#include "ee4308_turtle/controller.hpp"

namespace ee4308::turtle
{
    void Controller::configure(
        const rclcpp_lifecycle::LifecycleNode::WeakPtr &parent,
        std::string name, const std::shared_ptr<tf2_ros::Buffer> tf,
        const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
    {
        (void)costmap_ros;

        // initialize states / variables
        this->node_ = parent.lock(); // this class is not a node_. It is instantiated as part of a node_ `parent`.
        this->tf_ = tf;
        this->plugin_name_ = name;

        // initialize parameters
        ee4308::initParam(this->node_, this->plugin_name_ + ".desired_linear_vel", this->desired_linear_vel_, 0.2);
        ee4308::initParam(this->node_, this->plugin_name_ + ".desired_lookahead_dist", this->desired_lookahead_dist_, 0.4);
        ee4308::initParam(this->node_, this->plugin_name_ + ".max_angular_vel", this->max_angular_vel_, 1.0);
        ee4308::initParam(this->node_, this->plugin_name_ + ".max_linear_vel", this->max_linear_vel_, 0.22);
        ee4308::initParam(this->node_, this->plugin_name_ + ".xy_goal_thres", this->xy_goal_thres_, 0.05);
        ee4308::initParam(this->node_, this->plugin_name_ + ".yaw_goal_thres", this->yaw_goal_thres_, 0.25);

        // initialize topics
        this->sub_scan_ = this->node_->create_subscription<sensor_msgs::msg::LaserScan>(
            "scan", rclcpp::SensorDataQoS(),
            std::bind(&Controller::callbackSubScan_, this, std::placeholders::_1));
    }

    void Controller::callbackSubScan_(sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        this->scan_ranges_ = msg->ranges;
    }

    geometry_msgs::msg::TwistStamped Controller::computeVelocityCommands(
        const geometry_msgs::msg::PoseStamped &rbt_pose_odom,
        const geometry_msgs::msg::Twist &velocity,
        nav2_core::GoalChecker *goal_checker)
    {
        (void)velocity;     // not used
        (void)goal_checker; // not used

        // check if path exists
        if (global_plan_.poses.empty())
        {
            RCLCPP_WARN_STREAM(node_->get_logger(), "Global plan is empty!");
            return writeCmdVel(0, 0);
        }

        // get rbt's pose in map frame (DO NOT DELETE --> need the next two lines for rbt_pose)
        geometry_msgs::msg::PoseStamped rbt_pose;
        tf_->transform(rbt_pose_odom, rbt_pose, "map");

        const double xr = rbt_pose.pose.position.x;
        const double yr = rbt_pose.pose.position.y;
        const double phi_r = ee4308::getYawFromQuaternion(rbt_pose.pose.orientation);

        // get goal pose (contains the "clicked" goal rotation and position)
        geometry_msgs::msg::PoseStamped goal_pose = global_plan_.poses.back();
        const double xg = goal_pose.pose.position.x;
        const double yg = goal_pose.pose.position.y;
        const double phi_g = ee4308::getYawFromQuaternion(goal_pose.pose.orientation);

        // goal checker
        const double dist_to_goal = std::hypot(xg - xr, yg - yr);
        const double yaw_to_goal = phi_g - phi_r;
        if (dist_to_goal <= xy_goal_thres_)
        {
            if (abs(yaw_to_goal) <= yaw_goal_thres_){
            return writeCmdVel(0.0, 0.0);
            }
            else {
                return writeCmdVel(0.0, std::clamp(yaw_to_goal, -max_angular_vel_,max_angular_vel_));
            }
        }

        // get lookahead?
        geometry_msgs::msg::PoseStamped lookahead_pose = goal_pose;
        RCLCPP_WARN_STREAM(node_->get_logger(),
        "lookaheadDist" << desired_lookahead_dist_);

        for (std::size_t i = 0; i < global_plan_.poses.size(); i++){
            const double dist = std::hypot(global_plan_.poses[i].pose.position.x - xr, global_plan_.poses[i].pose.position.y - yr);
            if (dist>desired_lookahead_dist_)
                break;
            lookahead_pose = global_plan_.poses[i];            
        }

        const double xl = lookahead_pose.pose.position.x;
        const double yl = lookahead_pose.pose.position.y;
        
        const double dx = xl - xr;
        const double dy = yl - yr;
        
        const double cphi = std::cos(phi_r);
        const double sphi = std::sin(phi_r);
        
        const double x_prime = cphi * dx + sphi * dy;
        const double y_prime = -sphi * dx + cphi * dy;

        double curvature = 2*y_prime / (x_prime*x_prime + y_prime*y_prime);
        
        double linear_vel = desired_linear_vel_;
        // double angular_vel = linear_vel * curvature;
  
        const double curvature_threshold = 0.5;
        const double proximity_threshold = 0.1;
        const double lookahead_gain = 2.0;

        //curvature heuristic
        double lin_vel_c = (curvature_threshold < curvature) ? linear_vel * (curvature_threshold / curvature) : linear_vel;

        // Proximity Calc
        double d_o = std::numeric_limits<double>::infinity();

        for (float r : scan_ranges_) {
            if (std::isfinite(r) && r > 0.05f) {
                d_o = std::min(d_o, (double)r);
            }
        }
        //proximity heuristic
        double final_lin_vel_ = (d_o < proximity_threshold) ? lin_vel_c * (d_o / proximity_threshold) : lin_vel_c;

        desired_lookahead_dist_ = std::clamp(final_lin_vel_ * lookahead_gain, 0.3, 1.0);
        final_lin_vel_ = std::clamp(final_lin_vel_, -max_linear_vel_,max_linear_vel_);
        double angular_vel = final_lin_vel_ * curvature;
        double final_ang_vel_ = std::clamp(angular_vel,-max_angular_vel_,max_angular_vel_);
        return writeCmdVel(final_lin_vel_, final_ang_vel_);
    }

    geometry_msgs::msg::TwistStamped Controller::writeCmdVel(double linear_vel, double angular_vel)
    {
        geometry_msgs::msg::TwistStamped cmd_vel;
        cmd_vel.header.frame_id = "odom";
        cmd_vel.header.stamp = this->node_->now();
        cmd_vel.twist.linear.x = linear_vel;
        cmd_vel.twist.angular.z = angular_vel;
        return cmd_vel;
    }

    // ======================================== DO NOT TOUCH =================================

    void Controller::cleanup() { RCLCPP_INFO_STREAM(this->node_->get_logger(), "Cleaning up plugin " << plugin_name_ << " of type ee4308::turtle::Controller"); }

    void Controller::activate() { RCLCPP_INFO_STREAM(this->node_->get_logger(), "Activating plugin " << plugin_name_ << " of type ee4308::turtle::Controller"); }

    void Controller::deactivate() { RCLCPP_INFO_STREAM(this->node_->get_logger(), "Deactivating plugin " << plugin_name_ << " of type ee4308::turtle::Controller"); }

    void Controller::setSpeedLimit(const double &speed_limit, const bool &percentage)
    {
        (void)speed_limit;
        (void)percentage;
    }

    void Controller::setPlan(const nav_msgs::msg::Path &path) { this->global_plan_ = path; }
}

PLUGINLIB_EXPORT_CLASS(ee4308::turtle::Controller, nav2_core::Controller)