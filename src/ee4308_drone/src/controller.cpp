#include "ee4308_drone/controller.hpp"
#include <algorithm>
namespace ee4308::drone
{
    Controller::Controller(
        const rclcpp::NodeOptions &options,
        const std::string &name = "controller")
        : Node(name, options)
    {
        this->frequency_ = ee4308::getParameter<double>(this, "frequency", 20.0).as_double();
        this->enable_ = ee4308::getParameter<bool>(this, "enable", true).as_bool();
        this->lookahead_distance_ = ee4308::getParameter<double>(this, "lookahead_distance", 1.0).as_double();
        this->max_xy_vel_ = ee4308::getParameter<double>(this, "max_xy_vel", 1.0).as_double();
        this->max_z_vel_ = ee4308::getParameter<double>(this, "max_z_vel", 0.5).as_double();
        this->yaw_vel_ = ee4308::getParameter<double>(this, "yaw_vel", -0.5236).as_double();
        this->kp_xy_ = ee4308::getParameter<double>(this, "kp_xy", 1.0).as_double();
        this->kp_z_ = ee4308::getParameter<double>(this, "kp_z", 1.0).as_double();

        this->pub_cmd_vel_ = this->create_publisher<geometry_msgs::msg::Twist>(
            "cmd_vel", rclcpp::ServicesQoS());
        this->sub_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "odom", rclcpp::SensorDataQoS(),
            std::bind(&Controller::callbackSubOdom_, this, std::placeholders::_1));
        this->sub_plan_ = this->create_subscription<nav_msgs::msg::Path>(
            "plan", rclcpp::SensorDataQoS(),
            std::bind(&Controller::callbackSubPlan_, this, std::placeholders::_1));

        this->received_odom_ = false;

        this->timer_ = this->create_timer(1s / this->frequency_, std::bind(&Controller::callbackTimer_, this));
    }

    void Controller::callbackSubOdom_(const nav_msgs::msg::Odometry msg)
    {
        this->odom_ = msg;
        this->received_odom_ = true;
    }

    void Controller::callbackSubPlan_(const nav_msgs::msg::Path msg)
    {
        this->plan_ = msg;
    }

    void Controller::callbackTimer_()
    {
        if (!enable_)
            return;

        if (!this->received_odom_)
        {
            publishCmdVel_(0, 0, 0, 0);
            return;
        }

        if (plan_.poses.empty())
        {
            publishCmdVel_(0, 0, 0, 0);
            return;
        }
        
        const double drone_x = odom_.pose.pose.position.x;
        const double drone_y = odom_.pose.pose.position.y;
        const double drone_z = odom_.pose.pose.position.z;
        const double yaw = ee4308::getYawFromQuaternion(odom_.pose.pose.orientation);
        double dist = (0.0);

        geometry_msgs::msg::PoseStamped lookahead_pose = plan_.poses.back();

        for (std::size_t i = 0; i < plan_.poses.size(); i++)
        {
            dist = std::hypot(plan_.poses[i].pose.position.x - drone_x,
                plan_.poses[i].pose.position.y - drone_y,
                plan_.poses[i].pose.position.z - drone_z);
            if (dist >= lookahead_distance_)
            {
                lookahead_pose = plan_.poses[i];
                break;
            }
        }

        const double dx= lookahead_pose.pose.position.x - drone_x;
        const double dy = lookahead_pose.pose.position.y - drone_y;
        const double dz = lookahead_pose.pose.position.z - drone_z;

        const double cos_yaw = std::cos(yaw);
        const double sin_yaw = std::sin(yaw);

        const double x_prime = cos_yaw * dx + sin_yaw * dy;
        const double y_prime = -sin_yaw * dx + cos_yaw * dy;

        double cmd_x = kp_xy_ * x_prime;
        double cmd_y = kp_xy_ * y_prime;
        double cmd_z = kp_z_ * dz;

        const double xy_speed = std::hypot(cmd_x, cmd_y);
        if (xy_speed > max_xy_vel_ && xy_speed > ee4308::THRES)
        {
            const double scale = max_xy_vel_ / xy_speed;
            cmd_x *= scale;
            cmd_y *= scale;
        }

        cmd_z = std::clamp(cmd_z, -max_z_vel_, max_z_vel_);

        publishCmdVel_(cmd_x, cmd_y, cmd_z, yaw_vel_);
    }

    void Controller::publishCmdVel_(double x_vel, double y_vel, double z_vel, double yaw_vel)
    {
        geometry_msgs::msg::Twist cmd_vel;
        cmd_vel.linear.x = x_vel;
        cmd_vel.linear.y = y_vel;
        cmd_vel.linear.z = z_vel;
        cmd_vel.angular.z = yaw_vel;
        if (!std::isfinite(x_vel) || !std::isfinite(y_vel) || !std::isfinite(z_vel) || !std::isfinite(yaw_vel))
        {
           // RCLCPP_WARN(this->get_logger(),
             //   "Cmd velocities are inf or nan. Controller or estimator problem. CmdVels(x,y,z,yaw): %6.3f, %6.3f, %6.3f, %6.3f",
               // x_vel, y_vel, z_vel, yaw_vel);
        }
        pub_cmd_vel_->publish(cmd_vel);
    }
}

RCLCPP_COMPONENTS_REGISTER_NODE(ee4308::drone::Controller);
