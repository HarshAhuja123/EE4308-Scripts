#include "ee4308_turtle/planner.hpp"
#include "ee4308_turtle/controller.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>

namespace ee4308::turtle
{

    // ====================== Planner Node ===================
    AStarNode::AStarNode(int new_c, int new_r) : c(new_c), r(new_r)  {}

    // ======================== Nav2 Planner Plugin ===============================
    void Planner::configure(
        const rclcpp_lifecycle::LifecycleNode::WeakPtr &parent,
        std::string name, const std::shared_ptr<tf2_ros::Buffer> tf,
        const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
    {
        // initialize states / variables
        this->node_ = parent.lock(); // this class is not a node. It is instantiated as part of a node `parent`.
        this->tf_ = tf;
        this->plugin_name_ = name;
        this->costmap_ = costmap_ros->getCostmap();
        this->global_frame_id_ = costmap_ros->getGlobalFrameID();
        this->path_pub_ = node_->create_publisher<nav_msgs::msg::Path>("planned_path", 10);

        // declare parameters to let the node know we are using these params.
        ee4308::initParam(this->node_, this->plugin_name_ + ".max_access_cost", this->max_access_cost_, 200);
        ee4308::initParam(this->node_, this->plugin_name_ + ".interpolation_distance", this->interpolation_distance_, 0.05);

    }

    // Converts world coordinates to cell column and cell row.
    std::pair<int, int> Planner::XYToCR_(double x, double y)
    {
        // The following functions may be used:
        //   costmap_->getOriginX()
        //   costmap_->getOriginY()
        //   costmap_->getResolution()
        //   std::ceil()
        //   std::floor()

        int c = std::floor((x - costmap_->getOriginX())/costmap_->getResolution());
        int r = std::floor((y - costmap_->getOriginY())/costmap_->getResolution());
        return {c, r};
    }
    
    // Converts cell column and cell row to world coordinates.
    std::pair<double, double> Planner::CRToXY_(int c, int r)
    {
        // The following functions may be used:
        //   this->costmap_->getResolution()
        //   this->costmap_->getOriginX()
        //   this->costmap_->getOriginY()
        double x = (c+0.5)*costmap_->getResolution() + costmap_->getOriginX();
        double y = (r+0.5)*costmap_->getResolution() + costmap_->getOriginY(); // +0.5 to ensure that center of cell is attained instead of the corner

        return {x, y};
    }


    // Converts cell column and cell row to flattened array index.
    int Planner::CRToIndex_(int c, int r)
    {
        return (r*costmap_->getSizeInCellsX())+c; // if we let flatten mean arranging all rows as a one lined vector
    }

    // Returns true if out of map, false otherwise.
    bool Planner::outOfMap_(  int c,  int r)
    {
        // The following functions may be used:
        return c  >= static_cast<int>(this->costmap_->getSizeInCellsX()) || r >= static_cast<int>(this->costmap_->getSizeInCellsY()) || c < 0 || r < 0 ;
    }

    nav_msgs::msg::Path Planner::createPlan(
        const geometry_msgs::msg::PoseStamped &start,
        const geometry_msgs::msg::PoseStamped &goal,
        std::function<bool()> cancel_checker)
    {
        // =========== Initializations ===================
        const int size_x = static_cast<int>(this->costmap_->getSizeInCellsX());
        const int size_y = static_cast<int>(this->costmap_->getSizeInCellsY());
        const int total_cells = size_x * size_y;

        // Create a vector of nodes (modify accordingly)
        std::vector<AStarNode> nodes;
        nodes.reserve(total_cells);
        for (int r = 0; r < size_y; ++r)
        {
            for (int c = 0; c < size_x; ++c)
            {
                nodes.emplace_back(c, r);
            }
        }

        // Create an open list
        OpenList<AStarNode *> open_list;

        // get the c,r map coordinates of the start and goal points
        auto [start_c, start_r] = this->XYToCR_(start.pose.position.x, start.pose.position.y);
        auto [goal_c, goal_r] = this->XYToCR_(goal.pose.position.x, goal.pose.position.y);

        auto makeEmptyPath = [this]() {
            nav_msgs::msg::Path path;
            path.header.frame_id = this->global_frame_id_;
            path.header.stamp = this->node_->now();
            return path;
        };

        if (this->outOfMap_(start_c, start_r) || this->outOfMap_(goal_c, goal_r))
        {
            RCLCPP_WARN(this->node_->get_logger(), "Start or goal is outside costmap bounds.");
            nav_msgs::msg::Path path = makeEmptyPath();
            path_pub_->publish(path);
            return path;
        }

        if (this->costmap_->getCost(start_c, start_r) >= this->max_access_cost_ ||
            this->costmap_->getCost(goal_c, goal_r) >= this->max_access_cost_)
        {
            RCLCPP_WARN(this->node_->get_logger(), "Start or goal lies in an occupied/blocked cell.");
            nav_msgs::msg::Path path = makeEmptyPath();
            path_pub_->publish(path);
            return path;
        }

        // do some start node initialization (modify accordingly)
        const int start_idx = this->CRToIndex_(start_c, start_r);
        AStarNode *start_node = &nodes[start_idx];
        start_node->g = 0.0;
        double L = std::max(std::abs(start_c - goal_c), std::abs(start_r - goal_r));
        double S = std::min(std::abs(start_c - goal_c), std::abs(start_r - goal_r));
        start_node->h = S * std::sqrt(2.0) + L - S; // octile heuristic
        start_node->f = start_node->h;
        start_node->parent = nullptr;
        open_list.push(start_node);
        start_node->expanded = false;
        std::vector<double> costs(static_cast<size_t>(total_cells), std::numeric_limits<double>::infinity());
        costs[start_idx] = 0.0;

        static const std::array<std::pair<int, int>, 8> kNeighborOffsets{
            std::pair<int, int>{1, 0},
            std::pair<int, int>{1, 1},
            std::pair<int, int>{0, 1},
            std::pair<int, int>{-1, 1},
            std::pair<int, int>{-1, 0},
            std::pair<int, int>{-1, -1},
            std::pair<int, int>{0, -1},
            std::pair<int, int>{1, -1},
        };

        // ================ Expansion loop ========================
        while (rclcpp::ok() && !open_list.empty())
        {
            if (cancel_checker && cancel_checker())
            {
                RCLCPP_WARN(this->node_->get_logger(), "Global planning cancelled.");
                nav_msgs::msg::Path path = makeEmptyPath();
                path_pub_->publish(path);
                return path;
            }

            // pop the cheapest
            AStarNode *node = open_list.top(); // gives us the address so that we can edit into the open_list directly.
            open_list.pop();
            if (node->expanded == true)
            {
                continue;
            } // ignore if the area is already expanded
            // do something if goal found
            if (goal_c == node->c && goal_r == node->r)
            { // smoothing implemented within this already
                nav_msgs::msg::Path path = this->writeToPath_(node, goal);
                path_pub_->publish(path);
                return path;
            }
            node->expanded = true;
            // do stuff in expansion loop

            // ================ Neighbor loop ========================
            for (auto [dc, dr] : kNeighborOffsets)
            {
                int nb_c = node->c + dc;
                int nb_r = node->r + dr;
                if (outOfMap_(nb_c, nb_r))
                {
                    continue;
                }

                if (costmap_->getCost(nb_c, nb_r) >= max_access_cost_)
                {
                    continue;
                }

                // Avoid corner-cutting through diagonally touching obstacles.
                if (dc != 0 && dr != 0)
                {
                    if (costmap_->getCost(node->c + dc, node->r) >= max_access_cost_ ||
                        costmap_->getCost(node->c, node->r + dr) >= max_access_cost_)
                    {
                        continue;
                    }
                }

                AStarNode *nxt = &nodes[this->CRToIndex_(nb_c, nb_r)];
                const int nb_idx = this->CRToIndex_(nb_c, nb_r);
                const double step_cost = std::hypot(static_cast<double>(dc), static_cast<double>(dr));
                const double tentative_g = node->g + static_cast<double>(costmap_->getCost(nb_c, nb_r)) + step_cost;

                if (tentative_g < costs[nb_idx])
                {
                    nxt->g = tentative_g;
                    costs[nb_idx] = nxt->g;
                    nxt->parent = node;
                    L = std::max(std::abs(nb_c - goal_c), std::abs(nb_r - goal_r));
                    S = std::min(std::abs(nb_c - goal_c), std::abs(nb_r - goal_r));
                    nxt->h = S * std::sqrt(2.0) + L - S; // octile heuristic
                    nxt->f = nxt->g + nxt->h;
                    open_list.push(nxt);
                }
            }
        }

        RCLCPP_WARN(this->node_->get_logger(), "No path found from start to goal.");
        nav_msgs::msg::Path path = makeEmptyPath();
        path_pub_->publish(path);
        return path;
    }











    nav_msgs::msg::Path Planner::writeToPath_(
        AStarNode *goal_node,
        geometry_msgs::msg::PoseStamped goal)
    {
        // setup the path message
        nav_msgs::msg::Path path;
        path.poses.clear();
        path.header.frame_id = this->global_frame_id_;
        path.header.stamp = this->node_->now();

        if (goal_node == nullptr)
        {
            return path;
        }

        // Reconstruct coarse path (goal->start), then reverse to get start->goal.
        AStarNode *node = goal_node;
        while (node != nullptr)
        { 
            // convert map coordinates to world coordinates
            auto [wx, wy] = this->CRToXY_(node->c, node->r);
            
            // push the pose into the messages.
            geometry_msgs::msg::PoseStamped pose;
            pose.header.frame_id = this->global_frame_id_;
            pose.header.stamp = path.header.stamp;
            pose.pose.position.x = wx;
            pose.pose.position.y = wy;
            pose.pose.orientation.w = 1.0; // normalized quaternion
            path.poses.push_back(pose);

            // go to the next node
            node = node->parent;
        }
        
        std::reverse(path.poses.begin(), path.poses.end());

        // Densify the path for easier local tracking.
        if (this->interpolation_distance_ > 0.0 && path.poses.size() > 1)
        {
            std::vector<geometry_msgs::msg::PoseStamped> dense_poses;
            dense_poses.reserve(path.poses.size());
            dense_poses.push_back(path.poses.front());

            for (size_t i = 0; i < path.poses.size() - 1; ++i)
            {
                const auto &p0 = path.poses[i].pose.position;
                const auto &p1 = path.poses[i + 1].pose.position;
                const double segment_length = std::hypot(p1.x - p0.x, p1.y - p0.y);
                const int segments = std::max(1, static_cast<int>(std::ceil(segment_length / this->interpolation_distance_)));

                for (int s = 1; s <= segments; ++s)
                {
                    const double t = static_cast<double>(s) / static_cast<double>(segments);
                    geometry_msgs::msg::PoseStamped pose;
                    pose.header.frame_id = this->global_frame_id_;
                    pose.header.stamp = path.header.stamp;
                    pose.pose.position.x = p0.x + t * (p1.x - p0.x);
                    pose.pose.position.y = p0.y + t * (p1.y - p0.y);
                    pose.pose.position.z = p0.z + t * (p1.z - p0.z);
                    pose.pose.orientation.w = 1.0;
                    dense_poses.push_back(pose);
                }
            }

            path.poses = std::move(dense_poses);
        }

        // Use exact final goal pose (position + final yaw requested by Nav2).
        goal.header.frame_id = this->global_frame_id_;
        goal.header.stamp = path.header.stamp;
        path.poses.back() = goal;

        // Orient each pose along the next segment.
        for (size_t i = 0; i + 1 < path.poses.size(); ++i)
        {
            auto &p0 = path.poses[i].pose.position;
            auto &p1 = path.poses[i + 1].pose.position;

            const double yaw = std::atan2(p1.y - p0.y, p1.x - p0.x);
            tf2::Quaternion q;
            q.setRPY(0.0, 0.0, yaw);
            path.poses[i].pose.orientation = tf2::toMsg(q);
        }
                Eigen::MatrixXd J;
        J.resize(2*sg_half_window_+1, sg_order_+1); // r
        for(int r = 0 ; r<J.rows() ; ++r){
            for (int c = 0; c < J.cols() ; ++c){
                J(r,c)=std::pow((r-sg_half_window_),c);
            }
        }
        Eigen::MatrixXd A = ((J.transpose() * J).inverse() * J.transpose()).row(0);
        
        //return path;
        
        nav_msgs::msg::Path smooth;
        smooth.poses.clear();
        smooth.header.frame_id = this->global_frame_id_;
        smooth.header.stamp = this->node_->now();
        for (long unsigned int i = 0 ; i<path.poses.size(); ++i){
            double xn = 0.0;
            double yn = 0.0;
            for (long int j = -sg_half_window_ ; j<=sg_half_window_ ; ++j){
                int k = static_cast<int>(i)+j;
               if(k<0){
                    k = 0 ;
                }
                else if (i+j>path.poses.size()-1){ k=path.poses.size()-1;}
                xn+=path.poses[k].pose.position.x*A(0,j+sg_half_window_);
                yn+=path.poses[k].pose.position.y*A(0,j+sg_half_window_);
            }
            geometry_msgs::msg::PoseStamped pose; // do not fill the header with timestamp or frame information. 
            pose.pose.position.x = xn;
            pose.pose.position.y = yn;
            pose.pose.orientation.w = 1; // normalized quaternion
            smooth.poses.push_back(pose);
        }
        smooth.poses.back()=goal; // this is because the SMOOTHER DRIFTS POINTS, so the final goal state is possibly lost information. re-establishing fixes the bug (i.e. the controller was seen to follow some auto-linear path instead of the clearly plotted smoothened one without this line.)
        return smooth;
        //return path;
    }

    // ======================================== DO NOT TOUCH =================================

    void Planner::cleanup() { RCLCPP_INFO_STREAM(this->node_->get_logger(), "Cleaning up plugin " << plugin_name_ << " of type ee4308::turtle::Planner"); }

    void Planner::activate() { RCLCPP_INFO_STREAM(this->node_->get_logger(), "Activating plugin " << plugin_name_ << " of type ee4308::turtle::Planner"); }

    void Planner::deactivate() { RCLCPP_INFO_STREAM(this->node_->get_logger(), "Deactivating plugin " << plugin_name_ << " of type ee4308::turtle::Planner"); }
}

PLUGINLIB_EXPORT_CLASS(ee4308::turtle::Planner, nav2_core::GlobalPlanner)