
#include <memory>
#include <vector>
#include <cmath>
#include <algorithm>
#include <limits>
#include "nav2_core/controller.hpp"
#include "nav2_core/exceptions.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "nav2_util/node_utils.hpp"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include <angles/angles.h>

using nav2_util::declare_parameter_if_not_declared;
using nav2_util::geometry_utils::euclidean_distance;

namespace custom_dwa_controller
{

template<typename Iter, typename Getter>
Iter min_by(Iter begin, Iter end, Getter getCompareVal)
{
  if (begin == end) return end;
  auto lowest = getCompareVal(*begin);
  Iter lowest_it = begin;
  for (Iter it = ++begin; it != end; ++it) {
    auto comp = getCompareVal(*it);
    if (comp < lowest) {
      lowest = comp;
      lowest_it = it;
    }
  }
  return lowest_it;
}

class DWALocalPlanner : public nav2_core::Controller
{
public:
  DWALocalPlanner() = default;
  ~DWALocalPlanner() override = default;

  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name, const std::shared_ptr<tf2_ros::Buffer> tf,
    const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override
  {
    node_ = parent.lock();
    name_ = name;
    tf_ = tf;
    costmap_ros_ = costmap_ros;

    // Parameter declaration
    declare_parameter_if_not_declared(node_, name_ + ".max_vel_x", rclcpp::ParameterValue(0.5));
    declare_parameter_if_not_declared(node_, name_ + ".min_vel_x", rclcpp::ParameterValue(0.1));
    declare_parameter_if_not_declared(node_, name_ + ".max_vel_theta", rclcpp::ParameterValue(1.0));
    declare_parameter_if_not_declared(node_, name_ + ".acc_lim_x", rclcpp::ParameterValue(2.0));
    declare_parameter_if_not_declared(node_, name_ + ".acc_lim_theta", rclcpp::ParameterValue(3.2));
    declare_parameter_if_not_declared(node_, name_ + ".sim_time", rclcpp::ParameterValue(1.5));
    declare_parameter_if_not_declared(node_, name_ + ".vx_samples", rclcpp::ParameterValue(20));
    declare_parameter_if_not_declared(node_, name_ + ".vtheta_samples", rclcpp::ParameterValue(40));
    declare_parameter_if_not_declared(node_, name_ + ".goal_tolerance", rclcpp::ParameterValue(0.25));
    declare_parameter_if_not_declared(node_, name_ + ".obstacle_weight", rclcpp::ParameterValue(1.5));
    declare_parameter_if_not_declared(node_, name_ + ".goal_weight", rclcpp::ParameterValue(1.0));
    declare_parameter_if_not_declared(node_, name_ + ".heading_weight", rclcpp::ParameterValue(5.0));
     declare_parameter_if_not_declared(node_, name_ + ".velocity_weight", rclcpp::ParameterValue(5.0));
    declare_parameter_if_not_declared(node_, name_ + ".transform_tolerance", rclcpp::ParameterValue(1.0));

    // Parameter retrieval
    node_->get_parameter(name_ + ".max_vel_x", max_vel_x_);
    node_->get_parameter(name_ + ".min_vel_x", min_vel_x_);
    node_->get_parameter(name_ + ".max_vel_theta", max_vel_theta_);
    node_->get_parameter(name_ + ".acc_lim_x", acc_lim_x_);
    node_->get_parameter(name_ + ".acc_lim_theta", acc_lim_theta_);
    node_->get_parameter(name_ + ".sim_time", sim_time_);
    node_->get_parameter(name_ + ".vx_samples", vx_samples_);
    node_->get_parameter(name_ + ".vtheta_samples", vtheta_samples_);
    node_->get_parameter(name_ + ".goal_tolerance", goal_tolerance_);
    node_->get_parameter(name_ + ".obstacle_weight", obstacle_weight_);
    node_->get_parameter(name_ + ".goal_weight", goal_weight_);
    node_->get_parameter(name_ + ".heading_weight", heading_weight_);
    node_->get_parameter(name_ + ".velocity_weight", velocity_weight_);

    double transform_tolerance;
    node_->get_parameter(name_ + ".transform_tolerance", transform_tolerance);
    transform_tolerance_ = rclcpp::Duration::from_seconds(transform_tolerance);
    
    // Visualization publishers
    trajectory_pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>("best_plan", 1);
    all_trajectories_pub_ = node_->create_publisher<visualization_msgs::msg::MarkerArray>("all_trajectories", 1);
    best_trajectory_pub_ = node_->create_publisher<nav_msgs::msg::Path>("best_trajectory_path", 1);
    global_pub_ = node_->create_publisher<nav_msgs::msg::Path>("received_global_plan", 1);
  }

  void cleanup() override {
      RCLCPP_INFO(
      node_->get_logger(),
      "Cleaning up controller: %s of type custom_dwa_controller::DWALocalPlanner",
      name_.c_str());
      
      global_pub_.reset();
      trajectory_pub_.reset();
    all_trajectories_pub_.reset();
    best_trajectory_pub_.reset();
    }
  void activate() override {
    global_pub_->on_activate();
    trajectory_pub_->on_activate();
    all_trajectories_pub_->on_activate();
    best_trajectory_pub_->on_activate();
  }

  void deactivate() override {
    global_pub_->on_deactivate();
    trajectory_pub_->on_deactivate();
    all_trajectories_pub_->on_deactivate();
    best_trajectory_pub_->on_deactivate();
  }
    void setSpeedLimit(const double& speed_limit, const bool& percentage) override
{
  (void) speed_limit;
  (void) percentage;
} 

  void setPlan(const nav_msgs::msg::Path & path)
{

  // Build an odom-frame version
  nav_msgs::msg::Path odom_plan;
  odom_plan.header.frame_id = "odom";  // e.g. "odom"
  odom_plan.header.stamp    = path.header.stamp;


  for (const auto & p : path.poses) {
    // transformPose is your helper that uses tf_ and transform_tolerance_
        geometry_msgs::msg::PoseStamped stamped, transformed;
    // 1) Copy the plan’s header (frame_id & timestamp)
    stamped.header = path.header;
    // 2) Copy the pose data itself
    stamped.pose   = p.pose;
    
    if (!transformPose(
          tf_,
          "odom",   // target frame, e.g. "odom"
          stamped,
          transformed,
          transform_tolerance_))
    {
      RCLCPP_WARN(node_->get_logger(),
        "  Failed to TF plan point (%.2f,%.2f) at t=%u.%u",
        p.pose.position.x, p.pose.position.y,
        p.header.stamp.sec, p.header.stamp.nanosec);
      continue;
    }
    odom_plan.poses.push_back(transformed);
  }

  if (odom_plan.poses.empty()) {
    RCLCPP_ERROR(node_->get_logger(),
      "All global plan points failed to transform to odom!");
    throw nav2_core::PlannerException("No valid odom-frame plan");
  }

  global_plan_ = odom_plan;
  RCLCPP_INFO(node_->get_logger(),
    "  Stored %zu poses in odom-frame global_plan",
    global_plan_.poses.size());
}

  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,
    const geometry_msgs::msg::Twist & velocity,
    nav2_core::GoalChecker * goal_checker) override
  {
    
    
    (void)goal_checker;

    auto vx_samples = sampleLinearVelocities(velocity.linear.x);
    auto vtheta_samples = sampleAngularVelocities(velocity.angular.z);

    visualization_msgs::msg::MarkerArray all_markers;
    int marker_id = 0;
    double best_score = -std::numeric_limits<double>::infinity();
    geometry_msgs::msg::Twist best_cmd;
    std::vector<geometry_msgs::msg::PoseStamped> best_trajectory;

    for (const auto& vx : vx_samples) {
      for (const auto& vtheta : vtheta_samples) {
        auto trajectory = generateTrajectory(pose, vx, vtheta);
        
        // Visualize all trajectories
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "odom";
        marker.header.stamp = node_->now();
        marker.ns = "candidate_trajectories";
        marker.id = marker_id++;
        marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.scale.x = 0.02;
        marker.color.a = 0.3;
        marker.color.b = 1.0;
        
        for (const auto& pose : trajectory) {
          geometry_msgs::msg::Point p;
          p.x = pose.pose.position.x;
          p.y = pose.pose.position.y;
          marker.points.push_back(p);
        }
        all_markers.markers.push_back(marker);

        // Evaluate trajectory
        double score = evaluateTrajectory(trajectory);
        if (score > best_score) {
          best_score = score;
          best_cmd.linear.x = vx;
          best_cmd.angular.z = vtheta;
          best_trajectory = trajectory;
        }
      }
    }

    // Publish visualization data
    all_trajectories_pub_->publish(all_markers);
    publishBestTrajectory(best_trajectory);
    publishTrajectoryMarker(best_trajectory);

    geometry_msgs::msg::TwistStamped cmd_vel;
    cmd_vel.header.stamp = node_->now();
    cmd_vel.header.frame_id = "base_link";
    cmd_vel.twist = best_cmd;
    RCLCPP_INFO(
    node_->get_logger(),
    "Best velocity published is lin %f,%f,%f and ang %f,%f,%f",
    cmd_vel.twist.linear.x,cmd_vel.twist.linear.y,cmd_vel.twist.linear.z,cmd_vel.twist.angular.x,cmd_vel.twist.angular.y,cmd_vel.twist.angular.z);
  
    return cmd_vel;
  }

  
protected:
  // Core DWA functions
  std::vector<double> sampleLinearVelocities(double current_vx) {
    std::vector<double> samples;
    double max_acc_vx = current_vx + acc_lim_x_ * 0.1;
    double min_acc_vx = current_vx - acc_lim_x_ * 0.1;
    double min_vx = std::max(min_vel_x_, min_acc_vx);
    double max_vx = std::min(max_vel_x_, max_acc_vx);
    double step = (max_vx - min_vx) / (vx_samples_ - 1);
    
    for (int i = 0; i < vx_samples_; ++i) {
      samples.push_back(min_vx + i * step);
    }
    return samples;
  }

  std::vector<double> sampleAngularVelocities(double current_vtheta) {
    std::vector<double> samples;
    double max_acc_vtheta = current_vtheta + acc_lim_theta_ * 0.1;
    double min_acc_vtheta = current_vtheta - acc_lim_theta_ * 0.1;
    double min_vtheta = std::max(-max_vel_theta_, min_acc_vtheta);
    double max_vtheta = std::min(max_vel_theta_, max_acc_vtheta);
    double step = (max_vtheta - min_vtheta) / (vtheta_samples_ - 1);
    
    for (int i = 0; i < vtheta_samples_; ++i) {
      samples.push_back(min_vtheta + i * step);
    }
    return samples;
  }

  std::vector<geometry_msgs::msg::PoseStamped> generateTrajectory(
    const geometry_msgs::msg::PoseStamped& start_pose,
    double vx, double vtheta)
{
  std::vector<geometry_msgs::msg::PoseStamped> trajectory;
  auto current_pose = start_pose;
  double yaw = tf2::getYaw(current_pose.pose.orientation);
  
  // Use the incoming header stamp as your time base
  rclcpp::Time t0 = rclcpp::Time(start_pose.header.stamp);
  const double dt = 0.1;
  double elapsed = 0.0;

  while (elapsed < sim_time_) {
    // 1) Integrate
    yaw    = angles::normalize_angle(yaw + vtheta * dt);
    current_pose.pose.position.x += vx * std::cos(yaw) * dt;
    current_pose.pose.position.y += vx * std::sin(yaw) * dt;

    // 2) Pack back into quaternion (and normalize)
    tf2::Quaternion q; 
    q.setRPY(0, 0, yaw);
    q.normalize();
    current_pose.pose.orientation = tf2::toMsg(q);

    // 3) Stamp and frame
    current_pose.header.stamp = t0 + rclcpp::Duration::from_seconds(elapsed + dt);
    current_pose.header.frame_id = costmap_ros_->getBaseFrameID();  // e.g. "odom"

    trajectory.push_back(current_pose);
    elapsed += dt;
  }

  return trajectory;
}


double evaluateTrajectory(
  const std::vector<geometry_msgs::msg::PoseStamped>& trajectory)
{
  if (trajectory.empty() || global_plan_.poses.empty()) {
    return -std::numeric_limits<double>::infinity();
  }

  // Grab start, end, and goal
  const auto & start_pose = trajectory.front();
  const auto & end_pose   = trajectory.back();
  const auto & goal_pose  = global_plan_.poses.size()>20 ? global_plan_.poses[20]:global_plan_.poses.back();

  // 1) HEADING: compare end_yaw → bearing(end → goal)
  double end_yaw = tf2::getYaw(end_pose.pose.orientation);
  double dx = goal_pose.pose.position.x - end_pose.pose.position.x;
  double dy = goal_pose.pose.position.y - end_pose.pose.position.y;
  double goal_bearing = std::atan2(dy, dx);
  double heading_error = angles::shortest_angular_distance(end_yaw, goal_bearing);
  double heading_score = std::cos(heading_error);

   // 2. DISTANCE COMPONENT - β · dist(v,ω) 
    // Distance to closest obstacle (clearance)
    double min_obstacle_dist = std::numeric_limits<double>::max();
    bool collision_free = true;
    
    for (const auto& pose : trajectory) {
        unsigned int mx, my;
        if (!costmap_ros_->getCostmap()->worldToMap(pose.pose.position.x, pose.pose.position.y, mx, my)) {
            return -std::numeric_limits<double>::infinity();
        }
        
        unsigned char cost = costmap_ros_->getCostmap()->getCost(mx, my);
        if (cost >= nav2_costmap_2d::LETHAL_OBSTACLE) {
            collision_free = false;
            break;
        }
        
        // Calculate distance to nearest obstacle
        double obstacle_dist = calculateObstacleDistance(pose.pose.position.x, pose.pose.position.y);
        min_obstacle_dist = std::min(min_obstacle_dist, obstacle_dist);
    }
    
    if (!collision_free) {
        return -std::numeric_limits<double>::infinity();
    }
    
    // Normalize obstacle distance (higher is better)
    double clearance_score = std::min(min_obstacle_dist / 2.0, 1.0); // Cap at 2m
    // double clearance_score = min_obstacle_dist;
    
    // 3. VELOCITY COMPONENT - γ · vel(v,ω)
    // Encourage faster forward movement
    double trajectory_length = 0.0;
    for (size_t i = 1; i < trajectory.size(); ++i) {
        double dx = trajectory[i].pose.position.x - trajectory[i-1].pose.position.x;
        double dy = trajectory[i].pose.position.y - trajectory[i-1].pose.position.y;
        trajectory_length += std::hypot(dx, dy);
    }
    double avg_velocity = trajectory_length / sim_time_;
    double velocity_score = avg_velocity / max_vel_x_; // Normalize by max velocity

  // 4) GOAL PROGRESS: start→goal vs end→goal
  double start_dx = goal_pose.pose.position.x - start_pose.pose.position.x;
  double start_dy = goal_pose.pose.position.y - start_pose.pose.position.y;
  double initial_goal_dist = std::hypot(start_dx, start_dy);

  double final_goal_dist = std::hypot(dx, dy);

  double progress_score =
    initial_goal_dist > 0.0
    ? std::max(0.0, (initial_goal_dist - final_goal_dist) / initial_goal_dist)
    : 0.0;

  // Combine
  double total_score =
      heading_weight_  * heading_score +
      obstacle_weight_ * clearance_score +
      velocity_weight_ * velocity_score +
      goal_weight_     * progress_score;

  return total_score;
}



 
  
    double calculateObstacleDistance(double x, double y) {
        unsigned int mx, my;
        if (!costmap_ros_->getCostmap()->worldToMap(x, y, mx, my)) {
            return 0.0;
        }
        
        nav2_costmap_2d::Costmap2D* costmap = costmap_ros_->getCostmap();
        double min_dist = std::numeric_limits<double>::max();
        
        // Search in a small radius for obstacles
        int search_radius = 20; // cells
        for (int dx = -search_radius; dx <= search_radius; ++dx) {
            for (int dy = -search_radius; dy <= search_radius; ++dy) {
                unsigned int check_x = mx + dx;
                unsigned int check_y = my + dy;
                
                if (check_x < costmap->getSizeInCellsX() && check_y < costmap->getSizeInCellsY()) {
                    unsigned char cost = costmap->getCost(check_x, check_y);
                    if (cost >= nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE) {
                        double dist = std::hypot(dx, dy) * costmap->getResolution();
                        min_dist = std::min(min_dist, dist);
                    }
                }
            }
        }
        
        return (min_dist == std::numeric_limits<double>::max()) ? 2.0 : min_dist;
    }

  // Visualization functions
  void publishBestTrajectory(const std::vector<geometry_msgs::msg::PoseStamped>& trajectory) {
    nav_msgs::msg::Path path;
    path.header.frame_id = "odom";
    path.header.stamp = node_->now();
    path.poses = trajectory;
    best_trajectory_pub_->publish(path);
  }

  void publishTrajectoryMarker(const std::vector<geometry_msgs::msg::PoseStamped>& trajectory) {
    visualization_msgs::msg::MarkerArray markers;
    visualization_msgs::msg::Marker marker;
    
    marker.header.frame_id = "odom";
    marker.header.stamp = node_->now();
    marker.ns = "best_trajectory";
    marker.id = 0;
    marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.scale.x = 0.05;
    marker.color.a = 1.0;
    marker.color.g = 1.0;

    for (const auto& pose : trajectory) {
      geometry_msgs::msg::Point p;
      p.x = pose.pose.position.x;
      p.y = pose.pose.position.y;
      marker.points.push_back(p);
    }
    markers.markers.push_back(marker);
    trajectory_pub_->publish(markers);
  }

bool transformPose(
  const std::shared_ptr<tf2_ros::Buffer> tf,
  const std::string frame,
  const geometry_msgs::msg::PoseStamped & in_pose,
  geometry_msgs::msg::PoseStamped & out_pose,
  const rclcpp::Duration & transform_tolerance
) const
{
  // Implementation taken as is fron nav_2d_utils in nav2_dwb_controller

  if (in_pose.header.frame_id == frame) {
    out_pose = in_pose;
    return true;
  }

  try {
    tf->transform(in_pose, out_pose, frame);
    return true;
  } catch (tf2::ExtrapolationException & ex) {
    auto transform = tf->lookupTransform(
      frame,
      in_pose.header.frame_id,
      tf2::TimePointZero
    );
    if (
      (rclcpp::Time(in_pose.header.stamp) - rclcpp::Time(transform.header.stamp)) >
      transform_tolerance)
    {
      RCLCPP_ERROR(
        rclcpp::get_logger("tf_help"),
        "Transform data too old when converting from %s to %s",
        in_pose.header.frame_id.c_str(),
        frame.c_str()
      );
      RCLCPP_ERROR(
        rclcpp::get_logger("tf_help"),
        "Data time: %ds %uns, Transform time: %ds %uns",
        in_pose.header.stamp.sec,
        in_pose.header.stamp.nanosec,
        transform.header.stamp.sec,
        transform.header.stamp.nanosec
      );
      return false;
    } else {
      tf2::doTransform(in_pose, out_pose, transform);
      return true;
    }
  } catch (tf2::TransformException & ex) {
    RCLCPP_ERROR(
      rclcpp::get_logger("tf_help"),
      "Exception in transformPose: %s",
      ex.what()
    );
    return false;
  }
  return false;
}


private:
  // Member variables
  rclcpp_lifecycle::LifecycleNode::SharedPtr node_;
  std::string name_;
  std::shared_ptr<tf2_ros::Buffer> tf_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
  nav_msgs::msg::Path global_plan_;
  rclcpp::Duration transform_tolerance_{0, 0};

  // Publishers
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<visualization_msgs::msg::MarkerArray>> trajectory_pub_;
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<visualization_msgs::msg::MarkerArray>> all_trajectories_pub_;
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>> best_trajectory_pub_;
  std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<nav_msgs::msg::Path>> global_pub_;

  // Parameters
  double max_vel_x_{0.5}, min_vel_x_{0.1}, max_vel_theta_{1.0};
  double acc_lim_x_{2.0}, acc_lim_theta_{3.2};
  double sim_time_{1.5}, goal_tolerance_{0.25};
  int vx_samples_{20}, vtheta_samples_{40};
  double obstacle_weight_{1.5}, goal_weight_{1.0}, heading_weight_{5.0}, velocity_weight_;
};

}  // namespace custom_dwa_controller

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(custom_dwa_controller::DWALocalPlanner, nav2_core::Controller);

