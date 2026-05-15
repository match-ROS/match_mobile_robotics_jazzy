#include <cmath>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <unordered_set>

#include <gz/msgs/pose_v.pb.h>
#include <gz/transport/Node.hh>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>

namespace
{
double yaw_from_quaternion(const geometry_msgs::msg::Quaternion & q)
{
  const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
  const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
  return std::atan2(siny_cosp, cosy_cosp);
}

double normalize_angle(double angle)
{
  while (angle > M_PI) {
    angle -= 2.0 * M_PI;
  }
  while (angle < -M_PI) {
    angle += 2.0 * M_PI;
  }
  return angle;
}

rclcpp::Time stamp_from_gz_header(const gz::msgs::Header & header)
{
  if (header.has_stamp()) {
    return rclcpp::Time(
      static_cast<int32_t>(header.stamp().sec()),
      static_cast<uint32_t>(header.stamp().nsec()),
      RCL_ROS_TIME);
  }
  return rclcpp::Clock(RCL_ROS_TIME).now();
}
}  // namespace

class GzGroundTruthPublisher : public rclcpp::Node
{
public:
  GzGroundTruthPublisher()
  : Node("gz_ground_truth_publisher")
  {
    gz_pose_topic_ = declare_parameter<std::string>("gz_pose_topic", "/world/maze/pose/info");
    robot_name_ = declare_parameter<std::string>("robot_name", "mur620a");
    output_frame_id_ = declare_parameter<std::string>("output_frame_id", "map");
    child_frame_id_ = declare_parameter<std::string>(
      "child_frame_id", robot_name_ + "/base_footprint");
    const auto pose_topic = declare_parameter<std::string>(
      "pose_topic", "/" + robot_name_ + "/ground_truth/pose");
    const auto odom_topic = declare_parameter<std::string>(
      "odom_topic", "/" + robot_name_ + "/ground_truth/odom");

    target_names_ = {
      robot_name_,
      robot_name_ + "/base_footprint",
      robot_name_ + "::base_footprint",
      "base_footprint",
    };

    pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(pose_topic, 10);
    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>(odom_topic, 10);

    if (!gz_node_.Subscribe(gz_pose_topic_, &GzGroundTruthPublisher::on_pose_v, this)) {
      throw std::runtime_error("Failed to subscribe to Gazebo topic " + gz_pose_topic_);
    }

    RCLCPP_INFO(
      get_logger(),
      "Publishing ground truth from Gazebo topic '%s' for model '%s'",
      gz_pose_topic_.c_str(), robot_name_.c_str());
  }

private:
  void on_pose_v(const gz::msgs::Pose_V & msg)
  {
    const gz::msgs::Pose * robot_pose = nullptr;
    for (const auto & pose : msg.pose()) {
      if (is_target_pose(pose.name())) {
        robot_pose = &pose;
        break;
      }
    }

    if (!robot_pose) {
      log_available_names_once(msg);
      return;
    }

    if (!logged_match_) {
      logged_match_ = true;
      RCLCPP_INFO(get_logger(), "Using Gazebo pose '%s' as ground truth", robot_pose->name().c_str());
    }

    const auto stamp = stamp_from_gz_header(msg.header());
    auto pose_msg = make_pose(*robot_pose, stamp);
    auto odom_msg = make_odom(stamp, pose_msg);

    pose_pub_->publish(pose_msg);
    odom_pub_->publish(odom_msg);

    std::lock_guard<std::mutex> lock(previous_mutex_);
    previous_stamp_ = stamp;
    previous_pose_ = pose_msg.pose;
  }

  bool is_target_pose(const std::string & name) const
  {
    if (target_names_.count(name) > 0) {
      return true;
    }
    return name.size() > robot_name_.size() &&
      name.rfind(robot_name_) == name.size() - robot_name_.size() &&
      (name[name.size() - robot_name_.size() - 1] == '/' ||
       name[name.size() - robot_name_.size() - 1] == ':');
  }

  void log_available_names_once(const gz::msgs::Pose_V & msg)
  {
    if (logged_available_names_) {
      return;
    }
    logged_available_names_ = true;

    std::string names;
    const int count = std::min(msg.pose_size(), 40);
    for (int i = 0; i < count; ++i) {
      if (!names.empty()) {
        names += ", ";
      }
      names += msg.pose(i).name();
    }
    if (msg.pose_size() > count) {
      names += ", ...";
    }
    RCLCPP_WARN(
      get_logger(),
      "No ground truth match for '%s'. Available Gazebo pose names: %s",
      robot_name_.c_str(), names.c_str());
  }

  geometry_msgs::msg::PoseStamped make_pose(
    const gz::msgs::Pose & gz_pose,
    const rclcpp::Time & stamp) const
  {
    geometry_msgs::msg::PoseStamped pose_msg;
    pose_msg.header.stamp = stamp;
    pose_msg.header.frame_id = output_frame_id_;
    pose_msg.pose.position.x = gz_pose.position().x();
    pose_msg.pose.position.y = gz_pose.position().y();
    pose_msg.pose.position.z = gz_pose.position().z();
    pose_msg.pose.orientation.x = gz_pose.orientation().x();
    pose_msg.pose.orientation.y = gz_pose.orientation().y();
    pose_msg.pose.orientation.z = gz_pose.orientation().z();
    pose_msg.pose.orientation.w = gz_pose.orientation().w();
    return pose_msg;
  }

  nav_msgs::msg::Odometry make_odom(
    const rclcpp::Time & stamp,
    const geometry_msgs::msg::PoseStamped & pose_msg)
  {
    nav_msgs::msg::Odometry odom_msg;
    odom_msg.header = pose_msg.header;
    odom_msg.child_frame_id = child_frame_id_;
    odom_msg.pose.pose = pose_msg.pose;

    std::lock_guard<std::mutex> lock(previous_mutex_);
    if (previous_stamp_ && previous_pose_) {
      const double dt = (stamp - *previous_stamp_).seconds();
      if (dt > 0.0) {
        odom_msg.twist.twist.linear.x =
          (pose_msg.pose.position.x - previous_pose_->position.x) / dt;
        odom_msg.twist.twist.linear.y =
          (pose_msg.pose.position.y - previous_pose_->position.y) / dt;
        odom_msg.twist.twist.linear.z =
          (pose_msg.pose.position.z - previous_pose_->position.z) / dt;

        const double yaw = yaw_from_quaternion(pose_msg.pose.orientation);
        const double previous_yaw = yaw_from_quaternion(previous_pose_->orientation);
        odom_msg.twist.twist.angular.z = normalize_angle(yaw - previous_yaw) / dt;
      }
    }

    return odom_msg;
  }

  std::string gz_pose_topic_;
  std::string robot_name_;
  std::string output_frame_id_;
  std::string child_frame_id_;
  std::unordered_set<std::string> target_names_;

  gz::transport::Node gz_node_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;

  bool logged_available_names_{false};
  bool logged_match_{false};
  std::mutex previous_mutex_;
  std::optional<rclcpp::Time> previous_stamp_;
  std::optional<geometry_msgs::msg::Pose> previous_pose_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<GzGroundTruthPublisher>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
