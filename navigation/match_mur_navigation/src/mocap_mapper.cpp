#include "match_mur_navigation/mapping_core.hpp"
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

using namespace std::chrono_literals;

class MocapMapper : public rclcpp::Node {
 public:
  MocapMapper() : Node("mocap_mapper"),
    grid_(declare_parameter("resolution", 0.02), declare_parameter("max_cells", 25000000)),
    poses_(declare_parameter("max_pose_gap", 0.1)), buffer_(get_clock()), listener_(buffer_, this, false) {
    robot_ = declare_parameter<std::string>("robot", "mur620d");
    base_ = robot_ + "/base_link";
    timeout_ = declare_parameter("pose_timeout", 0.5);
    min_ = declare_parameter("min_range", 0.1);
    max_ = declare_parameter("max_range", 10.0);
    const auto period = declare_parameter("map_update_interval", 1.0);
    if (!std::isfinite(min_) || !std::isfinite(max_) || min_ < 0 || max_ <= min_ ||
        !std::isfinite(timeout_) || timeout_ <= 0 || !std::isfinite(period) || period <= 0)
      throw std::invalid_argument("Invalid mapping parameters");
    map_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>("map_raw", rclcpp::QoS(1).transient_local());
    status_pub_ = create_publisher<std_msgs::msg::String>("mapper_status", 1);
    pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>("pose", 100,
      [this](geometry_msgs::msg::PoseStamped::ConstSharedPtr msg) {
        if (msg->header.frame_id != "mocap") return;
        const auto &p = msg->pose.position; const auto &q = msg->pose.orientation;
        if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z) ||
            !std::isfinite(q.x) || !std::isfinite(q.y) || !std::isfinite(q.z) || !std::isfinite(q.w)) return;
        tf2::Quaternion quat(q.x, q.y, q.z, q.w);
        if (std::abs(quat.length2() - 1.0) > 0.01) return;
        const double t = rclcpp::Time(msg->header.stamp).seconds();
        if (std::abs(now().seconds() - t) > timeout_) return;
        poses_.add(t, tf2::Transform(quat.normalized(), tf2::Vector3(p.x, p.y, p.z)));
      });
    for (const auto &topic : {"front_scan", "rear_scan"})
      scan_subs_.push_back(create_subscription<sensor_msgs::msg::LaserScan>(topic, rclcpp::SensorDataQoS(),
        [this](sensor_msgs::msg::LaserScan::ConstSharedPtr msg) {
          if (!recording_) return;
          if (pending_.size() >= 40) { pending_.pop_front(); ++discarded_; }
          pending_.push_back({msg, std::chrono::steady_clock::now()});
        }));
    recording_sub_ = create_subscription<std_msgs::msg::Bool>("recording", rclcpp::QoS(1).transient_local(),
      [this](std_msgs::msg::Bool::ConstSharedPtr msg) {
        recording_ = msg->data;
        if (!recording_) { discarded_ += pending_.size(); pending_.clear(); publish_map(); }
      });
    work_timer_ = create_wall_timer(10ms, [this]() { process(); });
    map_timer_ = create_wall_timer(std::chrono::duration<double>(period), [this]() { publish_map(); });
    status_timer_ = create_wall_timer(500ms, [this]() {
      std_msgs::msg::String msg;
      msg.data = "{\"integrated\":" + std::to_string(integrated_) + ",\"discarded\":" +
        std::to_string(discarded_) + ",\"reason\":\"" + reason_ + "\"}";
      status_pub_->publish(msg);
    });
  }
 private:
  struct Pending {
    sensor_msgs::msg::LaserScan::ConstSharedPtr scan;
    std::chrono::steady_clock::time_point received;
  };
  bool integrate(const sensor_msgs::msg::LaserScan &scan) {
    if (scan.ranges.empty() || !std::isfinite(scan.angle_min) || !std::isfinite(scan.angle_increment) ||
        !std::isfinite(scan.range_min) || !std::isfinite(scan.range_max) || scan.range_max <= scan.range_min ||
        !std::isfinite(scan.time_increment) || scan.time_increment < 0) {
      reason_ = "invalid_scan"; ++discarded_; return true;
    }
    const double stamp = rclcpp::Time(scan.header.stamp).seconds();
    const double end = stamp + (scan.ranges.size() - 1) * scan.time_increment;
    tf2::Transform first, last;
    if (!poses_.at(stamp, first) || !poses_.at(end, last)) {
      reason_ = "waiting_for_pose"; return false;
    }
    tf2::Transform extrinsic;
    try {
      // Only ask for the local base-to-scanner transform, never a global robot pose.
      const auto transform = buffer_.lookupTransform(base_, scan.header.frame_id, tf2::TimePointZero);
      tf2::fromMsg(transform.transform, extrinsic);
    } catch (const tf2::TransformException &) { reason_ = "waiting_for_scanner_tf"; return false; }
    const double near = std::max(min_, static_cast<double>(scan.range_min));
    const double far = std::min(max_, static_cast<double>(scan.range_max));
    if (far <= near) { reason_ = "invalid_scan_limits"; ++discarded_; return true; }
    std::vector<mur_mapping::Ray> rays; rays.reserve(scan.ranges.size());
    for (size_t i = 0; i < scan.ranges.size(); ++i) {
      const double range = scan.ranges[i];
      if (std::isnan(range) || range < near || range == -INFINITY) continue;
      // Values outside the scanner's documented range are not valid returns.
      if (std::isfinite(range) && range > scan.range_max) continue;
      tf2::Transform pose;
      if (!poses_.at(stamp + i * scan.time_increment, pose)) { reason_ = "waiting_for_pose"; return false; }
      const auto laser = pose * extrinsic;
      const double angle = scan.angle_min + i * scan.angle_increment;
      const auto origin = laser.getOrigin();
      const double length = std::min(range, far);
      const auto point = laser * tf2::Vector3(length * std::cos(angle), length * std::sin(angle), 0);
      rays.push_back({origin.x(), origin.y(), point.x(), point.y(), std::isfinite(range) && range < far});
    }
    if (rays.empty()) { reason_ = "no_valid_rays"; ++discarded_; return true; }
    if (!grid_.integrate(rays)) { reason_ = "grid_limit"; ++discarded_; return true; }
    ++integrated_; reason_ = "recording"; return true;
  }
  void process() {
    const double clock = now().seconds();
    if (last_clock_ > clock) { poses_.clear(); discarded_ += pending_.size(); pending_.clear(); }
    last_clock_ = clock;
    // Process each scanner independently: an unavailable rear pose/TF must not block the front.
    for (auto it = pending_.begin(); it != pending_.end();) {
      const double age = clock - rclcpp::Time(it->scan->header.stamp).seconds();
      const double waiting = std::chrono::duration<double>(std::chrono::steady_clock::now() - it->received).count();
      if (age > timeout_ || waiting > timeout_ || age < -timeout_) {
        ++discarded_; reason_ = "scan_expired"; it = pending_.erase(it);
      } else if (integrate(*it->scan)) { it = pending_.erase(it); }
      else { ++it; }
    }
    if (recording_ && clock - poses_.latest() > timeout_) reason_ = "waiting_for_mocap";
    if (!recording_) reason_ = "stopped";
  }
  void publish_map() {
    if (!grid_.width()) return;
    nav_msgs::msg::OccupancyGrid msg;
    msg.header.frame_id = "mocap"; msg.header.stamp = now();
    msg.info.map_load_time = msg.header.stamp;
    msg.info.resolution = grid_.resolution(); msg.info.width = grid_.width(); msg.info.height = grid_.height();
    msg.info.origin.position.x = grid_.origin_x(); msg.info.origin.position.y = grid_.origin_y();
    msg.info.origin.orientation.w = 1; msg.data = grid_.data();
    map_pub_->publish(msg);
  }
  mur_mapping::Grid grid_;
  mur_mapping::PoseBuffer poses_;
  tf2_ros::Buffer buffer_;
  tf2_ros::TransformListener listener_;
  std::string robot_, base_, reason_ = "waiting_for_mocap";
  double min_, max_, timeout_, last_clock_ = 0;
  uint64_t integrated_ = 0, discarded_ = 0;
  bool recording_ = true;
  std::deque<Pending> pending_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr recording_sub_;
  std::vector<rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr> scan_subs_;
  rclcpp::TimerBase::SharedPtr work_timer_, map_timer_, status_timer_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  try { rclcpp::spin(std::make_shared<MocapMapper>()); }
  catch (const std::exception &e) {
    RCLCPP_FATAL(rclcpp::get_logger("mocap_mapper"), "%s", e.what());
    rclcpp::shutdown(); return 1;
  }
  rclcpp::shutdown(); return 0;
}
