#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/exceptions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

namespace
{
using Vector6 = std::array<double, 6>;

std::string armPrefix(const std::string & arm)
{
  return arm == "l" ? "UR10_l" : "UR10_r";
}

std::vector<double> checkedVector(
  rclcpp::Node & node,
  const std::string & name,
  const std::vector<double> & defaults)
{
  auto values = node.declare_parameter<std::vector<double>>(name, defaults);
  if (values.size() != defaults.size()) {
    RCLCPP_WARN(
      node.get_logger(), "Parameter '%s' has %zu entries, expected %zu; using defaults",
      name.c_str(), values.size(), defaults.size());
    return defaults;
  }
  return values;
}

double deadband(double value, double threshold)
{
  return std::abs(value) < threshold ? 0.0 : value;
}

double clamp(double value, double limit)
{
  const double abs_limit = std::abs(limit);
  if (abs_limit <= 0.0) {
    return value;
  }
  return std::clamp(value, -abs_limit, abs_limit);
}

tf2::Vector3 vectorFromMsg(const geometry_msgs::msg::Vector3 & msg)
{
  return {msg.x, msg.y, msg.z};
}

void vectorToMsg(const tf2::Vector3 & value, geometry_msgs::msg::Vector3 & msg)
{
  msg.x = value.x();
  msg.y = value.y();
  msg.z = value.z();
}

void vectorToMsg(const tf2::Vector3 & value, geometry_msgs::msg::Point & msg)
{
  msg.x = value.x();
  msg.y = value.y();
  msg.z = value.z();
}

tf2::Quaternion quaternionFromMsg(const geometry_msgs::msg::Quaternion & msg)
{
  tf2::Quaternion q(msg.x, msg.y, msg.z, msg.w);
  q.normalize();
  return q;
}

void quaternionToMsg(const tf2::Quaternion & value, geometry_msgs::msg::Quaternion & msg)
{
  tf2::Quaternion q = value;
  q.normalize();
  msg.x = q.x();
  msg.y = q.y();
  msg.z = q.z();
  msg.w = q.w();
}

tf2::Transform transformFromMsg(const geometry_msgs::msg::TransformStamped & msg)
{
  tf2::Transform transform;
  transform.setOrigin(vectorFromMsg(msg.transform.translation));
  transform.setRotation(quaternionFromMsg(msg.transform.rotation));
  return transform;
}

tf2::Quaternion quaternionFromRotationVector(const tf2::Vector3 & rotation_vector)
{
  const double angle = rotation_vector.length();
  tf2::Quaternion q;
  if (angle < 1.0e-12) {
    q.setValue(0.0, 0.0, 0.0, 1.0);
  } else {
    q.setRotation(rotation_vector.normalized(), angle);
  }
  q.normalize();
  return q;
}

tf2::Vector3 rotationVectorFromQuaternion(tf2::Quaternion q)
{
  q.normalize();
  if (q.w() < 0.0) {
    q = tf2::Quaternion(-q.x(), -q.y(), -q.z(), -q.w());
  }

  const double angle = q.getAngle();
  if (angle < 1.0e-12) {
    return {0.0, 0.0, 0.0};
  }
  return q.getAxis() * angle;
}

geometry_msgs::msg::PoseStamped poseMsg(
  const std::string & frame_id,
  const rclcpp::Time & stamp,
  const tf2::Vector3 & position,
  const tf2::Quaternion & orientation)
{
  geometry_msgs::msg::PoseStamped msg;
  msg.header.frame_id = frame_id;
  msg.header.stamp = stamp;
  vectorToMsg(position, msg.pose.position);
  quaternionToMsg(orientation, msg.pose.orientation);
  return msg;
}
}  // namespace

class CartesianAdmittanceController : public rclcpp::Node
{
public:
  explicit CartesianAdmittanceController(const rclcpp::NodeOptions & options)
  : Node("cartesian_admittance_controller", options),
    tf_buffer_(get_clock()),
    tf_listener_(tf_buffer_),
    tf_broadcaster_(std::make_unique<tf2_ros::TransformBroadcaster>(*this))
  {
    robot_name_ = declare_parameter<std::string>("robot_name", "mur620");
    arm_ = declare_parameter<std::string>("arm", "r");
    const auto prefix = armPrefix(arm_);

    tf_base_frame_ = declare_parameter<std::string>(
      "tf_base_frame", robot_name_ + "/" + prefix + "/base_link");
    tf_tcp_frame_ = declare_parameter<std::string>(
      "tf_tcp_frame", robot_name_ + "/" + prefix + "/tool0");
    command_frame_ = declare_parameter<std::string>(
      "command_frame", prefix + "/base_link");
    input_twist_topic_ = declare_parameter<std::string>(
      "input_twist_topic", "/" + robot_name_ + "/cartesian_admittance_controller_" + arm_ +
      "/equilibrium_twist_cmd");
    output_twist_topic_ = declare_parameter<std::string>(
      "output_twist_topic", "/" + robot_name_ + "/jparse_velocity_controller_" + arm_ + "/twist_cmd");
    wrench_topic_ = declare_parameter<std::string>(
      "wrench_topic", "/" + robot_name_ + "/" + prefix +
      "/force_torque_sensor_broadcaster/ft_data");
    equilibrium_frame_ = declare_parameter<std::string>(
      "equilibrium_frame", robot_name_ + "/" + prefix + "/admittance_equilibrium_pose");
    target_frame_ = declare_parameter<std::string>(
      "target_frame", robot_name_ + "/" + prefix + "/admittance_target_pose");

    rate_hz_ = std::max(1.0, declare_parameter<double>("rate_hz", 500.0));
    command_timeout_ = std::max(0.0, declare_parameter<double>("command_timeout", 0.12));
    wrench_timeout_ = std::max(0.0, declare_parameter<double>("wrench_timeout", 0.5));
    wrench_bias_duration_ = std::max(0.0, declare_parameter<double>("wrench_bias_duration", 1.0));
    wrench_filter_alpha_ = std::clamp(
      declare_parameter<double>("wrench_filter_alpha", 0.02), 0.0, 1.0);
    force_deadband_ = std::max(0.0, declare_parameter<double>("force_deadband", 0.0));
    torque_deadband_ = std::max(0.0, declare_parameter<double>("torque_deadband", 0.0));
    max_linear_velocity_ = std::max(0.0, declare_parameter<double>("max_linear_velocity", 0.10));
    max_angular_velocity_ = std::max(0.0, declare_parameter<double>("max_angular_velocity", 0.35));
    publish_tf_rate_hz_ = std::max(0.0, declare_parameter<double>("publish_tf_rate_hz", 50.0));
    require_wrench_ = declare_parameter<bool>("require_wrench", true);
    bias_ready_ = !require_wrench_;

    admittance_ = checkedVector(
      *this, "admittance", {0.0006, 0.0006, 0.0015, 0.0, 0.0, 0.0});
    pose_error_gain_ = checkedVector(
      *this, "pose_error_gain", {0.5, 0.5, 0.5, 0.4, 0.4, 0.4});
    wrench_sign_ = checkedVector(
      *this, "wrench_sign", {1.0, 1.0, 1.0, 1.0, 1.0, 1.0});

    twist_sub_ = create_subscription<geometry_msgs::msg::TwistStamped>(
      input_twist_topic_, rclcpp::SystemDefaultsQoS(),
      [this](const geometry_msgs::msg::TwistStamped::SharedPtr msg) {
        updateCommand(*msg);
      });
    wrench_sub_ = create_subscription<geometry_msgs::msg::WrenchStamped>(
      wrench_topic_, rclcpp::SensorDataQoS(),
      [this](const geometry_msgs::msg::WrenchStamped::SharedPtr msg) {
        updateWrench(*msg);
      });

    output_twist_pub_ = create_publisher<geometry_msgs::msg::TwistStamped>(
      output_twist_topic_, rclcpp::SystemDefaultsQoS());
    filtered_wrench_pub_ = create_publisher<geometry_msgs::msg::WrenchStamped>(
      "/" + robot_name_ + "/cartesian_admittance_controller_" + arm_ + "/filtered_wrench", 10);
    equilibrium_pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
      "/" + robot_name_ + "/cartesian_admittance_controller_" + arm_ + "/equilibrium_pose", 10);
    target_pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(
      "/" + robot_name_ + "/cartesian_admittance_controller_" + arm_ + "/target_pose", 10);

    const auto period = std::chrono::duration<double>(1.0 / rate_hz_);
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      [this]() { update(); });

    RCLCPP_INFO(
      get_logger(),
      "Cartesian admittance controller ready for %s %s: input=%s, wrench=%s, output=%s",
      robot_name_.c_str(), arm_.c_str(), input_twist_topic_.c_str(),
      wrench_topic_.c_str(), output_twist_topic_.c_str());
  }

private:
  void updateCommand(const geometry_msgs::msg::TwistStamped & msg)
  {
    const auto & frame = msg.header.frame_id;
    if (!frame.empty() && frame != command_frame_ && frame != tf_base_frame_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Equilibrium twist frame_id '%s' is not '%s'; interpreting it in the arm base frame",
        frame.c_str(), command_frame_.c_str());
    }

    target_twist_[0] = msg.twist.linear.x;
    target_twist_[1] = msg.twist.linear.y;
    target_twist_[2] = msg.twist.linear.z;
    target_twist_[3] = msg.twist.angular.x;
    target_twist_[4] = msg.twist.angular.y;
    target_twist_[5] = msg.twist.angular.z;
    last_command_time_ = now();
    have_command_ = true;
  }

  void updateWrench(const geometry_msgs::msg::WrenchStamped & msg)
  {
    const std::string source_frame =
      msg.header.frame_id.empty() ? tf_tcp_frame_ : msg.header.frame_id;

    tf2::Matrix3x3 rotation;
    try {
      const auto transform_msg =
        tf_buffer_.lookupTransform(tf_base_frame_, source_frame, tf2::TimePointZero);
      rotation = transformFromMsg(transform_msg).getBasis();
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Could not transform wrench from '%s' to '%s': %s",
        source_frame.c_str(), tf_base_frame_.c_str(), ex.what());
      return;
    }

    const tf2::Vector3 force = rotation * vectorFromMsg(msg.wrench.force);
    const tf2::Vector3 torque = rotation * vectorFromMsg(msg.wrench.torque);
    Vector6 sample = {
      force.x() * wrench_sign_[0],
      force.y() * wrench_sign_[1],
      force.z() * wrench_sign_[2],
      torque.x() * wrench_sign_[3],
      torque.y() * wrench_sign_[4],
      torque.z() * wrench_sign_[5],
    };

    if (!bias_ready_) {
      accumulateBias(sample);
      return;
    }

    for (std::size_t i = 0; i < filtered_wrench_.size(); ++i) {
      const double unbiased = sample[i] - wrench_bias_[i];
      filtered_wrench_[i] =
        (1.0 - wrench_filter_alpha_) * filtered_wrench_[i] + wrench_filter_alpha_ * unbiased;
    }
    filtered_wrench_[0] = deadband(filtered_wrench_[0], force_deadband_);
    filtered_wrench_[1] = deadband(filtered_wrench_[1], force_deadband_);
    filtered_wrench_[2] = deadband(filtered_wrench_[2], force_deadband_);
    filtered_wrench_[3] = deadband(filtered_wrench_[3], torque_deadband_);
    filtered_wrench_[4] = deadband(filtered_wrench_[4], torque_deadband_);
    filtered_wrench_[5] = deadband(filtered_wrench_[5], torque_deadband_);

    last_wrench_time_ = now();
    have_wrench_ = true;
  }

  void accumulateBias(const Vector6 & sample)
  {
    if (bias_sample_count_ == 0) {
      bias_start_time_ = now();
    }
    for (std::size_t i = 0; i < wrench_bias_.size(); ++i) {
      wrench_bias_[i] += sample[i];
    }
    ++bias_sample_count_;

    if (wrench_bias_duration_ <= 0.0 ||
      (now() - bias_start_time_).seconds() >= wrench_bias_duration_)
    {
      for (double & value : wrench_bias_) {
        value /= static_cast<double>(std::max<std::size_t>(1, bias_sample_count_));
      }
      bias_ready_ = true;
      RCLCPP_INFO(
        get_logger(),
        "Wrench bias ready from %zu samples: [%.3f %.3f %.3f %.3f %.3f %.3f]",
        bias_sample_count_, wrench_bias_[0], wrench_bias_[1], wrench_bias_[2],
        wrench_bias_[3], wrench_bias_[4], wrench_bias_[5]);
    }
  }

  bool lookupCurrentTcp(tf2::Transform & tcp) const
  {
    try {
      const auto transform_msg =
        tf_buffer_.lookupTransform(tf_base_frame_, tf_tcp_frame_, tf2::TimePointZero);
      tcp = transformFromMsg(transform_msg);
      return true;
    } catch (const tf2::TransformException & ex) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Could not get current TCP transform %s -> %s: %s",
        tf_base_frame_.c_str(), tf_tcp_frame_.c_str(), ex.what());
      return false;
    }
  }

  void initializeEquilibrium(const tf2::Transform & current_tcp)
  {
    equilibrium_position_ = current_tcp.getOrigin();
    equilibrium_orientation_ = current_tcp.getRotation();
    equilibrium_orientation_.normalize();
    last_update_time_ = now();
    last_tf_pub_time_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
    equilibrium_initialized_ = true;
    RCLCPP_INFO(
      get_logger(), "Captured admittance equilibrium pose from current TCP in %s",
      tf_base_frame_.c_str());
  }

  Vector6 activeCommand() const
  {
    if (!have_command_ || (now() - last_command_time_).seconds() > command_timeout_) {
      return {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    }
    return target_twist_;
  }

  bool wrenchFresh() const
  {
    if (!require_wrench_) {
      return true;
    }
    return have_wrench_ && (now() - last_wrench_time_).seconds() <= wrench_timeout_;
  }

  void integrateEquilibrium(const Vector6 & command, double dt)
  {
    equilibrium_position_ += tf2::Vector3(command[0], command[1], command[2]) * dt;

    const tf2::Vector3 angular_delta(command[3] * dt, command[4] * dt, command[5] * dt);
    const tf2::Quaternion delta_q = quaternionFromRotationVector(angular_delta);
    equilibrium_orientation_ = delta_q * equilibrium_orientation_;
    equilibrium_orientation_.normalize();
  }

  void update()
  {
    tf2::Transform current_tcp;
    if (!lookupCurrentTcp(current_tcp)) {
      publishZero();
      return;
    }
    if (!equilibrium_initialized_) {
      initializeEquilibrium(current_tcp);
    }

    const auto stamp = now();
    const double dt = std::clamp((stamp - last_update_time_).seconds(), 0.0, 0.02);
    last_update_time_ = stamp;

    if (!bias_ready_ || !wrenchFresh()) {
      publishZero();
      publishState(stamp, current_tcp, true);
      return;
    }

    const Vector6 command = activeCommand();
    integrateEquilibrium(command, dt);

    const tf2::Vector3 admittance_translation(
      filtered_wrench_[0] * admittance_[0],
      filtered_wrench_[1] * admittance_[1],
      filtered_wrench_[2] * admittance_[2]);
    const tf2::Vector3 admittance_rotation(
      filtered_wrench_[3] * admittance_[3],
      filtered_wrench_[4] * admittance_[4],
      filtered_wrench_[5] * admittance_[5]);

    const tf2::Vector3 target_position = equilibrium_position_ + admittance_translation;
    tf2::Quaternion target_orientation =
      quaternionFromRotationVector(admittance_rotation) * equilibrium_orientation_;
    target_orientation.normalize();

    const tf2::Vector3 position_error = target_position - current_tcp.getOrigin();
    const tf2::Quaternion orientation_error_q =
      target_orientation * current_tcp.getRotation().inverse();
    const tf2::Vector3 orientation_error = rotationVectorFromQuaternion(orientation_error_q);

    geometry_msgs::msg::TwistStamped out;
    out.header.stamp = stamp;
    out.header.frame_id = command_frame_;
    out.twist.linear.x = clamp(command[0] + position_error.x() * pose_error_gain_[0], max_linear_velocity_);
    out.twist.linear.y = clamp(command[1] + position_error.y() * pose_error_gain_[1], max_linear_velocity_);
    out.twist.linear.z = clamp(command[2] + position_error.z() * pose_error_gain_[2], max_linear_velocity_);
    out.twist.angular.x = clamp(command[3] + orientation_error.x() * pose_error_gain_[3], max_angular_velocity_);
    out.twist.angular.y = clamp(command[4] + orientation_error.y() * pose_error_gain_[4], max_angular_velocity_);
    out.twist.angular.z = clamp(command[5] + orientation_error.z() * pose_error_gain_[5], max_angular_velocity_);
    output_twist_pub_->publish(out);

    latest_target_position_ = target_position;
    latest_target_orientation_ = target_orientation;
    publishState(stamp, current_tcp, false);
  }

  void publishZero()
  {
    geometry_msgs::msg::TwistStamped out;
    out.header.stamp = now();
    out.header.frame_id = command_frame_;
    output_twist_pub_->publish(out);
  }

  void publishState(
    const rclcpp::Time & stamp,
    const tf2::Transform & current_tcp,
    bool target_is_equilibrium)
  {
    const tf2::Vector3 target_position =
      target_is_equilibrium ? equilibrium_position_ : latest_target_position_;
    const tf2::Quaternion target_orientation =
      target_is_equilibrium ? equilibrium_orientation_ : latest_target_orientation_;

    geometry_msgs::msg::WrenchStamped wrench_msg;
    wrench_msg.header.stamp = stamp;
    wrench_msg.header.frame_id = tf_base_frame_;
    wrench_msg.wrench.force.x = filtered_wrench_[0];
    wrench_msg.wrench.force.y = filtered_wrench_[1];
    wrench_msg.wrench.force.z = filtered_wrench_[2];
    wrench_msg.wrench.torque.x = filtered_wrench_[3];
    wrench_msg.wrench.torque.y = filtered_wrench_[4];
    wrench_msg.wrench.torque.z = filtered_wrench_[5];
    filtered_wrench_pub_->publish(wrench_msg);

    equilibrium_pose_pub_->publish(
      poseMsg(tf_base_frame_, stamp, equilibrium_position_, equilibrium_orientation_));
    target_pose_pub_->publish(
      poseMsg(tf_base_frame_, stamp, target_position, target_orientation));

    if (publish_tf_rate_hz_ <= 0.0) {
      return;
    }
    if (last_tf_pub_time_.nanoseconds() != 0 &&
      (stamp - last_tf_pub_time_).seconds() < 1.0 / publish_tf_rate_hz_)
    {
      return;
    }
    last_tf_pub_time_ = stamp;

    std::vector<geometry_msgs::msg::TransformStamped> transforms;
    transforms.reserve(2);
    transforms.push_back(makeTransform(stamp, equilibrium_frame_, equilibrium_position_, equilibrium_orientation_));
    transforms.push_back(makeTransform(stamp, target_frame_, target_position, target_orientation));
    (void)current_tcp;
    tf_broadcaster_->sendTransform(transforms);
  }

  geometry_msgs::msg::TransformStamped makeTransform(
    const rclcpp::Time & stamp,
    const std::string & child_frame,
    const tf2::Vector3 & translation,
    const tf2::Quaternion & rotation) const
  {
    geometry_msgs::msg::TransformStamped msg;
    msg.header.stamp = stamp;
    msg.header.frame_id = tf_base_frame_;
    msg.child_frame_id = child_frame;
    vectorToMsg(translation, msg.transform.translation);
    quaternionToMsg(rotation, msg.transform.rotation);
    return msg;
  }

  std::string robot_name_;
  std::string arm_;
  std::string tf_base_frame_;
  std::string tf_tcp_frame_;
  std::string command_frame_;
  std::string input_twist_topic_;
  std::string output_twist_topic_;
  std::string wrench_topic_;
  std::string equilibrium_frame_;
  std::string target_frame_;

  double rate_hz_{500.0};
  double command_timeout_{0.12};
  double wrench_timeout_{0.5};
  double wrench_bias_duration_{1.0};
  double wrench_filter_alpha_{0.02};
  double force_deadband_{0.0};
  double torque_deadband_{0.0};
  double max_linear_velocity_{0.10};
  double max_angular_velocity_{0.35};
  double publish_tf_rate_hz_{50.0};
  bool require_wrench_{true};

  std::vector<double> admittance_;
  std::vector<double> pose_error_gain_;
  std::vector<double> wrench_sign_;

  Vector6 target_twist_{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  Vector6 wrench_bias_{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  Vector6 filtered_wrench_{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

  tf2::Vector3 equilibrium_position_{0.0, 0.0, 0.0};
  tf2::Quaternion equilibrium_orientation_{0.0, 0.0, 0.0, 1.0};
  tf2::Vector3 latest_target_position_{0.0, 0.0, 0.0};
  tf2::Quaternion latest_target_orientation_{0.0, 0.0, 0.0, 1.0};

  bool have_command_{false};
  bool have_wrench_{false};
  bool bias_ready_{false};
  bool equilibrium_initialized_{false};
  std::size_t bias_sample_count_{0};
  rclcpp::Time bias_start_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_command_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_wrench_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_update_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_tf_pub_time_{0, 0, RCL_ROS_TIME};

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr twist_sub_;
  rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr wrench_sub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr output_twist_pub_;
  rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr filtered_wrench_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr equilibrium_pose_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr target_pose_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CartesianAdmittanceController>(rclcpp::NodeOptions()));
  rclcpp::shutdown();
  return 0;
}
