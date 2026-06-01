#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <limits>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2/exceptions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

using namespace std::chrono_literals;

namespace
{
constexpr double PI = 3.14159265358979323846;

std::string jointSuffix(const std::string & joint_name)
{
  const auto slash = joint_name.find_last_of('/');
  return slash == std::string::npos ? joint_name : joint_name.substr(slash + 1);
}

std::vector<double> defaultLimitsForJoints(
  const std::vector<std::string> & joint_names,
  const std::map<std::string, double> & values,
  double fallback)
{
  std::vector<double> limits;
  limits.reserve(joint_names.size());
  for (const auto & joint_name : joint_names) {
    const auto it = values.find(jointSuffix(joint_name));
    limits.push_back(it == values.end() ? fallback : it->second);
  }
  return limits;
}

double distance(
  const geometry_msgs::msg::Vector3 & a,
  const geometry_msgs::msg::Vector3 & b)
{
  const double dx = a.x - b.x;
  const double dy = a.y - b.y;
  const double dz = a.z - b.z;
  return std::sqrt(dx * dx + dy * dy + dz * dz);
}
}  // namespace

class ArmVelocitySafetyNode : public rclcpp::Node
{
public:
  explicit ArmVelocitySafetyNode(const rclcpp::NodeOptions & options)
  : Node("arm_velocity_safety_node", options),
    tf_buffer_(get_clock()),
    tf_listener_(tf_buffer_)
  {
    robot_name_ = declare_parameter<std::string>("robot_name", "mur620");
    joint_states_topic_ = declare_parameter<std::string>("joint_states_topic", "/joint_states");
    rate_hz_ = std::max(1.0, declare_parameter<double>("rate_hz", 500.0));
    command_timeout_ = declare_parameter<double>("command_timeout", 0.15);
    max_joint_velocity_ = declare_parameter<double>("max_joint_velocity", 0.6);
    max_joint_acceleration_ = declare_parameter<double>("max_joint_acceleration", 0.4);
    max_joint_jerk_ = declare_parameter<double>("max_joint_jerk", 1.0);
    joint_limit_margin_ = declare_parameter<double>("joint_limit_margin", 0.02);
    enable_collision_avoidance_ = declare_parameter<bool>("enable_collision_avoidance", true);
    collision_stop_distance_ = declare_parameter<double>("collision_stop_distance", 0.14);
    collision_release_distance_ = declare_parameter<double>("collision_release_distance", 0.18);
    collision_fail_safe_stop_ = declare_parameter<bool>("collision_fail_safe_stop", false);
    collision_common_frame_ = declare_parameter<std::string>(
      "collision_common_frame", robot_name_ + "/base_link");
    collision_frames_l_ = declare_parameter<std::vector<std::string>>(
      "collision_frames_l",
      {
        robot_name_ + "/UR10_l/shoulder_link",
        robot_name_ + "/UR10_l/upper_arm_link",
        robot_name_ + "/UR10_l/forearm_link",
        robot_name_ + "/UR10_l/wrist_1_link",
        robot_name_ + "/UR10_l/wrist_2_link",
        robot_name_ + "/UR10_l/wrist_3_link",
        robot_name_ + "/UR10_l/tool0",
      });
    collision_frames_r_ = declare_parameter<std::vector<std::string>>(
      "collision_frames_r",
      {
        robot_name_ + "/UR10_r/shoulder_link",
        robot_name_ + "/UR10_r/upper_arm_link",
        robot_name_ + "/UR10_r/forearm_link",
        robot_name_ + "/UR10_r/wrist_1_link",
        robot_name_ + "/UR10_r/wrist_2_link",
        robot_name_ + "/UR10_r/wrist_3_link",
        robot_name_ + "/UR10_r/tool0",
      });

    left_ = makeArm("l", "UR10_l");
    right_ = makeArm("r", "UR10_r");
    configureJointLimits(left_);
    configureJointLimits(right_);

    left_.input_sub = create_subscription<std_msgs::msg::Float64MultiArray>(
      left_.input_topic, rclcpp::SystemDefaultsQoS(),
      [this](const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
        updateCommand(left_, *msg);
      });
    right_.input_sub = create_subscription<std_msgs::msg::Float64MultiArray>(
      right_.input_topic, rclcpp::SystemDefaultsQoS(),
      [this](const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
        updateCommand(right_, *msg);
      });
    left_.output_pub = create_publisher<std_msgs::msg::Float64MultiArray>(
      left_.output_topic, rclcpp::SystemDefaultsQoS());
    right_.output_pub = create_publisher<std_msgs::msg::Float64MultiArray>(
      right_.output_topic, rclcpp::SystemDefaultsQoS());
    status_pub_ = create_publisher<std_msgs::msg::String>(
      "/" + robot_name_ + "/arm_velocity_safety/status", 10);
    min_collision_distance_pub_ = create_publisher<std_msgs::msg::Float64>(
      "/" + robot_name_ + "/arm_velocity_safety/min_collision_distance", 10);

    joint_state_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      joint_states_topic_, rclcpp::SensorDataQoS(),
      [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
        updateJointState(*msg);
      });

    const auto period = std::chrono::duration<double>(1.0 / rate_hz_);
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      [this]() { update(); });

    RCLCPP_INFO(
      get_logger(),
      "MUR arm velocity safety ready: L %s -> %s, R %s -> %s, collision=%s",
      left_.input_topic.c_str(), left_.output_topic.c_str(),
      right_.input_topic.c_str(), right_.output_topic.c_str(),
      enable_collision_avoidance_ ? "true" : "false");
  }

private:
  struct ArmState
  {
    std::string side;
    std::string prefix;
    std::vector<std::string> joint_names;
    std::string input_topic;
    std::string output_topic;
    std::vector<double> latest_command;
    std::map<std::string, double> last_velocity_commands;
    std::map<std::string, double> last_acceleration_commands;
    std::vector<double> joint_lower_limits;
    std::vector<double> joint_upper_limits;
    std::vector<double> joint_velocity_limits;
    std::vector<double> joint_acceleration_limits;
    std::vector<double> joint_jerk_limits;
    rclcpp::Time last_input_time{0, 0, RCL_ROS_TIME};
    rclcpp::Time last_output_time{0, 0, RCL_ROS_TIME};
    bool have_input{false};
    bool idle_zero_sent{false};
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr input_sub;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr output_pub;
  };

  ArmState makeArm(const std::string & side, const std::string & prefix)
  {
    ArmState arm;
    arm.side = side;
    arm.prefix = prefix;
    arm.joint_names = {
      prefix + "/shoulder_pan_joint",
      prefix + "/shoulder_lift_joint",
      prefix + "/elbow_joint",
      prefix + "/wrist_1_joint",
      prefix + "/wrist_2_joint",
      prefix + "/wrist_3_joint",
    };
    arm.input_topic = declare_parameter<std::string>(
      side + "_input_topic",
      "/" + robot_name_ + "/" + prefix + "/safe_forward_velocity_controller/commands");
    arm.output_topic = declare_parameter<std::string>(
      side + "_output_topic",
      "/" + robot_name_ + "/" + prefix + "/forward_velocity_controller/commands");
    arm.latest_command.assign(arm.joint_names.size(), 0.0);
    return arm;
  }

  std::vector<double> checkedLimitVector(
    const std::string & name,
    const std::vector<double> & defaults)
  {
    auto values = declare_parameter<std::vector<double>>(name, defaults);
    if (values.size() != defaults.size()) {
      RCLCPP_WARN(
        get_logger(), "Parameter '%s' has %zu entries, expected %zu; using defaults",
        name.c_str(), values.size(), defaults.size());
      return defaults;
    }
    return values;
  }

  void configureJointLimits(ArmState & arm)
  {
    const std::map<std::string, double> lower_defaults = {
      {"shoulder_pan_joint", -2.0 * PI},
      {"shoulder_lift_joint", -2.0 * PI},
      {"elbow_joint", -PI},
      {"wrist_1_joint", -2.0 * PI},
      {"wrist_2_joint", -2.0 * PI},
      {"wrist_3_joint", -2.0 * PI},
    };
    const std::map<std::string, double> upper_defaults = {
      {"shoulder_pan_joint", 2.0 * PI},
      {"shoulder_lift_joint", 2.0 * PI},
      {"elbow_joint", PI},
      {"wrist_1_joint", 2.0 * PI},
      {"wrist_2_joint", 2.0 * PI},
      {"wrist_3_joint", 2.0 * PI},
    };
    const std::map<std::string, double> velocity_defaults = {
      {"shoulder_pan_joint", 2.09439510239},
      {"shoulder_lift_joint", 2.09439510239},
      {"elbow_joint", PI},
      {"wrist_1_joint", PI},
      {"wrist_2_joint", PI},
      {"wrist_3_joint", PI},
    };
    const std::map<std::string, double> acceleration_defaults = {
      {"shoulder_pan_joint", max_joint_acceleration_},
      {"shoulder_lift_joint", max_joint_acceleration_},
      {"elbow_joint", max_joint_acceleration_},
      {"wrist_1_joint", max_joint_acceleration_},
      {"wrist_2_joint", max_joint_acceleration_},
      {"wrist_3_joint", max_joint_acceleration_},
    };
    const std::map<std::string, double> jerk_defaults = {
      {"shoulder_pan_joint", max_joint_jerk_},
      {"shoulder_lift_joint", max_joint_jerk_},
      {"elbow_joint", max_joint_jerk_},
      {"wrist_1_joint", max_joint_jerk_},
      {"wrist_2_joint", max_joint_jerk_},
      {"wrist_3_joint", max_joint_jerk_},
    };

    arm.joint_lower_limits = checkedLimitVector(
      arm.side + "_joint_lower_limits",
      defaultLimitsForJoints(arm.joint_names, lower_defaults, -2.0 * PI));
    arm.joint_upper_limits = checkedLimitVector(
      arm.side + "_joint_upper_limits",
      defaultLimitsForJoints(arm.joint_names, upper_defaults, 2.0 * PI));
    arm.joint_velocity_limits = checkedLimitVector(
      arm.side + "_joint_velocity_limits",
      defaultLimitsForJoints(arm.joint_names, velocity_defaults, max_joint_velocity_));
    arm.joint_acceleration_limits = checkedLimitVector(
      arm.side + "_joint_acceleration_limits",
      defaultLimitsForJoints(arm.joint_names, acceleration_defaults, max_joint_acceleration_));
    arm.joint_jerk_limits = checkedLimitVector(
      arm.side + "_joint_jerk_limits",
      defaultLimitsForJoints(arm.joint_names, jerk_defaults, max_joint_jerk_));
  }

  void updateCommand(
    ArmState & arm,
    const std_msgs::msg::Float64MultiArray & msg)
  {
    std::vector<double> command(arm.joint_names.size(), 0.0);
    const auto count = std::min(command.size(), msg.data.size());
    for (std::size_t i = 0; i < count; ++i) {
      command[i] = std::isfinite(msg.data[i]) ? msg.data[i] : 0.0;
    }
    arm.latest_command = command;
    arm.last_input_time = now();
    arm.have_input = true;
    arm.idle_zero_sent = false;
  }

  void updateJointState(const sensor_msgs::msg::JointState & msg)
  {
    for (std::size_t i = 0; i < msg.name.size(); ++i) {
      if (i < msg.position.size() && std::isfinite(msg.position[i])) {
        joint_positions_[msg.name[i]] = msg.position[i];
      }
    }
  }

  void publishCommand(ArmState & arm, const std::vector<double> & command)
  {
    std_msgs::msg::Float64MultiArray msg;
    msg.data = command;
    arm.output_pub->publish(msg);
  }

  void publishZero(ArmState & arm)
  {
    publishCommand(arm, std::vector<double>(arm.joint_names.size(), 0.0));
    arm.last_velocity_commands.clear();
    arm.last_acceleration_commands.clear();
  }

  std::vector<double> applyLimits(ArmState & arm, std::vector<double> command)
  {
    const rclcpp::Time current_time = now();
    double dt = arm.last_output_time.nanoseconds() == 0 ?
      1.0 / rate_hz_ : (current_time - arm.last_output_time).seconds();
    if (dt <= 0.0 || dt > 0.1) {
      dt = 1.0 / rate_hz_;
    }
    arm.last_output_time = current_time;

    for (std::size_t i = 0; i < command.size(); ++i) {
      const auto & joint_name = arm.joint_names[i];
      const double velocity_limit = std::min(
        std::abs(arm.joint_velocity_limits[i]), std::abs(max_joint_velocity_));
      command[i] = std::clamp(command[i], -velocity_limit, velocity_limit);

      const auto position = joint_positions_.find(joint_name);
      bool position_blocked = false;
      if (position != joint_positions_.end()) {
        const double lower = arm.joint_lower_limits[i] + joint_limit_margin_;
        const double upper = arm.joint_upper_limits[i] - joint_limit_margin_;
        if ((position->second >= upper && command[i] > 0.0) ||
            (position->second <= lower && command[i] < 0.0))
        {
          command[i] = 0.0;
          position_blocked = true;
        } else if (command[i] > 0.0) {
          command[i] = std::min(command[i], std::max(0.0, (upper - position->second) / dt));
        } else if (command[i] < 0.0) {
          command[i] = std::max(command[i], std::min(0.0, (lower - position->second) / dt));
        }
      }

      if (position_blocked) {
        arm.last_velocity_commands[joint_name] = 0.0;
        arm.last_acceleration_commands[joint_name] = 0.0;
        continue;
      }

      const double previous_velocity = arm.last_velocity_commands.count(joint_name) ?
        arm.last_velocity_commands[joint_name] : 0.0;
      const double previous_acceleration = arm.last_acceleration_commands.count(joint_name) ?
        arm.last_acceleration_commands[joint_name] : 0.0;
      double acceleration = (command[i] - previous_velocity) / dt;

      const double acceleration_limit = std::abs(arm.joint_acceleration_limits[i]);
      if (acceleration_limit > 0.0) {
        acceleration = std::clamp(acceleration, -acceleration_limit, acceleration_limit);
      }

      const double jerk_limit = std::abs(arm.joint_jerk_limits[i]);
      if (jerk_limit > 0.0) {
        const double max_acceleration_delta = jerk_limit * dt;
        acceleration = std::clamp(
          acceleration,
          previous_acceleration - max_acceleration_delta,
          previous_acceleration + max_acceleration_delta);
      }

      command[i] = previous_velocity + acceleration * dt;
      command[i] = std::clamp(command[i], -velocity_limit, velocity_limit);
      arm.last_velocity_commands[joint_name] = command[i];
      arm.last_acceleration_commands[joint_name] = acceleration;
    }

    return command;
  }

  bool computeMinCollisionDistance(double & min_distance)
  {
    min_distance = std::numeric_limits<double>::infinity();
    std::vector<geometry_msgs::msg::Vector3> points_l;
    std::vector<geometry_msgs::msg::Vector3> points_r;

    try {
      for (const auto & frame : collision_frames_l_) {
        points_l.push_back(
          tf_buffer_.lookupTransform(collision_common_frame_, frame, tf2::TimePointZero)
          .transform.translation);
      }
      for (const auto & frame : collision_frames_r_) {
        points_r.push_back(
          tf_buffer_.lookupTransform(collision_common_frame_, frame, tf2::TimePointZero)
          .transform.translation);
      }
    } catch (const tf2::TransformException & exc) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Collision TF lookup failed in frame '%s': %s",
        collision_common_frame_.c_str(), exc.what());
      return false;
    }

    for (const auto & left_point : points_l) {
      for (const auto & right_point : points_r) {
        min_distance = std::min(min_distance, distance(left_point, right_point));
      }
    }
    return std::isfinite(min_distance);
  }

  bool shouldStopForCollision()
  {
    if (!enable_collision_avoidance_) {
      return false;
    }

    double min_distance = std::numeric_limits<double>::infinity();
    const bool valid = computeMinCollisionDistance(min_distance);
    std_msgs::msg::Float64 min_msg;
    min_msg.data = valid ? min_distance : std::numeric_limits<double>::quiet_NaN();
    min_collision_distance_pub_->publish(min_msg);

    if (!valid) {
      return collision_fail_safe_stop_;
    }

    if (collision_stop_active_) {
      if (min_distance > collision_release_distance_) {
        collision_stop_active_ = false;
        publishStatus("collision_clear");
      }
    } else if (min_distance < collision_stop_distance_) {
      collision_stop_active_ = true;
      publishStatus("collision_stop");
    }
    return collision_stop_active_;
  }

  void publishStatus(const std::string & status)
  {
    std_msgs::msg::String msg;
    msg.data = status;
    status_pub_->publish(msg);
    RCLCPP_WARN(get_logger(), "Arm velocity safety status: %s", status.c_str());
  }

  bool commandTimedOut(const ArmState & arm) const
  {
    return !arm.have_input || (now() - arm.last_input_time).seconds() > command_timeout_;
  }

  void update()
  {
    const bool collision_stop = shouldStopForCollision();
    updateArm(left_, collision_stop);
    updateArm(right_, collision_stop);
  }

  void updateArm(ArmState & arm, bool force_zero)
  {
    if (force_zero) {
      publishZero(arm);
      return;
    }

    if (commandTimedOut(arm)) {
      if (!arm.idle_zero_sent) {
        publishZero(arm);
        arm.idle_zero_sent = true;
      }
      return;
    }

    publishCommand(arm, applyLimits(arm, arm.latest_command));
  }

  std::string robot_name_;
  std::string joint_states_topic_;
  double rate_hz_;
  double command_timeout_;
  double max_joint_velocity_;
  double max_joint_acceleration_;
  double max_joint_jerk_;
  double joint_limit_margin_;
  bool enable_collision_avoidance_;
  double collision_stop_distance_;
  double collision_release_distance_;
  bool collision_fail_safe_stop_;
  std::string collision_common_frame_;
  std::vector<std::string> collision_frames_l_;
  std::vector<std::string> collision_frames_r_;
  bool collision_stop_active_{false};

  ArmState left_;
  ArmState right_;
  std::map<std::string, double> joint_positions_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr min_collision_distance_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ArmVelocitySafetyNode>(rclcpp::NodeOptions()));
  rclcpp::shutdown();
  return 0;
}
