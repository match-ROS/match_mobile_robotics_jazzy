#include <algorithm>
#include <array>
#include <cmath>
#include <cctype>
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Dense>
#include <controller_interface/chainable_controller_interface.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/frames.hpp>
#include <kdl/jacobian.hpp>
#include <kdl/jntarray.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <realtime_tools/realtime_thread_safe_box.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2_ros/transform_broadcaster.h>

namespace mur_control
{
namespace
{
using CallbackReturn = controller_interface::CallbackReturn;
using return_type = controller_interface::return_type;
using Vector6 = std::array<double, 6>;

constexpr double PI = 3.14159265358979323846;

std::string lower_copy(std::string value)
{
  std::transform(value.begin(), value.end(), value.begin(), [](unsigned char character) {
    return static_cast<char>(std::tolower(character));
  });
  return value;
}

std::string arm_prefix(const std::string & arm)
{
  return arm == "l" ? "UR10_l" : "UR10_r";
}

double deadband(double value, double threshold)
{
  return std::abs(value) < threshold ? 0.0 : value;
}

double clamp_abs(double value, double limit)
{
  const double abs_limit = std::abs(limit);
  if (abs_limit <= 0.0) {
    return value;
  }
  return std::clamp(value, -abs_limit, abs_limit);
}

tf2::Quaternion quaternion_from_rotation_vector(const tf2::Vector3 & rotation_vector)
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

tf2::Vector3 rotation_vector_from_quaternion(tf2::Quaternion q)
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

void vector_to_msg(const tf2::Vector3 & value, geometry_msgs::msg::Vector3 & msg)
{
  msg.x = value.x();
  msg.y = value.y();
  msg.z = value.z();
}

void vector_to_msg(const tf2::Vector3 & value, geometry_msgs::msg::Point & msg)
{
  msg.x = value.x();
  msg.y = value.y();
  msg.z = value.z();
}

void quaternion_to_msg(const tf2::Quaternion & value, geometry_msgs::msg::Quaternion & msg)
{
  tf2::Quaternion q = value;
  q.normalize();
  msg.x = q.x();
  msg.y = q.y();
  msg.z = q.z();
  msg.w = q.w();
}

geometry_msgs::msg::PoseStamped pose_msg(
  const std::string & frame_id,
  const rclcpp::Time & stamp,
  const tf2::Vector3 & position,
  const tf2::Quaternion & orientation)
{
  geometry_msgs::msg::PoseStamped msg;
  msg.header.frame_id = frame_id;
  msg.header.stamp = stamp;
  vector_to_msg(position, msg.pose.position);
  quaternion_to_msg(orientation, msg.pose.orientation);
  return msg;
}

Eigen::MatrixXd pseudo_inverse(const Eigen::MatrixXd & matrix, double tolerance)
{
  if (matrix.size() == 0) {
    return Eigen::MatrixXd(matrix.cols(), matrix.rows());
  }
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(matrix, Eigen::ComputeFullU | Eigen::ComputeFullV);
  const auto & singular_values = svd.singularValues();
  Eigen::MatrixXd sigma_inv = Eigen::MatrixXd::Zero(matrix.cols(), matrix.rows());
  for (Eigen::Index i = 0; i < singular_values.size(); ++i) {
    if (singular_values(i) > tolerance) {
      sigma_inv(i, i) = 1.0 / singular_values(i);
    }
  }
  return svd.matrixV() * sigma_inv * svd.matrixU().transpose();
}

Eigen::MatrixXd damped_least_squares_inverse(const Eigen::MatrixXd & jacobian, double damping)
{
  if (jacobian.size() == 0) {
    return Eigen::MatrixXd(jacobian.cols(), jacobian.rows());
  }
  const double lambda = std::max(0.0, damping);
  if (lambda <= std::numeric_limits<double>::epsilon()) {
    return pseudo_inverse(jacobian, 1.0e-6);
  }
  const Eigen::MatrixXd identity =
    Eigen::MatrixXd::Identity(jacobian.rows(), jacobian.rows());
  const Eigen::MatrixXd regularized =
    jacobian * jacobian.transpose() + lambda * lambda * identity;
  return jacobian.transpose() * regularized.ldlt().solve(identity);
}

Eigen::MatrixXd compose_svd(
  const Eigen::MatrixXd & u,
  const std::vector<double> & singular_values,
  const Eigen::MatrixXd & vt)
{
  Eigen::MatrixXd sigma = Eigen::MatrixXd::Zero(u.cols(), vt.rows());
  const auto count = std::min<Eigen::Index>(
    static_cast<Eigen::Index>(singular_values.size()), std::min(sigma.rows(), sigma.cols()));
  for (Eigen::Index i = 0; i < count; ++i) {
    sigma(i, i) = singular_values[static_cast<std::size_t>(i)];
  }
  return u * sigma * vt;
}

Eigen::MatrixXd compute_jparse_inverse(
  const Eigen::MatrixXd & jacobian,
  double gamma,
  double singular_gain_position,
  double singular_gain_angular,
  double pinv_tolerance,
  Eigen::VectorXd * singular_values_out,
  double * inverse_condition_out)
{
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(jacobian, Eigen::ComputeFullU | Eigen::ComputeFullV);
  const Eigen::MatrixXd u = svd.matrixU();
  const Eigen::MatrixXd vt = svd.matrixV().transpose();
  const Eigen::VectorXd singular_values = svd.singularValues();
  if (singular_values_out != nullptr) {
    *singular_values_out = singular_values;
  }

  const double sigma_max = singular_values.size() > 0 ? singular_values.maxCoeff() : 0.0;
  if (sigma_max <= std::numeric_limits<double>::epsilon()) {
    if (inverse_condition_out != nullptr) {
      *inverse_condition_out = 0.0;
    }
    return Eigen::MatrixXd::Zero(jacobian.cols(), jacobian.rows());
  }

  const double sigma_min = singular_values.minCoeff();
  if (inverse_condition_out != nullptr) {
    *inverse_condition_out = sigma_min / sigma_max;
  }

  std::vector<Eigen::VectorXd> non_singular_u_cols;
  std::vector<double> non_singular_values;
  std::vector<Eigen::VectorXd> singular_u_cols;
  std::vector<double> singular_phi_values;
  std::vector<double> safety_values;
  const double threshold = gamma * sigma_max;

  for (Eigen::Index i = 0; i < singular_values.size(); ++i) {
    const double sigma = singular_values(i);
    const double adjusted_condition = sigma / sigma_max;
    if (sigma > threshold) {
      non_singular_u_cols.push_back(u.col(i));
      non_singular_values.push_back(sigma);
    } else {
      singular_u_cols.push_back(u.col(i));
      singular_phi_values.push_back(adjusted_condition / gamma);
    }
    safety_values.push_back(adjusted_condition > gamma ? sigma : threshold);
  }

  Eigen::MatrixXd j_proj;
  if (!non_singular_u_cols.empty()) {
    Eigen::MatrixXd u_proj(jacobian.rows(), static_cast<int>(non_singular_u_cols.size()));
    for (Eigen::Index i = 0; i < u_proj.cols(); ++i) {
      u_proj.col(i) = non_singular_u_cols[static_cast<std::size_t>(i)];
    }
    j_proj = compose_svd(u_proj, non_singular_values, vt);
  } else {
    j_proj = jacobian;
  }

  const Eigen::MatrixXd j_safety = compose_svd(u, safety_values, vt);
  const Eigen::MatrixXd j_safety_pinv = pseudo_inverse(j_safety, pinv_tolerance);
  const Eigen::MatrixXd j_proj_pinv = pseudo_inverse(j_proj, pinv_tolerance);
  Eigen::MatrixXd j_parse = j_safety_pinv * j_proj * j_proj_pinv;

  if (!singular_u_cols.empty()) {
    Eigen::MatrixXd u_sing(jacobian.rows(), static_cast<int>(singular_u_cols.size()));
    for (Eigen::Index i = 0; i < u_sing.cols(); ++i) {
      u_sing.col(i) = singular_u_cols[static_cast<std::size_t>(i)];
    }
    Eigen::MatrixXd phi = Eigen::MatrixXd::Zero(u_sing.cols(), u_sing.cols());
    for (Eigen::Index i = 0; i < phi.rows(); ++i) {
      phi(i, i) = singular_phi_values[static_cast<std::size_t>(i)];
    }
    Eigen::MatrixXd gains = Eigen::MatrixXd::Identity(jacobian.rows(), jacobian.rows());
    for (Eigen::Index i = 0; i < jacobian.rows(); ++i) {
      gains(i, i) = i < 3 ? singular_gain_position : singular_gain_angular;
    }
    const Eigen::MatrixXd phi_singular = u_sing * phi * u_sing.transpose() * gains;
    j_parse += j_safety_pinv * phi_singular;
  }

  return j_parse;
}
}  // namespace

struct TwistReference
{
  Vector6 values{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  rclcpp::Time stamp{0, 0, RCL_ROS_TIME};
  bool valid{false};
};

class IntegratedCartesianAdmittanceController
  : public controller_interface::ChainableControllerInterface
{
public:
  IntegratedCartesianAdmittanceController() = default;

  CallbackReturn on_init() override
  {
    try {
      robot_name_ = auto_declare<std::string>("robot_name", "mur620");
      arm_ = auto_declare<std::string>("arm", "r");
      prefix_ = auto_declare<std::string>("prefix", arm_prefix(arm_));
      base_link_ = auto_declare<std::string>("base_link", prefix_ + "/base_link");
      tip_link_ = auto_declare<std::string>("tip_link", prefix_ + "/tool0");
      debug_base_frame_ = auto_declare<std::string>(
        "debug_base_frame", robot_name_ + "/" + prefix_ + "/base_link");
      command_frame_ = auto_declare<std::string>("command_frame", prefix_ + "/base_link");
      equilibrium_frame_ = auto_declare<std::string>(
        "equilibrium_frame", robot_name_ + "/" + prefix_ + "/admittance_equilibrium_pose");
      target_frame_ = auto_declare<std::string>(
        "target_frame", robot_name_ + "/" + prefix_ + "/admittance_target_pose");

      joint_names_ = auto_declare<std::vector<std::string>>(
        "joints",
        {
          prefix_ + "/shoulder_pan_joint",
          prefix_ + "/shoulder_lift_joint",
          prefix_ + "/elbow_joint",
          prefix_ + "/wrist_1_joint",
          prefix_ + "/wrist_2_joint",
          prefix_ + "/wrist_3_joint",
        });
      command_joint_names_ = auto_declare<std::vector<std::string>>(
        "command_joints", joint_names_);

      inverse_mode_ = lower_copy(auto_declare<std::string>("inverse_mode", "jparse"));
      gamma_ = std::clamp(auto_declare<double>("gamma", 0.1), 1.0e-4, 0.999);
      damping_ = std::max(0.0, auto_declare<double>("damping", 0.03));
      pinv_tolerance_ = auto_declare<double>("pinv_tolerance", 1.0e-6);
      singular_gain_position_ = auto_declare<double>("singular_gain_position", 1.0);
      singular_gain_angular_ = auto_declare<double>("singular_gain_angular", 1.0);

      command_timeout_ = std::max(0.0, auto_declare<double>("command_timeout", 0.12));
      require_wrench_ = auto_declare<bool>("require_wrench", false);
      use_ft_sensor_ = auto_declare<bool>("use_ft_sensor", false);
      wrench_in_tcp_frame_ = auto_declare<bool>("wrench_in_tcp_frame", true);
      wrench_timeout_ = std::max(0.0, auto_declare<double>("wrench_timeout", 0.5));
      wrench_bias_duration_ = std::max(0.0, auto_declare<double>("wrench_bias_duration", 1.0));
      wrench_filter_alpha_ = std::clamp(auto_declare<double>("wrench_filter_alpha", 0.02), 0.0, 1.0);
      force_deadband_ = std::max(0.0, auto_declare<double>("force_deadband", 0.0));
      torque_deadband_ = std::max(0.0, auto_declare<double>("torque_deadband", 0.0));
      ft_sensor_name_ = auto_declare<std::string>("ft_sensor_name", prefix_ + "/tcp_fts_sensor");
      ft_state_interface_names_ = auto_declare<std::vector<std::string>>(
        "ft_state_interface_names",
        {
          ft_sensor_name_ + "/force.x",
          ft_sensor_name_ + "/force.y",
          ft_sensor_name_ + "/force.z",
          ft_sensor_name_ + "/torque.x",
          ft_sensor_name_ + "/torque.y",
          ft_sensor_name_ + "/torque.z",
        });

      max_linear_velocity_ = std::max(0.0, auto_declare<double>("max_linear_velocity", 0.10));
      max_angular_velocity_ = std::max(0.0, auto_declare<double>("max_angular_velocity", 0.35));
      max_joint_velocity_ = std::max(0.0, auto_declare<double>("max_joint_velocity", 0.6));
      max_joint_acceleration_ = std::max(0.0, auto_declare<double>("max_joint_acceleration", 0.4));
      max_joint_jerk_ = std::max(0.0, auto_declare<double>("max_joint_jerk", 1.0));
      joint_limit_margin_ = std::max(0.0, auto_declare<double>("joint_limit_margin", 0.02));
      preserve_command_direction_ = auto_declare<bool>("preserve_command_direction", true);
      immediate_zero_on_zero_command_ = auto_declare<bool>("immediate_zero_on_zero_command", true);
      zero_command_deadband_ = std::max(0.0, auto_declare<double>("zero_command_deadband", 1.0e-5));
      publish_state_rate_hz_ = std::max(0.0, auto_declare<double>("publish_state_rate_hz", 50.0));

      admittance_ = checked_vector("admittance", {0.0006, 0.0006, 0.0015, 0.0, 0.0, 0.0});
      pose_error_gain_ = checked_vector("pose_error_gain", {0.5, 0.5, 0.5, 0.4, 0.4, 0.4});
      wrench_sign_ = checked_vector("wrench_sign", {1.0, 1.0, 1.0, 1.0, 1.0, 1.0});
      joint_lower_limits_ = checked_vector(
        "joint_lower_limits", {-2.0 * PI, -2.0 * PI, -PI, -2.0 * PI, -2.0 * PI, -2.0 * PI});
      joint_upper_limits_ = checked_vector(
        "joint_upper_limits", {2.0 * PI, 2.0 * PI, PI, 2.0 * PI, 2.0 * PI, 2.0 * PI});
      joint_velocity_limits_ = checked_vector(
        "joint_velocity_limits", {2.09439510239, 2.09439510239, PI, PI, PI, PI});
      joint_acceleration_limits_ = checked_vector(
        "joint_acceleration_limits", std::vector<double>(joint_names_.size(), max_joint_acceleration_));
      joint_jerk_limits_ = checked_vector(
        "joint_jerk_limits", std::vector<double>(joint_names_.size(), max_joint_jerk_));
    } catch (const std::exception & exc) {
      RCLCPP_ERROR(get_node()->get_logger(), "Integrated controller init failed: %s", exc.what());
      return CallbackReturn::ERROR;
    }
    return CallbackReturn::SUCCESS;
  }

  controller_interface::InterfaceConfiguration command_interface_configuration() const override
  {
    controller_interface::InterfaceConfiguration config;
    config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    for (const auto & joint_name : command_joint_names_) {
      config.names.push_back(joint_name + "/" + hardware_interface::HW_IF_VELOCITY);
    }
    return config;
  }

  controller_interface::InterfaceConfiguration state_interface_configuration() const override
  {
    controller_interface::InterfaceConfiguration config;
    config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
    for (const auto & joint_name : joint_names_) {
      config.names.push_back(joint_name + "/" + hardware_interface::HW_IF_POSITION);
    }
    for (const auto & joint_name : joint_names_) {
      config.names.push_back(joint_name + "/" + hardware_interface::HW_IF_VELOCITY);
    }
    if (use_ft_sensor_) {
      config.names.insert(
        config.names.end(), ft_state_interface_names_.begin(), ft_state_interface_names_.end());
    }
    return config;
  }

  CallbackReturn on_configure(const rclcpp_lifecycle::State &) override
  {
    const std::string urdf = get_robot_description();
    if (urdf.empty()) {
      RCLCPP_ERROR(get_node()->get_logger(), "No robot_description available from controller manager");
      return CallbackReturn::ERROR;
    }

    KDL::Tree tree;
    if (!kdl_parser::treeFromString(urdf, tree)) {
      RCLCPP_ERROR(get_node()->get_logger(), "Could not parse robot_description into KDL tree");
      return CallbackReturn::ERROR;
    }
    if (!tree.getChain(base_link_, tip_link_, chain_)) {
      RCLCPP_ERROR(
        get_node()->get_logger(), "Could not build KDL chain from '%s' to '%s'",
        base_link_.c_str(), tip_link_.c_str());
      return CallbackReturn::ERROR;
    }

    chain_joint_names_.clear();
    for (unsigned int i = 0; i < chain_.getNrOfSegments(); ++i) {
      const auto joint = chain_.getSegment(i).getJoint();
      if (joint.getType() != KDL::Joint::None) {
        chain_joint_names_.push_back(joint.getName());
      }
    }
    if (chain_joint_names_.size() != joint_names_.size()) {
      RCLCPP_WARN(
        get_node()->get_logger(),
        "KDL chain has %zu movable joints while configured command list has %zu joints",
        chain_joint_names_.size(), joint_names_.size());
    }

    jac_solver_ = std::make_unique<KDL::ChainJntToJacSolver>(chain_);
    fk_solver_ = std::make_unique<KDL::ChainFkSolverPos_recursive>(chain_);

    twist_sub_ = get_node()->create_subscription<geometry_msgs::msg::TwistStamped>(
      "~/equilibrium_twist_cmd", rclcpp::SystemDefaultsQoS(),
      [this](const geometry_msgs::msg::TwistStamped::SharedPtr msg) {
        TwistReference ref;
        ref.values = {
          msg->twist.linear.x, msg->twist.linear.y, msg->twist.linear.z,
          msg->twist.angular.x, msg->twist.angular.y, msg->twist.angular.z};
        ref.stamp = msg->header.stamp.sec == 0 && msg->header.stamp.nanosec == 0 ?
          get_node()->now() : rclcpp::Time(msg->header.stamp);
        ref.valid = true;
        received_reference_.set(ref);
      });

    singular_values_pub_ =
      get_node()->create_publisher<std_msgs::msg::Float64MultiArray>("~/singular_values", 10);
    debug_twist_pub_ =
      get_node()->create_publisher<std_msgs::msg::Float64MultiArray>("~/debug_twist", 10);
    filtered_wrench_pub_ =
      get_node()->create_publisher<geometry_msgs::msg::WrenchStamped>("~/filtered_wrench", 10);
    equilibrium_pose_pub_ =
      get_node()->create_publisher<geometry_msgs::msg::PoseStamped>("~/equilibrium_pose", 10);
    target_pose_pub_ =
      get_node()->create_publisher<geometry_msgs::msg::PoseStamped>("~/target_pose", 10);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*get_node());

    RCLCPP_INFO(
      get_node()->get_logger(),
      "Configured integrated Cartesian controller for %s: %s -> %s, joints=%zu, ft=%s",
      prefix_.c_str(), base_link_.c_str(), tip_link_.c_str(), joint_names_.size(),
      use_ft_sensor_ ? "enabled" : "disabled");
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_activate(const rclcpp_lifecycle::State &) override
  {
    if (command_interfaces_.size() != command_joint_names_.size()) {
      RCLCPP_ERROR(
        get_node()->get_logger(), "Expected %zu command interfaces, got %zu",
        command_joint_names_.size(), command_interfaces_.size());
      return CallbackReturn::ERROR;
    }
    const std::size_t expected_states =
      joint_names_.size() * 2 + (use_ft_sensor_ ? ft_state_interface_names_.size() : 0);
    if (state_interfaces_.size() != expected_states) {
      RCLCPP_ERROR(
        get_node()->get_logger(), "Expected %zu state interfaces, got %zu",
        expected_states, state_interfaces_.size());
      return CallbackReturn::ERROR;
    }

    reset_filters();
    last_commanded_velocity_.assign(command_joint_names_.size(), 0.0);
    last_commanded_acceleration_.assign(command_joint_names_.size(), 0.0);
    last_update_time_ = get_node()->now();
    previous_publish_time_ = rclcpp::Time(0, 0, get_node()->get_clock()->get_clock_type());

    KDL::JntArray q;
    if (!read_positions(q)) {
      RCLCPP_ERROR(get_node()->get_logger(), "Cannot read initial joint positions");
      return CallbackReturn::ERROR;
    }
    tf2::Transform current_tcp;
    if (!compute_current_tcp(q, current_tcp)) {
      RCLCPP_ERROR(get_node()->get_logger(), "Cannot compute initial TCP pose");
      return CallbackReturn::ERROR;
    }
    equilibrium_position_ = current_tcp.getOrigin();
    equilibrium_orientation_ = current_tcp.getRotation();
    equilibrium_orientation_.normalize();
    latest_target_position_ = equilibrium_position_;
    latest_target_orientation_ = equilibrium_orientation_;
    equilibrium_initialized_ = true;

    write_zero();
    RCLCPP_INFO(get_node()->get_logger(), "Activated integrated Cartesian controller for %s", prefix_.c_str());
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override
  {
    write_zero();
    equilibrium_initialized_ = false;
    return CallbackReturn::SUCCESS;
  }

protected:
  std::vector<hardware_interface::CommandInterface::SharedPtr>
  on_export_reference_interfaces_list() override
  {
    reference_interfaces_.assign(6, 0.0);
    exported_reference_interface_names_ = {
      "linear.x", "linear.y", "linear.z", "angular.x", "angular.y", "angular.z"};
    ordered_exported_reference_interfaces_.clear();
    ordered_exported_reference_interfaces_.reserve(exported_reference_interface_names_.size());
    for (std::size_t i = 0; i < exported_reference_interface_names_.size(); ++i) {
      ordered_exported_reference_interfaces_.push_back(
        std::make_shared<hardware_interface::CommandInterface>(
          get_node()->get_name(), exported_reference_interface_names_[i], &reference_interfaces_[i]));
    }
    return ordered_exported_reference_interfaces_;
  }

  std::vector<hardware_interface::StateInterface::SharedPtr>
  on_export_state_interfaces_list() override
  {
    return {};
  }

  bool on_set_chained_mode(bool chained_mode) override
  {
    subscriber_is_active_ = !chained_mode;
    return true;
  }

  return_type update_reference_from_subscribers(
    const rclcpp::Time & time,
    const rclcpp::Duration &) override
  {
    if (!subscriber_is_active_) {
      return return_type::OK;
    }
    auto maybe_reference = received_reference_.try_get();
    const TwistReference reference = maybe_reference.value_or(last_reference_);
    last_reference_ = reference;
    if (!reference.valid || (time - reference.stamp).seconds() > command_timeout_) {
      std::fill(reference_interfaces_.begin(), reference_interfaces_.end(), 0.0);
      return return_type::OK;
    }
    for (std::size_t i = 0; i < reference.values.size(); ++i) {
      reference_interfaces_[i] = reference.values[i];
    }
    return return_type::OK;
  }

  return_type update_and_write_commands(
    const rclcpp::Time & time,
    const rclcpp::Duration & period) override
  {
    const double dt = sanitized_dt(time, period);

    KDL::JntArray q;
    if (!read_positions(q)) {
      write_zero();
      return return_type::ERROR;
    }

    tf2::Transform current_tcp;
    if (!compute_current_tcp(q, current_tcp)) {
      write_zero();
      return return_type::ERROR;
    }
    if (!equilibrium_initialized_) {
      equilibrium_position_ = current_tcp.getOrigin();
      equilibrium_orientation_ = current_tcp.getRotation();
      equilibrium_orientation_.normalize();
      equilibrium_initialized_ = true;
    }

    if (!update_wrench(time, current_tcp)) {
      write_zero();
      if (should_publish(time)) {
        publish_state(time, current_tcp, true);
      }
      return return_type::OK;
    }

    Vector6 command = active_reference();
    integrate_equilibrium(command, dt);

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
      quaternion_from_rotation_vector(admittance_rotation) * equilibrium_orientation_;
    target_orientation.normalize();

    const tf2::Vector3 position_error = target_position - current_tcp.getOrigin();
    const tf2::Quaternion orientation_error_q =
      target_orientation * current_tcp.getRotation().inverse();
    const tf2::Vector3 orientation_error = rotation_vector_from_quaternion(orientation_error_q);

    Eigen::VectorXd target_twist(6);
    target_twist <<
      clamp_abs(command[0] + position_error.x() * pose_error_gain_[0], max_linear_velocity_),
      clamp_abs(command[1] + position_error.y() * pose_error_gain_[1], max_linear_velocity_),
      clamp_abs(command[2] + position_error.z() * pose_error_gain_[2], max_linear_velocity_),
      clamp_abs(command[3] + orientation_error.x() * pose_error_gain_[3], max_angular_velocity_),
      clamp_abs(command[4] + orientation_error.y() * pose_error_gain_[4], max_angular_velocity_),
      clamp_abs(command[5] + orientation_error.z() * pose_error_gain_[5], max_angular_velocity_);

    const Eigen::VectorXd qdot_raw = compute_joint_velocity(q, target_twist, time);
    if (qdot_raw.size() == 0) {
      write_zero();
      return return_type::ERROR;
    }

    std::vector<double> qdot(command_joint_names_.size(), 0.0);
    for (std::size_t i = 0; i < command_joint_names_.size(); ++i) {
      const auto it = std::find(chain_joint_names_.begin(), chain_joint_names_.end(), command_joint_names_[i]);
      if (it != chain_joint_names_.end()) {
        const std::size_t chain_index = static_cast<std::size_t>(std::distance(chain_joint_names_.begin(), it));
        if (chain_index < static_cast<std::size_t>(qdot_raw.size())) {
          qdot[i] = qdot_raw(static_cast<Eigen::Index>(chain_index));
        }
      }
    }

    qdot = apply_safety_limits(qdot, q, dt);
    Eigen::VectorXd qdot_limited = Eigen::VectorXd::Zero(qdot_raw.size());
    for (std::size_t i = 0; i < command_joint_names_.size(); ++i) {
      const auto it = std::find(chain_joint_names_.begin(), chain_joint_names_.end(), command_joint_names_[i]);
      if (it != chain_joint_names_.end()) {
        const std::size_t chain_index = static_cast<std::size_t>(std::distance(chain_joint_names_.begin(), it));
        if (chain_index < static_cast<std::size_t>(qdot_limited.size())) {
          qdot_limited(static_cast<Eigen::Index>(chain_index)) = qdot[i];
        }
      }
    }
    write_command(qdot);

    latest_target_position_ = target_position;
    latest_target_orientation_ = target_orientation;
    latest_achieved_twist_ = latest_jacobian_ * qdot_limited;
    if (should_publish(time)) {
      publish_debug(time, target_twist, qdot_limited);
      publish_state(time, current_tcp, false);
    }
    return return_type::OK;
  }

private:
  std::vector<double> checked_vector(
    const std::string & name,
    const std::vector<double> & defaults)
  {
    auto values = auto_declare<std::vector<double>>(name, defaults);
    if (values.size() != defaults.size()) {
      RCLCPP_WARN(
        get_node()->get_logger(), "Parameter '%s' has %zu entries, expected %zu; using defaults",
        name.c_str(), values.size(), defaults.size());
      return defaults;
    }
    return values;
  }

  double sanitized_dt(const rclcpp::Time & time, const rclcpp::Duration & period)
  {
    double dt = period.seconds();
    if (last_update_time_.nanoseconds() != 0) {
      dt = (time - last_update_time_).seconds();
    }
    if (dt <= 0.0 || dt > 0.1) {
      dt = 0.002;
    }
    last_update_time_ = time;
    return dt;
  }

  bool read_positions(KDL::JntArray & q) const
  {
    q.resize(joint_names_.size());
    for (std::size_t i = 0; i < joint_names_.size(); ++i) {
      const auto value = state_interfaces_[i].get_optional<double>();
      if (!value.has_value() || !std::isfinite(value.value())) {
        return false;
      }
      q(static_cast<unsigned int>(i)) = value.value();
    }
    return true;
  }

  bool compute_current_tcp(const KDL::JntArray & q, tf2::Transform & tcp) const
  {
    if (!fk_solver_) {
      return false;
    }
    KDL::Frame frame;
    if (fk_solver_->JntToCart(q, frame) < 0) {
      return false;
    }
    tcp.setOrigin({frame.p.x(), frame.p.y(), frame.p.z()});
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
    double w = 1.0;
    frame.M.GetQuaternion(x, y, z, w);
    tf2::Quaternion q_tf(x, y, z, w);
    q_tf.normalize();
    tcp.setRotation(q_tf);
    return true;
  }

  bool update_wrench(const rclcpp::Time & time, const tf2::Transform & current_tcp)
  {
    if (!use_ft_sensor_) {
      return !require_wrench_;
    }

    Vector6 sample;
    const std::size_t offset = joint_names_.size() * 2;
    for (std::size_t i = 0; i < sample.size(); ++i) {
      const auto value = state_interfaces_[offset + i].get_optional<double>();
      if (!value.has_value() || !std::isfinite(value.value())) {
        return !require_wrench_;
      }
      sample[i] = value.value() * wrench_sign_[i];
    }

    if (wrench_in_tcp_frame_) {
      const tf2::Matrix3x3 rotation = current_tcp.getBasis();
      const tf2::Vector3 force = rotation * tf2::Vector3(sample[0], sample[1], sample[2]);
      const tf2::Vector3 torque = rotation * tf2::Vector3(sample[3], sample[4], sample[5]);
      sample = {force.x(), force.y(), force.z(), torque.x(), torque.y(), torque.z()};
    }

    if (!bias_ready_) {
      accumulate_bias(sample, time);
      return false;
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
    last_wrench_time_ = time;
    have_wrench_ = true;
    return !require_wrench_ || (have_wrench_ && (time - last_wrench_time_).seconds() <= wrench_timeout_);
  }

  void accumulate_bias(const Vector6 & sample, const rclcpp::Time & time)
  {
    if (bias_sample_count_ == 0) {
      bias_start_time_ = time;
    }
    for (std::size_t i = 0; i < wrench_bias_.size(); ++i) {
      wrench_bias_[i] += sample[i];
    }
    ++bias_sample_count_;
    if (wrench_bias_duration_ <= 0.0 || (time - bias_start_time_).seconds() >= wrench_bias_duration_) {
      for (double & value : wrench_bias_) {
        value /= static_cast<double>(std::max<std::size_t>(1, bias_sample_count_));
      }
      bias_ready_ = true;
      RCLCPP_INFO(
        get_node()->get_logger(),
        "Integrated controller wrench bias ready from %zu samples", bias_sample_count_);
    }
  }

  void reset_filters()
  {
    filtered_wrench_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    wrench_bias_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    bias_sample_count_ = 0;
    bias_ready_ = !use_ft_sensor_ || !require_wrench_;
    have_wrench_ = false;
  }

  Vector6 active_reference() const
  {
    Vector6 command{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    for (std::size_t i = 0; i < std::min<std::size_t>(command.size(), reference_interfaces_.size()); ++i) {
      command[i] = std::isfinite(reference_interfaces_[i]) ? reference_interfaces_[i] : 0.0;
    }
    return command;
  }

  void integrate_equilibrium(const Vector6 & command, double dt)
  {
    equilibrium_position_ += tf2::Vector3(command[0], command[1], command[2]) * dt;
    const tf2::Vector3 angular_delta(command[3] * dt, command[4] * dt, command[5] * dt);
    equilibrium_orientation_ = quaternion_from_rotation_vector(angular_delta) * equilibrium_orientation_;
    equilibrium_orientation_.normalize();
  }

  Eigen::VectorXd compute_joint_velocity(
    const KDL::JntArray & q,
    const Eigen::VectorXd & target_twist,
    const rclcpp::Time & time)
  {
    KDL::Jacobian kdl_jacobian(chain_joint_names_.size());
    if (jac_solver_->JntToJac(q, kdl_jacobian) < 0) {
      RCLCPP_WARN_THROTTLE(
        get_node()->get_logger(), *get_node()->get_clock(), 2000, "KDL Jacobian computation failed");
      return {};
    }

    latest_jacobian_.resize(6, static_cast<Eigen::Index>(chain_joint_names_.size()));
    for (unsigned int col = 0; col < kdl_jacobian.columns(); ++col) {
      for (unsigned int row = 0; row < 6; ++row) {
        latest_jacobian_(static_cast<Eigen::Index>(row), static_cast<Eigen::Index>(col)) =
          kdl_jacobian(row, col);
      }
    }

    Eigen::MatrixXd inverse;
    if (inverse_mode_ == "jparse") {
      inverse = compute_jparse_inverse(
        latest_jacobian_, gamma_, singular_gain_position_, singular_gain_angular_,
        pinv_tolerance_, &latest_singular_values_, &latest_inverse_condition_);
    } else if (inverse_mode_ == "dls" || inverse_mode_ == "damped_least_squares") {
      Eigen::JacobiSVD<Eigen::MatrixXd> svd(latest_jacobian_);
      latest_singular_values_ = svd.singularValues();
      const double sigma_max = latest_singular_values_.size() > 0 ? latest_singular_values_.maxCoeff() : 0.0;
      const double sigma_min = latest_singular_values_.size() > 0 ? latest_singular_values_.minCoeff() : 0.0;
      latest_inverse_condition_ = sigma_max > std::numeric_limits<double>::epsilon() ?
        sigma_min / sigma_max : 0.0;
      inverse = damped_least_squares_inverse(latest_jacobian_, damping_);
    } else {
      RCLCPP_WARN_THROTTLE(
        get_node()->get_logger(), *get_node()->get_clock(), 2000,
        "Unknown inverse_mode '%s'; using DLS", inverse_mode_.c_str());
      inverse = damped_least_squares_inverse(latest_jacobian_, damping_);
    }

    (void)time;
    return inverse * target_twist;
  }

  std::vector<double> apply_safety_limits(
    std::vector<double> command,
    const KDL::JntArray & q,
    double dt)
  {
    if (command.size() != last_commanded_velocity_.size()) {
      last_commanded_velocity_.assign(command.size(), 0.0);
      last_commanded_acceleration_.assign(command.size(), 0.0);
    }

    std::vector<double> velocity_limits(command.size(), 0.0);
    std::vector<double> desired_acceleration(command.size(), 0.0);
    std::vector<double> acceleration_delta(command.size(), 0.0);
    std::vector<bool> position_blocked(command.size(), false);

    for (std::size_t i = 0; i < command.size(); ++i) {
      velocity_limits[i] = std::min(std::abs(joint_velocity_limits_[i]), std::abs(max_joint_velocity_));
      command[i] = std::clamp(command[i], -velocity_limits[i], velocity_limits[i]);

      const double lower = joint_lower_limits_[i] + joint_limit_margin_;
      const double upper = joint_upper_limits_[i] - joint_limit_margin_;
      const double position = q(static_cast<unsigned int>(i));
      if ((position >= upper && command[i] > 0.0) || (position <= lower && command[i] < 0.0)) {
        command[i] = 0.0;
        position_blocked[i] = true;
      } else if (command[i] > 0.0) {
        command[i] = std::min(command[i], std::max(0.0, (upper - position) / dt));
      } else if (command[i] < 0.0) {
        command[i] = std::max(command[i], std::min(0.0, (lower - position) / dt));
      }
    }

    if (preserve_command_direction_) {
      double velocity_scale = 1.0;
      for (std::size_t i = 0; i < command.size(); ++i) {
        if (velocity_limits[i] > 0.0 && std::abs(command[i]) > velocity_limits[i]) {
          velocity_scale = std::min(velocity_scale, velocity_limits[i] / std::abs(command[i]));
        }
      }
      for (double & value : command) {
        value *= velocity_scale;
      }
    }

    for (std::size_t i = 0; i < command.size(); ++i) {
      desired_acceleration[i] = (command[i] - last_commanded_velocity_[i]) / dt;
    }

    double acceleration_scale = 1.0;
    if (preserve_command_direction_) {
      for (std::size_t i = 0; i < command.size(); ++i) {
        const double limit = std::abs(joint_acceleration_limits_[i]);
        if (limit > 0.0 && std::abs(desired_acceleration[i]) > limit) {
          acceleration_scale = std::min(acceleration_scale, limit / std::abs(desired_acceleration[i]));
        }
      }
    }
    for (std::size_t i = 0; i < command.size(); ++i) {
      desired_acceleration[i] *= acceleration_scale;
      if (!preserve_command_direction_) {
        const double limit = std::abs(joint_acceleration_limits_[i]);
        if (limit > 0.0) {
          desired_acceleration[i] = std::clamp(desired_acceleration[i], -limit, limit);
        }
      }
      acceleration_delta[i] = desired_acceleration[i] - last_commanded_acceleration_[i];
    }

    double jerk_scale = 1.0;
    if (preserve_command_direction_) {
      for (std::size_t i = 0; i < command.size(); ++i) {
        const double max_delta = std::abs(joint_jerk_limits_[i]) * dt;
        if (max_delta > 0.0 && std::abs(acceleration_delta[i]) > max_delta) {
          jerk_scale = std::min(jerk_scale, max_delta / std::abs(acceleration_delta[i]));
        }
      }
    }

    for (std::size_t i = 0; i < command.size(); ++i) {
      double acceleration = last_commanded_acceleration_[i] + acceleration_delta[i] * jerk_scale;
      if (!preserve_command_direction_) {
        const double max_delta = std::abs(joint_jerk_limits_[i]) * dt;
        if (max_delta > 0.0) {
          acceleration = std::clamp(
            acceleration,
            last_commanded_acceleration_[i] - max_delta,
            last_commanded_acceleration_[i] + max_delta);
        }
      }
      double next_velocity = last_commanded_velocity_[i] + acceleration * dt;
      const double remaining = command[i] - last_commanded_velocity_[i];
      const double next_remaining = command[i] - next_velocity;
      if (std::abs(remaining) < 1.0e-9 || remaining * next_remaining <= 0.0) {
        next_velocity = command[i];
        acceleration = 0.0;
      }
      next_velocity = std::clamp(next_velocity, -velocity_limits[i], velocity_limits[i]);
      if (position_blocked[i] || (immediate_zero_on_zero_command_ && std::abs(command[i]) <= zero_command_deadband_)) {
        next_velocity = 0.0;
        acceleration = 0.0;
      }
      last_commanded_velocity_[i] = next_velocity;
      last_commanded_acceleration_[i] = acceleration;
      command[i] = next_velocity;
    }
    return command;
  }

  void write_command(const std::vector<double> & command)
  {
    for (std::size_t i = 0; i < std::min(command.size(), command_interfaces_.size()); ++i) {
      (void)command_interfaces_[i].set_value(command[i]);
    }
  }

  void write_zero()
  {
    for (auto & command_interface : command_interfaces_) {
      (void)command_interface.set_value(0.0);
    }
    std::fill(last_commanded_velocity_.begin(), last_commanded_velocity_.end(), 0.0);
    std::fill(last_commanded_acceleration_.begin(), last_commanded_acceleration_.end(), 0.0);
  }

  void publish_debug(
    const rclcpp::Time & time,
    const Eigen::VectorXd & target_twist,
    const Eigen::VectorXd & qdot)
  {
    std_msgs::msg::Float64MultiArray singular_msg;
    singular_msg.data.reserve(static_cast<std::size_t>(latest_singular_values_.size()) + 1);
    singular_msg.data.push_back(latest_inverse_condition_);
    for (Eigen::Index i = 0; i < latest_singular_values_.size(); ++i) {
      singular_msg.data.push_back(latest_singular_values_(i));
    }
    singular_values_pub_->publish(singular_msg);

    std_msgs::msg::Float64MultiArray debug_msg;
    debug_msg.data.reserve(13 + static_cast<std::size_t>(qdot.size()));
    debug_msg.data.push_back(latest_inverse_condition_);
    for (Eigen::Index i = 0; i < target_twist.size(); ++i) {
      debug_msg.data.push_back(target_twist(i));
    }
    for (Eigen::Index i = 0; i < latest_achieved_twist_.size(); ++i) {
      debug_msg.data.push_back(latest_achieved_twist_(i));
    }
    for (Eigen::Index i = 0; i < qdot.size(); ++i) {
      debug_msg.data.push_back(qdot(i));
    }
    debug_twist_pub_->publish(debug_msg);

    geometry_msgs::msg::WrenchStamped wrench_msg;
    wrench_msg.header.stamp = time;
    wrench_msg.header.frame_id = debug_base_frame_;
    wrench_msg.wrench.force.x = filtered_wrench_[0];
    wrench_msg.wrench.force.y = filtered_wrench_[1];
    wrench_msg.wrench.force.z = filtered_wrench_[2];
    wrench_msg.wrench.torque.x = filtered_wrench_[3];
    wrench_msg.wrench.torque.y = filtered_wrench_[4];
    wrench_msg.wrench.torque.z = filtered_wrench_[5];
    filtered_wrench_pub_->publish(wrench_msg);
  }

  bool should_publish(const rclcpp::Time & time)
  {
    if (publish_state_rate_hz_ <= 0.0) {
      return false;
    }
    const double publish_period = 1.0 / publish_state_rate_hz_;
    if (previous_publish_time_.nanoseconds() != 0 &&
      (time - previous_publish_time_).seconds() < publish_period)
    {
      return false;
    }
    previous_publish_time_ = time;
    return true;
  }

  void publish_state(
    const rclcpp::Time & time,
    const tf2::Transform &,
    bool target_is_equilibrium)
  {
    const tf2::Vector3 target_position =
      target_is_equilibrium ? equilibrium_position_ : latest_target_position_;
    const tf2::Quaternion target_orientation =
      target_is_equilibrium ? equilibrium_orientation_ : latest_target_orientation_;

    equilibrium_pose_pub_->publish(
      pose_msg(debug_base_frame_, time, equilibrium_position_, equilibrium_orientation_));
    target_pose_pub_->publish(pose_msg(debug_base_frame_, time, target_position, target_orientation));

    geometry_msgs::msg::TransformStamped eq_tf;
    eq_tf.header.stamp = time;
    eq_tf.header.frame_id = debug_base_frame_;
    eq_tf.child_frame_id = equilibrium_frame_;
    vector_to_msg(equilibrium_position_, eq_tf.transform.translation);
    quaternion_to_msg(equilibrium_orientation_, eq_tf.transform.rotation);

    geometry_msgs::msg::TransformStamped target_tf;
    target_tf.header.stamp = time;
    target_tf.header.frame_id = debug_base_frame_;
    target_tf.child_frame_id = target_frame_;
    vector_to_msg(target_position, target_tf.transform.translation);
    quaternion_to_msg(target_orientation, target_tf.transform.rotation);
    tf_broadcaster_->sendTransform(eq_tf);
    tf_broadcaster_->sendTransform(target_tf);
  }

  std::string robot_name_;
  std::string arm_;
  std::string prefix_;
  std::string base_link_;
  std::string tip_link_;
  std::string debug_base_frame_;
  std::string command_frame_;
  std::string equilibrium_frame_;
  std::string target_frame_;
  std::string inverse_mode_;
  std::string ft_sensor_name_;
  std::vector<std::string> joint_names_;
  std::vector<std::string> command_joint_names_;
  std::vector<std::string> chain_joint_names_;
  std::vector<std::string> ft_state_interface_names_;

  KDL::Chain chain_;
  std::unique_ptr<KDL::ChainJntToJacSolver> jac_solver_;
  std::unique_ptr<KDL::ChainFkSolverPos_recursive> fk_solver_;

  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr twist_sub_;
  realtime_tools::RealtimeThreadSafeBox<TwistReference> received_reference_;
  TwistReference last_reference_;
  bool subscriber_is_active_{true};

  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr singular_values_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr debug_twist_pub_;
  rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr filtered_wrench_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr equilibrium_pose_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr target_pose_pub_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  double gamma_{0.1};
  double damping_{0.03};
  double pinv_tolerance_{1.0e-6};
  double singular_gain_position_{1.0};
  double singular_gain_angular_{1.0};
  double command_timeout_{0.12};
  double wrench_timeout_{0.5};
  double wrench_bias_duration_{1.0};
  double wrench_filter_alpha_{0.02};
  double force_deadband_{0.0};
  double torque_deadband_{0.0};
  double max_linear_velocity_{0.10};
  double max_angular_velocity_{0.35};
  double max_joint_velocity_{0.6};
  double max_joint_acceleration_{0.4};
  double max_joint_jerk_{1.0};
  double joint_limit_margin_{0.02};
  double zero_command_deadband_{1.0e-5};
  double publish_state_rate_hz_{50.0};
  bool require_wrench_{false};
  bool use_ft_sensor_{false};
  bool wrench_in_tcp_frame_{true};
  bool preserve_command_direction_{true};
  bool immediate_zero_on_zero_command_{true};

  std::vector<double> admittance_;
  std::vector<double> pose_error_gain_;
  std::vector<double> wrench_sign_;
  std::vector<double> joint_lower_limits_;
  std::vector<double> joint_upper_limits_;
  std::vector<double> joint_velocity_limits_;
  std::vector<double> joint_acceleration_limits_;
  std::vector<double> joint_jerk_limits_;
  std::vector<double> last_commanded_velocity_;
  std::vector<double> last_commanded_acceleration_;

  Vector6 filtered_wrench_{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  Vector6 wrench_bias_{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  std::size_t bias_sample_count_{0};
  bool bias_ready_{true};
  bool have_wrench_{false};

  tf2::Vector3 equilibrium_position_{0.0, 0.0, 0.0};
  tf2::Quaternion equilibrium_orientation_{0.0, 0.0, 0.0, 1.0};
  tf2::Vector3 latest_target_position_{0.0, 0.0, 0.0};
  tf2::Quaternion latest_target_orientation_{0.0, 0.0, 0.0, 1.0};
  bool equilibrium_initialized_{false};

  rclcpp::Time last_update_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_wrench_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time bias_start_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time previous_publish_time_{0, 0, RCL_ROS_TIME};

  Eigen::VectorXd latest_singular_values_;
  Eigen::VectorXd latest_achieved_twist_{Eigen::VectorXd::Zero(6)};
  Eigen::MatrixXd latest_jacobian_;
  double latest_inverse_condition_{0.0};
};

}  // namespace mur_control

PLUGINLIB_EXPORT_CLASS(
  mur_control::IntegratedCartesianAdmittanceController,
  controller_interface::ChainableControllerInterface)
