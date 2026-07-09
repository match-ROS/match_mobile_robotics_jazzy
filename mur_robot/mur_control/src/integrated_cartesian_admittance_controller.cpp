#include <algorithm>
#include <array>
#include <cmath>
#include <cctype>
#include <limits>
#include <map>
#include <memory>
#include <sstream>
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
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2_ros/transform_broadcaster.h>
#include <visualization_msgs/msg/marker_array.hpp>

#include "mur_control/collision_response.hpp"

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

bool vector_below_deadband(const Vector6 & values, double deadband_value)
{
  return std::all_of(values.begin(), values.end(), [deadband_value](double value) {
    return std::abs(value) <= deadband_value;
  });
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

tf2::Transform transform_from_xyz_rpy(
  const std::vector<double> & xyz,
  const std::vector<double> & rpy)
{
  tf2::Quaternion q;
  q.setRPY(rpy[0], rpy[1], rpy[2]);
  q.normalize();

  tf2::Transform transform;
  transform.setOrigin(tf2::Vector3(xyz[0], xyz[1], xyz[2]));
  transform.setRotation(q);
  return transform;
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

struct CollisionJointState
{
  std::map<std::string, double> positions;
  std::map<std::string, rclcpp::Time> stamps;
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
      reset_equilibrium_on_zero_command_ = auto_declare<bool>("reset_equilibrium_on_zero_command", true);
      zero_command_deadband_ = std::max(0.0, auto_declare<double>("zero_command_deadband", 1.0e-5));
      publish_state_rate_hz_ = std::max(0.0, auto_declare<double>("publish_state_rate_hz", 50.0));

      enable_collision_avoidance_ = auto_declare<bool>("enable_collision_avoidance", false);
      collision_other_prefix_ = auto_declare<std::string>(
        "collision_other_prefix", arm_ == "l" ? "UR10_r" : "UR10_l");
      collision_other_base_link_ = auto_declare<std::string>(
        "collision_other_base_link", collision_other_prefix_ + "/base_link");
      collision_other_tip_link_ = auto_declare<std::string>(
        "collision_other_tip_link", collision_other_prefix_ + "/tool0");
      collision_other_joint_names_ = auto_declare<std::vector<std::string>>(
        "collision_other_joint_names",
        {
          collision_other_prefix_ + "/shoulder_pan_joint",
          collision_other_prefix_ + "/shoulder_lift_joint",
          collision_other_prefix_ + "/elbow_joint",
          collision_other_prefix_ + "/wrist_1_joint",
          collision_other_prefix_ + "/wrist_2_joint",
          collision_other_prefix_ + "/wrist_3_joint",
        });
      collision_capsule_segments_ = auto_declare<std::vector<std::string>>(
        "collision_capsule_segments", {});
      collision_other_capsule_segments_ = auto_declare<std::vector<std::string>>(
        "collision_other_capsule_segments", {});
      collision_forbidden_box_specs_ = auto_declare<std::vector<std::string>>(
        "collision_forbidden_boxes", {});
      collision_forbidden_box_exempt_own_links_ = auto_declare<std::vector<std::string>>(
        "collision_forbidden_box_exempt_own_links", {prefix_ + "/shoulder_link"});
      collision_forbidden_box_exempt_initial_segments_ = std::max(
        0, static_cast<int>(
          auto_declare<int>("collision_forbidden_box_exempt_initial_segments", 1)));
      collision_common_link_ = auto_declare<std::string>("collision_common_link", "base_link");
      collision_own_base_xyz_ = checked_vector("collision_own_base_xyz", {0.0, 0.0, 0.0});
      collision_own_base_rpy_ = checked_vector("collision_own_base_rpy", {0.0, 0.0, 0.0});
      collision_other_base_xyz_ = checked_vector("collision_other_base_xyz", {0.0, 0.0, 0.0});
      collision_other_base_rpy_ = checked_vector("collision_other_base_rpy", {0.0, 0.0, 0.0});
      collision_joint_states_topic_ = auto_declare<std::string>("collision_joint_states_topic", "/joint_states");
      collision_joint_state_timeout_ = std::max(
        0.0, auto_declare<double>("collision_joint_state_timeout", 0.1));
      collision_sample_spacing_ = std::max(
        0.005, auto_declare<double>("collision_sample_spacing", 0.08));
      collision_sphere_radius_ = std::max(
        0.0, auto_declare<double>("collision_sphere_radius", 0.04));
      collision_activation_clearance_ = std::max(
        0.0, auto_declare<double>("collision_activation_clearance", 0.12));
      collision_stop_clearance_ = std::max(
        0.0, auto_declare<double>("collision_stop_clearance", 0.04));
      collision_response_mode_ = parse_collision_response_mode(
        auto_declare<std::string>("collision_response_mode", "scale"));
      collision_fail_safe_stop_ = auto_declare<bool>("collision_fail_safe_stop", true);
      publish_collision_markers_ = auto_declare<bool>("publish_collision_markers", false);
      collision_marker_publish_rate_hz_ = std::max(
        0.0, auto_declare<double>("collision_marker_publish_rate_hz", 10.0));
      collision_marker_topic_ = auto_declare<std::string>(
        "collision_marker_topic", "~/collision_markers");

      admittance_ = checked_vector("admittance", {0.0006, 0.0006, 0.0015, 0.0, 0.0, 0.0});
      wrench_twist_gain_ = checked_vector("wrench_twist_gain", {0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
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

    if (enable_collision_avoidance_) {
      if (!configure_collision_chains(tree)) {
        return CallbackReturn::ERROR;
      }
      collision_joint_state_sub_ = get_node()->create_subscription<sensor_msgs::msg::JointState>(
        collision_joint_states_topic_, rclcpp::SensorDataQoS(),
        [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
          CollisionJointState snapshot =
            received_collision_joint_state_.try_get().value_or(CollisionJointState{});
          const rclcpp::Time stamp = msg->header.stamp.sec == 0 && msg->header.stamp.nanosec == 0 ?
            get_node()->now() : rclcpp::Time(msg->header.stamp);
          snapshot.stamp = stamp;
          snapshot.valid = true;
          for (std::size_t i = 0; i < msg->name.size(); ++i) {
            if (i < msg->position.size() && std::isfinite(msg->position[i])) {
              snapshot.positions[msg->name[i]] = msg->position[i];
              snapshot.stamps[msg->name[i]] = stamp;
            }
          }
          received_collision_joint_state_.set(snapshot);
        });
    }

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
    collision_min_clearance_pub_ =
      get_node()->create_publisher<std_msgs::msg::Float64>("~/collision_min_clearance", 10);
    collision_status_pub_ =
      get_node()->create_publisher<std_msgs::msg::String>("~/collision_status", 10);
    collision_nearest_source_pub_ =
      get_node()->create_publisher<std_msgs::msg::String>("~/collision_nearest_source", 10);
    if (publish_collision_markers_) {
      collision_markers_pub_ =
        get_node()->create_publisher<visualization_msgs::msg::MarkerArray>(
          collision_marker_topic_, 10);
    }
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*get_node());

    RCLCPP_INFO(
      get_node()->get_logger(),
      "Configured integrated Cartesian controller for %s: %s -> %s, joints=%zu, ft=%s, collision=%s, collision_response=%s",
      prefix_.c_str(), base_link_.c_str(), tip_link_.c_str(), joint_names_.size(),
      use_ft_sensor_ ? "enabled" : "disabled",
      enable_collision_avoidance_ ? "enabled" : "disabled",
      collision_response_mode_name(collision_response_mode_).c_str());
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
  std::vector<hardware_interface::CommandInterface> on_export_reference_interfaces() override
  {
    reference_interfaces_.assign(6, 0.0);
    std::vector<hardware_interface::CommandInterface> exported_reference_interfaces;
    exported_reference_interfaces.reserve(reference_interfaces_.size());
    exported_reference_interfaces.emplace_back(get_node()->get_name(), "linear.x", &reference_interfaces_[0]);
    exported_reference_interfaces.emplace_back(get_node()->get_name(), "linear.y", &reference_interfaces_[1]);
    exported_reference_interfaces.emplace_back(get_node()->get_name(), "linear.z", &reference_interfaces_[2]);
    exported_reference_interfaces.emplace_back(get_node()->get_name(), "angular.x", &reference_interfaces_[3]);
    exported_reference_interfaces.emplace_back(get_node()->get_name(), "angular.y", &reference_interfaces_[4]);
    exported_reference_interfaces.emplace_back(get_node()->get_name(), "angular.z", &reference_interfaces_[5]);
    return exported_reference_interfaces;
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
    if (reset_equilibrium_on_zero_command_ && vector_below_deadband(command, zero_command_deadband_)) {
      equilibrium_position_ = current_tcp.getOrigin();
      equilibrium_orientation_ = current_tcp.getRotation();
      equilibrium_orientation_.normalize();
    } else {
      integrate_equilibrium(command, dt);
    }

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
      clamp_abs(
        command[0] + position_error.x() * pose_error_gain_[0] +
          filtered_wrench_[0] * wrench_twist_gain_[0],
        max_linear_velocity_),
      clamp_abs(
        command[1] + position_error.y() * pose_error_gain_[1] +
          filtered_wrench_[1] * wrench_twist_gain_[1],
        max_linear_velocity_),
      clamp_abs(
        command[2] + position_error.z() * pose_error_gain_[2] +
          filtered_wrench_[2] * wrench_twist_gain_[2],
        max_linear_velocity_),
      clamp_abs(
        command[3] + orientation_error.x() * pose_error_gain_[3] +
          filtered_wrench_[3] * wrench_twist_gain_[3],
        max_angular_velocity_),
      clamp_abs(
        command[4] + orientation_error.y() * pose_error_gain_[4] +
          filtered_wrench_[4] * wrench_twist_gain_[4],
        max_angular_velocity_),
      clamp_abs(
        command[5] + orientation_error.z() * pose_error_gain_[5] +
          filtered_wrench_[5] * wrench_twist_gain_[5],
        max_angular_velocity_);

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

    latest_qdot_raw_command_ = qdot;
    qdot = apply_collision_avoidance(qdot, q, time);
    latest_qdot_after_collision_ = qdot;
    qdot = apply_safety_limits(qdot, q, dt);
    latest_qdot_after_safety_ = qdot;
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
      publish_debug(time, target_twist);
      publish_collision_state(time);
      publish_collision_markers(time);
      publish_state(time, current_tcp, false);
    }
    return return_type::OK;
  }

private:
  struct CollisionCapsule
  {
    std::string link_name;
    tf2::Vector3 start;
    tf2::Vector3 end;
  };

  struct ForbiddenBox
  {
    std::string name;
    tf2::Vector3 center;
    tf2::Vector3 size;
  };

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

  static std::vector<std::string> movable_joint_names(const KDL::Chain & chain)
  {
    std::vector<std::string> names;
    for (unsigned int i = 0; i < chain.getNrOfSegments(); ++i) {
      const auto joint = chain.getSegment(i).getJoint();
      if (joint.getType() != KDL::Joint::None) {
        names.push_back(joint.getName());
      }
    }
    return names;
  }

  static std::string trim_copy(const std::string & value)
  {
    const auto first = value.find_first_not_of(" \t\n\r");
    if (first == std::string::npos) {
      return "";
    }
    const auto last = value.find_last_not_of(" \t\n\r");
    return value.substr(first, last - first + 1);
  }

  static bool parse_vector3_token(const std::string & token, tf2::Vector3 & value)
  {
    std::istringstream stream(token);
    std::string component;
    std::array<double, 3> values{0.0, 0.0, 0.0};
    for (std::size_t i = 0; i < values.size(); ++i) {
      if (!std::getline(stream, component, ',')) {
        return false;
      }
      try {
        values[i] = std::stod(trim_copy(component));
      } catch (const std::exception &) {
        return false;
      }
    }
    if (std::getline(stream, component, ',')) {
      return false;
    }
    value = tf2::Vector3(values[0], values[1], values[2]);
    return true;
  }

  static bool parse_collision_capsule(
    const std::string & specification,
    CollisionCapsule & capsule)
  {
    std::istringstream stream(specification);
    std::string link;
    std::string start;
    std::string end;
    if (!std::getline(stream, link, ':') || !std::getline(stream, start, ':') ||
      !std::getline(stream, end, ':'))
    {
      return false;
    }
    std::string extra;
    if (std::getline(stream, extra, ':')) {
      return false;
    }
    capsule.link_name = trim_copy(link);
    if (capsule.link_name.empty()) {
      return false;
    }
    return parse_vector3_token(start, capsule.start) && parse_vector3_token(end, capsule.end);
  }

  static std::vector<CollisionCapsule> parse_collision_capsules(
    const std::vector<std::string> & specifications,
    const rclcpp::Logger & logger,
    const std::string & parameter_name)
  {
    std::vector<CollisionCapsule> capsules;
    capsules.reserve(specifications.size());
    for (const auto & specification : specifications) {
      CollisionCapsule capsule;
      if (parse_collision_capsule(specification, capsule)) {
        capsules.push_back(capsule);
      } else {
        RCLCPP_WARN(
          logger,
          "Ignoring invalid %s entry '%s'. Expected 'link:x1,y1,z1:x2,y2,z2'",
          parameter_name.c_str(), specification.c_str());
      }
    }
    return capsules;
  }

  static bool parse_forbidden_box(
    const std::string & specification,
    ForbiddenBox & box)
  {
    std::istringstream stream(specification);
    std::string name;
    std::string center;
    std::string size;
    if (!std::getline(stream, name, ':') || !std::getline(stream, center, ':') ||
      !std::getline(stream, size, ':'))
    {
      return false;
    }
    std::string extra;
    if (std::getline(stream, extra, ':')) {
      return false;
    }
    box.name = trim_copy(name);
    if (box.name.empty()) {
      return false;
    }
    if (!parse_vector3_token(center, box.center) || !parse_vector3_token(size, box.size)) {
      return false;
    }
    return box.size.x() > 0.0 && box.size.y() > 0.0 && box.size.z() > 0.0;
  }

  static std::vector<ForbiddenBox> parse_forbidden_boxes(
    const std::vector<std::string> & specifications,
    const rclcpp::Logger & logger,
    const std::string & parameter_name)
  {
    std::vector<ForbiddenBox> boxes;
    boxes.reserve(specifications.size());
    for (const auto & specification : specifications) {
      ForbiddenBox box;
      if (parse_forbidden_box(specification, box)) {
        boxes.push_back(box);
      } else {
        RCLCPP_WARN(
          logger,
          "Ignoring invalid %s entry '%s'. Expected 'name:cx,cy,cz:sx,sy,sz' with positive size",
          parameter_name.c_str(), specification.c_str());
      }
    }
    return boxes;
  }

  static std::vector<CollisionCapsule> default_ur10e_collision_capsules(
    const std::string & prefix)
  {
    const auto link = [&prefix](const std::string & name) {
        return prefix + "/" + name;
      };

    return {
      // Local capsule centerlines in the UR link frames. The long arm links use the
      // visual/collision mesh offsets from the UR10e description instead of only
      // connecting joint-frame origins; that avoids diagonal shortcuts through the arm.
      {link("shoulder_link"), {0.0, 0.0, -0.06}, {0.0, 0.0, 0.10}},
      {link("upper_arm_link"), {0.02, 0.0, 0.1762}, {-0.6127, 0.0, 0.1762}},
      {link("forearm_link"), {0.02, 0.0, 0.0393}, {-0.57155, 0.0, 0.0393}},
      {link("forearm_link"), {-0.57155, 0.0, 0.0393}, {-0.57155, 0.0, 0.17415}},
      {link("wrist_1_link"), {0.0, 0.0, -0.155}, {0.0, 0.0, 0.02}},
      {link("wrist_1_link"), {0.0, -0.035, -0.02}, {0.0, -0.11985, -0.02}},
      {link("wrist_2_link"), {0.0, 0.0, -0.14}, {0.0, 0.0, 0.02}},
      {link("wrist_2_link"), {0.0, 0.035, -0.02}, {0.0, 0.11655, -0.02}},
      {link("wrist_3_link"), {0.0, 0.0, -0.135}, {0.0, 0.0, 0.025}},
    };
  }

  bool configure_collision_chains(const KDL::Tree & tree)
  {
    if (!tree.getChain(base_link_, tip_link_, own_collision_chain_)) {
      RCLCPP_ERROR(
        get_node()->get_logger(),
        "Could not build own collision KDL chain from '%s' to '%s'",
        base_link_.c_str(), tip_link_.c_str());
      return false;
    }
    bool other_chain_from_robot_description = true;
    if (!tree.getChain(collision_other_base_link_, collision_other_tip_link_, other_collision_chain_)) {
      other_chain_from_robot_description = false;
      other_collision_chain_ = own_collision_chain_;
      RCLCPP_WARN(
        get_node()->get_logger(),
        "Could not build other collision KDL chain from '%s' to '%s'; reusing own UR arm "
        "chain geometry and applying the configured other-arm mount transform",
        collision_other_base_link_.c_str(), collision_other_tip_link_.c_str());
    }

    collision_common_from_own_base_ =
      transform_from_xyz_rpy(collision_own_base_xyz_, collision_own_base_rpy_);
    collision_common_from_other_base_ =
      transform_from_xyz_rpy(collision_other_base_xyz_, collision_other_base_rpy_);

    own_collision_joint_names_ = movable_joint_names(own_collision_chain_);
    auto other_movable_joint_names = movable_joint_names(other_collision_chain_);
    if (collision_other_joint_names_.size() == other_movable_joint_names.size()) {
      other_collision_joint_names_ = collision_other_joint_names_;
    } else {
      RCLCPP_WARN(
        get_node()->get_logger(),
        "Parameter 'collision_other_joint_names' has %zu entries, expected %zu; using KDL joint names",
        collision_other_joint_names_.size(), other_movable_joint_names.size());
      other_collision_joint_names_ = std::move(other_movable_joint_names);
    }
    own_collision_fk_solver_ =
      std::make_unique<KDL::ChainFkSolverPos_recursive>(own_collision_chain_);
    other_collision_fk_solver_ =
      std::make_unique<KDL::ChainFkSolverPos_recursive>(other_collision_chain_);

    own_collision_capsules_ = collision_capsule_segments_.empty() ?
      default_ur10e_collision_capsules(prefix_) :
      parse_collision_capsules(
        collision_capsule_segments_, get_node()->get_logger(), "collision_capsule_segments");
    const std::string other_capsule_prefix =
      other_chain_from_robot_description ? collision_other_prefix_ : prefix_;
    other_collision_capsules_ = collision_other_capsule_segments_.empty() ?
      default_ur10e_collision_capsules(other_capsule_prefix) :
      parse_collision_capsules(
        collision_other_capsule_segments_, get_node()->get_logger(),
        "collision_other_capsule_segments");
    collision_forbidden_boxes_ = parse_forbidden_boxes(
      collision_forbidden_box_specs_, get_node()->get_logger(), "collision_forbidden_boxes");

    RCLCPP_INFO(
      get_node()->get_logger(),
      "Configured collision avoidance for %s against %s: common=%s, own_chain=%s->%s, "
      "other_chain=%s->%s%s, own_joints=%zu, other_joints=%zu, own_capsules=%zu, "
      "other_capsules=%zu, forbidden_boxes=%zu, other_capsule_prefix=%s",
      prefix_.c_str(), collision_other_prefix_.c_str(), collision_common_link_.c_str(),
      base_link_.c_str(), tip_link_.c_str(),
      collision_other_base_link_.c_str(), collision_other_tip_link_.c_str(),
      other_chain_from_robot_description ? "" : " (reused geometry)",
      own_collision_joint_names_.size(), other_collision_joint_names_.size(),
      own_collision_capsules_.size(), other_collision_capsules_.size(),
      collision_forbidden_boxes_.size(), other_capsule_prefix.c_str());
    return true;
  }

  bool fill_collision_q(
    const std::vector<std::string> & collision_joint_names,
    const CollisionJointState & joint_state,
    const rclcpp::Time & time,
    const KDL::JntArray * own_q,
    KDL::JntArray & q) const
  {
    q.resize(collision_joint_names.size());
    for (std::size_t i = 0; i < collision_joint_names.size(); ++i) {
      const auto & joint_name = collision_joint_names[i];
      bool assigned = false;
      if (own_q != nullptr) {
        const auto own_it = std::find(joint_names_.begin(), joint_names_.end(), joint_name);
        if (own_it != joint_names_.end()) {
          const auto own_index = static_cast<std::size_t>(std::distance(joint_names_.begin(), own_it));
          if (own_index < own_q->rows()) {
            q(static_cast<unsigned int>(i)) = (*own_q)(static_cast<unsigned int>(own_index));
            assigned = true;
          }
        }
      }
      if (!assigned) {
        const auto state_it = joint_state.positions.find(joint_name);
        if (state_it == joint_state.positions.end() || !std::isfinite(state_it->second)) {
          return false;
        }
        const auto stamp_it = joint_state.stamps.find(joint_name);
        if (stamp_it == joint_state.stamps.end() || stamp_it->second.nanoseconds() == 0 ||
          (time - stamp_it->second).seconds() > collision_joint_state_timeout_)
        {
          return false;
        }
        q(static_cast<unsigned int>(i)) = state_it->second;
      }
    }
    return true;
  }

  bool collision_joint_state_valid(
    const CollisionJointState & joint_state,
    const rclcpp::Time & time) const
  {
    if (!joint_state.valid) {
      return false;
    }
    if (joint_state.stamp.nanoseconds() == 0) {
      return false;
    }
    return (time - joint_state.stamp).seconds() <= collision_joint_state_timeout_;
  }

  static tf2::Vector3 point_from_kdl(const KDL::Vector & vector)
  {
    return {vector.x(), vector.y(), vector.z()};
  }

  static tf2::Transform transform_from_kdl(const KDL::Frame & frame)
  {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
    double w = 1.0;
    frame.M.GetQuaternion(x, y, z, w);
    tf2::Quaternion rotation(x, y, z, w);
    rotation.normalize();

    tf2::Transform transform;
    transform.setOrigin(point_from_kdl(frame.p));
    transform.setRotation(rotation);
    return transform;
  }

  void append_sampled_segment(
    const tf2::Vector3 & start,
    const tf2::Vector3 & end,
    std::vector<tf2::Vector3> & points) const
  {
    const double length = (end - start).length();
    const int steps = std::max(1, static_cast<int>(std::ceil(length / collision_sample_spacing_)));
    for (int step = 0; step <= steps; ++step) {
      const double t = static_cast<double>(step) / static_cast<double>(steps);
      points.push_back(start.lerp(end, t));
    }
  }

  bool forbidden_box_exempt_own_link(const std::string & link_name) const
  {
    return std::find(
      collision_forbidden_box_exempt_own_links_.begin(),
      collision_forbidden_box_exempt_own_links_.end(),
      link_name) != collision_forbidden_box_exempt_own_links_.end();
  }

  bool sample_collision_points(
    const KDL::Chain & chain,
    KDL::ChainFkSolverPos_recursive & solver,
    const KDL::JntArray & q,
    const tf2::Transform & common_from_chain_base,
    const std::vector<CollisionCapsule> & capsules,
    std::vector<tf2::Vector3> & points,
    std::vector<tf2::Vector3> * forbidden_box_points = nullptr) const
  {
    points.clear();
    if (forbidden_box_points != nullptr) {
      forbidden_box_points->clear();
    }

    if (!capsules.empty()) {
      std::map<std::string, unsigned int> segment_index_by_link;
      for (unsigned int i = 0; i < chain.getNrOfSegments(); ++i) {
        segment_index_by_link[chain.getSegment(i).getName()] = i + 1;
      }

      for (const auto & capsule : capsules) {
        const auto segment_it = segment_index_by_link.find(capsule.link_name);
        if (segment_it == segment_index_by_link.end()) {
          RCLCPP_WARN_THROTTLE(
            get_node()->get_logger(), *get_node()->get_clock(), 5000,
            "Collision capsule link '%s' is not part of chain '%s'->'%s'",
            capsule.link_name.c_str(),
            chain.getNrOfSegments() > 0 ? chain.getSegment(0).getName().c_str() : "",
            chain.getNrOfSegments() > 0 ?
            chain.getSegment(chain.getNrOfSegments() - 1).getName().c_str() : "");
          continue;
        }

        KDL::Frame link_frame;
        if (solver.JntToCart(q, link_frame, segment_it->second) < 0) {
          return false;
        }

        const tf2::Transform common_from_link =
          common_from_chain_base * transform_from_kdl(link_frame);
        const tf2::Vector3 start = common_from_link * capsule.start;
        const tf2::Vector3 end = common_from_link * capsule.end;
        std::vector<tf2::Vector3> segment_points;
        append_sampled_segment(start, end, segment_points);
        points.insert(points.end(), segment_points.begin(), segment_points.end());
        if (forbidden_box_points != nullptr && !forbidden_box_exempt_own_link(capsule.link_name)) {
          forbidden_box_points->insert(
            forbidden_box_points->end(), segment_points.begin(), segment_points.end());
        }
      }

      if (!points.empty()) {
        return true;
      }
      RCLCPP_WARN_THROTTLE(
        get_node()->get_logger(), *get_node()->get_clock(), 5000,
        "No configured collision capsules matched the KDL chain; falling back to joint-frame sampling");
    }

    KDL::Frame previous_frame;
    bool have_previous = false;
    bool started_movable_part = false;
    int sampled_segment_index = 0;

    for (unsigned int i = 0; i < chain.getNrOfSegments(); ++i) {
      KDL::Frame current_frame;
      if (solver.JntToCart(q, current_frame, i + 1) < 0) {
        return false;
      }

      const auto joint = chain.getSegment(i).getJoint();
      if (joint.getType() != KDL::Joint::None) {
        started_movable_part = true;
      }

      if (started_movable_part && have_previous) {
        const tf2::Vector3 start = common_from_chain_base * point_from_kdl(previous_frame.p);
        const tf2::Vector3 end = common_from_chain_base * point_from_kdl(current_frame.p);
        std::vector<tf2::Vector3> segment_points;
        append_sampled_segment(start, end, segment_points);
        points.insert(points.end(), segment_points.begin(), segment_points.end());
        if (forbidden_box_points != nullptr &&
          sampled_segment_index >= collision_forbidden_box_exempt_initial_segments_)
        {
          forbidden_box_points->insert(
            forbidden_box_points->end(), segment_points.begin(), segment_points.end());
        }
        ++sampled_segment_index;
      }

      previous_frame = current_frame;
      have_previous = true;
    }

    if (forbidden_box_points != nullptr && forbidden_box_points->empty()) {
      *forbidden_box_points = points;
    }
    return !points.empty();
  }

  void clear_latest_collision_points()
  {
    latest_own_collision_points_.clear();
    latest_other_collision_points_.clear();
    latest_nearest_own_collision_point_ = tf2::Vector3(0.0, 0.0, 0.0);
    latest_nearest_other_collision_point_ = tf2::Vector3(0.0, 0.0, 0.0);
    latest_collision_nearest_source_ = "none";
    latest_collision_points_valid_ = false;
  }

  static double signed_sphere_box_clearance(
    const tf2::Vector3 & point,
    const ForbiddenBox & box,
    double sphere_radius,
    tf2::Vector3 & nearest_box_point)
  {
    const tf2::Vector3 half_size = 0.5 * box.size;
    const tf2::Vector3 delta = point - box.center;
    const tf2::Vector3 abs_delta(std::abs(delta.x()), std::abs(delta.y()), std::abs(delta.z()));
    const tf2::Vector3 box_min = box.center - half_size;
    const tf2::Vector3 box_max = box.center + half_size;

    nearest_box_point = tf2::Vector3(
      std::clamp(point.x(), box_min.x(), box_max.x()),
      std::clamp(point.y(), box_min.y(), box_max.y()),
      std::clamp(point.z(), box_min.z(), box_max.z()));

    const bool inside = abs_delta.x() <= half_size.x() &&
      abs_delta.y() <= half_size.y() && abs_delta.z() <= half_size.z();
    if (!inside) {
      return (point - nearest_box_point).length() - sphere_radius;
    }

    const std::array<double, 3> margins{
      half_size.x() - abs_delta.x(),
      half_size.y() - abs_delta.y(),
      half_size.z() - abs_delta.z()};
    const auto axis_it = std::min_element(margins.begin(), margins.end());
    const std::size_t axis = static_cast<std::size_t>(std::distance(margins.begin(), axis_it));
    nearest_box_point = point;
    const double sign = axis == 0 ? (delta.x() >= 0.0 ? 1.0 : -1.0) :
      axis == 1 ? (delta.y() >= 0.0 ? 1.0 : -1.0) :
      (delta.z() >= 0.0 ? 1.0 : -1.0);
    if (axis == 0) {
      nearest_box_point.setX(box.center.x() + sign * half_size.x());
    } else if (axis == 1) {
      nearest_box_point.setY(box.center.y() + sign * half_size.y());
    } else {
      nearest_box_point.setZ(box.center.z() + sign * half_size.z());
    }
    return -*axis_it - sphere_radius;
  }

  bool compute_collision_clearance(
    const KDL::JntArray & own_collision_q,
    const KDL::JntArray & other_collision_q,
    double & clearance,
    bool store_debug_points = false)
  {
    if (!own_collision_fk_solver_ || !other_collision_fk_solver_) {
      if (store_debug_points) {
        clear_latest_collision_points();
      }
      return false;
    }

    std::vector<tf2::Vector3> own_points;
    std::vector<tf2::Vector3> own_forbidden_box_points;
    std::vector<tf2::Vector3> other_points;
    if (!sample_collision_points(
        own_collision_chain_, *own_collision_fk_solver_, own_collision_q,
        collision_common_from_own_base_, own_collision_capsules_, own_points,
        &own_forbidden_box_points) ||
      !sample_collision_points(
        other_collision_chain_, *other_collision_fk_solver_, other_collision_q,
        collision_common_from_other_base_, other_collision_capsules_, other_points))
    {
      if (store_debug_points) {
        clear_latest_collision_points();
      }
      return false;
    }

    double min_clearance = std::numeric_limits<double>::infinity();
    tf2::Vector3 nearest_own;
    tf2::Vector3 nearest_other;
    std::string nearest_source = "none";
    for (const auto & own_point : own_points) {
      for (const auto & other_point : other_points) {
        const double point_clearance =
          (own_point - other_point).length() - 2.0 * collision_sphere_radius_;
        if (point_clearance < min_clearance) {
          min_clearance = point_clearance;
          nearest_own = own_point;
          nearest_other = other_point;
          nearest_source = "other_arm";
        }
      }
    }
    for (const auto & own_point : own_forbidden_box_points) {
      for (const auto & box : collision_forbidden_boxes_) {
        tf2::Vector3 nearest_box_point;
        const double box_clearance = signed_sphere_box_clearance(
          own_point, box, collision_sphere_radius_, nearest_box_point);
        if (box_clearance < min_clearance) {
          min_clearance = box_clearance;
          nearest_own = own_point;
          nearest_other = nearest_box_point;
          nearest_source = box.name;
        }
      }
    }
    if (!std::isfinite(min_clearance)) {
      if (store_debug_points) {
        clear_latest_collision_points();
      }
      return false;
    }
    clearance = min_clearance;
    if (store_debug_points) {
      latest_own_collision_points_ = own_points;
      latest_other_collision_points_ = other_points;
      latest_nearest_own_collision_point_ = nearest_own;
      latest_nearest_other_collision_point_ = nearest_other;
      latest_collision_nearest_source_ = nearest_source;
      latest_collision_points_valid_ = true;
    }
    return true;
  }

  std::vector<double> apply_collision_avoidance(
    std::vector<double> qdot,
    const KDL::JntArray & own_q,
    const rclcpp::Time & time)
  {
    latest_collision_status_ = "disabled";
    latest_collision_min_clearance_ = std::numeric_limits<double>::quiet_NaN();
    latest_collision_scale_ = 1.0;

    if (!enable_collision_avoidance_) {
      clear_latest_collision_points();
      return qdot;
    }

    const auto maybe_joint_state = received_collision_joint_state_.try_get();
    const CollisionJointState joint_state = maybe_joint_state.value_or(CollisionJointState{});
    if (!collision_joint_state_valid(joint_state, time)) {
      latest_collision_status_ = "stale_other_arm";
      latest_collision_scale_ = 0.0;
      clear_latest_collision_points();
      return collision_fail_safe_stop_ ? std::vector<double>(qdot.size(), 0.0) : qdot;
    }

    KDL::JntArray own_collision_q;
    KDL::JntArray other_collision_q;
    if (!fill_collision_q(own_collision_joint_names_, joint_state, time, &own_q, own_collision_q) ||
      !fill_collision_q(other_collision_joint_names_, joint_state, time, nullptr, other_collision_q))
    {
      latest_collision_status_ = "stale_other_arm";
      latest_collision_scale_ = 0.0;
      clear_latest_collision_points();
      return collision_fail_safe_stop_ ? std::vector<double>(qdot.size(), 0.0) : qdot;
    }

    double clearance = std::numeric_limits<double>::quiet_NaN();
    if (!compute_collision_clearance(own_collision_q, other_collision_q, clearance, true)) {
      latest_collision_status_ = "stale_other_arm";
      latest_collision_scale_ = 0.0;
      clear_latest_collision_points();
      return collision_fail_safe_stop_ ? std::vector<double>(qdot.size(), 0.0) : qdot;
    }
    latest_collision_min_clearance_ = clearance;

    if (clearance >= collision_activation_clearance_) {
      latest_collision_status_ = "clear";
      return qdot;
    }

    std::vector<double> gradient(qdot.size(), 0.0);
    constexpr double gradient_step = 1.0e-4;
    for (std::size_t command_index = 0; command_index < command_joint_names_.size(); ++command_index) {
      const auto collision_it = std::find(
        own_collision_joint_names_.begin(), own_collision_joint_names_.end(),
        command_joint_names_[command_index]);
      if (collision_it == own_collision_joint_names_.end()) {
        continue;
      }

      const auto collision_index = static_cast<unsigned int>(
        std::distance(own_collision_joint_names_.begin(), collision_it));
      KDL::JntArray perturbed = own_collision_q;
      perturbed(collision_index) += gradient_step;

      double perturbed_clearance = clearance;
      if (compute_collision_clearance(perturbed, other_collision_q, perturbed_clearance)) {
        gradient[command_index] = (perturbed_clearance - clearance) / gradient_step;
      }
    }

    const CollisionResponse response = limit_collision_qdot(
      qdot, gradient, clearance, collision_stop_clearance_,
      collision_activation_clearance_, collision_response_mode_);
    latest_collision_status_ = response.status;
    latest_collision_scale_ = response.scale;
    return response.qdot;
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
    latest_safety_velocity_scale_ = 1.0;
    latest_safety_acceleration_scale_ = 1.0;
    latest_safety_jerk_scale_ = 1.0;
    latest_safety_limiting_joint_index_ = -1.0;
    latest_safety_limiting_stage_ = 0.0;

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
        latest_safety_limiting_joint_index_ = static_cast<double>(i);
        latest_safety_limiting_stage_ = 4.0;
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
          const double scale = velocity_limits[i] / std::abs(command[i]);
          if (scale < velocity_scale) {
            velocity_scale = scale;
            latest_safety_limiting_joint_index_ = static_cast<double>(i);
            latest_safety_limiting_stage_ = 1.0;
          }
        }
      }
      latest_safety_velocity_scale_ = velocity_scale;
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
          const double scale = limit / std::abs(desired_acceleration[i]);
          if (scale < acceleration_scale) {
            acceleration_scale = scale;
            latest_safety_limiting_joint_index_ = static_cast<double>(i);
            latest_safety_limiting_stage_ = 2.0;
          }
        }
      }
    }
    latest_safety_acceleration_scale_ = acceleration_scale;
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
          const double scale = max_delta / std::abs(acceleration_delta[i]);
          if (scale < jerk_scale) {
            jerk_scale = scale;
            latest_safety_limiting_joint_index_ = static_cast<double>(i);
            latest_safety_limiting_stage_ = 3.0;
          }
        }
      }
    }
    latest_safety_jerk_scale_ = jerk_scale;

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
        if (!position_blocked[i] && latest_safety_limiting_stage_ == 0.0) {
          latest_safety_limiting_joint_index_ = static_cast<double>(i);
          latest_safety_limiting_stage_ = 5.0;
        }
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
    const Eigen::VectorXd & target_twist)
  {
    std_msgs::msg::Float64MultiArray singular_msg;
    singular_msg.data.reserve(static_cast<std::size_t>(latest_singular_values_.size()) + 1);
    singular_msg.data.push_back(latest_inverse_condition_);
    for (Eigen::Index i = 0; i < latest_singular_values_.size(); ++i) {
      singular_msg.data.push_back(latest_singular_values_(i));
    }
    singular_values_pub_->publish(singular_msg);

    std_msgs::msg::Float64MultiArray debug_msg;
    debug_msg.data.reserve(20 + latest_qdot_raw_command_.size() +
      latest_qdot_after_collision_.size() + latest_qdot_after_safety_.size());
    debug_msg.data.push_back(latest_inverse_condition_);
    for (Eigen::Index i = 0; i < target_twist.size(); ++i) {
      debug_msg.data.push_back(target_twist(i));
    }
    for (Eigen::Index i = 0; i < latest_achieved_twist_.size(); ++i) {
      debug_msg.data.push_back(latest_achieved_twist_(i));
    }
    debug_msg.data.push_back(latest_collision_min_clearance_);
    debug_msg.data.push_back(latest_collision_scale_);
    debug_msg.data.push_back(latest_safety_velocity_scale_);
    debug_msg.data.push_back(latest_safety_acceleration_scale_);
    debug_msg.data.push_back(latest_safety_jerk_scale_);
    debug_msg.data.push_back(latest_safety_limiting_joint_index_);
    debug_msg.data.push_back(latest_safety_limiting_stage_);
    for (double value : latest_qdot_raw_command_) {
      debug_msg.data.push_back(value);
    }
    for (double value : latest_qdot_after_collision_) {
      debug_msg.data.push_back(value);
    }
    for (double value : latest_qdot_after_safety_) {
      debug_msg.data.push_back(value);
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

  void publish_collision_state(const rclcpp::Time &)
  {
    std_msgs::msg::Float64 clearance_msg;
    clearance_msg.data = latest_collision_min_clearance_;
    collision_min_clearance_pub_->publish(clearance_msg);

    std_msgs::msg::String status_msg;
    status_msg.data = latest_collision_status_;
    collision_status_pub_->publish(status_msg);

    std_msgs::msg::String source_msg;
    source_msg.data = latest_collision_nearest_source_;
    collision_nearest_source_pub_->publish(source_msg);
  }

  std::string collision_marker_frame() const
  {
    if (collision_common_link_.rfind(robot_name_ + "/", 0) == 0) {
      return collision_common_link_;
    }
    return robot_name_ + "/" + collision_common_link_;
  }

  static geometry_msgs::msg::Point point_msg(const tf2::Vector3 & point)
  {
    geometry_msgs::msg::Point msg;
    msg.x = point.x();
    msg.y = point.y();
    msg.z = point.z();
    return msg;
  }

  visualization_msgs::msg::Marker base_collision_marker(
    const rclcpp::Time & time,
    int id,
    const std::string & name_space,
    int type) const
  {
    visualization_msgs::msg::Marker marker;
    marker.header.stamp = time;
    marker.header.frame_id = collision_marker_frame();
    marker.ns = prefix_ + "_" + name_space;
    marker.id = id;
    marker.type = type;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    return marker;
  }

  void publish_collision_marker_delete_all(const rclcpp::Time & time)
  {
    if (!collision_markers_pub_) {
      return;
    }
    visualization_msgs::msg::MarkerArray markers;
    auto marker = base_collision_marker(
      time, 0, "collision_delete_all", visualization_msgs::msg::Marker::SPHERE_LIST);
    marker.action = visualization_msgs::msg::Marker::DELETEALL;
    markers.markers.push_back(marker);
    collision_markers_pub_->publish(markers);
  }

  void publish_collision_markers(const rclcpp::Time & time)
  {
    if (!publish_collision_markers_ || !collision_markers_pub_) {
      return;
    }
    if (collision_marker_publish_rate_hz_ <= 0.0) {
      return;
    }
    if (previous_collision_marker_publish_time_.nanoseconds() != 0 &&
      (time - previous_collision_marker_publish_time_).seconds() <
      1.0 / collision_marker_publish_rate_hz_)
    {
      return;
    }
    previous_collision_marker_publish_time_ = time;

    if (!enable_collision_avoidance_) {
      publish_collision_marker_delete_all(time);
      return;
    }
    if (!latest_collision_points_valid_ && collision_forbidden_boxes_.empty()) {
      publish_collision_marker_delete_all(time);
      return;
    }

    visualization_msgs::msg::MarkerArray markers;
    auto own_marker = base_collision_marker(
      time, 0, "own_collision_spheres", visualization_msgs::msg::Marker::SPHERE_LIST);
    own_marker.scale.x = 2.0 * collision_sphere_radius_;
    own_marker.scale.y = 2.0 * collision_sphere_radius_;
    own_marker.scale.z = 2.0 * collision_sphere_radius_;
    own_marker.color.r = 0.05;
    own_marker.color.g = 0.35;
    own_marker.color.b = 1.0;
    own_marker.color.a = 0.55;
    own_marker.points.reserve(latest_own_collision_points_.size());
    for (const auto & point : latest_own_collision_points_) {
      own_marker.points.push_back(point_msg(point));
    }

    auto other_marker = base_collision_marker(
      time, 1, "other_collision_spheres", visualization_msgs::msg::Marker::SPHERE_LIST);
    other_marker.scale.x = 2.0 * collision_sphere_radius_;
    other_marker.scale.y = 2.0 * collision_sphere_radius_;
    other_marker.scale.z = 2.0 * collision_sphere_radius_;
    other_marker.color.r = 1.0;
    other_marker.color.g = 0.45;
    other_marker.color.b = 0.0;
    other_marker.color.a = 0.55;
    other_marker.points.reserve(latest_other_collision_points_.size());
    for (const auto & point : latest_other_collision_points_) {
      other_marker.points.push_back(point_msg(point));
    }

    auto nearest_marker = base_collision_marker(
      time, 2, "nearest_collision_pair", visualization_msgs::msg::Marker::LINE_LIST);
    nearest_marker.scale.x = 0.012;
    nearest_marker.color.r = 1.0;
    nearest_marker.color.g = 0.0;
    nearest_marker.color.b = 0.0;
    nearest_marker.color.a = 0.95;
    if (latest_collision_points_valid_) {
      nearest_marker.points.push_back(point_msg(latest_nearest_own_collision_point_));
      nearest_marker.points.push_back(point_msg(latest_nearest_other_collision_point_));
    }

    markers.markers.push_back(own_marker);
    markers.markers.push_back(other_marker);
    markers.markers.push_back(nearest_marker);

    int box_id = 100;
    for (const auto & box : collision_forbidden_boxes_) {
      auto box_marker = base_collision_marker(
        time, box_id++, "forbidden_zones", visualization_msgs::msg::Marker::CUBE);
      box_marker.pose.position = point_msg(box.center);
      box_marker.pose.orientation.w = 1.0;
      box_marker.scale.x = box.size.x();
      box_marker.scale.y = box.size.y();
      box_marker.scale.z = box.size.z();
      box_marker.color.r = 0.9;
      box_marker.color.g = 0.0;
      box_marker.color.b = 0.1;
      box_marker.color.a = 0.25;
      markers.markers.push_back(box_marker);
    }

    collision_markers_pub_->publish(markers);
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
  std::string collision_other_prefix_;
  std::string collision_other_base_link_;
  std::string collision_other_tip_link_;
  std::string collision_common_link_;
  std::string collision_joint_states_topic_;
  std::string collision_marker_topic_;
  std::vector<std::string> joint_names_;
  std::vector<std::string> command_joint_names_;
  std::vector<std::string> chain_joint_names_;
  std::vector<std::string> ft_state_interface_names_;
  std::vector<std::string> collision_other_joint_names_;
  std::vector<std::string> own_collision_joint_names_;
  std::vector<std::string> other_collision_joint_names_;
  std::vector<std::string> collision_capsule_segments_;
  std::vector<std::string> collision_other_capsule_segments_;
  std::vector<std::string> collision_forbidden_box_specs_;
  std::vector<std::string> collision_forbidden_box_exempt_own_links_;
  int collision_forbidden_box_exempt_initial_segments_{1};
  std::vector<double> collision_own_base_xyz_;
  std::vector<double> collision_own_base_rpy_;
  std::vector<double> collision_other_base_xyz_;
  std::vector<double> collision_other_base_rpy_;
  std::vector<CollisionCapsule> own_collision_capsules_;
  std::vector<CollisionCapsule> other_collision_capsules_;
  std::vector<ForbiddenBox> collision_forbidden_boxes_;

  KDL::Chain chain_;
  KDL::Chain own_collision_chain_;
  KDL::Chain other_collision_chain_;
  std::unique_ptr<KDL::ChainJntToJacSolver> jac_solver_;
  std::unique_ptr<KDL::ChainFkSolverPos_recursive> fk_solver_;
  std::unique_ptr<KDL::ChainFkSolverPos_recursive> own_collision_fk_solver_;
  std::unique_ptr<KDL::ChainFkSolverPos_recursive> other_collision_fk_solver_;

  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr twist_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr collision_joint_state_sub_;
  realtime_tools::RealtimeThreadSafeBox<TwistReference> received_reference_;
  realtime_tools::RealtimeThreadSafeBox<CollisionJointState> received_collision_joint_state_;
  TwistReference last_reference_;
  bool subscriber_is_active_{true};

  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr singular_values_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr debug_twist_pub_;
  rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr filtered_wrench_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr equilibrium_pose_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr target_pose_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr collision_min_clearance_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr collision_status_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr collision_nearest_source_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr collision_markers_pub_;
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
  double collision_joint_state_timeout_{0.1};
  double collision_sample_spacing_{0.08};
  double collision_sphere_radius_{0.04};
  double collision_activation_clearance_{0.12};
  double collision_stop_clearance_{0.04};
  double collision_marker_publish_rate_hz_{10.0};
  CollisionResponseMode collision_response_mode_{CollisionResponseMode::Scale};
  bool require_wrench_{false};
  bool use_ft_sensor_{false};
  bool wrench_in_tcp_frame_{true};
  bool preserve_command_direction_{true};
  bool immediate_zero_on_zero_command_{true};
  bool reset_equilibrium_on_zero_command_{true};
  bool enable_collision_avoidance_{false};
  bool collision_fail_safe_stop_{true};
  bool publish_collision_markers_{false};

  std::vector<double> admittance_;
  std::vector<double> wrench_twist_gain_;
  std::vector<double> pose_error_gain_;
  std::vector<double> wrench_sign_;
  std::vector<double> joint_lower_limits_;
  std::vector<double> joint_upper_limits_;
  std::vector<double> joint_velocity_limits_;
  std::vector<double> joint_acceleration_limits_;
  std::vector<double> joint_jerk_limits_;
  std::vector<double> last_commanded_velocity_;
  std::vector<double> last_commanded_acceleration_;
  std::vector<double> latest_qdot_raw_command_;
  std::vector<double> latest_qdot_after_collision_;
  std::vector<double> latest_qdot_after_safety_;
  std::vector<tf2::Vector3> latest_own_collision_points_;
  std::vector<tf2::Vector3> latest_other_collision_points_;

  Vector6 filtered_wrench_{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  Vector6 wrench_bias_{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  std::size_t bias_sample_count_{0};
  bool bias_ready_{true};
  bool have_wrench_{false};

  tf2::Vector3 equilibrium_position_{0.0, 0.0, 0.0};
  tf2::Quaternion equilibrium_orientation_{0.0, 0.0, 0.0, 1.0};
  tf2::Vector3 latest_target_position_{0.0, 0.0, 0.0};
  tf2::Quaternion latest_target_orientation_{0.0, 0.0, 0.0, 1.0};
  tf2::Transform collision_common_from_own_base_{tf2::Transform::getIdentity()};
  tf2::Transform collision_common_from_other_base_{tf2::Transform::getIdentity()};
  tf2::Vector3 latest_nearest_own_collision_point_{0.0, 0.0, 0.0};
  tf2::Vector3 latest_nearest_other_collision_point_{0.0, 0.0, 0.0};
  bool equilibrium_initialized_{false};
  bool latest_collision_points_valid_{false};

  rclcpp::Time last_update_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_wrench_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time bias_start_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time previous_publish_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time previous_collision_marker_publish_time_{0, 0, RCL_ROS_TIME};

  Eigen::VectorXd latest_singular_values_;
  Eigen::VectorXd latest_achieved_twist_{Eigen::VectorXd::Zero(6)};
  Eigen::MatrixXd latest_jacobian_;
  double latest_inverse_condition_{0.0};
  double latest_collision_min_clearance_{std::numeric_limits<double>::quiet_NaN()};
  double latest_collision_scale_{1.0};
  double latest_safety_velocity_scale_{1.0};
  double latest_safety_acceleration_scale_{1.0};
  double latest_safety_jerk_scale_{1.0};
  double latest_safety_limiting_joint_index_{-1.0};
  double latest_safety_limiting_stage_{0.0};
  std::string latest_collision_status_{"disabled"};
  std::string latest_collision_nearest_source_{"none"};
};

}  // namespace mur_control

PLUGINLIB_EXPORT_CLASS(
  mur_control::IntegratedCartesianAdmittanceController,
  controller_interface::ChainableControllerInterface)
