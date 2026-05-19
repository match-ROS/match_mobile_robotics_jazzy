#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Dense>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/jacobian.hpp>
#include <kdl/jntarray.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_msgs/msg/string.hpp>

using namespace std::chrono_literals;

namespace
{
Eigen::MatrixXd pseudoInverse(const Eigen::MatrixXd & matrix, double tolerance)
{
  if (matrix.size() == 0) {
    return Eigen::MatrixXd(matrix.cols(), matrix.rows());
  }

  Eigen::JacobiSVD<Eigen::MatrixXd> svd(
    matrix, Eigen::ComputeFullU | Eigen::ComputeFullV);
  const auto & singular_values = svd.singularValues();

  Eigen::MatrixXd sigma_inv =
    Eigen::MatrixXd::Zero(matrix.cols(), matrix.rows());
  for (Eigen::Index i = 0; i < singular_values.size(); ++i) {
    if (singular_values(i) > tolerance) {
      sigma_inv(i, i) = 1.0 / singular_values(i);
    }
  }

  return svd.matrixV() * sigma_inv * svd.matrixU().transpose();
}

Eigen::MatrixXd composeSvd(
  const Eigen::MatrixXd & u,
  const std::vector<double> & singular_values,
  const Eigen::MatrixXd & vt)
{
  Eigen::MatrixXd sigma = Eigen::MatrixXd::Zero(u.cols(), vt.rows());
  const auto count = std::min<Eigen::Index>(
    static_cast<Eigen::Index>(singular_values.size()),
    std::min(sigma.rows(), sigma.cols()));
  for (Eigen::Index i = 0; i < count; ++i) {
    sigma(i, i) = singular_values[static_cast<std::size_t>(i)];
  }
  return u * sigma * vt;
}

Eigen::MatrixXd computeJParseInverse(
  const Eigen::MatrixXd & jacobian,
  double gamma,
  double singular_gain_position,
  double singular_gain_angular,
  double pinv_tolerance,
  Eigen::VectorXd * singular_values_out,
  double * inverse_condition_out)
{
  Eigen::JacobiSVD<Eigen::MatrixXd> svd(
    jacobian, Eigen::ComputeFullU | Eigen::ComputeFullV);
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
    j_proj = composeSvd(u_proj, non_singular_values, vt);
  } else {
    j_proj = jacobian;
  }

  const Eigen::MatrixXd j_safety = composeSvd(u, safety_values, vt);
  const Eigen::MatrixXd j_safety_pinv = pseudoInverse(j_safety, pinv_tolerance);
  const Eigen::MatrixXd j_proj_pinv = pseudoInverse(j_proj, pinv_tolerance);

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

class JParseVelocityController : public rclcpp::Node
{
public:
  explicit JParseVelocityController(const rclcpp::NodeOptions & options)
  : Node("jparse_velocity_controller", options)
  {
    robot_name_ = declare_parameter<std::string>("robot_name", "mur620a");
    arm_ = declare_parameter<std::string>("arm", "l");
    base_link_ = declare_parameter<std::string>(
      "base_link", arm_ == "l" ? "UR10_l/base_link" : "UR10_r/base_link");
    tip_link_ = declare_parameter<std::string>(
      "tip_link", arm_ == "l" ? "UR10_l/tool0" : "UR10_r/tool0");
    robot_description_topic_ = declare_parameter<std::string>(
      "robot_description_topic", "/" + robot_name_ + "/robot_description");
    twist_topic_ = declare_parameter<std::string>("twist_topic", "~/twist_cmd");
    command_topic_ = declare_parameter<std::string>(
      "command_topic", "/" + robot_name_ + "/forward_velocity_controller_" + arm_ + "/commands");
    joint_states_topic_ = declare_parameter<std::string>(
      "joint_states_topic", "/" + robot_name_ + "/joint_states");
    singular_values_topic_ = declare_parameter<std::string>(
      "singular_values_topic",
      "/" + robot_name_ + "/jparse_velocity_controller_" + arm_ + "/singular_values");
    debug_twist_topic_ = declare_parameter<std::string>(
      "debug_twist_topic",
      "/" + robot_name_ + "/jparse_velocity_controller_" + arm_ + "/debug_twist");
    rate_hz_ = declare_parameter<double>("rate_hz", 500.0);
    command_timeout_ = declare_parameter<double>("command_timeout", 0.12);
    gamma_ = declare_parameter<double>("gamma", 0.1);
    singular_gain_position_ = declare_parameter<double>("singular_gain_position", 1.0);
    singular_gain_angular_ = declare_parameter<double>("singular_gain_angular", 1.0);
    pinv_tolerance_ = declare_parameter<double>("pinv_tolerance", 1.0e-6);
    max_joint_velocity_ = declare_parameter<double>("max_joint_velocity", 1.5);
    max_cartesian_linear_velocity_ =
      declare_parameter<double>("max_cartesian_linear_velocity", 0.25);
    max_cartesian_angular_velocity_ =
      declare_parameter<double>("max_cartesian_angular_velocity", 0.8);

    gamma_ = std::clamp(gamma_, 1.0e-4, 0.999);
    rate_hz_ = std::max(1.0, rate_hz_);

    command_pub_ =
      create_publisher<std_msgs::msg::Float64MultiArray>(command_topic_, rclcpp::SystemDefaultsQoS());
    singular_values_pub_ =
      create_publisher<std_msgs::msg::Float64MultiArray>(singular_values_topic_, 10);
    debug_twist_pub_ =
      create_publisher<std_msgs::msg::Float64MultiArray>(debug_twist_topic_, 10);

    auto robot_description_qos = rclcpp::QoS(1).transient_local().reliable();
    robot_description_sub_ = create_subscription<std_msgs::msg::String>(
      robot_description_topic_, robot_description_qos,
      [this](const std_msgs::msg::String::SharedPtr msg) {
        if (!chain_ready_) {
          configureChain(msg->data);
        }
      });

    joint_state_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      joint_states_topic_, rclcpp::SensorDataQoS(),
      [this](const sensor_msgs::msg::JointState::SharedPtr msg) {
        updateJointState(*msg);
      });

    twist_sub_ = create_subscription<geometry_msgs::msg::TwistStamped>(
      twist_topic_, rclcpp::SystemDefaultsQoS(),
      [this](const geometry_msgs::msg::TwistStamped::SharedPtr msg) {
        updateTwist(*msg);
      });

    const auto period = std::chrono::duration<double>(1.0 / rate_hz_);
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      [this]() { update(); });

    RCLCPP_INFO(
      get_logger(),
      "J-PARSE velocity controller ready for arm %s: %s -> %s, command=%s, twist=%s, debug=%s",
      arm_.c_str(), base_link_.c_str(), tip_link_.c_str(),
      command_topic_.c_str(), twist_topic_.c_str(), debug_twist_topic_.c_str());
  }

private:
  void configureChain(const std::string & urdf_xml)
  {
    KDL::Tree tree;
    if (!kdl_parser::treeFromString(urdf_xml, tree)) {
      RCLCPP_ERROR(get_logger(), "Could not parse robot_description into KDL tree");
      return;
    }

    KDL::Chain chain;
    if (!tree.getChain(base_link_, tip_link_, chain)) {
      RCLCPP_ERROR(
        get_logger(), "Could not build KDL chain from '%s' to '%s'",
        base_link_.c_str(), tip_link_.c_str());
      return;
    }

    std::vector<std::string> joint_names;
    for (unsigned int i = 0; i < chain.getNrOfSegments(); ++i) {
      const auto joint = chain.getSegment(i).getJoint();
      if (joint.getType() != KDL::Joint::None) {
        joint_names.push_back(joint.getName());
      }
    }

    if (joint_names.empty()) {
      RCLCPP_ERROR(get_logger(), "KDL chain has no movable joints");
      return;
    }

    chain_ = chain;
    chain_joint_names_ = joint_names;
    command_joint_names_ = declare_parameter<std::vector<std::string>>(
      "command_joint_names", chain_joint_names_);
    jac_solver_ = std::make_unique<KDL::ChainJntToJacSolver>(chain_);
    chain_ready_ = true;

    RCLCPP_INFO(
      get_logger(), "Configured KDL chain with %zu joints from %s to %s",
      chain_joint_names_.size(), base_link_.c_str(), tip_link_.c_str());
  }

  void updateJointState(const sensor_msgs::msg::JointState & msg)
  {
    for (std::size_t i = 0; i < msg.name.size(); ++i) {
      if (i < msg.position.size() && std::isfinite(msg.position[i])) {
        joint_positions_[msg.name[i]] = msg.position[i];
      }
    }
  }

  void updateTwist(const geometry_msgs::msg::TwistStamped & msg)
  {
    const auto & frame = msg.header.frame_id;
    if (!frame.empty() && frame != base_link_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Twist command frame_id '%s' is not '%s'; interpreting it in the arm base frame for now",
        frame.c_str(), base_link_.c_str());
    }

    target_twist_ << msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z,
      msg.twist.angular.x, msg.twist.angular.y, msg.twist.angular.z;
    target_twist_.head<3>() = clampVectorNorm(
      target_twist_.head<3>(), max_cartesian_linear_velocity_);
    target_twist_.tail<3>() = clampVectorNorm(
      target_twist_.tail<3>(), max_cartesian_angular_velocity_);
    last_twist_time_ = now();
    have_twist_ = true;
    idle_zero_sent_ = false;
  }

  Eigen::Vector3d clampVectorNorm(const Eigen::Vector3d & value, double max_norm) const
  {
    if (max_norm <= 0.0) {
      return Eigen::Vector3d::Zero();
    }
    const double norm = value.norm();
    if (norm <= max_norm || norm <= std::numeric_limits<double>::epsilon()) {
      return value;
    }
    return value * (max_norm / norm);
  }

  bool readChainPositions(KDL::JntArray & q) const
  {
    q.resize(chain_joint_names_.size());
    for (std::size_t i = 0; i < chain_joint_names_.size(); ++i) {
      const auto it = joint_positions_.find(chain_joint_names_[i]);
      if (it == joint_positions_.end()) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "Waiting for joint state '%s'", chain_joint_names_[i].c_str());
        return false;
      }
      q(static_cast<unsigned int>(i)) = it->second;
    }
    return true;
  }

  void publishCommand(const std::vector<double> & velocities)
  {
    std_msgs::msg::Float64MultiArray msg;
    msg.data = velocities;
    command_pub_->publish(msg);
  }

  void publishZero()
  {
    publishCommand(std::vector<double>(command_joint_names_.size(), 0.0));
  }

  void publishIdleZeroOnce()
  {
    if (idle_zero_sent_) {
      return;
    }
    publishZero();
    idle_zero_sent_ = true;
  }

  void update()
  {
    if (!chain_ready_) {
      return;
    }

    if (!have_twist_ || (now() - last_twist_time_).seconds() > command_timeout_) {
      publishIdleZeroOnce();
      return;
    }

    KDL::JntArray q;
    if (!readChainPositions(q)) {
      publishZero();
      return;
    }

    KDL::Jacobian kdl_jacobian(chain_joint_names_.size());
    if (jac_solver_->JntToJac(q, kdl_jacobian) < 0) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "KDL Jacobian computation failed");
      publishZero();
      return;
    }

    Eigen::MatrixXd jacobian(6, static_cast<Eigen::Index>(chain_joint_names_.size()));
    for (unsigned int col = 0; col < kdl_jacobian.columns(); ++col) {
      for (unsigned int row = 0; row < 6; ++row) {
        jacobian(static_cast<Eigen::Index>(row), static_cast<Eigen::Index>(col)) =
          kdl_jacobian(row, col);
      }
    }

    Eigen::VectorXd singular_values;
    double inverse_condition = 0.0;
    const Eigen::MatrixXd jparse_inverse = computeJParseInverse(
      jacobian, gamma_, singular_gain_position_, singular_gain_angular_,
      pinv_tolerance_, &singular_values, &inverse_condition);
    Eigen::VectorXd qdot = jparse_inverse * target_twist_;

    double max_abs_velocity = 0.0;
    for (Eigen::Index i = 0; i < qdot.size(); ++i) {
      max_abs_velocity = std::max(max_abs_velocity, std::abs(qdot(i)));
    }
    if (max_abs_velocity > max_joint_velocity_ && max_abs_velocity > 0.0) {
      qdot *= max_joint_velocity_ / max_abs_velocity;
    }
    const Eigen::VectorXd achieved_twist = jacobian * qdot;

    std::map<std::string, double> qdot_by_joint;
    for (std::size_t i = 0; i < chain_joint_names_.size(); ++i) {
      qdot_by_joint[chain_joint_names_[i]] = qdot(static_cast<Eigen::Index>(i));
    }

    std::vector<double> command;
    command.reserve(command_joint_names_.size());
    for (const auto & joint_name : command_joint_names_) {
      const auto it = qdot_by_joint.find(joint_name);
      command.push_back(it == qdot_by_joint.end() ? 0.0 : it->second);
    }
    publishCommand(command);

    std_msgs::msg::Float64MultiArray singular_msg;
    singular_msg.data.reserve(static_cast<std::size_t>(singular_values.size()) + 1);
    singular_msg.data.push_back(inverse_condition);
    for (Eigen::Index i = 0; i < singular_values.size(); ++i) {
      singular_msg.data.push_back(singular_values(i));
    }
    singular_values_pub_->publish(singular_msg);

    std_msgs::msg::Float64MultiArray debug_msg;
    debug_msg.data.reserve(13 + static_cast<std::size_t>(qdot.size()));
    debug_msg.data.push_back(inverse_condition);
    for (Eigen::Index i = 0; i < target_twist_.size(); ++i) {
      debug_msg.data.push_back(target_twist_(i));
    }
    for (Eigen::Index i = 0; i < achieved_twist.size(); ++i) {
      debug_msg.data.push_back(achieved_twist(i));
    }
    for (Eigen::Index i = 0; i < qdot.size(); ++i) {
      debug_msg.data.push_back(qdot(i));
    }
    debug_twist_pub_->publish(debug_msg);
  }

  std::string robot_name_;
  std::string arm_;
  std::string base_link_;
  std::string tip_link_;
  std::string robot_description_topic_;
  std::string twist_topic_;
  std::string command_topic_;
  std::string joint_states_topic_;
  std::string singular_values_topic_;
  std::string debug_twist_topic_;
  double rate_hz_;
  double command_timeout_;
  double gamma_;
  double singular_gain_position_;
  double singular_gain_angular_;
  double pinv_tolerance_;
  double max_joint_velocity_;
  double max_cartesian_linear_velocity_;
  double max_cartesian_angular_velocity_;

  KDL::Chain chain_;
  std::unique_ptr<KDL::ChainJntToJacSolver> jac_solver_;
  std::vector<std::string> chain_joint_names_;
  std::vector<std::string> command_joint_names_;
  bool chain_ready_{false};

  std::map<std::string, double> joint_positions_;
  Eigen::Matrix<double, 6, 1> target_twist_{Eigen::Matrix<double, 6, 1>::Zero()};
  rclcpp::Time last_twist_time_{0, 0, RCL_ROS_TIME};
  bool have_twist_{false};
  bool idle_zero_sent_{false};

  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr command_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr singular_values_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr debug_twist_pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr robot_description_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr twist_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<JParseVelocityController>(rclcpp::NodeOptions()));
  rclcpp::shutdown();
  return 0;
}
