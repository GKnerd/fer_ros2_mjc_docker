#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/string.hpp"

#include <kdl/chain.hpp>
#include <kdl/chaindynparam.hpp>
#include <kdl/jntarray.hpp>
#include <kdl/jntspaceinertiamatrix.hpp>
#include <kdl/tree.hpp>
#include <kdl_parser/kdl_parser.hpp>

class MPCControllerNode : public rclcpp::Node
{
public:
  MPCControllerNode() : Node("mpc_controller_node")
  {
    joint_names_ = {
      "fer_joint1", "fer_joint2", "fer_joint3", "fer_joint4",
      "fer_joint5", "fer_joint6", "fer_joint7"
    };

    // Target pose (same as FL node target for easy comparison).
    q_des_   = {0.5, -1.0, 0.0, -1.5, 0.0, 2.0, 0.3};
    dq_des_  = std::vector<double>(7, 0.0);
    ddq_des_ = std::vector<double>(7, 0.0);

    tau_limits_ = {87.0, 87.0, 87.0, 87.0, 12.0, 12.0, 12.0};

    // Reflected motor inertia (motor_inertia * gear_ratio^2) from dynamics.yaml.
    armature_ = {0.605721456, 0.605721456,
                 0.462474144, 0.462474144,
                 0.205544064, 0.205544064, 0.205544064};

    effort_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
      "/effort_forward_controller/commands", 10);

    joint_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 10,
      std::bind(&MPCControllerNode::jointCallback, this, std::placeholders::_1));

    auto qos = rclcpp::QoS(1).transient_local().reliable();
    robot_description_sub_ = this->create_subscription<std_msgs::msg::String>(
      "/robot_description", qos,
      std::bind(&MPCControllerNode::robotDescriptionCallback, this, std::placeholders::_1));

    log_file_.open("mpc_full_log.csv");
    log_file_ << std::setprecision(17);

    log_file_ << "time";
    for (int i = 0; i < 7; ++i) log_file_ << ",q" << i + 1;
    for (int i = 0; i < 7; ++i) log_file_ << ",dq" << i + 1;
    for (int i = 0; i < 7; ++i) log_file_ << ",qdes" << i + 1;
    for (int i = 0; i < 7; ++i) log_file_ << ",error" << i + 1;
    for (int i = 0; i < 7; ++i) log_file_ << ",v" << i + 1;
    for (int i = 0; i < 7; ++i) log_file_ << ",mv" << i + 1;
    for (int i = 0; i < 7; ++i) log_file_ << ",c" << i + 1;
    for (int i = 0; i < 7; ++i) log_file_ << ",g" << i + 1;
    for (int i = 0; i < 7; ++i) log_file_ << ",tau_friction" << i + 1;
    for (int i = 0; i < 7; ++i) log_file_ << ",tau_raw" << i + 1;
    for (int i = 0; i < 7; ++i) log_file_ << ",tau_sat" << i + 1;
    log_file_ << "\n";

    RCLCPP_INFO(this->get_logger(), "MPC controller node started.");
  }

  ~MPCControllerNode()
  {
    if (log_file_.is_open()) {
      log_file_.close();
    }
  }

private:
  // Simple single-joint shooting MPC.
  // Sweeps candidate virtual accelerations v over [-v_max, v_max] and picks
  // the one with lowest cost over the prediction horizon.
  // Each joint is treated independently (decoupled approximation) because the
  // full coupled MPC would require solving a 7D optimization every 10 ms.
  double computeMPCAcceleration(double q, double dq, double q_des)
  {
    const double dt        = 0.01;    // prediction step [s]
    const int    horizon   = 15;      // prediction steps
    const double q_weight  = 100.0;   // penalize position error
    const double dq_weight = 5.0;     // penalize velocity at each step
    const double u_weight  = 0.01;    // penalize large virtual acceleration

    double best_v    = 0.0;
    double best_cost = 1e18;

    for (double v_candidate = -30.0; v_candidate <= 30.0; v_candidate += 1.0) {
      double q_sim  = q;
      double dq_sim = dq;
      double cost   = 0.0;

      for (int k = 0; k < horizon; ++k) {
        q_sim  += dq_sim * dt + 0.5 * v_candidate * dt * dt;
        dq_sim += v_candidate * dt;

        double e = q_des - q_sim;
        cost += q_weight  * e * e
              + dq_weight * dq_sim * dq_sim
              + u_weight  * v_candidate * v_candidate;
      }

      if (cost < best_cost) {
        best_cost = cost;
        best_v    = v_candidate;
      }
    }

    return best_v;
  }

  // Identified friction model for Franka Panda (Cognetti et al. 2019).
  // Returns torque the motor must supply to overcome friction (add to tau_raw).
  std::vector<double> computeFrictionTorque(const std::vector<double> & dq)
  {
    const std::vector<double> fi1 = {
      0.54615, 0.87224, 0.64068, 1.2794, 0.83904, 0.30301, 0.56489
    };
    const std::vector<double> fi2 = {
      5.1181, 9.0657, 10.136, 5.5903, 8.3469, 17.133, 10.336
    };
    const std::vector<double> fi3 = {
      0.039533, 0.025882, -0.04607, 0.036194, 0.026226, -0.021047, 0.0035526
    };

    std::vector<double> tau_friction(7, 0.0);
    for (size_t i = 0; i < 7; ++i) {
      const double tau_f_const = fi1[i] / (1.0 + std::exp(-fi2[i] * fi3[i]));
      tau_friction[i] = fi1[i] / (1.0 + std::exp(-fi2[i] * (dq[i] + fi3[i]))) - tau_f_const;
    }
    return tau_friction;
  }

  void robotDescriptionCallback(const std_msgs::msg::String::SharedPtr msg)
  {
    std::scoped_lock<std::mutex> lock(mutex_);

    if (model_ready_) {
      return;
    }

    KDL::Tree tree;
    if (!kdl_parser::treeFromString(msg->data, tree)) {
      RCLCPP_ERROR(this->get_logger(), "Failed to parse /robot_description");
      return;
    }

    if (!tree.getChain("base", "fer_hand_tcp", chain_)) {
      RCLCPP_ERROR(this->get_logger(), "Failed to extract KDL chain from base to fer_hand_tcp");
      return;
    }

    if (chain_.getNrOfJoints() != 7) {
      RCLCPP_ERROR(
        this->get_logger(),
        "KDL chain has %u joints, expected 7. Check base/tip link.",
        chain_.getNrOfJoints());
      return;
    }

    dyn_solver_ = std::make_unique<KDL::ChainDynParam>(
      chain_, KDL::Vector(0.0, 0.0, -9.81));

    q_kdl_.resize(7);
    dq_kdl_.resize(7);
    coriolis_kdl_.resize(7);
    gravity_kdl_.resize(7);
    mass_kdl_.resize(7);

    model_ready_ = true;
    RCLCPP_INFO(this->get_logger(), "KDL model initialized successfully.");
  }

  void jointCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    std::scoped_lock<std::mutex> lock(mutex_);

    if (!model_ready_) {
      return;
    }

    std::unordered_map<std::string, size_t> name_to_index;
    for (size_t i = 0; i < msg->name.size(); ++i) {
      name_to_index[msg->name[i]] = i;
    }

    std::vector<double> q(7, 0.0);
    std::vector<double> dq(7, 0.0);
    std::vector<double> error(7, 0.0);
    std::vector<double> v(7, 0.0);
    std::vector<double> mv_vec(7, 0.0);
    std::vector<double> c_vec(7, 0.0);
    std::vector<double> g_vec(7, 0.0);
    std::vector<double> tau_friction(7, 0.0);
    std::vector<double> tau_raw(7, 0.0);
    std::vector<double> tau_sat(7, 0.0);

    for (size_t i = 0; i < joint_names_.size(); ++i) {
      if (name_to_index.find(joint_names_[i]) == name_to_index.end()) {
        RCLCPP_WARN_THROTTLE(
          this->get_logger(), *this->get_clock(), 2000,
          "Joint %s not found in /joint_states yet.", joint_names_[i].c_str());
        return;
      }

      size_t idx = name_to_index[joint_names_[i]];
      if (idx >= msg->position.size() || idx >= msg->velocity.size()) {
        RCLCPP_WARN_THROTTLE(
          this->get_logger(), *this->get_clock(), 2000,
          "Incomplete joint state data.");
        return;
      }

      q[i]      = msg->position[idx];
      dq[i]     = msg->velocity[idx];
      q_kdl_(i) = q[i];
      dq_kdl_(i) = dq[i];
    }

    if (dyn_solver_->JntToMass(q_kdl_, mass_kdl_) != 0) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "JntToMass failed.");
      return;
    }
    if (dyn_solver_->JntToCoriolis(q_kdl_, dq_kdl_, coriolis_kdl_) != 0) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "JntToCoriolis failed.");
      return;
    }
    if (dyn_solver_->JntToGravity(q_kdl_, gravity_kdl_) != 0) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "JntToGravity failed.");
      return;
    }

    // MPC computes the virtual acceleration v for each joint independently.
    for (size_t i = 0; i < 7; ++i) {
      error[i] = q_des_[i] - q[i];
      v[i] = computeMPCAcceleration(q[i], dq[i], q_des_[i]);
    }

    tau_friction = computeFrictionTorque(dq);

    for (size_t i = 0; i < 7; ++i) {
      double mv = 0.0;
      for (size_t j = 0; j < 7; ++j) {
        double M_ij = mass_kdl_(i, j);
        if (i == j) M_ij += armature_[i];
        mv += M_ij * v[j];
      }

      mv_vec[i] = mv;
      c_vec[i]  = coriolis_kdl_(i);
      g_vec[i]  = gravity_kdl_(i);

      // gravcomp=1 in MJCF already cancels gravity — do not add g_vec.
      // Add tau_friction (+) to supply the torque needed to overcome friction.
      tau_raw[i] = mv_vec[i] + c_vec[i] + tau_friction[i];
      tau_sat[i] = std::clamp(tau_raw[i], -tau_limits_[i], tau_limits_[i]);
    }

    std_msgs::msg::Float64MultiArray cmd;
    cmd.data = tau_sat;
    effort_pub_->publish(cmd);

    double t_now = this->now().seconds();
    if (start_time_ < 0.0) {
      start_time_ = t_now;
    }
    double t = t_now - start_time_;

    if (log_file_.is_open()) {
      log_file_ << t;
      for (double val : q)            log_file_ << "," << val;
      for (double val : dq)           log_file_ << "," << val;
      for (double val : q_des_)       log_file_ << "," << val;
      for (double val : error)        log_file_ << "," << val;
      for (double val : v)            log_file_ << "," << val;
      for (double val : mv_vec)       log_file_ << "," << val;
      for (double val : c_vec)        log_file_ << "," << val;
      for (double val : g_vec)        log_file_ << "," << val;
      for (double val : tau_friction) log_file_ << "," << val;
      for (double val : tau_raw)      log_file_ << "," << val;
      for (double val : tau_sat)      log_file_ << "," << val;
      log_file_ << "\n";
    }

    RCLCPP_INFO_THROTTLE(
      this->get_logger(), *this->get_clock(), 2000,
      "MPC: raw=[%.2f %.2f %.2f], sat=[%.2f %.2f %.2f]",
      tau_raw[0], tau_raw[1], tau_raw[2],
      tau_sat[0], tau_sat[1], tau_sat[2]);
  }

  std::mutex mutex_;

  bool   model_ready_{false};
  double start_time_{-1.0};

  std::vector<std::string> joint_names_;
  std::vector<double>      q_des_;
  std::vector<double>      dq_des_;
  std::vector<double>      ddq_des_;
  std::vector<double>      tau_limits_;
  std::vector<double>      armature_;

  KDL::Chain                            chain_;
  std::unique_ptr<KDL::ChainDynParam>   dyn_solver_;
  KDL::JntArray                         q_kdl_;
  KDL::JntArray                         dq_kdl_;
  KDL::JntArray                         coriolis_kdl_;
  KDL::JntArray                         gravity_kdl_;
  KDL::JntSpaceInertiaMatrix            mass_kdl_;

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr  joint_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr         robot_description_sub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr effort_pub_;

  std::ofstream log_file_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MPCControllerNode>());
  rclcpp::shutdown();
  return 0;
}
