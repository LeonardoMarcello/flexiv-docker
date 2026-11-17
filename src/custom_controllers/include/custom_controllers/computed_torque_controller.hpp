/**
 * @file computed_torque_controller.hpp
 * @brief Joint-space computed torque controller as ROS 2 controller, using the Flexiv RDK
 * dynamic model (flexiv::rdk::Model). Ported from panda_controllers/ComputedTorque (ROS 1).
 *
 *   tau = M(q) ddq_d + c(q, dq) + Kp (q_d - q) + Kv (dq_d - dq)   [+ g(q)]
 *
 * Gravity is compensated by the Flexiv robot itself (enable_gravity_comp in the hardware
 * interface), so g is added only when 'compensate_gravity' is true.
 */

#ifndef CUSTOM_CONTROLLERS__COMPUTED_TORQUE_CONTROLLER_HPP_
#define CUSTOM_CONTROLLERS__COMPUTED_TORQUE_CONTROLLER_HPP_

#include <memory>
#include <string>
#include <vector>

#include <Eigen/Dense>

#include "controller_interface/controller_interface.hpp"
#include "custom_controllers/computed_torque_controller_parameters.hpp"

#include "flexiv/rdk/robot.hpp"
#include <flexiv/rdk/model.hpp>
#include <flexiv/rdk/utility.hpp>

#include "realtime_tools/realtime_buffer.hpp"
#include "realtime_tools/realtime_publisher.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

namespace custom_controllers {

/** Number of arm joints. */
constexpr size_t kNumJoints = 7;

using Vector7d = Eigen::Matrix<double, kNumJoints, 1>;
using Matrix7d = Eigen::Matrix<double, kNumJoints, kNumJoints>;
using JointStateMsg = sensor_msgs::msg::JointState;

class ComputedTorqueController : public controller_interface::ControllerInterface
{
public:
    ComputedTorqueController();

    controller_interface::InterfaceConfiguration command_interface_configuration() const override;

    controller_interface::InterfaceConfiguration state_interface_configuration() const override;

    controller_interface::return_type update(
        const rclcpp::Time& time, const rclcpp::Duration& period) override;

    CallbackReturn on_init() override;

    CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;

    CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;

    CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

    CallbackReturn on_cleanup(const rclcpp_lifecycle::State& previous_state) override;

protected:
    /** Desired joint trajectory point. */
    struct Reference
    {
        Vector7d q;
        Vector7d dq;
        Vector7d ddq;
    };

    std::shared_ptr<ParamListener> param_listener_;
    Params params_;

    // Flexiv RDK robot model, built on the connection owned by the hardware interface
    std::unique_ptr<flexiv::rdk::Model> model_;
    // vectorizeed robot state to use with rdk
    std::vector<double> q_rdk_;
    std::vector<double> dq_rdk_;

    // gains and limits
    Vector7d kp_;
    Vector7d kv_;
    Vector7d tau_limit_;
    Vector7d dq_limit_;

    // state (std::vector copies for flexiv::rdk::Model::Update)
    Vector7d q_;
    Vector7d dq_;

    // reference
    Reference reference_;
    realtime_tools::RealtimeBuffer<std::shared_ptr<Reference>> reference_buffer_;

    // dynamics and command
    Matrix7d M_;
    Vector7d c_;
    Vector7d g_;
    Vector7d error_;
    Vector7d dot_error_;
    Vector7d tau_cmd_;
    Vector7d tau_prev_;

    void updateGains();
    Vector7d saturateTorqueRate(const Vector7d& tau_calculated, const Vector7d& tau_prev) const;

    // subscriber
    rclcpp::Subscription<JointStateMsg>::SharedPtr command_subscriber_;
    void commandCallback(const JointStateMsg::SharedPtr msg);

    // publishers
    using JointStatePublisher = realtime_tools::RealtimePublisher<JointStateMsg>;
    std::shared_ptr<rclcpp::Publisher<JointStateMsg>> error_publisher_;
    std::unique_ptr<JointStatePublisher> rt_error_publisher_;
    std::shared_ptr<rclcpp::Publisher<JointStateMsg>> state_publisher_;
    std::unique_ptr<JointStatePublisher> rt_state_publisher_;
};

} /* namespace custom_controllers */
#endif /* CUSTOM_CONTROLLERS__COMPUTED_TORQUE_CONTROLLER_HPP_ */
