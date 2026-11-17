/**
 * @file computed_torque_controller.cpp
 * @brief Joint-space computed torque controller as ROS 2 controller, using the Flexiv RDK
 * dynamic model (flexiv::rdk::Model). Ported from panda_controllers/ComputedTorque (ROS 1).
 */

#include "custom_controllers/computed_torque_controller.hpp"

#include <algorithm>
#include <string>

#include "hardware_interface/types/hardware_interface_type_values.hpp"

namespace custom_controllers {

ComputedTorqueController::ComputedTorqueController()
: controller_interface::ControllerInterface()
{
}

controller_interface::CallbackReturn ComputedTorqueController::on_init()
{
    try {
        param_listener_ = std::make_shared<ParamListener>(get_node());
        params_ = param_listener_->get_params();
    } catch (const std::exception& e) {
        fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
        return controller_interface::CallbackReturn::ERROR;
    }
    return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
ComputedTorqueController::command_interface_configuration() const
{
    controller_interface::InterfaceConfiguration config;
    config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

    for (const auto& joint : params_.joints) {
        config.names.emplace_back(joint + "/" + hardware_interface::HW_IF_EFFORT);
    }

    return config;
}

controller_interface::InterfaceConfiguration
ComputedTorqueController::state_interface_configuration() const
{
    controller_interface::InterfaceConfiguration config;
    config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

    // Order: [q_1, dq_1, q_2, dq_2, ...], see update()
    for (const auto& joint : params_.joints) {
        config.names.emplace_back(joint + "/" + hardware_interface::HW_IF_POSITION);
        config.names.emplace_back(joint + "/" + hardware_interface::HW_IF_VELOCITY);
        config.names.emplace_back(joint + "/" + hardware_interface::HW_IF_EFFORT);
    }

    return config;
}

controller_interface::CallbackReturn ComputedTorqueController::on_configure(
    const rclcpp_lifecycle::State& /*previous_state*/)
{
    params_ = param_listener_->get_params();

    if (params_.robot_sn.empty()) {
        RCLCPP_ERROR(get_node()->get_logger(), "'robot_sn' parameter has to be specified.");
        return CallbackReturn::ERROR;
    }
    if (params_.joints.size() != kNumJoints) {
        RCLCPP_ERROR(get_node()->get_logger(), "'joints' parameter must contain %zu joints, got %zu.",
            kNumJoints, params_.joints.size());
        return CallbackReturn::ERROR;
    }

    updateGains();
    tau_limit_ = Eigen::Map<const Vector7d>(params_.tau_limit.data());
    dq_limit_ = Eigen::Map<const Vector7d>(params_.dq_limit.data());

    // Build the RDK model on the connection owned by the Flexiv hardware interface
    flexiv::rdk::Robot* robot = new flexiv::rdk::Robot(params_.robot_sn);
    if (robot == nullptr) {
        RCLCPP_ERROR(get_node()->get_logger(),
            "No Flexiv robot '%s' registered by the hardware interface.", params_.robot_sn.c_str());
        return CallbackReturn::ERROR;
    }
    try {
        const Eigen::Vector3d gravity(params_.gravity[0], params_.gravity[1], params_.gravity[2]);
        model_ = std::make_unique<flexiv::rdk::Model>(*robot, gravity);
    } catch (const std::exception& e) {
        RCLCPP_ERROR(
            get_node()->get_logger(), "Failed to create the Flexiv RDK model: %s", e.what());
        return CallbackReturn::ERROR;
    }

    // Check that the RDK model matches the controlled arm (RDK v1.x has no Model::joint_names():
    // Update() throws if the size differs from the robot DoF, M() has the model size)
    q_rdk_.assign(kNumJoints, 0.0);
    dq_rdk_.assign(kNumJoints, 0.0);
    try {
        model_->Update(q_rdk_, dq_rdk_);
        const auto model_dof = model_->M().rows();
        if (model_dof != static_cast<Eigen::Index>(kNumJoints)) {
            RCLCPP_ERROR(get_node()->get_logger(),
                "Flexiv RDK model has %ld joints, the controller expects %zu.",
                static_cast<long>(model_dof), kNumJoints);
            model_.reset();
            return CallbackReturn::ERROR;
        }
    } catch (const std::exception& e) {
        RCLCPP_ERROR(get_node()->get_logger(),
            "Flexiv RDK model does not match %zu joints: %s", kNumJoints, e.what());
        model_.reset();
        return CallbackReturn::ERROR;
    }

    try {
        // register subscriber
        command_subscriber_ = get_node()->create_subscription<JointStateMsg>("~/command",
            rclcpp::SystemDefaultsQoS(),
            [this](const JointStateMsg::SharedPtr msg) { commandCallback(msg); });

        // register publishers
        error_publisher_ = get_node()->create_publisher<JointStateMsg>(
            "~/tracking_error", rclcpp::SystemDefaultsQoS());
        rt_error_publisher_ = std::make_unique<JointStatePublisher>(error_publisher_);

        state_publisher_ = get_node()->create_publisher<JointStateMsg>(
            "~/controller_state", rclcpp::SystemDefaultsQoS());
        rt_state_publisher_ = std::make_unique<JointStatePublisher>(state_publisher_);
    } catch (...) {
        return LifecycleNodeInterface::CallbackReturn::ERROR;
    }

    // Preallocate published messages
    for (auto* rt_pub : {rt_error_publisher_.get(), rt_state_publisher_.get()}) {
        rt_pub->lock();
        rt_pub->msg_.name = params_.joints;
        rt_pub->msg_.position.resize(kNumJoints, 0.0);
        rt_pub->msg_.velocity.resize(kNumJoints, 0.0);
        rt_pub->msg_.effort.resize(kNumJoints, 0.0);
        rt_pub->unlock();
    }

    return LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn ComputedTorqueController::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/)
{
    // Secure initialization: hold the current configuration
    for (size_t i = 0; i < kNumJoints; ++i) {
        q_(i) = state_interfaces_[3 * i].get_value();
        dq_(i) = state_interfaces_[3 * i + 1].get_value();
    }
    reference_.q = q_;
    reference_.dq.setZero();
    reference_.ddq.setZero();
    reference_buffer_.reset();

    // The hardware interface holds the joints with zero torque when switching to torque mode
    tau_prev_.setZero();
    tau_cmd_.setZero();

    return LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn ComputedTorqueController::on_deactivate(
    const rclcpp_lifecycle::State& /*previous_state*/)
{
    for (auto& command_interface : command_interfaces_) {
        command_interface.set_value(0.0);
    }
    return LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn ComputedTorqueController::on_cleanup(
    const rclcpp_lifecycle::State& /*previous_state*/)
{
    model_.reset();
    return LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

controller_interface::return_type ComputedTorqueController::update(
    const rclcpp::Time& time, const rclcpp::Duration& /*period*/)
{
    // Update gains if changed at runtime
    if (param_listener_->is_old(params_)) {
        params_ = param_listener_->get_params();
        updateGains();
    }

    /* Actual position and velocity of the joints */
    for (size_t i = 0; i < kNumJoints; ++i) {
        q_(i) = state_interfaces_[3 * i].get_value();
        dq_(i) = state_interfaces_[3 * i + 1].get_value();
        q_rdk_[i] = q_(i);
        dq_rdk_[i] = dq_(i);
    }
    /* Dynamic model from the Flexiv RDK */
    try {
        model_->Update(q_rdk_, dq_rdk_);
        M_ = model_->M();
        c_ = model_->c();
        if (params_.compensate_gravity) {
            g_ = model_->g();
        }
    } catch (const std::exception& e) {
        RCLCPP_ERROR_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 1000,
            "Flexiv RDK model update failed: %s", e.what());
        return controller_interface::return_type::ERROR;
    }

    // get reference
    /* Latest reference from the command topic */
    const auto new_reference = *reference_buffer_.readFromRT();
    if (new_reference) {
        // reference_.q:   joint position reference (position)
        // reference_.dq:  joint velocity reference (velocity)
        // reference_.ddq: joint acceleration reference (effort)
        reference_ = *new_reference;
    }
    /* Scale desired velocity to stay within limits */
    Vector7d dq_d = reference_.dq;
    const double vel_ratio = (dq_d.cwiseAbs().array() / dq_limit_.array()).maxCoeff();
    if (vel_ratio > 1.0) {
        dq_d /= vel_ratio;
    }
    reference_.dq = dq_d;



    /* -----------------------------------*
     * Computed Torque control law        *
     * ---------------------------------- */
    error_ = reference_.q - q_;
    dot_error_ = reference_.dq - dq_;

    tau_cmd_ = M_ * reference_.ddq + c_ + kp_.cwiseProduct(error_) + kv_.cwiseProduct(dot_error_);

    if (params_.compensate_gravity) {
        tau_cmd_ += g_;
    }
    /* ---------------------------------- */



    /* Limit torque rate and magnitude */
    tau_cmd_ = saturateTorqueRate(tau_cmd_, tau_prev_);
    tau_cmd_ = tau_cmd_.cwiseMax(-tau_limit_).cwiseMin(tau_limit_);
    tau_prev_ = tau_cmd_;

    /* Set the command for each joint */
    for (size_t i = 0; i < kNumJoints; ++i) {
        command_interfaces_[i].set_value(tau_cmd_(i));
    }

    /* Publish tracking errors and controller state as joint states */
    if (rt_error_publisher_->trylock()) {
        auto& msg = rt_error_publisher_->msg_;
        msg.header.stamp = time;
        for (size_t i = 0; i < kNumJoints; ++i) {
            msg.position[i] = error_(i);
            msg.velocity[i] = dot_error_(i);
        }
        rt_error_publisher_->unlockAndPublish();
    }
    if (rt_state_publisher_->trylock()) {
        auto& msg = rt_state_publisher_->msg_;
        msg.header.stamp = time;
        for (size_t i = 0; i < kNumJoints; ++i) {
            msg.position[i] = q_(i);
            msg.velocity[i] = dq_(i);
            msg.effort[i] = tau_cmd_(i);
        }
        rt_state_publisher_->unlockAndPublish();
    }

    return controller_interface::return_type::OK;
}

void ComputedTorqueController::updateGains()
{
    kp_ = Eigen::Map<const Vector7d>(params_.kp.data());
    kv_ = Eigen::Map<const Vector7d>(params_.kv.data());
}

/* Limit the torque change per control cycle */
Vector7d ComputedTorqueController::saturateTorqueRate(
    const Vector7d& tau_calculated, const Vector7d& tau_prev) const
{
    const double delta = params_.delta_tau_max;
    return tau_prev + (tau_calculated - tau_prev).cwiseMax(-delta).cwiseMin(delta);
}

void ComputedTorqueController::commandCallback(const JointStateMsg::SharedPtr msg)
{
    // position is mandatory; velocity and effort (used as desired acceleration) default to zero
    if (msg->position.size() != kNumJoints) {
        RCLCPP_ERROR_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 1000,
            "Desired position must have dimension %zu, got %zu. Command ignored.", kNumJoints,
            msg->position.size());
        return;
    }
    if (!msg->velocity.empty() && msg->velocity.size() != kNumJoints) {
        RCLCPP_ERROR_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 1000,
            "Desired velocity must be empty or have dimension %zu. Command ignored.", kNumJoints);
        return;
    }
    // TODO: Here we assign acceleration to effort (use trajectory_msgs::JointTrajectoryPoint)
    if (!msg->effort.empty() && msg->effort.size() != kNumJoints) {
        RCLCPP_ERROR_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 1000,
            "Desired acceleration (effort) must be empty or have dimension %zu. Command ignored.",
            kNumJoints);
        return;
    }

    auto reference = std::make_shared<Reference>();
    reference->q = Eigen::Map<const Vector7d>(msg->position.data());
    reference->dq = msg->velocity.empty() ? Vector7d::Zero()
                                          : Vector7d(Eigen::Map<const Vector7d>(msg->velocity.data()));
    reference->ddq = msg->effort.empty() ? Vector7d::Zero()
                                         : Vector7d(Eigen::Map<const Vector7d>(msg->effort.data()));
    reference_buffer_.writeFromNonRT(reference);
}

} // namespace custom_controllers

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
    custom_controllers::ComputedTorqueController, controller_interface::ControllerInterface)
