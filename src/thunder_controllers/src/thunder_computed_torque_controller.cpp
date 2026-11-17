/**
 * @file thunder_computed_torque_controller.cpp
 * @brief Joint-space computed torque controller as ROS 2 controller, using a Thunder-generated
 * dynamic model. Ported from panda_controllers/ComputedTorque (ROS 1).
 */

#include "thunder_controllers/thunder_computed_torque_controller.hpp"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <string>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"

namespace thunder_controllers {

ThunderComputedTorqueController::ThunderComputedTorqueController()
: controller_interface::ControllerInterface()
{
}

controller_interface::CallbackReturn ThunderComputedTorqueController::on_init()
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
ThunderComputedTorqueController::command_interface_configuration() const
{
    controller_interface::InterfaceConfiguration config;
    config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

    for (const auto& joint : params_.joints) {
        config.names.emplace_back(joint + "/" + hardware_interface::HW_IF_EFFORT);
    }

    return config;
}

controller_interface::InterfaceConfiguration
ThunderComputedTorqueController::state_interface_configuration() const
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

controller_interface::CallbackReturn ThunderComputedTorqueController::on_configure(
    const rclcpp_lifecycle::State& /*previous_state*/)
{
    params_ = param_listener_->get_params();

    if (params_.joints.size() != kNumJoints) {
        RCLCPP_ERROR(get_node()->get_logger(), "'joints' parameter must contain %zu joints, got %zu.",
            kNumJoints, params_.joints.size());
        return CallbackReturn::ERROR;
    }

    updateGains();
    tau_limit_ = Eigen::Map<const Vector7d>(params_.tau_limit.data());
    dq_limit_ = Eigen::Map<const Vector7d>(params_.dq_limit.data());

    if (!loadThunderParameters()) {
        return CallbackReturn::ERROR;
    }

    // Check that the Thunder model matches the controlled arm
    if (robot_.ndof != static_cast<int>(kNumJoints)) {
        RCLCPP_ERROR(get_node()->get_logger(),
            "Thunder model '%s' has %d DOF, the controller expects %zu.", THUNDER_ROBOT_NAME,
            robot_.ndof, kNumJoints);
        return CallbackReturn::ERROR;
    }

    // Without dynamic parameters M and C are zero and the controller degenerates into a PD
    if (robot_.get_par_DYN().isZero()) {
        RCLCPP_ERROR(get_node()->get_logger(),
            "Thunder model '%s' has all-zero dynamic parameters (par_DYN). Set 'thunder_par_file' "
            "to a parameters file with the robot inertial parameters.",
            THUNDER_ROBOT_NAME);
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

controller_interface::CallbackReturn ThunderComputedTorqueController::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/)
{
    // Secure initialization: hold the current configuration, filters start from the raw state
    readState();
    q_ = q_raw_;
    dq_ = dq_raw_;
    reference_.q = q_;
    reference_.dq.setZero();
    reference_.ddq.setZero();
    reference_buffer_.reset();

    // The hardware interface holds the joints with zero torque when switching to torque mode
    tau_prev_.setZero();
    tau_cmd_.setZero();

    return LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn ThunderComputedTorqueController::on_deactivate(
    const rclcpp_lifecycle::State& /*previous_state*/)
{
    for (auto& command_interface : command_interfaces_) {
        command_interface.set_value(0.0);
    }
    return LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

controller_interface::return_type ThunderComputedTorqueController::update(
    const rclcpp::Time& time, const rclcpp::Duration& period)
{
    // Update gains if changed at runtime
    if (param_listener_->is_old(params_)) {
        params_ = param_listener_->get_params();
        updateGains();
    }

    /* Actual position and velocity of the joints, low-pass filtered */
    readState();
    const double dt = period.seconds();
    q_ = q_ * 0.7 + 0.3 * q_raw_;
    dq_ = dq_ * 0.1 + 0.9 * dq_raw_;
    // q_ += lowPassAlpha(params_.q_filter_cutoff_hz, dt) * (q_raw_ - q_);
    // dq_ += lowPassAlpha(params_.dq_filter_cutoff_hz, dt) * (dq_raw_ - dq_);

    /* Latest reference from the command topic */
    const auto new_reference = *reference_buffer_.readFromRT();
    if (new_reference) {
        reference_ = *new_reference;
    }

    /* Scale desired velocity to stay within limits */
    // Vector7d dq_d = reference_.dq;
    // const double vel_ratio = (dq_d.cwiseAbs().array() / dq_limit_.array()).maxCoeff();
    // if (vel_ratio > 1.0) {
    //     dq_d /= vel_ratio;
    // }
    // reference_.dq = dq_d;

    /* Dynamic model from Thunder */
    robot_.set_q(q_);
    robot_.set_dq(dq_);
    M_ = robot_.get_M();
    C_ = robot_.get_C();
    Jee_ = robot_.get_J_flange();



    /* -----------------------------------*
     * Computed Torque control law        *
     * ---------------------------------- */
    error_ = reference_.q - q_;
    dot_error_ = - dq_;//reference_.dq - dq_;

    tau_cmd_ = M_ * reference_.ddq + C_ * dq_ + kp_.cwiseProduct(error_)
               + kv_.cwiseProduct(dot_error_);

    if (params_.compensate_gravity) {
        G_ = robot_.get_G();
        tau_cmd_ += G_;
    }

    //tau_cmd_ = G_; // comment out
    /* ---------------------------------- */


    /* Limit torque rate and magnitude */
    //tau_cmd_ = saturateTorqueRate(tau_cmd_, tau_prev_);
    //tau_cmd_ = tau_cmd_.cwiseMax(-tau_limit_).cwiseMin(tau_limit_);
    tau_prev_ = tau_cmd_;

    /* Set the command for each joint */
    for (size_t i = 0; i < kNumJoints; ++i) {
        //command_interfaces_[i].set_value(0.0);
        command_interfaces_[i].set_value(tau_cmd_(i));//<- uncomment
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

void ThunderComputedTorqueController::readState()
{
    for (size_t i = 0; i < kNumJoints; ++i) {
        q_raw_(i) = state_interfaces_[3 * i].get_value();
        dq_raw_(i) = state_interfaces_[3 * i + 1].get_value();
    }
}

/* First-order low-pass filter gain: y += alpha * (x - y). cutoff <= 0 disables the filter */
double ThunderComputedTorqueController::lowPassAlpha(double cutoff_hz, double dt)
{
    if (cutoff_hz <= 0.0 || dt <= 0.0) {
        return 1.0;
    }
    return dt / (dt + 1.0 / (2.0 * M_PI * cutoff_hz));
}

void ThunderComputedTorqueController::updateGains()
{
    kp_ = Eigen::Map<const Vector7d>(params_.kp.data());
    kv_ = Eigen::Map<const Vector7d>(params_.kv.data());
}

bool ThunderComputedTorqueController::loadThunderParameters()
{
    if (params_.thunder_par_file.empty()) {
        RCLCPP_INFO(get_node()->get_logger(),
            "Using the dynamic parameters built into the Thunder model '%s'.", THUNDER_ROBOT_NAME);
        return true;
    }

    std::filesystem::path par_file(params_.thunder_par_file);
    if (par_file.is_relative()) {
        par_file = std::filesystem::path(
                       ament_index_cpp::get_package_share_directory("thunder_controllers"))
                   / "thunder" / par_file;
    }
    if (!std::filesystem::exists(par_file)) {
        RCLCPP_ERROR(get_node()->get_logger(), "Thunder parameters file '%s' not found.",
            par_file.c_str());
        return false;
    }

    if (!robot_.load_par(par_file.string())) {
        RCLCPP_ERROR(get_node()->get_logger(), "Failed to load Thunder parameters '%s'.",
            par_file.c_str());
        return false;
    }
    RCLCPP_INFO(get_node()->get_logger(), "Loaded Thunder parameters from '%s'.", par_file.c_str());
    
    
    auto par_dyn = robot_.get_par_DYN(); // Può essere double* o std::vector<double>/std::array
    std::string par_dyn_str = "[ ";
    for (size_t i = 0; i < 70; ++i) {
    par_dyn_str += std::to_string(par_dyn[i]);
    if (i < 69) par_dyn_str += ", ";
    }
    par_dyn_str += " ]";
    RCLCPP_INFO(get_node()->get_logger(), "Loaded par_dyn:\n%s", par_dyn_str.c_str());

    return true;
}

/* Limit the torque change per control cycle */
Vector7d ThunderComputedTorqueController::saturateTorqueRate(
    const Vector7d& tau_calculated, const Vector7d& tau_prev) const
{
    const double delta = params_.delta_tau_max;
    return tau_prev + (tau_calculated - tau_prev).cwiseMax(-delta).cwiseMin(delta);
}

void ThunderComputedTorqueController::commandCallback(const JointStateMsg::SharedPtr msg)
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

} // namespace thunder_controllers

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
    thunder_controllers::ThunderComputedTorqueController, controller_interface::ControllerInterface)
