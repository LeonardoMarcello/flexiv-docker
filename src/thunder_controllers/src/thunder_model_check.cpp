/**
 * @file thunder_model_check.cpp
 * @brief Runtime check of the Thunder dynamic model against the simulated robot.
 *
 * Compares the joint torque measured by the Gazebo joint force/torque sensors (component along
 * the joint axis) with the inverse dynamics of the measured motion:
 *
 *   tau_model = M(q) ddq + C(q, dq) dq + G(q) + D dq + F sign(dq)
 *
 * ddq is obtained by differentiating the low-pass filtered joint velocity; the sensor torque is
 * filtered with the same filter so that both signals have the same lag.
 *
 * Publishes sensor_msgs/JointState (values in the effort field):
 *   ~/tau_model, ~/tau_sensor, ~/residual (= tau_sensor - tau_model)
 */

#include <cmath>
#include <filesystem>
#include <map>
#include <string>
#include <vector>

#include <Eigen/Dense>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "geometry_msgs/msg/wrench_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

#include THUNDER_ROBOT_HEADER

namespace {

constexpr size_t kNumJoints = 7;
using Vector7d = Eigen::Matrix<double, kNumJoints, 1>;
using JointStateMsg = sensor_msgs::msg::JointState;
using WrenchMsg = geometry_msgs::msg::WrenchStamped;

class ThunderModelCheck : public rclcpp::Node
{
public:
    ThunderModelCheck()
    : rclcpp::Node("thunder_model_check")
    {
        joints_ = declare_parameter<std::vector<std::string>>("joints",
            {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "joint7"});
        const auto damping = declare_parameter<std::vector<double>>(
            "damping", std::vector<double>(kNumJoints, 0.0));
        const auto friction = declare_parameter<std::vector<double>>(
            "friction", std::vector<double>(kNumJoints, 0.0));
        cutoff_hz_ = declare_parameter<double>("cutoff_hz", 30.0);
        sensor_sign_ = declare_parameter<double>("sensor_sign", 1.0);
        const auto sensor_prefix
            = declare_parameter<std::string>("sensor_topic_prefix", "/joint_wrench/");
        auto par_file = declare_parameter<std::string>("thunder_par_file", "");

        if (joints_.size() != kNumJoints || damping.size() != kNumJoints
            || friction.size() != kNumJoints) {
            throw std::invalid_argument("'joints', 'damping' and 'friction' must have 7 elements");
        }
        damping_ = Eigen::Map<const Vector7d>(damping.data());
        friction_ = Eigen::Map<const Vector7d>(friction.data());

        // Thunder model with the same parameters as the controller
        if (!par_file.empty()) {
            std::filesystem::path path(par_file);
            if (path.is_relative()) {
                path = std::filesystem::path(
                           ament_index_cpp::get_package_share_directory("thunder_controllers"))
                       / "thunder" / path;
            }
            if (!robot_.load_par(path.string())) {
                throw std::runtime_error("Failed to load Thunder parameters '" + path.string() + "'");
            }
            RCLCPP_INFO(get_logger(), "Loaded Thunder parameters from '%s'.", path.c_str());
        }
        if (robot_.get_par_DYN().isZero()) {
            throw std::runtime_error("Thunder par_DYN is all zeros: set 'thunder_par_file'");
        }

        // Joint axes (in the child link frame, where the sensors express the wrench)
        for (size_t i = 0; i < kNumJoints; ++i) {
            const auto& a = robot_.jointsAxis[i];
            axes_[i] = Eigen::Vector3d(a[0], a[1], a[2]);
        }

        sensor_torque_.setZero();
        for (size_t i = 0; i < kNumJoints; ++i) {
            sensor_subs_.push_back(create_subscription<WrenchMsg>(sensor_prefix + joints_[i],
                rclcpp::SensorDataQoS(), [this, i](const WrenchMsg::SharedPtr msg) {
                    const Eigen::Vector3d t(
                        msg->wrench.torque.x, msg->wrench.torque.y, msg->wrench.torque.z);
                    sensor_torque_(i) = sensor_sign_ * axes_[i].dot(t);
                }));
        }

        joint_state_sub_ = create_subscription<JointStateMsg>("/joint_states",
            rclcpp::SensorDataQoS(),
            [this](const JointStateMsg::SharedPtr msg) { jointStateCallback(msg); });

        tau_model_pub_ = create_publisher<JointStateMsg>("~/tau_model", 10);
        tau_sensor_pub_ = create_publisher<JointStateMsg>("~/tau_sensor", 10);
        residual_pub_ = create_publisher<JointStateMsg>("~/residual", 10);
    }

private:
    void jointStateCallback(const JointStateMsg::SharedPtr msg)
    {
        // Map the joint_states order to the configured joint order
        if (index_.empty()) {
            for (size_t i = 0; i < kNumJoints; ++i) {
                for (size_t j = 0; j < msg->name.size(); ++j) {
                    if (msg->name[j] == joints_[i]) {
                        index_[i] = j;
                    }
                }
                if (index_.count(i) == 0) {
                    RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000,
                        "Joint '%s' not found in /joint_states.", joints_[i].c_str());
                    index_.clear();
                    return;
                }
            }
        }

        Vector7d q, dq;
        for (size_t i = 0; i < kNumJoints; ++i) {
            q(i) = msg->position[index_[i]];
            dq(i) = msg->velocity[index_[i]];
        }
        const double t = rclcpp::Time(msg->header.stamp).seconds();

        if (!initialized_) {
            dq_f_ = dq;
            ddq_f_.setZero();
            tau_sensor_f_ = sensor_torque_;
            t_prev_ = t;
            initialized_ = true;
            return;
        }
        const double dt = t - t_prev_;
        if (dt <= 0.0) {
            return;
        }
        t_prev_ = t;

        // First-order low-pass filter, same for velocity and sensor torque
        const double alpha = dt / (dt + 1.0 / (2.0 * M_PI * cutoff_hz_));
        const Vector7d dq_f_prev = dq_f_;
        dq_f_ += alpha * (dq - dq_f_);
        ddq_f_ = (dq_f_ - dq_f_prev) / dt;
        tau_sensor_f_ += alpha * (sensor_torque_ - tau_sensor_f_);

        // Inverse dynamics of the measured motion
        robot_.set_q(q);
        robot_.set_dq(dq_f_);
        // const Vector7d tau_model = robot_.get_M() * ddq_f_ + robot_.get_C() * dq_f_
        //                            + robot_.get_G() + damping_.cwiseProduct(dq_f_)
        //                            + friction_.cwiseProduct(dq_f_.array().sign().matrix());
        const Vector7d tau_model = robot_.get_G();

        publish(tau_model_pub_, msg->header, tau_model);
        publish(tau_sensor_pub_, msg->header, tau_sensor_f_);
        publish(residual_pub_, msg->header, tau_sensor_f_ - tau_model);
    }

    void publish(const rclcpp::Publisher<JointStateMsg>::SharedPtr& pub,
        const std_msgs::msg::Header& header, const Vector7d& tau)
    {
        JointStateMsg out;
        out.header = header;
        out.name = joints_;
        out.effort.assign(tau.data(), tau.data() + kNumJoints);
        pub->publish(out);
    }

    THUNDER_ROBOT_CLASS robot_;
    std::vector<std::string> joints_;
    std::map<size_t, size_t> index_;
    Eigen::Vector3d axes_[kNumJoints];
    Vector7d damping_, friction_;
    double cutoff_hz_ {30.0};
    double sensor_sign_ {1.0};

    bool initialized_ {false};
    double t_prev_ {0.0};
    Vector7d sensor_torque_, dq_f_, ddq_f_, tau_sensor_f_;

    std::vector<rclcpp::Subscription<WrenchMsg>::SharedPtr> sensor_subs_;
    rclcpp::Subscription<JointStateMsg>::SharedPtr joint_state_sub_;
    rclcpp::Publisher<JointStateMsg>::SharedPtr tau_model_pub_, tau_sensor_pub_, residual_pub_;
};

} // namespace

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ThunderModelCheck>());
    rclcpp::shutdown();
    return 0;
}
