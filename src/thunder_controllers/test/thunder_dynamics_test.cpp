/**
 * @file thunder_dynamics_test.cpp
 * @brief Evaluate the Thunder dynamic model at a given joint state and print the dynamic
 * matrices.
 *
 * Usage:
 *   ros2 run thunder_controllers thunder_dynamics_test \
 *       --q   q1 ... q7      [rad]      (default: 0)
 *       --dq  dq1 ... dq7    [rad/s]    (default: 0)
 *       --ddq ddq1 ... ddq7  [rad/s^2]  (default: 0)
 *       --par <file>         Thunder parameters file (default: installed <robot>_par.yaml)
 *
 * Prints M(q), C(q,dq), C(q,dq) dq, G(q), tau = M ddq + C dq + G, J_flange(q), T_w_flange(q).
 */

#include <cstdlib>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <string>

#include <Eigen/Dense>

#include "ament_index_cpp/get_package_share_directory.hpp"

#include THUNDER_ROBOT_HEADER

namespace {

constexpr int kNumJoints = 7;
using Vector7d = Eigen::Matrix<double, kNumJoints, 1>;

void usage(const char* prog)
{
    std::cout << "Usage: " << prog
              << " [--q q1..q7] [--dq dq1..dq7] [--ddq ddq1..ddq7] [--par <file>]\n"
              << "  Missing vectors default to zero. Units: rad, rad/s, rad/s^2.\n"
              << "  --par defaults to share/thunder_controllers/thunder/" << THUNDER_ROBOT_NAME
              << "_par.yaml\n";
}

/** Read 7 numbers following argv[i] into v. */
bool readVector(int argc, char** argv, int& i, Vector7d& v)
{
    if (i + kNumJoints >= argc) {
        std::cerr << "Error: " << argv[i] << " needs " << kNumJoints << " values\n";
        return false;
    }
    for (int j = 0; j < kNumJoints; ++j) {
        char* end = nullptr;
        v(j) = std::strtod(argv[i + 1 + j], &end);
        if (end == argv[i + 1 + j] || *end != '\0') {
            std::cerr << "Error: invalid number '" << argv[i + 1 + j] << "' for " << argv[i] << "\n";
            return false;
        }
    }
    i += kNumJoints;
    return true;
}

void print(const std::string& name, const Eigen::MatrixXd& m)
{
    const Eigen::IOFormat fmt(6, 0, "  ", "\n", "  [", "]");
    std::cout << name << " (" << m.rows() << "x" << m.cols() << "):\n"
              << m.format(fmt) << "\n\n";
}

} // namespace

int main(int argc, char** argv)
{
    Vector7d q = Vector7d::Zero(), dq = Vector7d::Zero(), ddq = Vector7d::Zero();
    std::string par_file;

    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        bool ok = true;
        if (arg == "--q") {
            ok = readVector(argc, argv, i, q);
        } else if (arg == "--dq") {
            ok = readVector(argc, argv, i, dq);
        } else if (arg == "--ddq") {
            ok = readVector(argc, argv, i, ddq);
        } else if (arg == "--par" && i + 1 < argc) {
            par_file = argv[++i];
        } else if (arg == "-h" || arg == "--help") {
            usage(argv[0]);
            return 0;
        } else {
            std::cerr << "Error: unknown argument '" << arg << "'\n";
            ok = false;
        }
        if (!ok) {
            usage(argv[0]);
            return 1;
        }
    }

    if (par_file.empty()) {
        par_file = (std::filesystem::path(
                        ament_index_cpp::get_package_share_directory("thunder_controllers"))
                    / "thunder" / (std::string(THUNDER_ROBOT_NAME) + "_par.yaml"))
                       .string();
    }

    THUNDER_ROBOT_CLASS robot;
    if (!robot.load_par(par_file)) {
        std::cerr << "Error: failed to load Thunder parameters '" << par_file << "'\n";
        return 1;
    }
    if (robot.get_par_DYN().isZero()) {
        std::cerr << "Warning: par_DYN is all zeros, dynamic terms will be zero\n";
    }

    robot.set_q(q);
    robot.set_dq(dq);

    const Eigen::Matrix<double, 7, 7> M = robot.get_M();
    const Eigen::Matrix<double, 7, 7> C = robot.get_C();
    const Vector7d G = robot.get_G();
    const Vector7d Cdq = C * dq;
    const Vector7d tau = M * ddq + Cdq + G;

    std::cout << std::fixed << std::setprecision(6);
    std::cout << "Thunder model: " << THUNDER_ROBOT_NAME << "\nParameters: " << par_file << "\n\n";
    print("q", q.transpose());
    print("dq", dq.transpose());
    print("ddq", ddq.transpose());
    print("M(q)", M);
    print("C(q,dq)", C);
    print("C(q,dq) dq", Cdq.transpose());
    print("G(q)", G.transpose());
    print("tau = M ddq + C dq + G", tau.transpose());
    print("J_flange(q)", robot.get_J_flange());
    print("T_w_flange(q)", robot.get_T_w_flange());

    return 0;
}
