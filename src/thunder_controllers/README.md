# thunder_controllers

ros2_control controllers for Flexiv arms based on a dynamic model generated with Thunder
(`thunder gen`: M, C, G, Jacobians), plus tools to inspect and validate the model.

| Component | Type | Purpose |
|---|---|---|
| `ThunderComputedTorqueController` | ros2_control controller (plugin) | Joint-space computed torque on the effort interface |
| `thunder_model_check` | ROS 2 node | Runtime comparison of the model with the Gazebo joint torque sensors |
| `thunder_dynamics_test` | command-line program | Evaluate the model at a given (q, dq, ddq) and print the dynamic matrices |

## Thunder model

The generated model lives in `thunder/<robot>_generatedFiles/` (default robot: `Rizon4s`):

```
thunder/Rizon4s_generatedFiles/
    thunder_Rizon4s.h / .cpp     # wrapper class (set_q, set_dq, get_M, get_C, get_G, ...)
    Rizon4s_gen.h / .cpp         # CasADi-generated code (~63 MB, compiled with -O1)
    Rizon4s_par.yaml             # parameters, installed to share/thunder_controllers/thunder
```

- If the files are missing, the package still builds but skips everything (CMake warning).
- For another robot: `colcon build --cmake-args -DTHUNDER_ROBOT=<robot>`.
- `par_DYN` in `<robot>_par.yaml` holds 10 values per body, in Thunder body order
  (`base_link`, `link1` ... `link7`, `flange`):
  `[mass, com_x, com_y, com_z, Ixx, Ixy, Ixz, Iyy, Iyz, Izz]`, with the CoM in the link frame and
  the inertia about the CoM (same convention as the URDF `<inertial>`). With all-zero `par_DYN`
  the controller refuses to configure.

See also [thunder/README.md](thunder/README.md).

## Build

```bash
cd ~/flexiv_ros2_ws
colcon build --symlink-install --packages-select thunder_controllers
source install/setup.bash
```

The first build compiles the generated model and takes a while; later builds only recompile the
controller and the tools.

## ThunderComputedTorqueController

Plugin: `thunder_controllers/ThunderComputedTorqueController`

Control law (current implementation in `src/thunder_computed_torque_controller.cpp`):

```
tau = M(q) ddq_d + C(q, dq) dq + Kp (q_d - q) + Kv (0 - dq)   [+ G(q) if compensate_gravity]
```

- q and dq go through an optional first-order low-pass filter before being used.
- The desired velocity is currently ignored in the damping term (`dot_error = -dq`), and the
  torque rate/magnitude saturation is commented out.
- On the real Flexiv robot gravity is compensated by the robot itself: keep
  `compensate_gravity: false`. In Gazebo set it to `true`.

### Interfaces

- Command: `<joint>/effort`
- State: `<joint>/position`, `<joint>/velocity`, `<joint>/effort`

### Topics

| Topic | Type | Content |
|---|---|---|
| `~/command` (sub) | `sensor_msgs/JointState` | `position` = q_d (required), `velocity` = dq_d, `effort` = ddq_d (optional, default 0) |
| `~/tracking_error` (pub) | `sensor_msgs/JointState` | `position` = q_d - q, `velocity` = dot error |
| `~/controller_state` (pub) | `sensor_msgs/JointState` | filtered q, dq and commanded torque (`effort`) |

### Parameters

| Parameter | Default | Runtime | Description |
|---|---|---|---|
| `joints` | `[]` | no | 7 joint names, e.g. `<robot_sn>_joint1` ... |
| `kp` | `[300, 300, 300, 300, 150, 150, 30]` | yes | Proportional gains [Nm/rad] |
| `kv` | `[30, 30, 30, 30, 15, 15, 3]` | yes | Derivative gains [Nm s/rad] (the name is `kv`, not `kd`) |
| `tau_limit` | Rizon4s limits | no | Torque limits [Nm] |
| `dq_limit` | Rizon4s limits | no | Velocity limits [rad/s] |
| `delta_tau_max` | `1.0` | yes | Max torque change per cycle [Nm] |
| `q_filter_cutoff_hz` | `0.0` | yes | Low-pass cutoff on q, 0 = off |
| `dq_filter_cutoff_hz` | `0.0` | yes | Low-pass cutoff on dq, 0 = off |
| `compensate_gravity` | `false` | yes | Add G(q) to the command |
| `thunder_par_file` | `""` | no | Thunder parameters file; relative paths are resolved in `share/thunder_controllers/thunder` |

Example configuration: [config/thunder_computed_torque_controller.yaml](config/thunder_computed_torque_controller.yaml).

Change gains at runtime:

```bash
ros2 param set /thunder_computed_torque_controller kp "[100.0, 100.0, 100.0, 100.0, 30.0, 30.0, 10.0]"
ros2 param set /thunder_computed_torque_controller kv "[10.0, 10.0, 10.0, 10.0, 3.0, 3.0, 0.3]"
```

> In simulation the URDF has no motor (rotor) inertia, so the wrist joints are very light
> (joint 7: about 0.0018 kg m^2). High damping gains such as `kv[6] = 3` at 1 kHz are close to
> the discrete stability limit (kv * dt / I < 2) and make the arm chatter. Use smaller gains on
> the light joints.

### Sending a reference

```bash
# fixed target
ros2 topic pub -w 1 -1 /thunder_computed_torque_controller/command sensor_msgs/msg/JointState \
  "{position: [0.0, -0.69813, 0.0, 1.570796, 0.0, 0.69813, 0.0]}"

# random target within +/-0.3 rad of the Rizon4s initial pose
ros2 topic pub -w 1 -1 /thunder_computed_torque_controller/command sensor_msgs/msg/JointState \
  "{position: $(python3 -c "import random; q0=[0,-0.69813,0,1.570796,0,0.69813,0]; print([round(q+random.uniform(-0.3,0.3),3) for q in q0])")}"
```

`-w 1` waits for the controller subscription; without it the message can be lost.

## thunder_dynamics_test

Evaluates the Thunder model at a joint state given on the command line and prints:
`M(q)`, `C(q,dq)`, `C(q,dq) dq`, `G(q)`, `tau = M ddq + C dq + G`, `J_flange(q)`, `T_w_flange(q)`.
It does not need ROS running.

```bash
ros2 run thunder_controllers thunder_dynamics_test \
    --q   q1 ... q7       # [rad]       default 0
    --dq  dq1 ... dq7     # [rad/s]     default 0
    --ddq ddq1 ... ddq7   # [rad/s^2]   default 0
    --par <file>          # default: share/thunder_controllers/thunder/Rizon4s_par.yaml
```

Examples:

```bash
# everything zero
ros2 run thunder_controllers thunder_dynamics_test

# gravity torque at the Rizon4s initial pose
ros2 run thunder_controllers thunder_dynamics_test --q 0 -0.69813 0 1.570796 0 0.69813 0

# full inverse dynamics with velocity and acceleration
ros2 run thunder_controllers thunder_dynamics_test \
    --q   0 -0.69813 0 1.570796 0 0.69813 0 \
    --dq  0.1 0 0 0 0 0 0 \
    --ddq 0 0.5 0 0 0 0 0

# with another parameters file
ros2 run thunder_controllers thunder_dynamics_test --q 0.3 -0.5 0.2 1.4 -0.3 0.7 0.5 \
    --par /path/to/Rizon4s_par.yaml
```

Sanity check: at the initial pose (only `--q` above) with the URDF inertial parameters, `G(q)` is
`[0, 50.990, 0.798, -22.109, -2.608, 3.093, 0]` Nm.

- Missing vectors default to zero; each vector needs exactly 7 numbers.
- A warning is printed if `par_DYN` is all zeros (all dynamic terms would be zero).

## thunder_model_check

Runtime check of the model against the simulated robot (started by
`flexiv_gazebo/launch/flexiv_gazebo.launch.py`, disable with `model_check:=false`). It computes the
inverse dynamics of the measured motion and compares it with the Gazebo joint torque sensors:

```
tau_model = M(q) ddq + C(q, dq) dq + G(q) + D dq + F sign(dq)
```

ddq is the derivative of the low-pass filtered dq; the sensor torque is filtered with the same
filter.

| Topic | Content (`effort` field of `sensor_msgs/JointState`) |
|---|---|
| `~/tau_model` | model torque |
| `~/tau_sensor` | sensor torque along the joint axis |
| `~/residual` | `tau_sensor - tau_model` |

| Parameter | Default | Description |
|---|---|---|
| `joints` | `joint1` ... `joint7` | joint names in `/joint_states` |
| `damping`, `friction` | `0` | joint damping / Coulomb friction, match the URDF `<dynamics>` |
| `cutoff_hz` | `30.0` | filter cutoff |
| `sensor_sign` | `1.0` | set `-1.0` if the sensor reports the opposite convention |
| `sensor_topic_prefix` | `/joint_wrench/` | sensor topics `<prefix><joint>` |
| `thunder_par_file` | `""` | Thunder parameters file (required) |

Live plot of all joints (from `flexiv_gazebo`):

```bash
ros2 run flexiv_gazebo plot_model_check.py
```

> The axis component of a Gazebo joint wrench is essentially the applied (commanded) torque:
> a revolute joint transmits no constraint torque about its own axis. Comparing the model with
> the commanded torque `~/controller_state` effort gives the same information.
