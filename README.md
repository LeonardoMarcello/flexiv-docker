# flexiv-docker

Docker development environment for **Flexiv Rizon** arms with **ROS 2 Humble**, based on
[flexiv_ros2](https://github.com/flexivrobotics/flexiv_ros2) **humble-v1.7** (Flexiv RDK **1.7.0**),
plus custom controllers (Thunder computed torque) and a Gazebo simulation.

> **Why humble-v1.7?** The RDK version must match the robot software. Our Rizon4s runs
> Flexiv software v3.9.3, which needs RDK 1.7: RDK v2.x reports the Rizon4s as not supported.

## Contents

```
flexiv-docker/
├── install.sh            # downloads flexiv_ros2 humble-v1.7 + RDK 1.7 and builds the image
├── run.sh                # starts the container and opens a shell in it
├── Dockerfile            # ROS 2 Humble + Gazebo + RDK 1.7 + workspace build
├── docker-compose.yaml   # container 'flexiv', host network, realtime limits, X11, GPU
├── .devcontainer/        # VS Code Dev Container configuration
├── docker/               # bashrc, inputrc, persistent bash history (docker/history)
└── flexiv_ros2/          # sources, mounted into the container (see below)
    ├── flexiv_hardware/rdk/                    # Flexiv RDK 1.7 (git submodule)
    ├── flexiv_controllers/thunder_controllers/ # Thunder computed torque + model tools
    ├── flexiv_controllers/custom_controllers/  # computed torque with the RDK model
    └── flexiv_gazebo/                          # Gazebo Classic simulation
```

`thunder_controllers`, `custom_controllers`, `flexiv_gazebo` and
`flexiv_description/urdf/Rizon4s.urdf` are our own files: they are **not** part of the upstream
`flexiv_ros2` repository (untracked in git).

## Requirements

- Linux host with Docker and the Docker Compose v2 plugin (`docker compose version`)
- git
- X11 for the GUIs (RViz, Gazebo, Terminator)
- For the real robot: Ethernet connection to the robot (e.g. host `192.168.2.10/24`, robot
  `192.168.2.100`)

## Install

```bash
./install.sh               # get the sources on humble-v1.7 + RDK 1.7, then build the image
./install.sh --no-cache    # same, rebuilding the image from scratch
./install.sh --skip-build  # only prepare the sources
```

What `install.sh` does:

1. If `flexiv_ros2/` does not exist, clones `flexiv_ros2` at tag `humble-v1.7`. If it exists,
   switches it to the `humble-v1.7` branch, leaving untracked files untouched. It stops if
   tracked files have uncommitted changes. An untracked local `flexiv_description` (which would
   block the switch) is moved to `flexiv_description_local_<date>`.
2. Initializes the RDK submodule (`flexiv_hardware/rdk`) and checks it is RDK 1.7.0.
3. Warns if `flexiv_description/urdf/Rizon4s.urdf` (needed by the simulation) is missing.
4. Builds the image (`docker compose build`, image `flexiv:1.7.1`).

The first build takes a while (RDK third-party dependencies, the ~63 MB Thunder generated model).

## Run

```bash
./run.sh
```

Allows X11 access, starts the `flexiv` container in the background (if not already running) and
opens a bash shell in it. Run `./run.sh` again in other terminals to open more shells in the same
container.

Alternatively, open the folder in VS Code and use **Dev Containers: Reopen in Container**.

### Inside the container

| Path / variable | Content |
|---|---|
| `~/flexiv_ros2_ws` | colcon workspace (built in the image, ROS sourced in `.bashrc`) |
| `~/flexiv_ros2_ws/src/flexiv_ros2` | **mounted from the host** `./flexiv_ros2`: edits are shared live |
| `$RDK_INSTALL` = `~/rdk_install` | installed RDK 1.7 and its dependencies |
| `$RDK_SRC` | RDK sources (`src/flexiv_ros2/flexiv_hardware/rdk`) |
| `~/.history` | bash history, persisted on the host in `docker/history` |

After editing the sources, rebuild only what changed:

```bash
cd ~/flexiv_ros2_ws
colcon build --symlink-install --packages-select <package> --cmake-args -DCMAKE_PREFIX_PATH=$RDK_INSTALL
source install/setup.bash
```

> Anything built **inside** `src/flexiv_ros2` during the image build is hidden at runtime by the
> host mount. For example the RDK examples: build them in the container outside the mounted folder:
>
> ```bash
> cmake -S $RDK_SRC/example -B ~/rdk_examples -DCMAKE_PREFIX_PATH=$RDK_INSTALL
> cmake --build ~/rdk_examples -j 4
> ```

## Real robot

1. On the robot (Flexiv Elements): **Remote mode over Ethernet**, operation mode **Automatic**,
   no faults or E-stop active.
2. Check the connection with an RDK example:

   ```bash
   cd ~/rdk_examples
   sudo LD_LIBRARY_PATH=$RDK_INSTALL/lib ./basics1_display_robot_states Rizon4s-062951
   ```

3. Start the ROS 2 driver:

   ```bash
   ros2 launch flexiv_bringup rizon.launch.py rizon_type:=Rizon4s robot_sn:=Rizon4s-062951
   ros2 launch flexiv_bringup rizon.launch.py --show-args   # all options
   ```

On the real robot gravity is compensated by the robot itself: keep `compensate_gravity: false` in
the controllers. See [thunder_controllers/README.md](flexiv_ros2/flexiv_controllers/thunder_controllers/README.md)
for the controller parameters and how to send references.

## Simulation (Gazebo)

```bash
ros2 launch flexiv_gazebo flexiv_gazebo.launch.py                 # runs on its own
ros2 launch flexiv_gazebo flexiv_gazebo.launch.py paused:=true    # pause after activation, step with ⏭ or 'gz world -s'
```

- The robot model is `flexiv_description/urdf/Rizon4s.urdf` (same URDF used to generate the
  Thunder model), the initial pose comes from `flexiv_gazebo/config/Rizon4s/initial_positions.yaml`.
- The Thunder computed torque controller runs on the effort interfaces at 1 kHz, with
  `compensate_gravity: true` (Gazebo does not compensate gravity).
- `thunder_model_check` compares the Thunder inverse dynamics with the simulated joint torques;
  plot all joints with `ros2 run flexiv_gazebo plot_model_check.py`.

## Troubleshooting

| Symptom | Cause / fix |
|---|---|
| `Searching for [Rizon4s-...]` then `Failed to connect` | RDK discovery gets no answer: check Remote mode + Automatic on the robot, RDK version vs robot software, firewall (`sudo ufw status`), turn Wi-Fi off, no other RDK program connected |
| `gzserver ... exit code 255`, `Entity [flexiv] already exists`, `Controller already loaded` | a Gazebo server from a previous run is still alive: `pkill -9 gzserver; pkill -9 gzclient` (or `docker restart flexiv`) |
| `Skipping joint ... not in the gazebo model`, `Not existing: jointN/effort` | the URDF links have no `<inertial>` block: Gazebo drops massless links and their joints |
| `no ros2_control tag` from `ros2_control_node` | the `flexiv_description` in use has no ros2_control part (e.g. a different description copy) |
| `Failed to connect` after switching flexiv_ros2 branch | rebuild the image: the RDK version comes from the branch (`./install.sh`) |
| Controller chatters in Gazebo | gains too high for the light wrist joints (no motor inertia in simulation); see the controller README |

## Switching flexiv_ros2 version

Our own packages are untracked, so they survive branch switches. `install.sh` makes a safe switch
to humble-v1.7; for other versions keep in mind:

- the RDK version comes from the branch (submodule up to v1.8, `flexiv.humble.repos` from v1.9)
  and must match the robot software;
- from v1.9 `flexiv_description` is a separate repository and the Dockerfile must use
  `vcs import` again;
- back up first, e.g.
  `tar czf ../my_work.tar.gz flexiv_controllers/custom_controllers flexiv_controllers/thunder_controllers flexiv_gazebo`.
