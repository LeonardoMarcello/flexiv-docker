#!/bin/bash
set -e

source /opt/ros/humble/setup.bash
cd "${WORKSPACE:-/home/flexiv/flexiv_ros2_ws}"

# Installa eventuali nuove dipendenze aggiunte ai package.xml montati dall'host
sudo apt-get update
rosdep update
rosdep install --from-paths src --ignore-src --rosdistro humble -r -y || true

# Ricompila il workspace sui sorgenti montati dall'host
colcon build --symlink-install \
    --cmake-args -DCMAKE_PREFIX_PATH="${RDK_INSTALL:-/home/flexiv/rdk_install}" -DCMAKE_EXPORT_COMPILE_COMMANDS=ON

echo "Post-create completato."
