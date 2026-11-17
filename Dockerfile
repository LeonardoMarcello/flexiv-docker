# Usa l'immagine ufficiale di ROS 2 Humble basata su Ubuntu 22.04
FROM osrf/ros:humble-desktop

# Evita richieste interattive durante l'installazione dei pacchetti
ENV DEBIAN_FRONTEND=noninteractive

# Installa le dipendenze di sistema, sudo e gli strumenti di sviluppo necessari
RUN apt-get update && apt-get install -y \
    software-properties-common \
    wget \
    git \
    sudo \
    terminator \
    build-essential \
    python3-pip \
    python3-colcon-common-extensions \
    libeigen3-dev \
    ros-humble-xacro \
    ros-humble-tinyxml2-vendor \
    ros-humble-ros2-control \
    ros-humble-realtime-tools \
    ros-humble-control-toolbox \
    ros-humble-moveit \
    ros-humble-ros2-controllers \
    ros-humble-test-msgs \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-gazebo-ros2-control \
    ros-humble-joint-state-publisher \
    ros-humble-joint-state-publisher-gui \
    ros-humble-robot-state-publisher \
    ros-humble-rviz2 \
    python3-rosdep \
    bash-completion \
    python3-argcomplete \
    dbus-x11 \
    libcanberra-gtk3-module \
    && rm -rf /var/lib/apt/lists/*

# Crea un utente non-root chiamato 'flexiv' (UID/GID 1000 per compatibilità con l'host)
ARG USER_UID=1000
ARG USER_GID=1000
RUN groupadd --gid ${USER_GID} flexiv && \
    useradd --uid ${USER_UID} --gid ${USER_GID} -m -s /bin/bash flexiv && \
    usermod -aG sudo flexiv && \
    echo "flexiv ALL=(ALL) NOPASSWD:ALL" >> /etc/sudoers

# Inizializza rosdep (lo facciamo come root prima di cambiare utente)
RUN rosdep init || true && rosdep update

# Imposta variabili d'ambiente per il workspace e l'utente
ENV WORKSPACE=/home/flexiv/flexiv_ros2_ws
WORKDIR ${WORKSPACE}

# Simulazione: Gazebo Classic + ros2_control (pacchetto flexiv_gazebo)
RUN apt-get update && apt-get install -y \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-gazebo-ros2-control \
    && rm -rf /var/lib/apt/lists/*

# Copia i sorgenti assegnandoli all'utente flexiv
RUN chown -R flexiv:flexiv ${WORKSPACE}
COPY --chown=flexiv:flexiv flexiv_ros2/ ./src/flexiv_ros2/
COPY --chown=flexiv:flexiv src/ ./src/flexiv_packages/

# Passa all'utente 'flexiv' per le operazioni successive (installazione dipendenze e compilazione)
USER flexiv

# Installa le dipendenze del workspace tramite rosdep (specificando --rosdistro humble)
# humble-v1.7: RDK e flexiv_description sono dentro flexiv_ros2 (submodule + pacchetto).
# Il submodule va inizializzato sull'host prima della build: git submodule update --init --recursive
RUN rosdep update && \
    rosdep install --from-paths src --ignore-src --rosdistro humble -r -y || true

# Compila flexiv_rdk (submodule flexiv_hardware/rdk) e le sue dipendenze terze
# installandole nella home dell'utente (/home/flexiv/rdk_install)
ENV RDK_INSTALL=/home/flexiv/rdk_install
ENV RDK_SRC=/home/flexiv/flexiv_ros2_ws/src/flexiv_ros2/flexiv_hardware/rdk
RUN test -f ${RDK_SRC}/thirdparty/build_and_install_dependencies.sh || \
    (echo "RDK submodule mancante: esegui 'git submodule update --init --recursive' in flexiv_ros2" && exit 1)
RUN cd ${RDK_SRC}/thirdparty && \
    bash build_and_install_dependencies.sh ${RDK_INSTALL}
# install rdk
RUN cd ${RDK_SRC} && \
    rm -rf build && mkdir build && cd build && \
    cmake .. -DCMAKE_INSTALL_PREFIX=${RDK_INSTALL} && \
    make install
# install examples (not they are located outside rdk)
RUN cmake -S ${RDK_SRC}/example -B /home/flexiv/rdk_examples -DCMAKE_PREFIX_PATH=${RDK_INSTALL} && \
    cmake --build /home/flexiv/rdk_examples --config Release -j 4

# Compila il workspace ROS 2
SHELL ["/bin/bash", "-c"]
RUN source /opt/ros/humble/setup.bash && \
    colcon build --symlink-install --cmake-args -DCMAKE_PREFIX_PATH=${RDK_INSTALL}

# Configura l'ambiente bash dell'utente flexiv
# (ROS, autocompletamento, cronologia persistente, ricerca con le frecce)
COPY --chown=flexiv:flexiv docker/bashrc /home/flexiv/.bashrc_flexiv
COPY --chown=flexiv:flexiv docker/inputrc /home/flexiv/.inputrc
RUN echo "source ~/.bashrc_flexiv" >> ~/.bashrc

CMD ["bash"]
