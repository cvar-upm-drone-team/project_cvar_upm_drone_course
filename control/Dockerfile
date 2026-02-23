FROM alvgaona/ros:humble-desktop

ARG DEBIAN_FRONTEND=noninteractive

SHELL ["/bin/bash", "-c"]

USER root

RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-colcon-common-extensions \
    tmux \
    tmuxinator \
    iputils-ping \
    && rm -rf /var/lib/apt/lists/*

RUN mkdir -p /home/humble/drone_course_ws && chown -R humble:humble /home/humble/drone_course_ws

WORKDIR /home/humble/drone_course_ws

COPY --chown=humble:humble drone_course_ws/src ./src
COPY --chown=humble:humble *.py ./
COPY --chown=humble:humble *.bash ./
COPY --chown=humble:humble config ./config

USER humble

RUN rosdep update --rosdistro humble

USER root

RUN apt-get update && \
    su - humble -c "cd /home/humble/drone_course_ws && rosdep install --from-paths src --ignore-src -r -y --rosdistro humble" && \
    rm -rf /var/lib/apt/lists/*

USER humble

RUN echo "source /opt/ros/humble/setup.bash" >> /home/humble/.bashrc && \
    echo "source /home/humble/drone_course_ws/install/setup.bash" >> /home/humble/.bashrc && \
    touch /home/humble/.Xauthority

USER root
