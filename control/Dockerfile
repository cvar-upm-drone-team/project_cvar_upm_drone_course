FROM aerostack2/nightly-humble:81518c85217bf95a34691263bfc7e766aec295a4

USER root
SHELL ["/bin/bash", "-c"]

ARG DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-colcon-common-extensions \
    python3-rosdep \
    tmux \
    tmuxinator \
  && rm -rf /var/lib/apt/lists/*

WORKDIR /root/project_cvar_upm_drone_course
COPY . /root/project_cvar_upm_drone_course

WORKDIR /root/project_cvar_upm_drone_course/control/drone_course_ws
RUN source /opt/ros/humble/setup.bash \
  && source /root/aerostack2_ws/install/setup.bash \
  && (rosdep init 2>/dev/null || true) \
  && rosdep update \
  && rosdep install --from-paths src --ignore-src -r -y \
  && colcon build --symlink-install

RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc \
  && echo "source /root/aerostack2_ws/install/setup.bash" >> /root/.bashrc \
  && echo "[ -f /root/project_cvar_upm_drone_course/control/drone_course_ws/install/setup.bash ] && source /root/project_cvar_upm_drone_course/control/drone_course_ws/install/setup.bash" >> /root/.bashrc

WORKDIR /root/project_cvar_upm_drone_course
CMD ["bash"]
