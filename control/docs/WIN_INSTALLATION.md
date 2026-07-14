# Windows Installation (Docker)

This guide covers running the `control` track's Docker environment on **Windows**. Since ROS 2 Humble and Aerostack2 only support Linux natively, Docker is the recommended (and simplest) way to run this course on Windows.

> **⚠️ WSL2 is required, not optional.** As long as the Docker image is built and run from **inside a WSL2 terminal** (not PowerShell/CMD), GUI apps (RViz, Gazebo) display automatically on your Windows desktop through WSLg — no VNC Viewer or extra display setup needed. If you build/run from PowerShell instead, GUI forwarding will **not** work. All commands in this guide must be run from a WSL2 terminal (Ubuntu).

## 1. Prerequisites

- **WSL2**, with a Linux distro installed (e.g. Ubuntu): [WSL Install Guide](https://learn.microsoft.com/en-us/windows/wsl/install). WSLg (bundled with modern WSL2) provides GUI forwarding out of the box — this is what makes VNC unnecessary.
- **Docker Desktop for Windows**, with the **WSL2 backend** enabled: [Docker Desktop Installation Guide](https://docs.docker.com/desktop/install/windows-install/)
  - During/after install, make sure WSL2 integration is enabled (Docker Desktop → Settings → General → "Use the WSL 2 based engine", and Settings → Resources → WSL Integration).
  - `docker compose` ships bundled with Docker Desktop, no separate install needed.
- **Git** — use `git` from within WSL2 (recommended), or [Git for Windows](https://git-scm.com/download/win).

All commands below must be run from a **WSL2 terminal (Ubuntu)** — not PowerShell — so that paths, shell scripts (`launch_as2.bash`, `stop.bash`), and GUI forwarding all behave the same way as on native Linux.

## 2. Clone the Repository

From a **WSL2 terminal (Ubuntu)**:

```bash
cd ~
git clone https://github.com/cvar-upm-drone-team/project_cvar_upm_drone_course.git
```

## 3. Building and Running the Container

Run all of the following from the **same WSL2 terminal**:

1. Navigate to the `control` directory:

```bash
cd ~/project_cvar_upm_drone_course/control
```

2. Build the Docker image:

```bash
docker compose build
```

3. Start the container:

```bash
docker compose up -d
```

4. Connect to the container using `docker exec` in a new WSL2 terminal:

```bash
docker exec -it project_cvar_upm_drone_course_control bash
```

5. Build the workspace inside the container:

```bash
cd ~/project_cvar_upm_drone_course/control
cd drone_course_ws
colcon build --symlink-install
source install/setup.bash
```

## 4. Accessing the GUI (RViz / Gazebo)

No VNC Viewer or extra setup is needed. As long as the container was built and launched from a WSL2 terminal, GUI windows (RViz, Gazebo) are forwarded automatically through WSLg and appear directly on your Windows desktop, just like on native Linux.

## 5. Run the Course Environment

Inside the container (via `docker exec`, as in step 3.4):

```bash
cd ~/project_cvar_upm_drone_course/control
```

### Fix scripts format for Windows

Run the following commands to make the scripts executable:
```bash
sed -i 's/\r$//' launch_as2.bash
sed -i 's/\r$//' stop.bash
sed -i 's/\r$//' config/initialize.bash
```

### Launch simulation

```bash
./launch_as2.bash
```

Platform options:

- `-p ms`: Multirotor Simulator (default)
- `-p gz`: Gazebo

Examples:

```bash
./launch_as2.bash -p ms
./launch_as2.bash -p gz
```

A window like this should appear directly on your Windows desktop (via WSLg):

<img src="resources/rviz_view.png" alt="RViz simulation view" width="500" />

## 6. Stop Simulation

Launch the stop script in the tmuxinator session, or in a new `docker exec` terminal:

```bash
./stop.bash
```

## Troubleshooting

- **Docker Desktop fails to start / WSL2 error**: Make sure virtualization is enabled in your BIOS and the WSL2 kernel update is installed ([Microsoft WSL install guide](https://learn.microsoft.com/en-us/windows/wsl/install)).
- **`docker compose` not found**: Update Docker Desktop to a recent version; `docker compose` (v2, no hyphen) is bundled by default.
- **GUI windows (RViz/Gazebo) don't appear**: Double-check that you built and ran the container from a **WSL2 terminal**, not PowerShell/CMD — GUI forwarding via WSLg only works for the WSL2 flow. Also ensure WSL2/WSLg is up to date (`wsl --update` from PowerShell) and that you're on a Windows version with WSLg support (Windows 11, or Windows 10 with a recent WSL2 update).
- **Slow performance / file changes not syncing**: If cloning the repo onto the Windows filesystem (e.g. `C:\Users\...`) feels slow, clone it inside your WSL2 filesystem instead (e.g. `~` inside WSL2) and run Docker Desktop with the WSL2 backend for near-native performance.
