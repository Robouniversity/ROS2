# 05 - Setting Docker for Unitree G1

## Lesson Goal
Set up Docker, VS Code, and the recommended development extensions so the Unitree G1 ROS 2 Foxy development container can run cleanly on the Ubuntu host.

## Main Repository
- Dev container repository: https://github.com/Robouniversity/ROS2

This repository includes a README with the full container workflow and a helper script for installing Docker on Ubuntu.

## Key Links
- Repository README: https://github.com/Robouniversity/ROS2/blob/main/README.md
- Docker setup helper script in repo: https://github.com/Robouniversity/ROS2/blob/main/scripts/setup_docker_ubuntu.sh
- VS Code official site: https://code.visualstudio.com/
- VS Code Linux setup docs: https://code.visualstudio.com/docs/setup/linux
- Dev Containers extension: https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.remote-containers
- ROS extension for VS Code: https://marketplace.visualstudio.com/items?itemName=ms-iot.vscode-ros
- C/C++ extension: https://marketplace.visualstudio.com/items?itemName=ms-vscode.cpptools
- Python extension: https://marketplace.visualstudio.com/items?itemName=ms-python.python
- CMake Tools extension: https://marketplace.visualstudio.com/items?itemName=ms-vscode.cmake-tools
- Docker official Ubuntu install docs: https://docs.docker.com/engine/install/ubuntu/
- Docker Compose plugin install docs: https://docs.docker.com/compose/install/linux/
- Docker Linux post-install docs: https://docs.docker.com/engine/install/linux-postinstall/

## Core Notes
For this lesson, the simplest path is:

1. Start from a working Ubuntu host.
2. Install Docker on the host.
3. Install VS Code.
4. Install the VS Code extensions needed for robotics and dev containers.
5. Open the `Robouniversity/ROS2` repo in VS Code.
6. Reopen the project in the dev container and let the setup finish.

The repository README is the main workflow reference for this video because it already explains both:

- the VS Code Dev Containers flow
- the plain Docker Compose flow

## Install Docker on Ubuntu
The repo already includes a host-side helper script:

- `scripts/setup_docker_ubuntu.sh`

The script installs:

- Docker Engine
- Docker Compose plugin
- NVIDIA Container Toolkit when an NVIDIA driver is already present

Recommended repo-based path:

```bash
chmod +x scripts/setup_docker_ubuntu.sh
./scripts/setup_docker_ubuntu.sh
```

Important note:

- Run this on the Ubuntu host, not inside a container.
- If Docker group membership does not apply immediately, log out and log back in.

Official Docker documentation path using `apt`:

- Docker Engine on Ubuntu: https://docs.docker.com/engine/install/ubuntu/
- Docker Compose plugin on Linux: https://docs.docker.com/compose/install/linux/
- Linux post-install steps: https://docs.docker.com/engine/install/linux-postinstall/

What to focus on from the official Docker docs:

- Add Docker's official apt repository and GPG key.
- Install `docker-ce`, `docker-ce-cli`, `containerd.io`, `docker-buildx-plugin`, and `docker-compose-plugin`.
- Add your user to the `docker` group for non-root usage.
- Verify with `docker --version` and `docker compose version`.

## Install VS Code
Use the official VS Code website:

- https://code.visualstudio.com/

Linux-specific setup docs:

- https://code.visualstudio.com/docs/setup/linux

What to do:

- Download and install VS Code on the Ubuntu host.
- Open the `Robouniversity/ROS2` repository in VS Code.
- Sign in to sync settings only if you want that workflow, but it is optional.

## VS Code Plugins for Robotics Development
Recommended extensions for this repo:

- Dev Containers: lets VS Code reopen the project directly inside the Docker development container.
- ROS: adds ROS and ROS 2 support such as terminals, launch support, and workspace helpers.
- C/C++: useful for IntelliSense, navigation, and debugging of native ROS packages.
- Python: useful for ROS tools, scripts, and Python nodes.
- CMake Tools: helpful for CMake-based package workflows and build configuration.

Official extension links:

- Dev Containers: https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.remote-containers
- ROS: https://marketplace.visualstudio.com/items?itemName=ms-iot.vscode-ros
- C/C++: https://marketplace.visualstudio.com/items?itemName=ms-vscode.cpptools
- Python: https://marketplace.visualstudio.com/items?itemName=ms-python.python
- CMake Tools: https://marketplace.visualstudio.com/items?itemName=ms-vscode.cmake-tools

Optional extra:

- Docker DX extension: https://marketplace.visualstudio.com/items?itemName=docker.docker

## Start the Unitree G1 Dev Container
The repo README explains the main VS Code workflow:

1. Open the repository in VS Code.
2. Install the `Dev Containers` extension if needed.
3. Run `Dev Containers: Reopen in Container`.
4. Wait for the image build and container startup.
5. Wait for the post-create step to clone dependencies and build the workspace.

The README also explains the terminal-based alternative with Docker Compose if you do not want to use the full VS Code container workflow.

## Host-Side Verification
Before opening the dev container, useful checks on the Ubuntu host are:

```bash
docker --version
docker compose version
nvidia-smi
echo $DISPLAY
echo $XAUTHORITY
```

## Dev-Container Verification
Once the container is running, useful checks are:

```bash
printenv | grep -E 'ROS_DOMAIN_ID|RMW_IMPLEMENTATION|DISPLAY|XAUTHORITY'
ros2 --help
nvidia-smi
```

Optional GUI and rendering checks:

```bash
xeyes
glxinfo | grep "OpenGL renderer"
```

## Main Takeaway
Use the `Robouniversity/ROS2` repository README as the main setup guide for this lesson. Install Docker on the Ubuntu host either with the repo helper script or the official Docker apt-based instructions, install VS Code from the official site, add the recommended robotics extensions, and then reopen the project in the dev container.
