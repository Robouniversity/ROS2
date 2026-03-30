# 06 - Setting ROS 2 Using Docker

## Lesson Goal
Explain how to open the Unitree G1 ROS 2 Foxy dev container and understand what the dev container is doing behind the scenes.

## Main Repository
- Dev container repo: https://github.com/Robouniversity/ROS2
- Repository README: https://github.com/Robouniversity/ROS2/blob/main/README.md

This lesson is about using the repository's Docker-based development environment instead of installing and managing everything directly on the host.

## Key Idea
The repo provides a ready-made development container for ROS 2 Foxy and Unitree G1 work.

That means:

- the development environment is reproducible
- ROS 2 Foxy tools are already prepared inside the container
- Unitree dependencies are cloned and built automatically
- VS Code can attach directly to that environment

## What a Dev Container Means
A dev container is a Docker container configured specifically for software development.

Instead of only running one program, it becomes the full workspace environment:

- shell
- ROS tools
- compilers
- dependencies
- extensions inside VS Code

This is useful because robotics environments often depend on very specific package versions, DDS middleware setup, GPU access, GUI forwarding, and build tools.

## Main Files That Control This Setup
- Dev container config: https://github.com/Robouniversity/ROS2/blob/main/.devcontainer/devcontainer.json
- Docker image definition: https://github.com/Robouniversity/ROS2/blob/main/.devcontainer/Dockerfile
- Post-create setup script: https://github.com/Robouniversity/ROS2/blob/main/.devcontainer/post_create.sh
- Compose file: https://github.com/Robouniversity/ROS2/blob/main/docker-compose.yml

## How to Open the Dev Container in VS Code
Before this step, the Ubuntu host should already have:

- Docker installed
- Docker Compose available
- VS Code installed
- Dev Containers extension installed

Recommended workflow:

1. Clone or download the repository on the Ubuntu host.
2. Open the `Robouniversity/ROS2` folder in VS Code.
3. If prompted, install the `Dev Containers` extension.
4. Open the Command Palette.
5. Run `Dev Containers: Reopen in Container`.
6. Wait for the image build to complete.
7. Wait for the post-create setup to complete.

Important note:

- The first container startup can take a while because it installs dependencies, clones repositories, and builds ROS-related workspaces.

## What Happens When You Reopen in Container
When VS Code reopens the repo in the container, it uses `.devcontainer/devcontainer.json`.

That file tells VS Code:

- use `../docker-compose.yml`
- connect to the `unitree_dev` service
- use `/workspace` as the workspace folder
- install recommended extensions such as ROS, C/C++, and Python
- run `bash .devcontainer/post_create.sh` after the container starts

In simple terms, `devcontainer.json` is the bridge between VS Code and Docker.

## What the Dockerfile Is Doing
The Dockerfile starts from:

- `osrf/ros:foxy-desktop`

That gives the container a ROS 2 Foxy desktop base image.

Then it installs development tools and ROS-related packages such as:

- `build-essential`
- `cmake`
- `git`
- `python3-colcon-common-extensions`
- `python3-rosdep`
- `python3-vcstool`
- GUI/X11-related packages
- `ros-foxy-rmw-cyclonedds-cpp`
- `ros-foxy-rosidl-generator-dds-idl`

It also:

- initializes `rosdep`
- updates `rosdep`
- copies this repository into `/workspace`
- auto-sources `/opt/ros/foxy/setup.bash` in the container shell

So the Dockerfile creates the base ROS 2 Foxy development image.

## What docker-compose.yml Is Doing
The compose file defines the running container service:

- service name: `unitree_dev`
- container name: `unitree_dev`
- host networking enabled
- privileged mode enabled
- GPU access enabled
- X11 display forwarding enabled
- `ROS_DOMAIN_ID=0`
- `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`

Why this matters for robotics:

- host networking helps many robot communication workflows
- privileged mode is often needed for hardware-related access
- GPU support helps graphics, simulation, and accelerated workloads
- X11 forwarding allows GUI applications from the container to appear on the host desktop

The compose file also mounts:

- `/tmp/.X11-unix`
- the host Xauthority file

That is what helps GUI tools from inside the container display correctly on the Ubuntu host.

## What post_create.sh Is Doing
This script is one of the most important parts of the setup.

After the container starts, the script:

- creates and prepares the workspace structure
- clones `unitree_sdk2` if missing
- clones `unitree_ros2` if missing
- patches Unitree setup scripts
- copies local example content into workspace locations
- builds `unitree_sdk2`
- installs ROS dependencies with `rosdep`
- builds Cyclone DDS for Foxy
- builds Unitree ROS 2 packages
- sets `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`
- updates `.bashrc` so needed setup scripts are sourced automatically
- sets up MuJoCo support
- builds the local basic ROS 2 example workspace

This means the container is not just a blank Ubuntu image. It becomes a prepared Unitree ROS 2 development environment automatically.

## Why Cyclone DDS Is Mentioned
This repo uses:

- `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`

That means ROS 2 communication is configured to use Cyclone DDS instead of the default middleware path.

For robotics work, middleware configuration matters because discovery, communication reliability, and compatibility can depend on the DDS setup.

## Daily Workflow Inside the Container
After the dev container is ready, common steps are:

1. Open a terminal inside VS Code.
2. Confirm ROS is available.
3. Build packages when needed.
4. Run examples or ROS nodes from the container terminal.

Useful checks:

```bash
printenv | grep -E 'ROS_DOMAIN_ID|RMW_IMPLEMENTATION|DISPLAY|XAUTHORITY'
ros2 --help
nvidia-smi
```

Optional GUI checks:

```bash
xeyes
glxinfo | grep "OpenGL renderer"
```

## If You Do Not Use VS Code
The same repo also supports a terminal-first Docker flow.

Build and start:

```bash
docker compose up --build -d
```

Open a shell:

```bash
docker exec -it unitree_dev bash
```

Run the same post-create setup manually if needed:

```bash
bash .devcontainer/post_create.sh
```

## Main Takeaway
This repository's dev container gives you a consistent ROS 2 Foxy environment for Unitree G1 work. VS Code uses `devcontainer.json` to open the Docker environment, `Dockerfile` builds the base image, `docker-compose.yml` configures runtime features like networking, GPU, and X11, and `post_create.sh` turns the container into a ready-to-use robotics workspace.
