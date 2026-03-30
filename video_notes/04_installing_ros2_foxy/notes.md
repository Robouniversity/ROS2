# 04 - Installing ROS 2 Foxy

## Lesson Goal
Install ROS 2 Foxy natively on Ubuntu 20.04 so the machine is ready for ROS development and Unitree G1 work.

## Key Links
- ROS 2 Foxy documentation: https://docs.ros.org/en/foxy/
- ROS 2 Foxy installation page: https://docs.ros.org/en/foxy/Installation.html
- ROS 2 Foxy Ubuntu install docs: https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Debians.html

## Core Notes
The main reference for this part should be the official Ubuntu Debian install guide:

- https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Debians.html

This is the standard package-based installation path for ROS 2 Foxy on Ubuntu 20.04.

What to focus on from the Foxy docs:

- Foxy is the correct distro for this repository.
- Ubuntu 20.04 is the matching operating system target.
- The official docs explain how to add the ROS 2 package sources and install Foxy from Debian packages.
- They also show the environment setup step so ROS 2 commands are available in the terminal.
- Foxy is end-of-life now, but it is still the right reference for this specific repo because the container and dependencies are Foxy-based.

Important repo-specific idea:

- Foxy is end-of-life, so use its documentation here because it matches this repository, not because it is the newest ROS 2 release.

Suggested setup order:

1. Install Ubuntu on the host.
2. Update Ubuntu 20.04 fully after first boot.
3. Review the ROS 2 Foxy Debian install guide.
4. Add the ROS 2 apt repository and keys as shown in the docs.
5. Install ROS 2 Foxy packages.
6. Source the Foxy environment.
7. Verify the installation with ROS 2 commands.

Why ROS 2 installation docs still matter:

- They explain Foxy platform compatibility.
- They show the standard package-based install flow for Ubuntu.
- They help you understand the normal ROS environment layout on a native Ubuntu machine.

## Quick Verification Checklist / Takeaway
- `source /opt/ros/foxy/setup.bash`
- `ros2 --help`
- `printenv | grep ROS`

Main takeaway: after Ubuntu 20.04 is installed and updated, follow the official ROS 2 Foxy Debian-package guide so the machine has a correct native Foxy setup.
