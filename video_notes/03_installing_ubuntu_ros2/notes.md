# 03 - Installing Ubuntu 20.04 and ROS 2 Foxy

## Lesson Goal
Set up Ubuntu 20.04 and install ROS 2 Foxy natively so the machine is ready for ROS learning and Unitree G1 development.

## Key Links
- Ubuntu 20.04 install tutorial: https://ubuntu.com/tutorials/install-ubuntu-desktop#1-overview
- Ubuntu install video: https://youtu.be/woXkRPiWLvc?si=GqL1hV2MtPdb_gy_
- ROS 2 Foxy documentation: https://docs.ros.org/en/foxy/
- ROS 2 Foxy installation page: https://docs.ros.org/en/foxy/Installation.html
- ROS 2 Foxy Ubuntu install docs: https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Debians.html

## Core Notes
Ubuntu 20.04 is the cleanest match for ROS 2 Foxy. For this video, the important part is getting a stable Ubuntu installation first and then following the official ROS 2 Foxy Debian-package instructions.

## Ubuntu 20.04 Installation
Use the Ubuntu desktop tutorial as the main written guide and the YouTube video as a visual walkthrough. The goal is to finish with a stable Ubuntu 20.04 desktop before moving into Docker and ROS-related tools.

### Installation Resources
- Ubuntu written tutorial: https://ubuntu.com/tutorials/install-ubuntu-desktop#1-overview
- Ubuntu install video: https://youtu.be/woXkRPiWLvc?si=GqL1hV2MtPdb_gy_

[![Ubuntu 20.04 installation video thumbnail](https://img.youtube.com/vi/woXkRPiWLvc/hqdefault.jpg)](https://youtu.be/woXkRPiWLvc?si=GqL1hV2MtPdb_gy_)

Click the thumbnail above to open the Ubuntu installation video.

What to focus on during Ubuntu installation:

- Create a bootable USB installer.
- Choose Ubuntu Desktop 20.04 during installation.
- Complete normal user setup, disk selection, and regional settings.
- Update the system after first boot.
- Confirm networking works correctly before starting development-tool setup.

Practical takeaway: do not rush into ROS or Docker setup until the base Ubuntu install is clean, updated, and connected to the internet.

Recommended host baseline after Ubuntu installation:

- Ubuntu 20.04 is the cleanest match for ROS 2 Foxy.
- Internet connection working.
- System fully updated after first boot.
- Terminal access ready for package installation and environment setup.

## Installing ROS 2 Foxy
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

Main takeaway: start with a clean Ubuntu 20.04 installation, then follow the official ROS 2 Foxy Debian-package guide so the machine has a correct native Foxy setup.
