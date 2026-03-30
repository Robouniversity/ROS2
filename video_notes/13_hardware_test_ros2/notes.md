# 13 - Hardware Test with ROS 2

## Lesson Goal
Use the official `unitree_ros2` repository as the reference for hardware-side ROS 2 setup, verify communication with the robot, inspect topics, visualize point cloud data in RViz, and understand how to run a low-level G1 example such as `g1_ankle_swing_example.cpp`.

## Main References
- Unitree ROS 2 repository: https://github.com/unitreerobotics/unitree_ros2
- G1 ankle example: https://github.com/unitreerobotics/unitree_ros2/blob/master/example/src/src/g1/lowlevel/g1_ankle_swing_example.cpp

## Core Idea
This video is about testing the real ROS 2 communication path with Unitree hardware.

The workflow is:

1. set up the Unitree ROS 2 environment
2. connect the robot by Ethernet
3. confirm ROS 2 topics are visible
4. read robot data from topics
5. optionally visualize point cloud in RViz
6. run a low-level G1 example carefully

## What the `unitree_ros2` Repo Provides
The repository gives you:

- Unitree ROS 2 message packages
- Cyclone DDS setup
- example programs
- setup scripts for real robot and simulation
- low-level examples for G1 and other robots

Important idea:

- this repo is not only for simulation
- it is also the driver-side ROS 2 communication path for Unitree hardware testing

## System and ROS 2 Requirements
The repo documents these tested combinations:

- Ubuntu 20.04 with ROS 2 Foxy
- Ubuntu 22.04 with ROS 2 Humble

For your current notes sequence, Foxy is the main reference path.

## Install and Build `unitree_ros2`
The repo’s general flow is:

```bash
git clone https://github.com/unitreerobotics/unitree_ros2
cd ~/unitree_ros2
```

Install dependencies:

```bash
sudo apt install ros-foxy-rmw-cyclonedds-cpp
sudo apt install ros-foxy-rosidl-generator-dds-idl
sudo apt install libyaml-cpp-dev
```

Compile Cyclone DDS workspace:

```bash
cd ~/unitree_ros2/cyclonedds_ws/src
git clone https://github.com/ros2/rmw_cyclonedds -b foxy
git clone https://github.com/eclipse-cyclonedds/cyclonedds -b releases/0.10.x
cd ..
colcon build --packages-select cyclonedds
```

Then source ROS 2 and compile the Unitree packages:

```bash
source /opt/ros/foxy/setup.bash
colcon build
```

## Hardware Network Setup
The repo’s hardware instructions are very specific about Ethernet setup.

### Physical connection
- connect the Unitree robot and the development machine with Ethernet

### Find the network interface
Use:

```bash
ifconfig
```

or:

```bash
ip addr
```

Find the interface connected to the robot, for example:

- `enp3s0`

### Set static IPv4
The repo instructs setting the host network interface manually to:

- IP address: `192.168.123.100`
- netmask: `255.255.255.0`

This is important because the ROS 2 DDS communication depends on the host being on the correct robot-side network.

## `setup.sh` for Real Hardware
The real-robot setup script in the repo looks like this:

```bash
source /opt/ros/foxy/setup.bash
source $HOME/unitree_ros2/cyclonedds_ws/install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI='<CycloneDDS><Domain><General><Interfaces>
                            <NetworkInterface name="enp3s0" priority="default" multicast="default" />
                        </Interfaces></General></Domain></CycloneDDS>'
```

What this does:

- loads ROS 2
- loads the Unitree Cyclone DDS workspace
- forces ROS 2 to use Cyclone DDS
- forces DDS traffic onto the robot network interface

The interface name must match your real hardware connection.

If your robot is connected through a different interface, replace `enp3s0` with the correct one.

## Source the Environment
After editing `setup.sh`, source it:

```bash
source ~/unitree_ros2/setup.sh
```

This should be done in every new terminal used for hardware testing.

## First Hardware Communication Test
After setup, the repo suggests:

```bash
source ~/unitree_ros2/setup.sh
ros2 topic list
```

What you are checking here:

- DDS discovery is working
- ROS 2 can see the Unitree topics
- the network configuration is correct

If this step fails, do not continue to motion commands yet.

## Basic Data Verification
The repo shows reading robot state by echoing a topic such as:

```bash
ros2 topic echo /sportmodestate
```

This verifies that:

- the host is receiving live robot state
- DDS transport is working
- the ROS 2 environment is pointed at the actual robot

This is one of the safest early tests because it is read-only.

## Build and Run the Example Workspace
The repo says to build the example workspace like this:

```bash
source ~/unitree_ros2/setup.sh
cd ~/unitree_ros2/example
colcon build
```

After that, the example executables appear under the install tree.

Example:

```bash
./install/unitree_ros2_example/bin/read_motion_state
```

This is a good first executable to run because it reads state rather than commanding motion.

## Useful Read-Only Tests Before Low-Level Commands
Recommended progression:

1. `ros2 topic list`
2. `ros2 topic echo /sportmodestate`
3. run `read_motion_state`
4. run a low-state reader for HG robots if needed

This helps confirm the pipeline before sending commands to actuators.

## Reading G1 Low-Level State
The repo includes a reader for HG-based robots:

- `read_low_state_hg.cpp`

This is relevant for G1 because G1 uses `unitree_hg` message types.

What that example does:

- subscribes to `lowstate` or `lf/lowstate`
- reads IMU data
- reads motor states
- prints robot feedback to the terminal

This is a very good diagnostic step before attempting any low-level motion example.

## Point Cloud Visualization in RViz
The repo also includes an RViz example for visualizing robot lidar data.

The README describes this flow:

### 1. List topics
```bash
ros2 topic list
```

Look for the lidar point-cloud topic:

- `utlidar/cloud`

### 2. Check the frame id
```bash
ros2 topic echo --no-arr /utlidar/cloud
```

The repo’s example shows the frame id as:

- `utlidar_lidar`

### 3. Start RViz
```bash
ros2 run rviz2 rviz2
```

### 4. Add the point cloud display
In RViz:

- add a `PointCloud2` display
- set the topic to `utlidar/cloud`
- set the fixed frame to `utlidar_lidar`

Then the lidar point cloud should appear.

Important note:

- this point-cloud example is described in the upstream repo as a robot lidar visualization example
- available topics depend on the robot hardware and enabled sensors

So in your real hardware test, confirm the topic exists first with `ros2 topic list`.

## Low-Level G1 Example: `g1_ankle_swing_example.cpp`
This is one of the most relevant low-level G1 examples in the repo.

File:

- https://github.com/unitreerobotics/unitree_ros2/blob/master/example/src/src/g1/lowlevel/g1_ankle_swing_example.cpp

After building the example workspace, run it with:

```bash
./install/unitree_ros2_example/bin/g1_ankle_swing_example
```

Use this only after read-only topic checks succeed and the robot is in a safe testing condition.

## What This Example Is Doing
This program is a ROS 2 node that:

- subscribes to G1 low-level state
- subscribes to the G1 torso IMU topic
- publishes low-level motor commands
- drives ankle motion in several stages
- prints periodic status output

This is a real low-level control example, not just a topic reader.

## Key Topics Used by the Example
The file defines:

```cpp
const auto HG_CMD_TOPIC = "lowcmd";
const auto HG_IMU_TORSO = "secondary_imu";
const auto HG_STATE_TOPIC = "lowstate";
```

Meaning:

- command topic: `lowcmd`
- state topic: `lowstate`
- torso IMU topic: `secondary_imu`

This matches the HG/G1 low-level communication style.

## Important G1 Joint Mapping
The example includes a full G1 joint index enum.

This is important because ankle control depends on exact motor indices:

- `LEFT_ANKLE_PITCH = 4`
- `LEFT_ANKLE_ROLL = 5`
- `RIGHT_ANKLE_PITCH = 10`
- `RIGHT_ANKLE_ROLL = 11`

If those indices are wrong, the wrong joints move.

## How the G1 Ankle Example Is Structured
The node creates:

- a low-state subscriber
- an IMU subscriber
- a low-command publisher
- two timers running at 2 ms

The two-timer pattern is important:

- one timer computes the control targets
- one timer packages and publishes the command

This separates command generation from message publication.

## Motion Stages in the Example
The control function uses three stages:

1. move the robot toward a zero posture
2. swing the ankles in PR mode
3. swing the ankles in AB mode

That means the example is not only “move ankle once.”

It is also demonstrating:

- safe transition into a known pose
- pitch/roll-style ankle control
- alternate A/B series-parallel style control mode

## PR Mode vs AB Mode
The example defines:

```cpp
enum class Mode {
  PR = 0,
  AB = 1
};
```

This is an important G1 concept because some joints can be controlled in different actuator representations.

The example explicitly switches:

- first to PR mode
- later to AB mode

So this is not only a motion demo, it is also a control-mode demo.

## Publishing the Low-Level Command
The example writes the outgoing command like this conceptually:

- set `mode_pr`
- copy the current `mode_machine`
- fill each joint command
- compute CRC
- publish the `LowCmd`

That means low-level motion control is not just setting joint angles. It also requires:

- preserving machine mode
- publishing valid HG messages
- filling CRC correctly

## Reading State and Safety-Relevant Data
The low-state callback does more than just store positions.

It also:

- reads motor error codes
- reads IMU data
- reads wireless controller state
- logs detailed robot status periodically

This is very useful during hardware testing because it gives:

- actuator mode visibility
- position and velocity feedback
- torque estimation
- temperature and voltage info

That makes it a stronger test than a simple blind command sender.

## Suggested Hardware Test Order for G1
For a cautious workflow, use this order:

1. connect Ethernet and configure the host network
2. source `~/unitree_ros2/setup.sh`
3. run `ros2 topic list`
4. run `ros2 topic echo /sportmodestate`
5. build the example workspace
6. run a read-only state example first
7. confirm `lowstate` and `secondary_imu` are active
8. only then run `g1_ankle_swing_example`

This order reduces the chance of trying low-level motion before confirming the communication pipeline.

## Practical Command Sequence
Example hardware-side sequence:

```bash
source ~/unitree_ros2/setup.sh
ros2 topic list
ros2 topic echo /sportmodestate
```

Build the examples:

```bash
source ~/unitree_ros2/setup.sh
cd ~/unitree_ros2/example
colcon build
```

Run a read-only example:

```bash
./install/unitree_ros2_example/bin/read_motion_state
```

If available for your hardware setup, check point cloud:

```bash
ros2 topic echo --no-arr /utlidar/cloud
ros2 run rviz2 rviz2
```

Then run the low-level G1 ankle node after confirming the robot is in a safe test condition:

```bash
./install/unitree_ros2_example/bin/g1_ankle_swing_example
```

## What This Video Should Emphasize
- `setup.sh` is the hardware communication gateway
- network interface selection is critical
- `ros2 topic list` and `ros2 topic echo` are the first validation tools
- point-cloud visualization is a good sensor-side ROS 2 validation test
- `g1_ankle_swing_example.cpp` is a real low-level actuator example and should be treated carefully
- read-only checks should happen before low-level command tests

## Main Takeaway
The `unitree_ros2` repository already provides the full hardware-test path: configure the Ethernet interface, source the Cyclone DDS ROS 2 environment, verify topics, read live state, optionally visualize lidar point cloud in RViz, and then move into low-level G1 examples like `g1_ankle_swing_example.cpp`. The most important practical lesson is to treat low-level G1 nodes as the final step, only after communication, state visibility, and sensor topics have already been verified.
