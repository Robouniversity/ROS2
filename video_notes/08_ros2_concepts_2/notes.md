# 08 - ROS 2 Concepts 2

## Lesson Goal
Use the ROS 2 CLI tools to explore a running ROS 2 system, interact with `turtlesim`, and understand publisher-subscriber behavior through hands-on commands.

## Main Reference
- ROS 2 Foxy Beginner CLI Tools: https://docs.ros.org/en/foxy/Tutorials/Beginner-CLI-Tools.html

This lesson is an abstract, guided note based on the official Foxy beginner CLI tools tutorial series.

## Key Links
- Beginner CLI tools index: https://docs.ros.org/en/foxy/Tutorials/Beginner-CLI-Tools.html
- Configuring ROS 2 environment: https://docs.ros.org/en/foxy/Tutorials/Beginner-CLI-Tools/Configuring-ROS2-Environment.html
- Using `turtlesim`, `ros2`, and `rqt`: https://docs.ros.org/en/foxy/Tutorials/Beginner-CLI-Tools/Introducing-Turtlesim/Introducing-Turtlesim.html
- Understanding nodes: https://docs.ros.org/en/foxy/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Nodes/Understanding-ROS2-Nodes.html
- Understanding topics: https://docs.ros.org/en/foxy/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Topics/Understanding-ROS2-Topics.html
- Understanding services: https://docs.ros.org/en/foxy/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Services/Understanding-ROS2-Services.html
- Understanding parameters: https://docs.ros.org/en/foxy/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Parameters/Understanding-ROS2-Parameters.html
- Understanding actions: https://docs.ros.org/en/foxy/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Actions/Understanding-ROS2-Actions.html

## Core Notes
This lesson is about learning ROS 2 by interacting with a live system from the terminal.

The ROS 2 CLI is important because it helps you:

- start packages and nodes
- inspect the ROS graph
- view topics, services, and parameters
- publish test messages
- debug communication without writing full application code first

For learning, `turtlesim` is a very good example because it gives a visible robot-like system with simple commands and immediate feedback.

## Step 1: Source the ROS 2 Environment
In every new terminal, source the ROS 2 setup first:

```bash
source /opt/ros/foxy/setup.bash
```

Useful quick check:

```bash
printenv | grep -i ROS
```

You should see variables such as `ROS_DISTRO=foxy`.

## Step 2: Install and Check turtlesim
If `turtlesim` is not already installed:

```bash
sudo apt update
sudo apt install ros-foxy-turtlesim
```

Check the package executables:

```bash
ros2 pkg executables turtlesim
```

You should see tools such as:

- `turtlesim_node`
- `turtle_teleop_key`

## Step 3: Start turtlesim
Open a terminal and run:

```bash
ros2 run turtlesim turtlesim_node
```

This starts the simulator window and creates a ROS 2 node for the turtle.

## Step 4: Start Keyboard Teleoperation
Open another terminal and run:

```bash
source /opt/ros/foxy/setup.bash
ros2 run turtlesim turtle_teleop_key
```

Now use the keyboard to move the turtle.

This gives you a simple live ROS 2 system with:

- one node for the simulator
- one node for keyboard input
- topics and services connecting them

## Interacting With ROS 2 CLI
Open a third terminal and use the ROS 2 CLI to inspect the running system.

### List Nodes
```bash
source /opt/ros/foxy/setup.bash
ros2 node list
```

You should see nodes such as:

- `/turtlesim`
- `/teleop_turtle`

Inspect one node:

```bash
ros2 node info /turtlesim
```

This shows its publishers, subscribers, services, and actions.

### List Topics
```bash
ros2 topic list
```

This shows all active topics in the system.

To include message types:

```bash
ros2 topic list -t
```

Inspect one topic:

```bash
ros2 topic info /turtle1/cmd_vel
```

This helps you see who publishes and who subscribes.

### Echo Topic Data
To watch data on a topic:

```bash
ros2 topic echo /turtle1/pose
```

Now move the turtle and watch the live pose data update.

This is one of the most useful ROS 2 debugging commands.

### Show Topic Type
```bash
ros2 topic type /turtle1/cmd_vel
```

This tells you which message type the topic uses.

You can then inspect the interface if needed:

```bash
ros2 interface show geometry_msgs/msg/Twist
```

## Publisher and Subscriber Example
This is the easiest way to understand publisher-subscriber behavior using CLI tools.

In the running `turtlesim` system:

- the teleop node publishes velocity commands
- the turtlesim node subscribes to them
- the turtlesim node publishes pose updates
- your terminal can subscribe by echoing the pose topic

### CLI Publisher Example
Publish a velocity command manually:

```bash
ros2 topic pub /turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 2.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 1.8}}"
```

This command acts like a publisher node.

What happens:

- the CLI publishes a `Twist` message
- `/turtlesim` receives it as a subscriber
- the turtle starts moving in the simulator window

### Repeated Publisher
To keep publishing continuously:

```bash
ros2 topic pub --rate 1 /turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 1.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.5}}"
```

This publishes at 1 Hz.

### CLI Subscriber Example
Use the terminal as a subscriber:

```bash
ros2 topic echo /turtle1/pose
```

Now the terminal subscribes to the pose topic and prints the turtle state continuously.

This is the simplest publisher-subscriber demo:

- one terminal publishes commands
- another terminal subscribes to state
- the simulator shows the result visually

## Interacting With turtlesim
`turtlesim` is useful because it lets you test multiple ROS 2 concepts in one place.

Things you can do:

- move the turtle with keyboard teleop
- publish velocity commands manually from CLI
- inspect topics and nodes
- call services such as clear, reset, spawn, and kill
- view parameters

## Working With Services in turtlesim
List services:

```bash
ros2 service list
```

You will see services such as:

- `/clear`
- `/reset`
- `/spawn`
- `/kill`

Check a service type:

```bash
ros2 service type /clear
```

Call the clear service:

```bash
ros2 service call /clear std_srvs/srv/Empty
```

Call reset:

```bash
ros2 service call /reset std_srvs/srv/Empty
```

Spawn another turtle:

```bash
ros2 service call /spawn turtlesim/srv/Spawn "{x: 4.0, y: 2.0, theta: 0.0, name: 'turtle2'}"
```

This is a very nice way to demonstrate that ROS 2 services perform request-response interactions.

## Working With Parameters
List parameters of a node:

```bash
ros2 param list /turtlesim
```

Get a parameter:

```bash
ros2 param get /turtlesim background_r
```

Set a parameter:

```bash
ros2 param set /turtlesim background_r 150
```

You can also change:

- `background_g`
- `background_b`

After changing parameters, use:

```bash
ros2 service call /clear std_srvs/srv/Empty
```

This refreshes the window so the new background color is visible.

## Understanding ROS 2 CLI as a Learning Tool
The CLI is not only for running commands. It is one of the best ways to understand how ROS 2 systems are wired together.

With the CLI, you can:

- inspect the graph without opening code
- verify whether topics are active
- confirm node names
- test communication manually
- debug whether a publisher or subscriber is working

That is why ROS 2 CLI tools are so important in real robotics development.

## Practical Learning Sequence
A good order for beginners is:

1. Start `turtlesim`.
2. Start keyboard teleop.
3. List nodes.
4. List topics.
5. Echo `/turtle1/pose`.
6. Publish manually to `/turtle1/cmd_vel`.
7. List and call services.
8. View and change parameters.

This sequence gives a full introduction to the ROS graph through direct interaction.

## Main Takeaway
The ROS 2 CLI gives you direct control over a live robot-style system. With `turtlesim`, you can see nodes, topics, services, and parameters in action, manually publish and subscribe from the terminal, and build strong intuition about how ROS 2 communication works before writing larger applications.
