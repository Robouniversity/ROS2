# 09 - ROS 2 Programming Practice

## Lesson Goal
Understand how to create the `cpp_pubsub` package from the ROS 2 Foxy tutorial and learn the core C++ publisher-subscriber programming pattern used in ROS 2.

## Main Reference
- Beginner: Client libraries: https://docs.ros.org/en/foxy/Tutorials/Beginner-Client-Libraries.html
- Writing a simple publisher and subscriber (C++): https://docs.ros.org/en/foxy/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Cpp-Publisher-And-Subscriber.html

This lesson is an abstraction of the official tutorial, focused on the C++ publisher and subscriber example.

## Key Idea
In ROS 2 C++, a node becomes a publisher or subscriber by using the `rclcpp` client library.

The main workflow is:

1. Create a workspace.
2. Create a C++ package.
3. Add a publisher node.
4. Add a subscriber node.
5. Register executables in `CMakeLists.txt`.
6. Build with `colcon`.
7. Run both nodes and observe topic communication.

## Main Concepts Behind the Example
The `cpp_pubsub` tutorial demonstrates a very common ROS 2 design pattern:

- one node publishes messages on a topic
- another node subscribes to that same topic
- both use a known message type
- the topic name and message type must match

The example is usually described as:

- talker: the publisher
- listener: the subscriber

This is the first real programming pattern most ROS 2 users learn.

## Step 1: Create a Workspace
Start by sourcing ROS 2 Foxy:

```bash
source /opt/ros/foxy/setup.bash
```

Create a workspace if you do not already have one:

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
```

Why this matters:

- `ros2_ws` is the workspace root
- `src` is where package source code lives

## Step 2: Create the C++ Package
Create the package:

```bash
ros2 pkg create --build-type ament_cmake cpp_pubsub
```

This generates the standard files for a C++ ROS 2 package.

Important generated files:

- `package.xml`
- `CMakeLists.txt`
- `src/`
- `include/` if needed later

The package uses:

- `ament_cmake` as the build system
- `rclcpp` for ROS 2 C++ programming
- standard ROS 2 package structure

## Step 3: Understand the Package Structure
After creation, the package looks conceptually like this:

```text
cpp_pubsub/
├── CMakeLists.txt
├── package.xml
└── src/
```

Then you add the two C++ source files:

- `publisher_member_function.cpp`
- `subscriber_member_function.cpp`

These files contain the two nodes.

## Step 4: Publisher Node Abstraction
The publisher node usually does the following:

1. Inherit from `rclcpp::Node`.
2. Create a publisher object.
3. Create a timer.
4. On each timer callback, publish a message.

This means the publisher is not sending data randomly. It publishes at a regular interval.

Typical responsibilities of the publisher:

- create a topic publisher
- prepare a message
- fill the message contents
- publish the message repeatedly

Important concepts inside the publisher:

- `rclcpp::Node`: base class for the node
- `create_publisher<msg_type>()`: creates the publisher
- timer callback: decides when publishing happens
- queue depth: affects buffering behavior

In the tutorial, the publisher sends a string message repeatedly to a topic.

### Example Talker Code in C++
```cpp
#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

class MinimalPublisher : public rclcpp::Node
{
public:
  MinimalPublisher()
  : Node("minimal_publisher"), count_(0)
  {
    publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);
    timer_ = this->create_wall_timer(
      500ms, std::bind(&MinimalPublisher::timer_callback, this));
  }

private:
  void timer_callback()
  {
    auto message = std_msgs::msg::String();
    message.data = "Hello, world! " + std::to_string(count_++);
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    publisher_->publish(message);
  }

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  size_t count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}
```

What this code does:

- creates a node called `minimal_publisher`
- creates a publisher on the `topic` topic
- runs a timer every 500 ms
- publishes a `std_msgs/msg/String` message each time
- increments a counter so each message is different

## Step 5: Subscriber Node Abstraction
The subscriber node usually does the following:

1. Inherit from `rclcpp::Node`.
2. Create a subscription object.
3. Register a callback function.
4. React whenever a new message arrives.

Typical responsibilities of the subscriber:

- subscribe to the topic
- receive messages of the expected type
- execute code inside the callback
- print or process the received data

Important concepts inside the subscriber:

- `create_subscription<msg_type>()`: creates the subscriber
- callback function: runs when data arrives
- message type must match the publisher
- topic name must match the publisher

In the tutorial, the subscriber listens for the string messages and logs what it receives.

### Example Listener Code in C++
```cpp
#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class MinimalSubscriber : public rclcpp::Node
{
public:
  MinimalSubscriber()
  : Node("minimal_subscriber")
  {
    subscription_ = this->create_subscription<std_msgs::msg::String>(
      "topic", 10,
      std::bind(&MinimalSubscriber::topic_callback, this, std::placeholders::_1));
  }

private:
  void topic_callback(const std_msgs::msg::String::SharedPtr msg) const
  {
    RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
  }

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}
```

What this code does:

- creates a node called `minimal_subscriber`
- subscribes to the `topic` topic
- waits for `std_msgs/msg/String` messages
- runs the callback whenever a message arrives
- prints the received data to the terminal

## Step 6: Core C++ Pattern in the Example
The tutorial is less about the specific string content and more about the pattern.

The pattern is:

- initialize ROS 2
- create a node object
- create publisher or subscriber resources
- spin the node so callbacks can run
- shut down cleanly

Very important idea:

- publisher timers need spinning to keep running
- subscriber callbacks need spinning to receive messages

So `rclcpp::spin(...)` is a key part of the runtime behavior.

## Step 7: Dependencies
The tutorial example needs package dependencies such as:

- `rclcpp`
- `std_msgs`

These dependencies must be declared in:

- `package.xml`
- `CMakeLists.txt`

Why this matters:

- `package.xml` describes package metadata and dependencies
- `CMakeLists.txt` tells CMake how to build the executables and link libraries

## Step 8: CMakeLists.txt Abstraction
For the example to build correctly, `CMakeLists.txt` must do several jobs:

- find required packages
- create the publisher executable
- create the subscriber executable
- link them against ROS 2 dependencies
- install the executables

Conceptually, it must include:

- `find_package(rclcpp REQUIRED)`
- `find_package(std_msgs REQUIRED)`
- `add_executable(...)`
- `ament_target_dependencies(...)`
- `install(TARGETS ...)`

This is the bridge between your C++ files and the ROS 2 build system.

## Step 9: package.xml Abstraction
`package.xml` is the metadata file for the package.

For this tutorial, it should describe:

- package name
- version
- maintainer
- license
- build tool dependency
- runtime/build dependencies

The important dependency ideas are:

- `ament_cmake`
- `rclcpp`
- `std_msgs`

Without correct dependency declarations, the package may not build or run correctly.

## Step 10: Build the Package
Go to the workspace root:

```bash
cd ~/ros2_ws
```

Build:

```bash
colcon build --packages-select cpp_pubsub
```

Why this matters:

- `colcon` discovers the package
- builds the C++ executables
- creates an install space for the package

After building, source the workspace:

```bash
source install/setup.bash
```

This makes the newly built package available to ROS 2 commands.

## Step 11: Run the Publisher and Subscriber
Open one terminal for the publisher:

```bash
source /opt/ros/foxy/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 run cpp_pubsub talker
```

Open another terminal for the subscriber:

```bash
source /opt/ros/foxy/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 run cpp_pubsub listener
```

Expected behavior:

- the talker keeps publishing string messages
- the listener receives those messages and prints them

This is the simplest complete ROS 2 C++ communication program.

## What the Talker and Listener Teach
This small example teaches several major ROS 2 programming ideas at once:

- how to create a node in C++
- how to publish messages
- how to subscribe to messages
- how topics connect separate processes
- how callbacks work
- how ROS 2 packages are built and run

Even though the example is simple, the same pattern scales into real robot applications.

## How This Connects to Real Robotics
In a real robot system, the same publisher-subscriber idea appears everywhere.

Examples:

- a sensor node publishes IMU data
- a control node subscribes to robot state
- a perception node publishes detections
- a planner node subscribes to maps and robot pose
- a command node publishes velocity or joint commands

So the `cpp_pubsub` example is not just a toy example. It is the basic communication pattern used throughout ROS 2 systems.

## Common Mistakes Beginners Hit
- Forgetting to source `/opt/ros/foxy/setup.bash`
- Forgetting to source the workspace `install/setup.bash`
- Building from the wrong directory instead of the workspace root
- Creating the package outside the `src` folder
- Using mismatched topic names
- Using mismatched message types
- Forgetting to add dependencies in `CMakeLists.txt` or `package.xml`
- Forgetting to install the executables in `CMakeLists.txt`

## Practical Learning Sequence
Good beginner sequence:

1. Create the workspace.
2. Create `cpp_pubsub`.
3. Study the publisher node structure.
4. Study the subscriber node structure.
5. Update `CMakeLists.txt` and `package.xml`.
6. Build with `colcon`.
7. Run talker and listener in separate terminals.
8. Use `ros2 topic list` and `ros2 topic echo` to inspect the communication.

## Main Takeaway
The `cpp_pubsub` tutorial teaches the core ROS 2 C++ programming model. A publisher node creates and sends messages on a topic, a subscriber node receives them through a callback, and the package is built with `ament_cmake` and `colcon`. Once you understand this pattern, you have the foundation for writing real ROS 2 robot software in C++.
