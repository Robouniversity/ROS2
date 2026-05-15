# 07 - ROS 2 Concepts 1

## Lesson Goal
Understand the architecture of ROS 1 and ROS 2, why ROS 2 changed the communication model, and learn the main ROS 2 concepts in enough detail to read, run, and design real robot software.

## Key Links
- ROS project homepage: https://www.ros.org/
- ROS 2 Foxy concepts index: https://docs.ros.org/en/foxy/Concepts.html
- ROS 2 Foxy tutorials index: https://docs.ros.org/en/foxy/Tutorials.html
- ROS 2 nodes concept: https://docs.ros.org/en/foxy/Concepts/About-Nodes.html
- ROS 2 topics concept: https://docs.ros.org/en/foxy/Concepts/About-Topics.html
- ROS 2 services concept: https://docs.ros.org/en/foxy/Concepts/About-Services.html
- ROS 2 actions concept: https://docs.ros.org/en/foxy/Concepts/About-Actions.html
- ROS 2 parameters concept: https://docs.ros.org/en/foxy/Concepts/About-ROS-2-Parameters.html
- ROS 2 QoS concept: https://docs.ros.org/en/foxy/Concepts/About-Quality-of-Service-Settings.html
- ROS 2 interfaces concept: https://docs.ros.org/en/foxy/Concepts/About-ROS-Interfaces.html
- ROS 2 middleware concept: https://docs.ros.org/en/foxy/Concepts/About-Middleware-Implementations.html
- ROS 2 executors concept: https://docs.ros.org/en/foxy/Concepts/About-Executors.html
- ROS 2 launch system: https://docs.ros.org/en/foxy/Tutorials/Intermediate/Launch/Launch-system.html
- ROS 2 tf2 concept: https://docs.ros.org/en/foxy/Tutorials/Intermediate/Tf2/Introduction-To-Tf2.html

## Core Notes
ROS is a robotics software framework and ecosystem. It gives robot developers a standard way to structure software into modular processes that communicate through well-defined interfaces.

The big idea is simple:

- split robot software into small modules
- let those modules communicate through standard patterns
- make the system observable, debuggable, and reusable

ROS 2 keeps that same philosophy, but changes the communication foundation and many system-level assumptions so it works better for modern robotics.

## ROS 1 Architecture
The easiest way to think about ROS 1 is:

- many nodes
- one ROS master
- named topics and services
- message passing over a ROS communication stack

Main ROS 1 architecture pieces:

- `roscore`: starts the master and core coordination services
- ROS master: handles name registration and discovery
- nodes: independent processes that do work
- topics: streaming communication channels
- services: request-response communication
- parameter server: shared configuration storage
- messages: typed data structures passed between nodes

How ROS 1 communication works at a high level:

1. A node registers with the ROS master.
2. A publisher advertises a topic.
3. A subscriber asks the master who publishes that topic.
4. The master returns connection information.
5. The nodes then communicate directly.

Why ROS 1 became popular:

- very easy to learn
- huge package ecosystem
- excellent tools for visualization and debugging
- made robotics software modular and collaborative

Main ROS 1 limitations:

- central dependency on the master for discovery
- weaker support for real-time and embedded systems
- limited QoS control
- weaker support for distributed industrial systems
- security was not a core design priority
- parameter server was centralized

ROS 1 was amazing for research and education, but larger and more demanding robot systems needed a more modern communication model.

## ROS 2 Architecture
ROS 2 keeps the node-based programming model, but changes the underlying communication layer.

The easiest way to think about ROS 2 is:

- many nodes
- no single ROS master required
- discovery handled by DDS
- communication behavior controlled by QoS
- better support for distributed, embedded, and production-style systems

Main ROS 2 architecture pieces:

- nodes
- topics
- services
- actions
- parameters
- interfaces
- DDS-based middleware
- client libraries such as `rclcpp` and `rclpy`
- executors and callback groups
- launch system
- lifecycle support for managed nodes

The most important architecture change is this:

- ROS 1 uses a master-based discovery model
- ROS 2 uses DDS-based peer discovery

That means ROS 2 nodes do not need a central master to find each other in the same way ROS 1 did.

## ROS 1 vs ROS 2
High-level comparison:

- ROS 1: simpler original model, master-based discovery, very common in research
- ROS 2: DDS-based, more configurable, better for modern distributed robot systems

Main differences:

- Discovery:
  ROS 1 depends on the ROS master.
  ROS 2 uses DDS discovery.

- Communication quality:
  ROS 1 has simpler communication behavior.
  ROS 2 adds QoS policies such as reliability, durability, and history.

- Middleware:
  ROS 1 uses its own communication stack.
  ROS 2 uses DDS through an RMW layer.

- Parameters:
  ROS 1 uses a centralized parameter server.
  ROS 2 parameters belong to nodes.

- Long-running commands:
  ROS 1 mainly used topics or services for many tasks.
  ROS 2 adds actions as a first-class pattern.

- Real-time and embedded support:
  ROS 2 is designed with these needs more seriously in mind.

- Security:
  ROS 2 has a better path for secure deployments because of DDS-related security support and updated system design.

## Layered View of ROS 2
It helps to imagine ROS 2 as layers:

1. Application layer
   Your nodes, launch files, robot logic, control software, perception, planning, dashboards.

2. Client library layer
   `rclcpp` for C++, `rclpy` for Python, and other language bindings.

3. ROS client library core
   The common ROS client interfaces under the user-facing APIs.

4. RMW layer
   The ROS middleware abstraction layer.

5. DDS implementation
   Cyclone DDS, Fast DDS, Connext, and others.

6. Transport/network layer
   UDP, multicast, discovery traffic, and machine-to-machine communication.

This is why people often say:

- ROS 2 application code does not talk directly to DDS
- it talks to ROS client libraries
- which talk to RMW
- which talks to DDS

## Node
A node is a single executable process in a ROS graph.

A node usually does one focused job:

- read a sensor
- publish joint states
- process camera data
- compute control outputs
- visualize data
- bridge hardware to higher-level software

Why nodes matter:

- they keep systems modular
- they make debugging easier
- they allow separate pieces to be restarted or replaced
- they let different teams own different parts of the robot stack

Good node design mindset:

- one clear responsibility per node
- communicate through interfaces, not direct code coupling
- keep inputs and outputs explicit

## Topic
A topic is a named bus for streaming data.

Topics are for continuous or repeated information such as:

- camera images
- IMU data
- laser scans
- robot state
- velocity commands

Topic communication pattern:

- publisher sends messages
- subscriber receives messages
- publisher and subscriber do not need to know each other directly

Why topics are useful:

- many subscribers can observe the same data
- publishers and subscribers stay loosely coupled
- the system becomes easy to inspect with command-line and visualization tools

Typical examples:

- `/camera/image_raw`
- `/joint_states`
- `/cmd_vel`

## Service
A service is request-response communication.

Use a service when:

- a client asks for one operation
- a server returns one response
- the task is short and immediate

Examples:

- reset a sensor
- request map metadata
- toggle a mode

Service mindset:

- use services for quick transactional behavior
- do not use services for long-running tasks

## Action
An action is for long-running goals with feedback and cancellation.

An action includes:

- goal
- feedback
- result
- cancel support

Use actions when the task takes time, such as:

- navigate to a goal
- move a robot arm to a pose
- execute a manipulation sequence

Why actions matter:

- topics are too loose for tracked goal execution
- services are too short-lived for progress-aware operations
- actions are built for command workflows that need status and control

## Parameter
Parameters are configuration values associated with a node.

Examples:

- control gains
- topic names
- frame names
- robot dimensions
- thresholds

ROS 2 parameter mindset:

- parameters belong to nodes
- they are not stored in one global central server like ROS 1
- they can often be declared, read, updated, and monitored at runtime

This makes configuration more local and explicit.

## Message, Service, and Action Interfaces
ROS communication depends on typed interfaces.

The three main interface categories are:

- messages: for topics
- services: for request-response
- actions: for goal-feedback-result workflows

Why typed interfaces matter:

- both sides know the exact data structure
- tools can inspect and validate communication
- teams can share common robot data definitions

Examples of interface content:

- position, velocity, and effort arrays
- pose and twist data
- command flags
- task goal/result structures

## Package
A package is the basic organizational unit in ROS.

A package usually contains some combination of:

- nodes
- launch files
- interface definitions
- configuration files
- scripts
- CMake or Python build metadata

Why packages matter:

- they make code reusable
- they define dependencies clearly
- they are the unit you build and distribute

## Workspace
A workspace is a directory structure where multiple ROS packages are developed and built together.

Typical workspace idea:

- source code in `src`
- build outputs in `build`
- installed artifacts in `install`
- logs in `log`

Why workspaces matter:

- they let you build many related packages together
- they make overlay development possible
- they help you test local changes without modifying system packages

## Colcon
`colcon` is the standard build tool commonly used with ROS 2 workspaces.

What it does:

- discovers packages
- resolves build order
- builds packages
- creates install spaces

Common idea:

- source the ROS environment
- build with `colcon`
- source the workspace install setup script

## Launch System
The launch system starts and configures multiple nodes together.

Use launch files when you need to:

- start many nodes at once
- pass parameters
- remap topic names
- organize full robot application startup

Why launch matters:

- real robots rarely run just one node
- launch files capture system-level startup structure

## Names
ROS systems use names heavily.

Examples:

- node names
- topic names
- service names
- action names
- parameter names

Why names matter:

- they define how components find and refer to each other
- they help separate subsystems
- they support namespaces for larger robot systems

## Namespace
A namespace groups names under a prefix.

Examples:

- `/robot1/cmd_vel`
- `/robot2/cmd_vel`

Why namespaces matter:

- useful for multi-robot systems
- useful for separating subsystems
- reduce naming collisions

## Remapping
Remapping changes names at runtime without changing the source code.

This is useful when:

- the same node is reused in different robots
- you need to connect a node to a differently named topic
- you want flexibility without editing code

## QoS: Quality of Service
QoS is one of the most important ROS 2 concepts.

QoS controls how data is delivered.

Main QoS ideas:

- reliability
- durability
- history
- depth
- liveliness
- deadline

Examples:

- reliable delivery is important for critical control or command data
- best-effort delivery may be better for high-rate sensor streams where dropping old data is acceptable
- transient local durability helps late-joining subscribers receive the last stored message

Why QoS matters:

- robotics data is not all the same
- camera streams, control commands, maps, and state updates have different communication needs
- ROS 2 lets you tune that behavior explicitly

This is one of the biggest conceptual upgrades from ROS 1.

## DDS and RMW
DDS stands for Data Distribution Service.

ROS 2 uses DDS as the communication middleware layer.

RMW stands for ROS Middleware interface.

That means:

- your node code uses ROS APIs
- ROS uses the RMW abstraction
- the RMW implementation talks to a DDS vendor implementation

Common point to remember:

- DDS is the transport/middleware technology
- RMW is the ROS abstraction over that middleware

In this repository, the middleware choice is:

- `rmw_cyclonedds_cpp`

That means the ROS 2 stack is configured around Cyclone DDS.

## Executor
An executor is the mechanism that decides how callbacks are processed.

Callbacks can come from:

- subscriptions
- timers
- services
- actions
- parameter events

Why executors matter:

- they affect concurrency
- they affect responsiveness
- they influence how node callbacks are scheduled

Two common ideas:

- single-threaded execution
- multi-threaded execution

## Callback Group
Callback groups let you control which callbacks can run concurrently.

This matters when:

- multiple callbacks share resources
- some callbacks must not run in parallel
- you need more control over node concurrency

## Timer
A timer lets a node run code periodically.

Typical uses:

- publish sensor polling data
- run a control loop
- print diagnostics
- perform periodic checks

## TF2
`tf2` is the transform library used to track coordinate frames over time.

Examples of frames:

- `map`
- `odom`
- `base_link`
- `camera_link`
- `laser`

Why `tf2` matters:

- robots reason about spatial relationships constantly
- sensors and actuators live in different frames
- perception and control need consistent coordinate transforms

Without `tf2`, multi-sensor robotics becomes very difficult to manage.

## Lifecycle Nodes
Lifecycle nodes are managed nodes with explicit states.

Typical states include ideas like:

- unconfigured
- inactive
- active
- finalized

Why lifecycle nodes matter:

- startup becomes more controlled
- systems become easier to supervise
- large deployments can activate and deactivate components safely

This is especially useful in more production-like robot systems.

## ROS Graph
The ROS graph is the live network of nodes and their communication relationships.

It includes:

- which nodes exist
- which topics exist
- who publishes and subscribes
- what services and actions are available

Why the graph matters:

- it makes the system inspectable
- it supports debugging and discovery
- it helps developers understand the running robot software

## Introspection and Tools
One of ROS's biggest strengths is introspection.

Useful tooling ideas:

- list nodes
- inspect topics
- check message types
- echo data
- visualize transforms
- inspect parameters

This is a major reason ROS is so productive for robotics development.

## How These Concepts Fit Together
A practical mental model:

1. A package contains one or more nodes.
2. Nodes run inside a workspace.
3. Nodes communicate using topics, services, and actions.
4. Messages, services, and actions define the typed interfaces.
5. Parameters configure node behavior.
6. Launch files start the whole system.
7. Executors and callback groups control runtime behavior.
8. DDS and QoS control communication behavior underneath.
9. `tf2` manages spatial frame relationships.

## How This Applies to Unitree G1
In the Unitree G1 workflow from this repository:

- Unitree packages provide robot-specific nodes and interfaces
- ROS 2 provides the communication model
- Cyclone DDS provides the middleware path
- the dev container provides a repeatable workspace
- your applications become additional ROS 2 nodes on top of that stack

So when you work with Unitree G1, you are not just learning one robot API. You are learning how robot-specific software plugs into the general ROS 2 architecture.

## What To Understand From This Video
- I can explain the core architectural difference between ROS 1 and ROS 2.
- I know why DDS and QoS are important in ROS 2.
- I understand nodes, topics, services, actions, parameters, packages, and workspaces.
- I understand how launch, executors, callback groups, and `tf2` fit into real systems.
- I can relate the abstract ROS 2 model to the Unitree G1 development workflow.

## Quick Checklist / Takeaway
- ROS 1 is master-based; ROS 2 is DDS-based.
- Topics stream data, services do request-response, and actions handle long-running goals.
- Parameters configure nodes, packages organize code, and workspaces organize builds.
- QoS, DDS, executors, and `tf2` are key ROS 2 concepts for real robot systems.
