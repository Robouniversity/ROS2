# 02 - What Is ROS 2

## Lesson Goal
Understand where ROS came from, why ROS 2 was created, and how the project evolved into the middleware stack used today in robotics platforms like Unitree G1.

## Key Links
- ROS project homepage: https://www.ros.org/
- ROS 2 Foxy documentation: https://docs.ros.org/en/foxy/
- ROS 2 Foxy installation docs: https://docs.ros.org/en/foxy/Installation.html
- ROS 2 concepts index: https://docs.ros.org/en/foxy/Concepts.html
- ROS 2 tutorials index: https://docs.ros.org/en/foxy/Tutorials.html
- Unitree ROS 2 repository: https://github.com/unitreerobotics/unitree_ros2

## Core Notes
ROS began as a robotics software project from Willow Garage. The goal was to give robot developers a common framework for building software without every team needing to invent its own communication system, tooling, package layout, and developer workflow from scratch.

Over time, ROS became widely adopted in research and development because it made robotics software more modular, more shareable, and easier to inspect. As robots became more complex and more commercial systems needed stronger reliability, security, and distributed communication, ROS 2 was developed as the next generation of the ecosystem.

Useful timeline to remember:

- Early ROS work started in the late 2000s under Willow Garage.
- ROS 1 became the standard learning and research platform for many robotics teams.
- ROS 2 was started to address limitations in ROS 1, especially around production-grade communication, real-time support, multi-robot systems, and long-term maintainability.
- The ROS ecosystem later came under Open Robotics stewardship, helping standardize and grow the platform further.
- ROS 2 distributions such as Foxy, Humble, and newer releases reflect the project’s long-term release model.

Why ROS 2 was needed:

- ROS 1 was extremely influential, but it was designed in an earlier stage of robotics development.
- Modern robots need stronger support for distributed systems, QoS control, embedded targets, and industrial/commercial reliability.
- ROS 2 adopted DDS-based communication to support more serious deployment scenarios.
- This shift made ROS 2 more suitable for real robots and not only lab demos or academic prototypes.

The ROS equation:

- A simple way to think about ROS is: hardware + middleware + tools + reusable packages + community.
- ROS is not just a library. It is an ecosystem that includes communication, package standards, build workflows, visualization tools, simulation hooks, debugging tools, and shared conventions.
- ROS 2 extends that idea by replacing older communication assumptions with a more modern distributed systems foundation.

Why ROS became so important:

- It lowered the barrier to entry for robot software development.
- It created a common language across labs, startups, and robotics companies.
- It made software reuse normal in robotics, which dramatically sped up learning and prototyping.
- It helped robotics development feel more like software engineering and less like isolated one-off system integration.

How this connects to Unitree G1 and this repo:

- Unitree provides robot-specific packages, but ROS 2 gives the communication model.
- This repository uses ROS 2 Foxy as the software foundation around the Unitree G1 stack.
- The reason that matters is historical as well as technical: ROS 2 is the ecosystem that lets vendor-specific robot software plug into a standard robotics workflow.
- That is why learning the project background is useful before diving into commands and APIs.

In this repository, ROS 2 Foxy is the chosen distro because the container and setup scripts are designed around it. The README also shows that Cyclone DDS is the selected ROS middleware implementation.

## Quick Checklist / Takeaway
- I know ROS started before ROS 2 and understand why ROS 2 was created.
- I can explain the broad ROS timeline from Willow Garage to modern ROS 2 releases.
- I understand the ROS equation as an ecosystem, not just a coding library.
- I know this repo uses ROS 2 Foxy and Cyclone DDS for the Unitree G1 workflow.
