---

# Running the Example in Simulation

Before executing the program, ensure that the Unitree G1 simulation environment is already running and publishing the required DDS topics.

This example is designed to work with the **Unitree MuJoCo simulator** using the loopback (`lo`) network interface.

## Step 1 – Start the Unitree G1 Simulation

Launch the Unitree G1 simulation following the installation procedure described in the previous lesson.

Once the simulator is running, verify that the robot is standing correctly and that the simulation is stable.

---

## Step 2 – Open a New Terminal

Open another terminal window and navigate to your project directory containing the Python examples.

For example:

```bash
cd ~/ROS2/examples_unitree_python_sdk
```

Adjust the directory if your examples are stored in a different location.

---

## Step 3 – Run the Example

Execute the program using the loopback interface.

```bash
python3 low_level_example_2.py
```

or explicitly specify the loopback interface:

```bash
python3 low_level_example_2.py lo
```

The loopback interface automatically uses **DDS Domain ID 1**, which is commonly used for local simulation.

The terminal should display messages similar to:

```text
Using DDS domain 1 on interface 'lo'

Waiting for low-level robot state...

State received.

Starting simulation dance sequence.
```

---

## Step 4 – Observe the Robot Motion

After the first robot state is received, the controller automatically begins executing the motion sequence.

The robot should perform the following stages:

1. Move to the neutral pose
2. Raise both arms into a T-pose
3. Perform an elbow wave
4. Cross both arms
5. Return to the neutral posture
6. Twist the waist
7. Lean from side to side
8. Move into the final dance pose
9. Return to the zero pose
10. Release SDK control

The entire sequence runs automatically without further user interaction.

---

# Running on Real Hardware

This example is intended primarily for simulation because it contains large upper-body motions.

If you intentionally want to run it on a physical Unitree G1 robot, specify the robot's network interface together with the hardware override option.

Example:

```bash
python3 low_level_example_2.py eth0 --allow-hardware
```

where:

- `eth0` is the Ethernet interface connected to the robot.
- `--allow-hardware` disables the built-in safety protection that prevents accidental execution on real hardware.

> **Warning**
>
> Always test new low-level controllers in simulation before running them on a physical robot. Large or incorrect joint commands may lead to unstable robot behavior or hardware damage.

---

# Stopping the Program

The controller runs until the complete motion sequence finishes.

To stop the program manually, press:

```text
Ctrl + C
```

When interrupted, the controller immediately publishes a final command that releases the SDK control weight before exiting.

This prevents the robot from remaining under low-level SDK control after the application closes.

---

# Expected Terminal Output

A successful execution typically produces output similar to the following:

```text
Using DDS domain 1 on interface 'lo'

Waiting for low-level robot state...

State received.

Stage 1: moving upper body to zero.

Stage 2: moving to wide T-pose.

Stage 3: elbow wave dance.

Stage 4: crossing arms.

Stage 5: opening arms back to neutral.

Stage 6: waist twist with arms raised.

Stage 7: side lean with arms out.

Stage 8: final dance pose.

Stage 9: returning to zero.

Stage 10: releasing arm SDK control weight.

Dance sequence complete.
```

If you do not see these messages, verify:

- The Unitree simulator is running.
- DDS communication has been initialized correctly.
- The loopback interface (`lo`) is being used.
- The simulator is publishing the `rt/lowstate` topic.

---