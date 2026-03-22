#include <cmath>
#include <iostream>
#include <thread>
#include <unistd.h>

#include <unitree/common/thread/thread.hpp>
#include <unitree/idl/hg/LowCmd_.hpp>
#include <unitree/idl/hg/LowState_.hpp>
#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>

// Bring the Unitree robot DDS helpers into the local namespace.
using namespace unitree::robot;

// Bring the Unitree thread helper types into the local namespace.
using namespace unitree::common;

// DDS topic used to send low-level motor commands.
#define TOPIC_LOWCMD "rt/lowcmd"

// DDS topic used to receive low-level robot state feedback.
#define TOPIC_LOWSTATE "rt/lowstate"

// Control loop period in seconds, which corresponds to 500 Hz.
constexpr double dt = 0.002;

// Enumerate the high-level motions this demo cycles through.
enum MotionState
{
    // Hold a neutral standing pose.
    STAND,

    // Alternate knee bending to imitate stepping.
    WALK,

    // Combine waist and arm motion.
    DANCE,

    // Lower both knees into a squat.
    SQUAT,

    // Wave the right arm.
    RIGHT_WAVE,

    // Wave the left arm.
    LEFT_WAVE,

    // Wave both arms together.
    BOTH_WAVE,

    // Swing both arms in opposite directions.
    ARM_SWING,

    // Raise one arm into a salute-like pose.
    SALUTE,

    // Move one arm in a handshake-like motion.
    HANDSHAKE
};

// Simple controller that publishes desired joint positions for G1.
class G1Controller
{
public:
    // Initialize commands, DDS channels, and the periodic control thread.
    void Init()
    {
        // Fill the outgoing command with default motor mode and gains.
        InitCmd();

        // Create the publisher that sends low-level motor commands.
        pub.reset(new ChannelPublisher<unitree_hg::msg::dds_::LowCmd_>(TOPIC_LOWCMD));

        // Open the command publisher channel.
        pub->InitChannel();

        // Create the subscriber that receives low-level motor states.
        sub.reset(new ChannelSubscriber<unitree_hg::msg::dds_::LowState_>(TOPIC_LOWSTATE));

        // Register the callback that copies each incoming state message.
        sub->InitChannel(std::bind(&G1Controller::StateHandler, this, std::placeholders::_1), 1);

        // Start a recurrent thread that runs the controller every dt seconds.
        thread = CreateRecurrentThreadEx("ctrl", UT_CPU_ID_NONE,
                                         int(dt * 1e6),
                                         &G1Controller::ControlLoop, this);
    }

    // Allow the motion sequence to start running.
    void Start()
    {
        // Enable the control loop body.
        running = true;

        // Print a small status message for the user.
        std::cout << "STARTED\n";
    }

    // Stop advancing the motion sequence.
    void Stop()
    {
        // Disable the control loop body.
        running = false;

        // Print a small status message for the user.
        std::cout << "STOPPED\n";
    }

private:
    // Outgoing low-level command packet.
    unitree_hg::msg::dds_::LowCmd_ cmd{};

    // Most recent low-level state packet.
    unitree_hg::msg::dds_::LowState_ state{};

    // Publisher used to write `cmd` onto DDS.
    ChannelPublisherPtr<unitree_hg::msg::dds_::LowCmd_> pub;

    // Subscriber used to receive state feedback.
    ChannelSubscriberPtr<unitree_hg::msg::dds_::LowState_> sub;

    // Background thread that executes the control loop.
    ThreadPtr thread;

    // Measured joint positions from the robot.
    double q[29]{};

    // Measured joint velocities from the robot.
    double dq[29]{};

    // Instantaneous target pose selected by the current motion state.
    double q_target[29]{};

    // Smoothed target pose actually sent to the motors.
    double q_smooth[29]{};

    // Controller time in seconds.
    double t = 0;

    // Gate used to keep the controller idle until the user starts it.
    bool running = false;

    // Current motion mode in the small demo state machine.
    MotionState state_machine = STAND;

    // Time at which the current motion mode started.
    double state_start_time = 0;

    // Time constant used by the low-pass smoothing filter.
    double tau = 0.08;

    // Configure default motor mode and gains for every joint.
    void InitCmd()
    {
        // Iterate over all 29 G1 joints.
        for (int i = 0; i < 29; i++)
        {
            // Enable normal position-control mode.
            cmd.motor_cmd()[i].mode() = 0x01;

            // Set proportional gain for position tracking.
            cmd.motor_cmd()[i].kp() = 40;

            // Set derivative gain for damping.
            cmd.motor_cmd()[i].kd() = 2;
        }
    }

    // Copy the latest measured joint state from DDS into local arrays.
    void StateHandler(const void *msg)
    {
        // Store the full incoming state message locally.
        state = *(unitree_hg::msg::dds_::LowState_ *)msg;

        // Copy each motor's position and velocity into compact arrays.
        for (int i = 0; i < 29; i++)
        {
            // Cache the joint position.
            q[i] = state.motor_state()[i].q();

            // Cache the joint velocity.
            dq[i] = state.motor_state()[i].dq();
        }
    }

    // Reset the whole body target to a neutral zero-angle posture.
    void Stand()
    {
        // Set every target joint angle to zero.
        for (int i = 0; i < 29; i++) q_target[i] = 0;
    }

    // Bend both knees to form a simple squat.
    void Squat()
    {
        // Start from the neutral standing posture.
        Stand();

        // Bend the left knee.
        q_target[3] = -0.6;

        // Bend the right knee.
        q_target[9] = -0.6;
    }

    // Alternate left and right knee flexion to suggest walking in place.
    void Walk()
    {
        // Start from the neutral standing posture.
        Stand();

        // Compute a sinusoidal phase value from controller time.
        double phase = sin(2 * t);

        // Bend the left knee only during the positive half cycle.
        q_target[3] = -0.4 * std::max(0.0, phase);

        // Bend the right knee only during the negative half cycle.
        q_target[9] = -0.4 * std::max(0.0, -phase);
    }

    // Combine waist sway with opposite arm motion for a playful dance.
    void Dance()
    {
        // Start from the neutral standing posture.
        Stand();

        // Sway the waist a little.
        q_target[1] = 0.2 * sin(2 * t);

        // Move one arm forward and back.
        q_target[20] = 0.5 * sin(4 * t);

        // Move the opposite arm in the reverse direction.
        q_target[16] = -0.5 * sin(4 * t);
    }

    // Raise the right arm and oscillate the wrist/elbow joint for a wave.
    void RightWave()
    {
        // Start from the neutral standing posture.
        Stand();

        // Lift the right shoulder/arm joint.
        q_target[20] = 0.4;

        // Bend the next right arm joint.
        q_target[21] = -0.6;

        // Oscillate the end joint to create the wave motion.
        q_target[22] = 0.5 * sin(4 * t);
    }

    // Raise the left arm and oscillate the wrist/elbow joint for a wave.
    void LeftWave()
    {
        // Start from the neutral standing posture.
        Stand();

        // Lift the left shoulder/arm joint.
        q_target[16] = 0.4;

        // Bend the next left arm joint.
        q_target[17] = -0.6;

        // Oscillate the end joint to create the wave motion.
        q_target[18] = 0.5 * sin(4 * t);
    }

    // Raise both arms and wave them in opposite directions.
    void BothWave()
    {
        // Start from the neutral standing posture.
        Stand();

        // Lift the right shoulder/arm joint.
        q_target[20] = 0.4;

        // Bend the next right arm joint.
        q_target[21] = -0.6;

        // Oscillate the right end joint.
        q_target[22] = 0.5 * sin(4 * t);

        // Lift the left shoulder/arm joint.
        q_target[16] = 0.4;

        // Bend the next left arm joint.
        q_target[17] = -0.6;

        // Oscillate the left end joint in the opposite direction.
        q_target[18] = -0.5 * sin(4 * t);
    }

    // Swing both arms back and forth while keeping them slightly bent.
    void ArmSwing()
    {
        // Start from the neutral standing posture.
        Stand();

        // Compute the shared swing value.
        double s = sin(3 * t);

        // Move one arm with the sine wave.
        q_target[20] = 0.5 * s;

        // Move the other arm in the opposite direction.
        q_target[16] = -0.5 * s;

        // Keep the right arm slightly bent.
        q_target[21] = -0.4;

        // Keep the left arm slightly bent.
        q_target[17] = -0.4;
    }

    // Place the right arm into a salute-like pose.
    void Salute()
    {
        // Start from the neutral standing posture.
        Stand();

        // Raise the right arm.
        q_target[20] = 0.6;

        // Bend the next joint more sharply.
        q_target[21] = -1.0;

        // Angle the last joint slightly.
        q_target[22] = 0.2;
    }

    // Place the right arm into a handshake pose and oscillate the hand.
    void Handshake()
    {
        // Start from the neutral standing posture.
        Stand();

        // Raise the right arm.
        q_target[20] = 0.5;

        // Bend the elbow-like joint.
        q_target[21] = -0.8;

        // Oscillate the end joint to mimic a handshake.
        q_target[22] = 0.3 * sin(5 * t);
    }

    // Advance the demo state machine based on how long the current motion ran.
    void UpdateState()
    {
        // Compute elapsed time since the last state transition.
        double e = t - state_start_time;

        // Stay in STAND for 2 seconds, then start walking.
        if (state_machine == STAND && e > 2) Switch(WALK);

        // Stay in WALK for 4 seconds, then wave the right arm.
        else if (state_machine == WALK && e > 4) Switch(RIGHT_WAVE);

        // Stay in RIGHT_WAVE for 4 seconds, then wave the left arm.
        else if (state_machine == RIGHT_WAVE && e > 4) Switch(LEFT_WAVE);

        // Stay in LEFT_WAVE for 4 seconds, then wave both arms.
        else if (state_machine == LEFT_WAVE && e > 4) Switch(BOTH_WAVE);

        // Stay in BOTH_WAVE for 4 seconds, then swing both arms.
        else if (state_machine == BOTH_WAVE && e > 4) Switch(ARM_SWING);

        // Stay in ARM_SWING for 4 seconds, then dance.
        else if (state_machine == ARM_SWING && e > 4) Switch(DANCE);

        // Stay in DANCE for 5 seconds, then salute.
        else if (state_machine == DANCE && e > 5) Switch(SALUTE);

        // Stay in SALUTE for 3 seconds, then handshake.
        else if (state_machine == SALUTE && e > 3) Switch(HANDSHAKE);

        // Stay in HANDSHAKE for 4 seconds, then squat.
        else if (state_machine == HANDSHAKE && e > 4) Switch(SQUAT);

        // Stay in SQUAT for 3 seconds, then return to stand.
        else if (state_machine == SQUAT && e > 3) Switch(STAND);
    }

    // Record a new state machine mode and reset its timer.
    void Switch(MotionState s)
    {
        // Store the new motion state.
        state_machine = s;

        // Record the time of this transition.
        state_start_time = t;

        // Print the numeric state id for simple debugging.
        std::cout << "State -> " << s << std::endl;
    }

    // Smooth abrupt target changes before sending them to the robot.
    void Smooth()
    {
        // Convert the time constant into a per-step blend factor.
        double alpha = dt / tau;

        // Process every joint command.
        for (int i = 0; i < 29; i++)
        {
            // Move the smoothed command a small step toward the raw target.
            q_smooth[i] += alpha * (q_target[i] - q_smooth[i]);

            // Clamp the outgoing command to a conservative range.
            if (q_smooth[i] > 1.5) q_smooth[i] = 1.5;

            // Clamp the outgoing command to a conservative range.
            if (q_smooth[i] < -1.5) q_smooth[i] = -1.5;
        }
    }

    // Main periodic control loop executed by the background thread.
    void ControlLoop()
    {
        // Advance the controller clock.
        t += dt;

        // Do nothing until the user starts the sequence.
        if (!running) return;

        // Check whether it is time to move to the next motion state.
        UpdateState();

        // Select the target posture for the current motion state.
        switch (state_machine)
        {
            // Command the neutral standing pose.
            case STAND: Stand(); break;

            // Command the walk-in-place pose.
            case WALK: Walk(); break;

            // Command the dance pose.
            case DANCE: Dance(); break;

            // Command the squat pose.
            case SQUAT: Squat(); break;

            // Command the right-hand wave pose.
            case RIGHT_WAVE: RightWave(); break;

            // Command the left-hand wave pose.
            case LEFT_WAVE: LeftWave(); break;

            // Command the both-hands wave pose.
            case BOTH_WAVE: BothWave(); break;

            // Command the arm swing pose.
            case ARM_SWING: ArmSwing(); break;

            // Command the salute pose.
            case SALUTE: Salute(); break;

            // Command the handshake pose.
            case HANDSHAKE: Handshake(); break;
        }

        // Filter the target pose to avoid sharp changes.
        Smooth();

        // Copy the smoothed pose into the outgoing DDS command.
        for (int i = 0; i < 29; i++)
        {
            // Set the desired position for this joint.
            cmd.motor_cmd()[i].q() = q_smooth[i];

            // Keep desired velocity at zero for simple position control.
            cmd.motor_cmd()[i].dq() = 0;

            // Do not use feedforward torque in this demo.
            cmd.motor_cmd()[i].tau() = 0;
        }

        // Publish the command packet to the robot.
        pub->Write(cmd);
    }
};

// Entry point for the demo program.
int main(int argc, const char **argv)
{
    // Use loopback if no network interface is provided.
    if (argc < 2)
        ChannelFactory::Instance()->Init(1, "lo");

    // Otherwise initialize DDS on the requested network interface.
    else
        ChannelFactory::Instance()->Init(0, argv[1]);

    // Create the controller object.
    G1Controller robot;

    // Set up the controller internals.
    robot.Init();

    // Show the user the available keyboard commands.
    std::cout << "\nPress 's' to START, 'q' to QUIT\n";

    // Keep reading commands until the user quits.
    while (true)
    {
        // Read one character from standard input.
        char c;

        // Wait for the next key command.
        std::cin >> c;

        // Start the controller if the user presses s.
        if (c == 's')
            robot.Start();

        // Exit the program if the user presses q.
        else if (c == 'q')
            break;
    }

    // Return success to the shell.
    return 0;
}
