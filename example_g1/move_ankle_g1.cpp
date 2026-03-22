#include <cmath>
#include <iostream>
#include <thread>
#include <unistd.h>

#include <unitree/common/thread/thread.hpp>
#include <unitree/idl/hg/LowCmd_.hpp>
#include <unitree/idl/hg/LowState_.hpp>
#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>

using namespace unitree::common;
using namespace unitree::robot;

// Publish joint commands on the low-level command topic.
#define TOPIC_LOWCMD "rt/lowcmd"

// Subscribe to measured joint states from the low-level state topic.
#define TOPIC_LOWSTATE "rt/lowstate"

// Run the controller at 500 Hz, which matches the existing example.
constexpr double kDt = 0.002;

// G1 exposes 29 controllable joints in the 29-DOF configuration.
constexpr int kMotorCount = 29;

// These indices come from Unitree's G1 ankle example.
constexpr int kLeftAnklePitch = 4;
constexpr int kRightAnklePitch = 10;

class G1AnkleController
{
public:
    // Set up command defaults, DDS channels, and the periodic control thread.
    void Init()
    {
        InitCmd();

        // Create the publisher used to send low-level joint commands.
        pub_.reset(new ChannelPublisher<unitree_hg::msg::dds_::LowCmd_>(TOPIC_LOWCMD));
        pub_->InitChannel();

        // Create the subscriber used to receive the latest measured robot state.
        sub_.reset(new ChannelSubscriber<unitree_hg::msg::dds_::LowState_>(TOPIC_LOWSTATE));
        sub_->InitChannel(std::bind(&G1AnkleController::StateHandler, this, std::placeholders::_1), 1);

        // Start the recurrent control loop thread.
        thread_ = CreateRecurrentThreadEx("ankle_ctrl", UT_CPU_ID_NONE,
                                          int(kDt * 1e6),
                                          &G1AnkleController::ControlLoop, this);
    }

    // Arm the controller after the user confirms it is safe to move.
    void Start()
    {
        running_ = true;
        std::cout << "Ankle controller started\n";
    }

private:
    // Outgoing low-level motor command message.
    unitree_hg::msg::dds_::LowCmd_ cmd_{};

    // Latest low-level state message received from the robot.
    unitree_hg::msg::dds_::LowState_ state_{};

    // DDS publisher handle.
    ChannelPublisherPtr<unitree_hg::msg::dds_::LowCmd_> pub_;

    // DDS subscriber handle.
    ChannelSubscriberPtr<unitree_hg::msg::dds_::LowState_> sub_;

    // Periodic control loop thread handle.
    ThreadPtr thread_;

    // Current measured joint positions.
    double q_[kMotorCount]{};

    // Smoothed command sent to each joint.
    double q_cmd_[kMotorCount]{};

    // Target command before smoothing.
    double q_target_[kMotorCount]{};

    // Controller time in seconds.
    double t_ = 0.0;

    // Gate that prevents motion until the user explicitly starts it.
    bool running_ = false;

    // Smoothing time constant for gentle command changes.
    double tau_ = 0.08;

    // Fill in default command mode and gains for all joints.
    void InitCmd()
    {
        for (int i = 0; i < kMotorCount; ++i)
        {
            // Enable position control mode for each motor.
            cmd_.motor_cmd()[i].mode() = 0x01;

            // Use moderate stiffness for a simple demo.
            cmd_.motor_cmd()[i].kp() = 40.0;

            // Use light damping to reduce oscillation.
            cmd_.motor_cmd()[i].kd() = 2.0;
        }
    }

    // Store the latest measured joint positions from the robot.
    void StateHandler(const void *msg)
    {
        // Copy the DDS message into our local state object.
        state_ = *(unitree_hg::msg::dds_::LowState_ *)msg;

        for (int i = 0; i < kMotorCount; ++i)
        {
            // Cache the current joint angle for initialization and feedback.
            q_[i] = state_.motor_state()[i].q();
        }
    }

    // Default all joints to zero so only the ankle joints are moved on purpose.
    void SetNeutralPose()
    {
        for (int i = 0; i < kMotorCount; ++i)
        {
            // Ask every joint to stay at the neutral position.
            q_target_[i] = 0.0;
        }
    }

    // Move only the ankle pitch joints with a small sinusoidal motion.
    void SetAnkleMotion()
    {
        // Start from the neutral pose each cycle.
        SetNeutralPose();

        // Limit the motion to about 8 degrees for a gentle demo.
        const double amplitude = 8.0 * M_PI / 180.0;

        // Move slowly at 0.5 Hz so the ankle motion is easy to observe.
        const double ankle_pitch = amplitude * std::sin(2.0 * M_PI * 0.5 * t_);

        // Apply the same pitch command to the left ankle.
        q_target_[kLeftAnklePitch] = ankle_pitch;

        // Apply the same pitch command to the right ankle.
        q_target_[kRightAnklePitch] = ankle_pitch;
    }

    // Blend toward the target command instead of stepping abruptly.
    void SmoothCommands()
    {
        // Compute the first-order filter coefficient for this loop period.
        const double alpha = kDt / tau_;

        for (int i = 0; i < kMotorCount; ++i)
        {
            // Move a small step from the current command toward the target.
            q_cmd_[i] += alpha * (q_target_[i] - q_cmd_[i]);
        }
    }

    // Periodic control loop that prepares and publishes motor commands.
    void ControlLoop()
    {
        // Advance the local controller clock.
        t_ += kDt;

        // Wait until the user explicitly starts the motion.
        if (!running_)
        {
            return;
        }

        // Hold the current measured joint positions for the first second.
        if (t_ < 1.0)
        {
            for (int i = 0; i < kMotorCount; ++i)
            {
                // Use the live measured position as the temporary target.
                q_target_[i] = q_[i];

                // Initialize the smoothed command from the same value.
                q_cmd_[i] = q_[i];
            }
        }
        else
        {
            // After the hold period, command the ankle pitch motion.
            SetAnkleMotion();

            // Smooth the outgoing command so the motion ramps gently.
            SmoothCommands();
        }

        for (int i = 0; i < kMotorCount; ++i)
        {
            // Send the desired position for each joint.
            cmd_.motor_cmd()[i].q() = q_cmd_[i];

            // Keep desired joint velocity at zero for position control.
            cmd_.motor_cmd()[i].dq() = 0.0;

            // Do not add feedforward torque in this simple example.
            cmd_.motor_cmd()[i].tau() = 0.0;
        }

        // Publish the new command packet to the robot.
        pub_->Write(cmd_);
    }
};

int main(int argc, const char **argv)
{
    // Use loopback by default when no network interface argument is provided.
    if (argc < 2)
    {
        ChannelFactory::Instance()->Init(1, "lo");
    }
    else
    {
        // Use the provided network interface for DDS traffic.
        ChannelFactory::Instance()->Init(0, argv[1]);
    }

    // Create the controller object.
    G1AnkleController robot;

    // Initialize DDS channels and the control thread.
    robot.Init();

    // Explain the single command this small demo accepts.
    std::cout << "\nPress 's' to start ankle motion, 'q' to quit\n";

    while (true)
    {
        // Read one character from standard input.
        char c;
        std::cin >> c;

        // Start moving the ankle when the user presses s.
        if (c == 's')
        {
            robot.Start();
        }
        else if (c == 'q')
        {
            // Exit the program when the user presses q.
            break;
        }
    }

    // Return success to the shell.
    return 0;
}
