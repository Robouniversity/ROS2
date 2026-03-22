#include <array>
#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "unitree_hg/msg/imu_state.hpp"
#include "unitree_hg/msg/low_state.hpp"
#include "unitree_hg/msg/motor_state.hpp"

// Match the high-frequency topic choice used by Unitree's reference examples.
constexpr bool kHighFrequency = true;

// G1 29-DOF robots expose 29 low-level motor slots.
constexpr int kG1MotorCount = 29;

// Joint indices copied from the Unitree G1 reference example.
enum G1JointIndex
{
  LEFT_HIP_PITCH = 0,
  LEFT_HIP_ROLL = 1,
  LEFT_HIP_YAW = 2,
  LEFT_KNEE = 3,
  LEFT_ANKLE_PITCH = 4,
  LEFT_ANKLE_ROLL = 5,
  RIGHT_HIP_PITCH = 6,
  RIGHT_HIP_ROLL = 7,
  RIGHT_HIP_YAW = 8,
  RIGHT_KNEE = 9,
  RIGHT_ANKLE_PITCH = 10,
  RIGHT_ANKLE_ROLL = 11
};

// Minimal ROS 2 node based on unitree_ros2/example/src/src/read_low_state_hg.cpp.
class G1BasicLowStateNode : public rclcpp::Node
{
public:
  G1BasicLowStateNode() : Node("g1_basic_low_state")
  {
    const char *topic_name = kHighFrequency ? "lowstate" : "lf/lowstate";

    subscription_ = this->create_subscription<unitree_hg::msg::LowState>(
        topic_name, 10,
        [this](const unitree_hg::msg::LowState::SharedPtr message) {
          HandleLowState(message);
        });

    // Print a compact summary once per second instead of logging every packet.
    summary_timer_ = this->create_wall_timer(
        std::chrono::seconds(1),
        [this]() { PrintSummary(); });
  }

private:
  void HandleLowState(const unitree_hg::msg::LowState::SharedPtr &message)
  {
    imu_ = message->imu_state;

    for (int i = 0; i < kG1MotorCount; ++i) {
      motors_[i] = message->motor_state[i];
    }

    mode_machine_ = static_cast<int>(message->mode_machine);
    received_state_ = true;
  }

  void PrintSummary()
  {
    if (!received_state_) {
      RCLCPP_INFO(this->get_logger(), "Waiting for Unitree lowstate messages...");
      return;
    }

    RCLCPP_INFO(
        this->get_logger(),
        "mode=%d imu_rpy=[%.3f, %.3f, %.3f]",
        mode_machine_,
        imu_.rpy[0], imu_.rpy[1], imu_.rpy[2]);

    RCLCPP_INFO(
        this->get_logger(),
        "left_ankle[pitch=%.3f roll=%.3f] right_ankle[pitch=%.3f roll=%.3f]",
        motors_[LEFT_ANKLE_PITCH].q, motors_[LEFT_ANKLE_ROLL].q,
        motors_[RIGHT_ANKLE_PITCH].q, motors_[RIGHT_ANKLE_ROLL].q);

    RCLCPP_INFO(
        this->get_logger(),
        "left_knee=%.3f right_knee=%.3f",
        motors_[LEFT_KNEE].q, motors_[RIGHT_KNEE].q);
  }

  rclcpp::Subscription<unitree_hg::msg::LowState>::SharedPtr subscription_;
  rclcpp::TimerBase::SharedPtr summary_timer_;
  unitree_hg::msg::IMUState imu_{};
  std::array<unitree_hg::msg::MotorState, kG1MotorCount> motors_{};
  int mode_machine_{0};
  bool received_state_{false};
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<G1BasicLowStateNode>());
  rclcpp::shutdown();
  return 0;
}
