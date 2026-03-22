#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "unitree_hg/msg/imu_state.hpp"
#include "unitree_hg/msg/low_cmd.hpp"
#include "unitree_hg/msg/low_state.hpp"
#include "unitree_hg/msg/motor_cmd.hpp"
#include "unitree_hg/msg/motor_state.hpp"

namespace
{

constexpr bool kHighFrequency = true;
constexpr int kG1MotorCount = 29;
constexpr int kHgMotorSlotCount = 35;
constexpr double kControlDt = 0.002;
constexpr auto kControlPeriod = std::chrono::milliseconds(2);
constexpr double kTransitionDuration = 3.0;

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

struct PackedMotorCmd
{
  uint8_t mode;
  float q;
  float dq;
  float tau;
  float kp;
  float kd;
  uint32_t reserve;
};

struct PackedLowCmd
{
  uint8_t mode_pr;
  uint8_t mode_machine;
  std::array<PackedMotorCmd, kHgMotorSlotCount> motor_cmd;
  std::array<uint32_t, 4> reserve;
  uint32_t crc;
};

double Clamp(double value, double low, double high)
{
  if (value < low) {
    return low;
  }
  if (value > high) {
    return high;
  }
  return value;
}

uint32_t Crc32Core(uint32_t *ptr, uint32_t len)
{
  uint32_t crc = 0xFFFFFFFF;
  constexpr uint32_t polynomial = 0x04c11db7;

  for (uint32_t i = 0; i < len; ++i) {
    uint32_t xbit = 1u << 31;
    uint32_t data = ptr[i];
    for (uint32_t bits = 0; bits < 32; ++bits) {
      if (crc & 0x80000000) {
        crc <<= 1;
        crc ^= polynomial;
      } else {
        crc <<= 1;
      }
      if (data & xbit) {
        crc ^= polynomial;
      }
      xbit >>= 1;
    }
  }

  return crc;
}

void FillCrc(unitree_hg::msg::LowCmd &msg)
{
  PackedLowCmd raw{};
  raw.mode_pr = msg.mode_pr;
  raw.mode_machine = msg.mode_machine;

  for (int i = 0; i < kHgMotorSlotCount; ++i) {
    raw.motor_cmd[i].mode = msg.motor_cmd[i].mode;
    raw.motor_cmd[i].q = msg.motor_cmd[i].q;
    raw.motor_cmd[i].dq = msg.motor_cmd[i].dq;
    raw.motor_cmd[i].tau = msg.motor_cmd[i].tau;
    raw.motor_cmd[i].kp = msg.motor_cmd[i].kp;
    raw.motor_cmd[i].kd = msg.motor_cmd[i].kd;
    raw.motor_cmd[i].reserve = msg.motor_cmd[i].reserve;
  }

  std::memcpy(raw.reserve.data(), msg.reserve.data(), sizeof(uint32_t) * raw.reserve.size());
  raw.crc = Crc32Core(reinterpret_cast<uint32_t *>(&raw), (sizeof(PackedLowCmd) >> 2) - 1);
  msg.crc = raw.crc;
}

}  // namespace

class G1BasicLowStateNode : public rclcpp::Node
{
public:
  G1BasicLowStateNode() : Node("g1_basic_low_state")
  {
    const char *state_topic = kHighFrequency ? "lowstate" : "lf/lowstate";

    state_subscription_ = this->create_subscription<unitree_hg::msg::LowState>(
        state_topic, 10,
        [this](const unitree_hg::msg::LowState::SharedPtr message) {
          HandleLowState(message);
        });

    command_publisher_ =
        this->create_publisher<unitree_hg::msg::LowCmd>("/lowcmd", 10);

    command_timer_ = this->create_wall_timer(
        kControlPeriod,
        [this]() { PublishStandCommand(); });

    summary_timer_ = this->create_wall_timer(
        std::chrono::seconds(1),
        [this]() { PrintSummary(); });

    InitializeCommand();
    stand_up_joint_pos_.fill(0.0);
    stand_down_joint_pos_.fill(0.0);

    // Keep the pose conservative: only bend the knees for the "down" phase.
    stand_down_joint_pos_[LEFT_KNEE] = -0.6;
    stand_down_joint_pos_[RIGHT_KNEE] = -0.6;
  }

private:
  void InitializeCommand()
  {
    low_command_.mode_pr = 0;
    low_command_.mode_machine = 0;

    for (int i = 0; i < kHgMotorSlotCount; ++i) {
      low_command_.motor_cmd[i].mode = 0x01;
      low_command_.motor_cmd[i].q = 0.0f;
      low_command_.motor_cmd[i].dq = 0.0f;
      low_command_.motor_cmd[i].tau = 0.0f;
      low_command_.motor_cmd[i].kp = 0.0f;
      low_command_.motor_cmd[i].kd = 0.0f;
      low_command_.motor_cmd[i].reserve = 0;
    }

    low_command_.reserve.fill(0);
    low_command_.crc = 0;
  }

  void HandleLowState(const unitree_hg::msg::LowState::SharedPtr &message)
  {
    imu_ = message->imu_state;
    mode_machine_ = static_cast<int>(message->mode_machine);

    for (int i = 0; i < kG1MotorCount; ++i) {
      motors_[i] = message->motor_state[i];
    }

    if (!received_state_) {
      for (int i = 0; i < kG1MotorCount; ++i) {
        initial_joint_pos_[i] = motors_[i].q;
      }
      received_state_ = true;
      RCLCPP_INFO(this->get_logger(), "Received first G1 lowstate packet, starting stand sequence.");
    }
  }

  void PublishStandCommand()
  {
    if (!received_state_) {
      return;
    }

    running_time_ += kControlDt;
    low_command_.mode_pr = 0;
    low_command_.mode_machine = static_cast<uint8_t>(mode_machine_);

    const bool standing_up = running_time_ < kTransitionDuration;
    const double phase_time = standing_up ? running_time_ : (running_time_ - kTransitionDuration);
    const double phase = std::tanh(phase_time / 1.2);

    for (int i = 0; i < kG1MotorCount; ++i) {
      const double start = standing_up ? initial_joint_pos_[i] : stand_up_joint_pos_[i];
      const double goal = standing_up ? stand_up_joint_pos_[i] : stand_down_joint_pos_[i];
      const double command = phase * goal + (1.0 - phase) * start;

      low_command_.motor_cmd[i].mode = 0x01;
      low_command_.motor_cmd[i].q = static_cast<float>(Clamp(command, -1.5, 1.5));
      low_command_.motor_cmd[i].dq = 0.0f;
      low_command_.motor_cmd[i].tau = 0.0f;
      low_command_.motor_cmd[i].kp = static_cast<float>(standing_up ? (phase * 60.0 + (1.0 - phase) * 20.0) : 60.0);
      low_command_.motor_cmd[i].kd = 2.0f;
    }

    for (int i = kG1MotorCount; i < kHgMotorSlotCount; ++i) {
      low_command_.motor_cmd[i].mode = 0x01;
      low_command_.motor_cmd[i].q = 0.0f;
      low_command_.motor_cmd[i].dq = 0.0f;
      low_command_.motor_cmd[i].tau = 0.0f;
      low_command_.motor_cmd[i].kp = 0.0f;
      low_command_.motor_cmd[i].kd = 0.0f;
    }

    FillCrc(low_command_);
    command_publisher_->publish(low_command_);
  }

  void PrintSummary()
  {
    if (!received_state_) {
      RCLCPP_INFO(this->get_logger(), "Waiting for Unitree lowstate messages...");
      return;
    }

    const double phase_time =
        running_time_ < kTransitionDuration ? running_time_ : (running_time_ - kTransitionDuration);
    const char *phase_name = running_time_ < kTransitionDuration ? "stand_up" : "stand_down";

    RCLCPP_INFO(
        this->get_logger(),
        "phase=%s t=%.2f mode=%d imu_rpy=[%.3f, %.3f, %.3f]",
        phase_name, phase_time, mode_machine_,
        imu_.rpy[0], imu_.rpy[1], imu_.rpy[2]);

    RCLCPP_INFO(
        this->get_logger(),
        "knee_cmd=[%.3f, %.3f] knee_state=[%.3f, %.3f]",
        low_command_.motor_cmd[LEFT_KNEE].q, low_command_.motor_cmd[RIGHT_KNEE].q,
        motors_[LEFT_KNEE].q, motors_[RIGHT_KNEE].q);
  }

  rclcpp::Subscription<unitree_hg::msg::LowState>::SharedPtr state_subscription_;
  rclcpp::Publisher<unitree_hg::msg::LowCmd>::SharedPtr command_publisher_;
  rclcpp::TimerBase::SharedPtr command_timer_;
  rclcpp::TimerBase::SharedPtr summary_timer_;

  unitree_hg::msg::LowCmd low_command_{};
  unitree_hg::msg::IMUState imu_{};
  std::array<unitree_hg::msg::MotorState, kG1MotorCount> motors_;
  std::array<double, kG1MotorCount> initial_joint_pos_{};
  std::array<double, kG1MotorCount> stand_up_joint_pos_{};
  std::array<double, kG1MotorCount> stand_down_joint_pos_{};

  double running_time_{0.0};
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
