#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "unitree_hg/msg/low_cmd.hpp"
#include "unitree_hg/msg/low_state.hpp"

namespace
{

constexpr int kG1MotorCount = 29;
constexpr int kHgMotorSlotCount = 35;
constexpr double kControlDt = 0.002;
constexpr auto kControlPeriod = std::chrono::milliseconds(2);
constexpr int kLeftAnklePitch = 4;
constexpr int kLeftAnkleRoll = 5;
constexpr int kRightAnklePitch = 10;
constexpr int kRightAnkleRoll = 11;
constexpr double kPi = 3.14159265358979323846;
constexpr double kMoveToZeroDuration = 3.0;

enum class Mode : uint8_t
{
  kPr = 0,
  kAb = 1
};

constexpr std::array<float, kG1MotorCount> kJointKp{
  60.0F, 60.0F, 60.0F, 100.0F, 40.0F, 40.0F,
  60.0F, 60.0F, 60.0F, 100.0F, 40.0F, 40.0F,
  60.0F, 40.0F, 40.0F,
  40.0F, 40.0F, 40.0F, 40.0F, 40.0F, 40.0F, 40.0F,
  40.0F, 40.0F, 40.0F, 40.0F, 40.0F, 40.0F, 40.0F
};

constexpr std::array<float, kG1MotorCount> kJointKd{
  1.0F, 1.0F, 1.0F, 2.0F, 1.0F, 1.0F,
  1.0F, 1.0F, 1.0F, 2.0F, 1.0F, 1.0F,
  1.0F, 1.0F, 1.0F,
  1.0F, 1.0F, 1.0F, 1.0F, 1.0F, 1.0F, 1.0F,
  1.0F, 1.0F, 1.0F, 1.0F, 1.0F, 1.0F, 1.0F
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

uint32_t Crc32Core(uint32_t * ptr, uint32_t len)
{
  uint32_t crc = 0xFFFFFFFF;
  constexpr uint32_t poly = 0x04c11db7;

  for (uint32_t i = 0; i < len; ++i) {
    uint32_t data = ptr[i];
    for (int b = 0; b < 32; ++b) {
      bool bit = (crc >> 31) ^ (data >> (31 - b));
      crc <<= 1;
      if (bit) {
        crc ^= poly;
      }
    }
  }

  return crc;
}

void FillCrc(unitree_hg::msg::LowCmd & msg)
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
  }

  std::memcpy(raw.reserve.data(), msg.reserve.data(), sizeof(uint32_t) * 4);
  raw.crc = Crc32Core(reinterpret_cast<uint32_t *>(&raw), (sizeof(PackedLowCmd) >> 2) - 1);
  msg.crc = raw.crc;
}

}  // namespace

class G1MoveAnkleNode : public rclcpp::Node
{
public:
  G1MoveAnkleNode()
  : Node("g1_move_ankle")
  {
    sub_ = create_subscription<unitree_hg::msg::LowState>(
      "lowstate", 10,
      std::bind(&G1MoveAnkleNode::StateCallback, this, std::placeholders::_1));

    pub_ = create_publisher<unitree_hg::msg::LowCmd>("lowcmd", 10);
    timer_ = create_wall_timer(kControlPeriod, std::bind(&G1MoveAnkleNode::ControlLoop, this));

    InitCmd();

    RCLCPP_INFO(
      get_logger(),
      "g1_move_ankle is ready. Waiting for low state, moving to zero pose, then starting ankle motion.");
  }

private:
  rclcpp::Subscription<unitree_hg::msg::LowState>::SharedPtr sub_;
  rclcpp::Publisher<unitree_hg::msg::LowCmd>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  unitree_hg::msg::LowCmd cmd_{};
  std::array<double, kG1MotorCount> q_{};
  std::array<double, kG1MotorCount> q_init_{};
  std::array<double, kG1MotorCount> q_cmd_{};
  std::array<double, kG1MotorCount> q_target_{};

  double t_{0.0};
  bool received_state_{false};
  bool initialized_pose_{false};
  uint8_t mode_machine_{0};
  const double tau_{0.08};

  void InitCmd()
  {
    for (int i = 0; i < kG1MotorCount; ++i) {
      cmd_.motor_cmd[i].mode = 0x01;
      cmd_.motor_cmd[i].kp = kJointKp[i];
      cmd_.motor_cmd[i].kd = kJointKd[i];
    }
    cmd_.mode_pr = static_cast<uint8_t>(Mode::kPr);
    cmd_.mode_machine = 0;
  }

  void StateCallback(const unitree_hg::msg::LowState::SharedPtr msg)
  {
    received_state_ = true;
    mode_machine_ = msg->mode_machine;

    for (int i = 0; i < kG1MotorCount; ++i) {
      q_[i] = msg->motor_state[i].q;
    }
  }

  void SetNeutralPose()
  {
    q_target_.fill(0.0);
  }

  void SetAnkleMotion()
  {
    SetNeutralPose();

    const double phase_time = t_ - kMoveToZeroDuration;
    const double ankle_pitch = (30.0 * kPi / 180.0) * std::sin(2.0 * kPi * phase_time);
    const double ankle_roll = (10.0 * kPi / 180.0) * std::sin(2.0 * kPi * phase_time);

    q_target_[kLeftAnklePitch] = ankle_pitch;
    q_target_[kLeftAnkleRoll] = ankle_roll;
    q_target_[kRightAnklePitch] = ankle_pitch;
    q_target_[kRightAnkleRoll] = -ankle_roll;
  }

  void SmoothCommands()
  {
    const double alpha = kControlDt / tau_;

    for (int i = 0; i < kG1MotorCount; ++i) {
      q_cmd_[i] += alpha * (q_target_[i] - q_cmd_[i]);
    }
  }

  void PublishCommand()
  {
    cmd_.mode_pr = static_cast<uint8_t>(Mode::kPr);
    cmd_.mode_machine = mode_machine_;

    for (int i = 0; i < kG1MotorCount; ++i) {
      cmd_.motor_cmd[i].q = static_cast<float>(q_cmd_[i]);
      cmd_.motor_cmd[i].dq = 0.0F;
      cmd_.motor_cmd[i].tau = 0.0F;
    }

    FillCrc(cmd_);
    pub_->publish(cmd_);
  }

  void ControlLoop()
  {
    if (!received_state_) {
      return;
    }

    if (!initialized_pose_) {
      for (int i = 0; i < kG1MotorCount; ++i) {
        q_init_[i] = q_[i];
        q_target_[i] = q_[i];
        q_cmd_[i] = q_[i];
      }
      initialized_pose_ = true;
    }

    t_ += kControlDt;

    if (t_ < kMoveToZeroDuration) {
      const double ratio = std::clamp(t_ / kMoveToZeroDuration, 0.0, 1.0);
      for (int i = 0; i < kG1MotorCount; ++i) {
        q_target_[i] = (1.0 - ratio) * q_init_[i];
      }
      SmoothCommands();
    } else {
      SetAnkleMotion();
      SmoothCommands();
    }

    PublishCommand();
  }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<G1MoveAnkleNode>());
  rclcpp::shutdown();
  return 0;
}
