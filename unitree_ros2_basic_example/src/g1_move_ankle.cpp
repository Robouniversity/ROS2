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
constexpr int kRightAnklePitch = 10;

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

    pub_ = create_publisher<unitree_hg::msg::LowCmd>("/lowcmd", 10);
    timer_ = create_wall_timer(kControlPeriod, std::bind(&G1MoveAnkleNode::ControlLoop, this));

    InitCmd();

    RCLCPP_INFO(
      get_logger(),
      "g1_move_ankle is ready. Waiting for low state, then holding pose for 1 s before ankle motion.");
  }

private:
  rclcpp::Subscription<unitree_hg::msg::LowState>::SharedPtr sub_;
  rclcpp::Publisher<unitree_hg::msg::LowCmd>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  unitree_hg::msg::LowCmd cmd_{};
  std::array<double, kG1MotorCount> q_{};
  std::array<double, kG1MotorCount> q_cmd_{};
  std::array<double, kG1MotorCount> q_target_{};

  double t_{0.0};
  bool received_state_{false};
  bool initialized_pose_{false};
  const double tau_{0.08};

  void InitCmd()
  {
    for (int i = 0; i < kG1MotorCount; ++i) {
      cmd_.motor_cmd[i].mode = 0x01;
      cmd_.motor_cmd[i].kp = 40.0F;
      cmd_.motor_cmd[i].kd = 2.0F;
    }
  }

  void StateCallback(const unitree_hg::msg::LowState::SharedPtr msg)
  {
    received_state_ = true;

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

    const double amplitude = 8.0 * M_PI / 180.0;
    const double ankle_pitch = amplitude * std::sin(2.0 * M_PI * 0.5 * t_);

    q_target_[kLeftAnklePitch] = ankle_pitch;
    q_target_[kRightAnklePitch] = ankle_pitch;
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
        q_target_[i] = q_[i];
        q_cmd_[i] = q_[i];
      }
      initialized_pose_ = true;
    }

    t_ += kControlDt;

    if (t_ < 1.0) {
      for (int i = 0; i < kG1MotorCount; ++i) {
        q_target_[i] = q_[i];
        q_cmd_[i] = q_[i];
      }
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
