#include <algorithm>
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

// ---------------- CRC ----------------
struct PackedMotorCmd
{
  uint8_t mode;
  float q, dq, tau, kp, kd;
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

double Clamp(double v, double lo, double hi)
{
  return std::max(lo, std::min(v, hi));
}

uint32_t Crc32Core(uint32_t *ptr, uint32_t len)
{
  uint32_t crc = 0xFFFFFFFF;
  constexpr uint32_t poly = 0x04c11db7;

  for (uint32_t i = 0; i < len; ++i) {
    uint32_t data = ptr[i];
    for (int b = 0; b < 32; b++) {
      bool bit = (crc >> 31) ^ (data >> (31 - b));
      crc <<= 1;
      if (bit) crc ^= poly;
    }
  }
  return crc;
}

void FillCrc(unitree_hg::msg::LowCmd &msg)
{
  PackedLowCmd raw{};
  raw.mode_pr = msg.mode_pr;
  raw.mode_machine = msg.mode_machine;

  for (int i = 0; i < kHgMotorSlotCount; i++) {
    raw.motor_cmd[i].mode = msg.motor_cmd[i].mode;
    raw.motor_cmd[i].q = msg.motor_cmd[i].q;
    raw.motor_cmd[i].dq = msg.motor_cmd[i].dq;
    raw.motor_cmd[i].tau = msg.motor_cmd[i].tau;
    raw.motor_cmd[i].kp = msg.motor_cmd[i].kp;
    raw.motor_cmd[i].kd = msg.motor_cmd[i].kd;
  }

  std::memcpy(raw.reserve.data(), msg.reserve.data(), sizeof(uint32_t) * 4);
  raw.crc = Crc32Core(reinterpret_cast<uint32_t *>(&raw),
                      (sizeof(PackedLowCmd) >> 2) - 1);
  msg.crc = raw.crc;
}

} // namespace

// ============================================================
// 🚀 ROS2 NODE
// ============================================================
class G1AdvancedMotionNode : public rclcpp::Node
{
public:
  G1AdvancedMotionNode() : Node("g1_advanced_motion")
  {
    const char *topic = kHighFrequency ? "lowstate" : "lf/lowstate";

    sub_ = create_subscription<unitree_hg::msg::LowState>(
        topic, 10,
        std::bind(&G1AdvancedMotionNode::StateCallback, this, std::placeholders::_1));

    pub_ = create_publisher<unitree_hg::msg::LowCmd>("lowcmd", 10);

    timer_ = create_wall_timer(kControlPeriod,
                               std::bind(&G1AdvancedMotionNode::ControlLoop, this));

    InitCmd();

    RCLCPP_INFO(
      get_logger(),
      "g1_advanced_motion is ready. Waiting for low state, moving to zero pose, then starting the motion state machine.");
  }

private:

  // =========================
  // 🧠 STATE MACHINE
  // =========================
  enum MotionState
  {
    STAND,
    WALK,
    RIGHT_WAVE,
    LEFT_WAVE,
    BOTH_WAVE,
    ARM_SWING,
    DANCE,
    SALUTE,
    HANDSHAKE,
    SQUAT
  };

  MotionState state_{STAND};
  double state_start_{0.0};

  // =========================
  // 📡 ROS
  // =========================
  rclcpp::Subscription<unitree_hg::msg::LowState>::SharedPtr sub_;
  rclcpp::Publisher<unitree_hg::msg::LowCmd>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  // =========================
  // 📊 DATA
  // =========================
  unitree_hg::msg::LowCmd cmd_{};
  std::array<double, kG1MotorCount> q_{};
  std::array<double, kG1MotorCount> dq_{};
  std::array<double, kG1MotorCount> q_init_{};
  double q_target_[29]{};
  double q_smooth_[29]{};
  double t_{0.0};
  bool received_{false};
  bool initialized_pose_{false};
  uint8_t mode_machine_{0};
  const double tau_{0.08};

  // =========================
  void InitCmd()
  {
    for (int i = 0; i < 29; i++) {
      cmd_.motor_cmd[i].mode = 0x01;
      cmd_.motor_cmd[i].kp = kJointKp[i];
      cmd_.motor_cmd[i].kd = kJointKd[i];
    }
    cmd_.mode_pr = static_cast<uint8_t>(Mode::kPr);
    cmd_.mode_machine = 0;
  }

  void StateCallback(const unitree_hg::msg::LowState::SharedPtr msg)
  {
    received_ = true;
    mode_machine_ = msg->mode_machine;

    for (int i = 0; i < kG1MotorCount; ++i) {
      q_[i] = msg->motor_state[i].q;
      dq_[i] = msg->motor_state[i].dq;
    }
  }

  // =========================
  // 🔁 STATE TRANSITIONS
  // =========================
  void UpdateState()
  {
    double e = t_ - state_start_;

    if (state_ == STAND && e > 2) Switch(WALK);
    else if (state_ == WALK && e > 4) Switch(RIGHT_WAVE);
    else if (state_ == RIGHT_WAVE && e > 4) Switch(LEFT_WAVE);
    else if (state_ == LEFT_WAVE && e > 4) Switch(BOTH_WAVE);
    else if (state_ == BOTH_WAVE && e > 4) Switch(ARM_SWING);
    else if (state_ == ARM_SWING && e > 4) Switch(DANCE);
    else if (state_ == DANCE && e > 5) Switch(SALUTE);
    else if (state_ == SALUTE && e > 3) Switch(HANDSHAKE);
    else if (state_ == HANDSHAKE && e > 4) Switch(SQUAT);
    else if (state_ == SQUAT && e > 3) Switch(STAND);
  }

  void Switch(MotionState s)
  {
    state_ = s;
    state_start_ = t_;
    RCLCPP_INFO(get_logger(), "State -> %d", s);
  }

  // =========================
  // 🤖 MOTIONS (from your demo)
  // =========================
  void Stand()
  {
    for (int i = 0; i < 29; i++) q_target_[i] = 0;
  }

  void Walk()
  {
    Stand();
    double p = sin(2 * t_);
    q_target_[3] = -0.4 * std::max(0.0, p);
    q_target_[9] = -0.4 * std::max(0.0, -p);
  }

  void Squat()
  {
    Stand();
    q_target_[3] = -0.6;
    q_target_[9] = -0.6;
  }

  void Dance()
  {
    Stand();
    q_target_[1] = 0.2 * sin(2 * t_);
    q_target_[20] = 0.5 * sin(4 * t_);
    q_target_[16] = -0.5 * sin(4 * t_);
  }

  void RightWave()
  {
    Stand();
    q_target_[20] = 0.4;
    q_target_[21] = -0.6;
    q_target_[22] = 0.5 * sin(4 * t_);
  }

  void LeftWave()
  {
    Stand();
    q_target_[16] = 0.4;
    q_target_[17] = -0.6;
    q_target_[18] = 0.5 * sin(4 * t_);
  }

  void BothWave()
  {
    Stand();
    q_target_[20] = 0.4;
    q_target_[21] = -0.6;
    q_target_[22] = 0.5 * sin(4 * t_);

    q_target_[16] = 0.4;
    q_target_[17] = -0.6;
    q_target_[18] = -0.5 * sin(4 * t_);
  }

  void ArmSwing()
  {
    Stand();
    double s = sin(3 * t_);
    q_target_[20] = 0.5 * s;
    q_target_[16] = -0.5 * s;
    q_target_[21] = -0.4;
    q_target_[17] = -0.4;
  }

  void Salute()
  {
    Stand();
    q_target_[20] = 0.6;
    q_target_[21] = -1.0;
    q_target_[22] = 0.2;
  }

  void Handshake()
  {
    Stand();
    q_target_[20] = 0.5;
    q_target_[21] = -0.8;
    q_target_[22] = 0.3 * sin(5 * t_);
  }

  // =========================
  // 🔁 CONTROL LOOP
  // =========================
  void ControlLoop()
  {
    if (!received_) return;

    if (!initialized_pose_) {
      for (int i = 0; i < kG1MotorCount; ++i) {
        q_init_[i] = q_[i];
        q_target_[i] = q_[i];
        q_smooth_[i] = q_[i];
      }
      initialized_pose_ = true;
    }

    t_ += kControlDt;

    if (t_ < kMoveToZeroDuration) {
      const double ratio = std::clamp(t_ / kMoveToZeroDuration, 0.0, 1.0);
      for (int i = 0; i < kG1MotorCount; ++i) {
        q_target_[i] = (1.0 - ratio) * q_init_[i];
      }
    } else {
      UpdateState();

      switch (state_)
      {
        case STAND: Stand(); break;
        case WALK: Walk(); break;
        case RIGHT_WAVE: RightWave(); break;
        case LEFT_WAVE: LeftWave(); break;
        case BOTH_WAVE: BothWave(); break;
        case ARM_SWING: ArmSwing(); break;
        case DANCE: Dance(); break;
        case SALUTE: Salute(); break;
        case HANDSHAKE: Handshake(); break;
        case SQUAT: Squat(); break;
      }
    }

    cmd_.mode_pr = static_cast<uint8_t>(Mode::kPr);
    cmd_.mode_machine = mode_machine_;

    const double alpha = kControlDt / tau_;
    for (int i = 0; i < 29; i++) {
      q_smooth_[i] += alpha * (q_target_[i] - q_smooth_[i]);

      cmd_.motor_cmd[i].q =
          static_cast<float>(Clamp(q_smooth_[i], -1.5, 1.5));
      cmd_.motor_cmd[i].dq = 0;
      cmd_.motor_cmd[i].tau = 0;
    }

    FillCrc(cmd_);
    pub_->publish(cmd_);
  }
};

// =========================
// MAIN
// =========================
int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<G1AdvancedMotionNode>());
  rclcpp::shutdown();
  return 0;
}
