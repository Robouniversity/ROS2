#include <iostream>
#include <cmath>
#include <unistd.h>
#include <thread>

#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/idl/hg/LowState_.hpp>
#include <unitree/idl/hg/LowCmd_.hpp>
#include <unitree/common/thread/thread.hpp>

using namespace unitree::robot;
using namespace unitree::common;

#define TOPIC_LOWCMD "rt/lowcmd"
#define TOPIC_LOWSTATE "rt/lowstate"

constexpr double dt = 0.002;

// ================= STATES =================
enum MotionState
{
    STAND,
    WALK,
    DANCE,
    SQUAT,
    RIGHT_WAVE,
    LEFT_WAVE,
    BOTH_WAVE,
    ARM_SWING,
    SALUTE,
    HANDSHAKE
};

// ================= CLASS =================
class G1Controller
{
public:
    void Init()
    {
        InitCmd();

        pub.reset(new ChannelPublisher<unitree_hg::msg::dds_::LowCmd_>(TOPIC_LOWCMD));
        pub->InitChannel();

        sub.reset(new ChannelSubscriber<unitree_hg::msg::dds_::LowState_>(TOPIC_LOWSTATE));
        sub->InitChannel(std::bind(&G1Controller::StateHandler, this, std::placeholders::_1), 1);

        thread = CreateRecurrentThreadEx("ctrl", UT_CPU_ID_NONE,
                                         int(dt * 1e6),
                                         &G1Controller::ControlLoop, this);
    }

    void Start()
    {
        running = true;
        std::cout << "STARTED\n";
    }

    void Stop()
    {
        running = false;
        std::cout << "STOPPED\n";
    }

private:
    unitree_hg::msg::dds_::LowCmd_ cmd{};
    unitree_hg::msg::dds_::LowState_ state{};

    ChannelPublisherPtr<unitree_hg::msg::dds_::LowCmd_> pub;
    ChannelSubscriberPtr<unitree_hg::msg::dds_::LowState_> sub;
    ThreadPtr thread;

    double q[29]{}, dq[29]{};
    double q_target[29]{};
    double q_smooth[29]{};

    double t = 0;
    bool running = false;

    MotionState state_machine = STAND;
    double state_start_time = 0;

    double tau = 0.08;

    // ================= INIT =================
    void InitCmd()
    {
        for (int i = 0; i < 29; i++)
        {
            cmd.motor_cmd()[i].mode() = 0x01;
            cmd.motor_cmd()[i].kp() = 40;
            cmd.motor_cmd()[i].kd() = 2;
        }
    }

    void StateHandler(const void *msg)
    {
        state = *(unitree_hg::msg::dds_::LowState_ *)msg;
        for (int i = 0; i < 29; i++)
        {
            q[i] = state.motor_state()[i].q();
            dq[i] = state.motor_state()[i].dq();
        }
    }

    // ================= BASE =================
    void Stand()
    {
        for (int i = 0; i < 29; i++) q_target[i] = 0;
    }

    void Squat()
    {
        Stand();
        q_target[3] = -0.6;
        q_target[9] = -0.6;
    }

    void Walk()
    {
        Stand();
        double phase = sin(2 * t);
        q_target[3] = -0.4 * std::max(0.0, phase);
        q_target[9] = -0.4 * std::max(0.0, -phase);
    }

    void Dance()
    {
        Stand();
        q_target[1] = 0.2 * sin(2 * t);
        q_target[20] = 0.5 * sin(4 * t);
        q_target[16] = -0.5 * sin(4 * t);
    }

    // ================= HAND =================
    void RightWave()
    {
        Stand();
        q_target[20] = 0.4;
        q_target[21] = -0.6;
        q_target[22] = 0.5 * sin(4 * t);
    }

    void LeftWave()
    {
        Stand();
        q_target[16] = 0.4;
        q_target[17] = -0.6;
        q_target[18] = 0.5 * sin(4 * t);
    }

    void BothWave()
    {
        Stand();

        q_target[20] = 0.4;
        q_target[21] = -0.6;
        q_target[22] = 0.5 * sin(4 * t);

        q_target[16] = 0.4;
        q_target[17] = -0.6;
        q_target[18] = -0.5 * sin(4 * t);
    }

    void ArmSwing()
    {
        Stand();
        double s = sin(3 * t);

        q_target[20] = 0.5 * s;
        q_target[16] = -0.5 * s;

        q_target[21] = -0.4;
        q_target[17] = -0.4;
    }

    void Salute()
    {
        Stand();
        q_target[20] = 0.6;
        q_target[21] = -1.0;
        q_target[22] = 0.2;
    }

    void Handshake()
    {
        Stand();
        q_target[20] = 0.5;
        q_target[21] = -0.8;
        q_target[22] = 0.3 * sin(5 * t);
    }

    // ================= STATE =================
    void UpdateState()
    {
        double e = t - state_start_time;

        if (state_machine == STAND && e > 2) Switch(WALK);
        else if (state_machine == WALK && e > 4) Switch(RIGHT_WAVE);
        else if (state_machine == RIGHT_WAVE && e > 4) Switch(LEFT_WAVE);
        else if (state_machine == LEFT_WAVE && e > 4) Switch(BOTH_WAVE);
        else if (state_machine == BOTH_WAVE && e > 4) Switch(ARM_SWING);
        else if (state_machine == ARM_SWING && e > 4) Switch(DANCE);
        else if (state_machine == DANCE && e > 5) Switch(SALUTE);
        else if (state_machine == SALUTE && e > 3) Switch(HANDSHAKE);
        else if (state_machine == HANDSHAKE && e > 4) Switch(SQUAT);
        else if (state_machine == SQUAT && e > 3) Switch(STAND);
    }

    void Switch(MotionState s)
    {
        state_machine = s;
        state_start_time = t;
        std::cout << "State -> " << s << std::endl;
    }

    // ================= SMOOTH =================
    void Smooth()
    {
        double alpha = dt / tau;

        for (int i = 0; i < 29; i++)
        {
            q_smooth[i] += alpha * (q_target[i] - q_smooth[i]);

            // manual clamp
            if (q_smooth[i] > 1.5) q_smooth[i] = 1.5;
            if (q_smooth[i] < -1.5) q_smooth[i] = -1.5;
        }
    }

    // ================= LOOP =================
    void ControlLoop()
    {
        t += dt;

        if (!running) return;

        UpdateState();

        switch (state_machine)
        {
            case STAND: Stand(); break;
            case WALK: Walk(); break;
            case DANCE: Dance(); break;
            case SQUAT: Squat(); break;

            case RIGHT_WAVE: RightWave(); break;
            case LEFT_WAVE: LeftWave(); break;
            case BOTH_WAVE: BothWave(); break;
            case ARM_SWING: ArmSwing(); break;
            case SALUTE: Salute(); break;
            case HANDSHAKE: Handshake(); break;
        }

        Smooth();

        for (int i = 0; i < 29; i++)
        {
            cmd.motor_cmd()[i].q() = q_smooth[i];
            cmd.motor_cmd()[i].dq() = 0;
            cmd.motor_cmd()[i].tau() = 0;
        }

        pub->Write(cmd);
    }
};

// ================= MAIN =================
int main(int argc, const char **argv)
{
    if (argc < 2)
        ChannelFactory::Instance()->Init(1, "lo");
    else
        ChannelFactory::Instance()->Init(0, argv[1]);

    G1Controller robot;
    robot.Init();

    std::cout << "\nPress 's' to START, 'q' to QUIT\n";

    while (true)
    {
        char c;
        std::cin >> c;

        if (c == 's')
            robot.Start();
        else if (c == 'q')
            break;
    }

    return 0;
}
