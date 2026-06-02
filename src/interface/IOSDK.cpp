#include "interface/IOSDK.h"
#include <algorithm>
#include <cctype>
#include <cerrno>
#include <cstring>
#include <fcntl.h>
#include <unistd.h>
#include <stdio.h>
#include <cmath>
#include <iostream>

uint32_t crc32_core(uint32_t *ptr, uint32_t len)
{
    unsigned int xbit = 0;
    unsigned int data = 0;
    unsigned int CRC32 = 0xFFFFFFFF;
    const unsigned int dwPolynomial = 0x04c11db7;

    for (unsigned int i = 0; i < len; i++)
    {
        xbit = 1 << 31;
        data = ptr[i];
        for (unsigned int bits = 0; bits < 32; bits++)
        {
            if (CRC32 & 0x80000000)
            {
                CRC32 <<= 1;
                CRC32 ^= dwPolynomial;
            }
            else
            {
                CRC32 <<= 1;
            }

            if (data & xbit)
                CRC32 ^= dwPolynomial;
            xbit >>= 1;
        }
    }

    return CRC32;
}

IOSDK::IOSDK()
{
    lowcmd_publisher_.reset(new ChannelPublisher<LowCmd_>(HG_CMD_TOPIC));
    lowcmd_publisher_->InitChannel();

    lowstate_subscriber_.reset(new ChannelSubscriber<LowState_>(HG_STATE_TOPIC));
    lowstate_subscriber_->InitChannel(std::bind(&IOSDK::LowStateHandler, this, std::placeholders::_1), 1);
    sportstate_subscriber_.reset(new ChannelSubscriber<unitree_go::msg::dds_::SportModeState_>(HG_SPORT_STATE_TOPIC));
    sportstate_subscriber_->InitChannel(std::bind(&IOSDK::SportStateHandler, this, std::placeholders::_1), 1);
    soccer_ball_subscriber_.reset(new ChannelSubscriber<geometry_msgs::msg::dds_::Point_>(HG_SOCCER_BALL_POS_TOPIC));
    soccer_ball_subscriber_->InitChannel(std::bind(&IOSDK::SoccerBallHandler, this, std::placeholders::_1), 1);
    soccer_target_subscriber_.reset(new ChannelSubscriber<geometry_msgs::msg::dds_::Point_>(HG_SOCCER_TARGET_POS_TOPIC));
    soccer_target_subscriber_->InitChannel(std::bind(&IOSDK::SoccerTargetHandler, this, std::placeholders::_1), 1);

    counter_ = 0;
    userCmd_ = UserCommand::NONE;
    userValue_.setZero();
    mode_machine_ = 0;
    keyboard_enabled_ = false;
    terminal_configured_ = false;
    stdin_flags_backup_ = -1;
    axis_hold_timeout_ = std::chrono::milliseconds(120);
    discrete_release_timeout_ = std::chrono::milliseconds(800);
    const auto now = std::chrono::steady_clock::now() - std::chrono::milliseconds(10000);
    axis_last_seen_.fill(now);
    discrete_last_seen_.fill(now);
    discrete_pressed_.fill(false);
    pending_keyboard_cmd_ = UserCommand::NONE;
    soccer_ball_pos_[0] = 0.7f;
    soccer_ball_pos_[1] = 0.0f;
    soccer_ball_pos_[2] = 0.11f;
    soccer_target_pos_[0] = 1.2f;
    soccer_target_pos_[1] = 0.0f;
    soccer_target_pos_[2] = 0.11f;
    std::memset(&terminal_orig_, 0, sizeof(terminal_orig_));
    initKeyboardInput();
}

IOSDK::~IOSDK()
{
    restoreKeyboardInput();
}

void IOSDK::sendRecv(const LowlevelCmd *cmd, LowlevelState *state)
{
    // send control cmd
    LowCmd_ dds_low_command;
    dds_low_command.mode_pr() = static_cast<uint8_t>(Mode::PR);
    dds_low_command.mode_machine() = mode_machine_;
    for (size_t i = 0; i < G1_NUM_MOTOR; i++)
    {
        
        dds_low_command.motor_cmd().at(i).mode() = 1; // 1:Enable, 0:Disable
        dds_low_command.motor_cmd().at(i).tau() = cmd->motorCmd[i].tau;
        dds_low_command.motor_cmd().at(i).q() = cmd->motorCmd[i].q;
        dds_low_command.motor_cmd().at(i).dq() = cmd->motorCmd[i].dq;
        dds_low_command.motor_cmd().at(i).kp() = cmd->motorCmd[i].Kp;
        dds_low_command.motor_cmd().at(i).kd() = cmd->motorCmd[i].Kd;
        // std::cout<<"des_q: "<<dds_low_command.motor_cmd().at(i).q()<<std::endl;
    }

    dds_low_command.crc() = crc32_core((uint32_t *)&dds_low_command, (sizeof(dds_low_command) >> 2) - 1);
    bool wrt = lowcmd_publisher_->Write(dds_low_command);

    for (int i = 0; i < G1_NUM_MOTOR; i++)
    {
        state->motorState[i].q = _lowState.motorState[i].q;
        state->motorState[i].dq = _lowState.motorState[i].dq;
    }
    for (int i = 0; i < 3; i++)
    {
        state->imu.quaternion[i] = _lowState.imu.quaternion[i];
        state->imu.accelerometer[i] = _lowState.imu.accelerometer[i];
        state->imu.gyroscope[i] = _lowState.imu.gyroscope[i];
        state->basePos[i] = _lowState.basePos[i];
        state->baseLinVel[i] = _lowState.baseLinVel[i];
    }
    state->imu.quaternion[3] = _lowState.imu.quaternion[3];

    {
        std::lock_guard<std::mutex> lock(input_mutex_);
        state->userCmd = userCmd_;
        state->userValue = userValue_;
        // Consume latched discrete keyboard command once.
        if (pending_keyboard_cmd_ != UserCommand::NONE && state->userCmd == pending_keyboard_cmd_)
        {
            pending_keyboard_cmd_ = UserCommand::NONE;
        }
    }
}

void IOSDK::SportStateHandler(const void *message)
{
    const auto state = *(const unitree_go::msg::dds_::SportModeState_ *)message;
    _lowState.basePos[0] = static_cast<float>(state.position()[0]);
    _lowState.basePos[1] = static_cast<float>(state.position()[1]);
    _lowState.basePos[2] = static_cast<float>(state.position()[2]);
    _lowState.baseLinVel[0] = static_cast<float>(state.velocity()[0]);
    _lowState.baseLinVel[1] = static_cast<float>(state.velocity()[1]);
    _lowState.baseLinVel[2] = static_cast<float>(state.velocity()[2]);
}

void IOSDK::SoccerBallHandler(const void *message)
{
    const auto pos = *(const geometry_msgs::msg::dds_::Point_ *)message;
    std::lock_guard<std::mutex> lock(soccer_mutex_);
    soccer_ball_pos_[0] = static_cast<float>(pos.x());
    soccer_ball_pos_[1] = static_cast<float>(pos.y());
    soccer_ball_pos_[2] = static_cast<float>(pos.z());
}

void IOSDK::SoccerTargetHandler(const void *message)
{
    const auto pos = *(const geometry_msgs::msg::dds_::Point_ *)message;
    std::lock_guard<std::mutex> lock(soccer_mutex_);
    soccer_target_pos_[0] = static_cast<float>(pos.x());
    soccer_target_pos_[1] = static_cast<float>(pos.y());
    soccer_target_pos_[2] = static_cast<float>(pos.z());
}

void IOSDK::LowStateHandler(const void *message)
{
    LowState_ low_state = *(const LowState_ *)message;
    if (low_state.crc() != crc32_core((uint32_t *)&low_state, (sizeof(LowState_) >> 2) - 1))
    {
        std::cout << "[ERROR] CRC Error" << std::endl;
        return;
    }

    // get motor state
    for (int i = 0; i < G1_NUM_MOTOR; ++i)
    {
        _lowState.motorState[i].q = low_state.motor_state()[i].q();
        _lowState.motorState[i].dq = low_state.motor_state()[i].dq();
    }
    
    // get imu state
    _lowState.imu.gyroscope[0] = low_state.imu_state().gyroscope()[0];
    _lowState.imu.gyroscope[1] = low_state.imu_state().gyroscope()[1];
    _lowState.imu.gyroscope[2] = low_state.imu_state().gyroscope()[2];

    _lowState.imu.quaternion[0] = low_state.imu_state().quaternion()[0];
    _lowState.imu.quaternion[1] = low_state.imu_state().quaternion()[1];
    _lowState.imu.quaternion[2] = low_state.imu_state().quaternion()[2];
    _lowState.imu.quaternion[3] = low_state.imu_state().quaternion()[3];

    _lowState.imu.accelerometer[0] = low_state.imu_state().accelerometer()[0];
    _lowState.imu.accelerometer[1] = low_state.imu_state().accelerometer()[1];
    _lowState.imu.accelerometer[2] = low_state.imu_state().accelerometer()[2];

    // update gamepad
    memcpy(rx_.buff, &low_state.wireless_remote()[0], 40);
    gamepad_.update(rx_.RF_RX);

    // update mode machine
    if (mode_machine_ != low_state.mode_machine())
    {
        if (mode_machine_ == 0)
            std::cout << "G1 type: " << unsigned(low_state.mode_machine()) << std::endl;
        mode_machine_ = low_state.mode_machine();
    }

    std::lock_guard<std::mutex> lock(input_mutex_);
    userCmd_ = UserCommand::NONE;
    UserValue keyboard_value;
    keyboard_value.setZero();
    bool keyboard_axis_active = false;
    UserCommand keyboard_cmd = UserCommand::NONE;
    pollKeyboard(keyboard_cmd, keyboard_value, keyboard_axis_active);

    if (keyboard_cmd != UserCommand::NONE)
    {
        pending_keyboard_cmd_ = keyboard_cmd;
    }

    if (pending_keyboard_cmd_ != UserCommand::NONE)
    {
        userCmd_ = pending_keyboard_cmd_;
    }
    else
    {
        userCmd_ = getGamepadCommand();
    }

    if (keyboard_axis_active)
    {
        userValue_ = keyboard_value;
    }
    else
    {
        userValue_ = getGamepadValue();
    }
}

void IOSDK::initKeyboardInput()
{
    if (!isatty(STDIN_FILENO))
    {
        std::cout << "[IOSDK] Keyboard input disabled: stdin is not a TTY, fallback to gamepad only." << std::endl;
        keyboard_enabled_ = false;
        return;
    }

    stdin_flags_backup_ = fcntl(STDIN_FILENO, F_GETFL, 0);
    if (stdin_flags_backup_ == -1)
    {
        std::perror("[IOSDK] fcntl(F_GETFL) failed");
        keyboard_enabled_ = false;
        return;
    }

    if (tcgetattr(STDIN_FILENO, &terminal_orig_) == -1)
    {
        std::perror("[IOSDK] tcgetattr failed");
        keyboard_enabled_ = false;
        return;
    }

    struct termios raw = terminal_orig_;
    raw.c_lflag &= static_cast<unsigned long>(~(ICANON | ECHO));
    raw.c_cc[VMIN] = 0;
    raw.c_cc[VTIME] = 0;
    if (tcsetattr(STDIN_FILENO, TCSANOW, &raw) == -1)
    {
        std::perror("[IOSDK] tcsetattr raw mode failed");
        keyboard_enabled_ = false;
        return;
    }

    if (fcntl(STDIN_FILENO, F_SETFL, stdin_flags_backup_ | O_NONBLOCK) == -1)
    {
        std::perror("[IOSDK] fcntl(F_SETFL) O_NONBLOCK failed");
        tcsetattr(STDIN_FILENO, TCSANOW, &terminal_orig_);
        keyboard_enabled_ = false;
        return;
    }

    terminal_configured_ = true;
    keyboard_enabled_ = true;
    std::cout << "[IOSDK] Keyboard input enabled. Use WASD/QE for motion and 1-0,-,[,] for commands." << std::endl;
}

void IOSDK::restoreKeyboardInput()
{
    if (!terminal_configured_)
    {
        return;
    }

    tcsetattr(STDIN_FILENO, TCSANOW, &terminal_orig_);
    if (stdin_flags_backup_ >= 0)
    {
        fcntl(STDIN_FILENO, F_SETFL, stdin_flags_backup_);
    }
    terminal_configured_ = false;
}

UserCommand IOSDK::mapKeyboardCommand(char key) const
{
    switch (key)
    {
    case '1': return UserCommand::START;
    case '2': return UserCommand::SELECT;
    case '3': return UserCommand::R2_A;
    case '4': return UserCommand::R1_UP;
    case '5': return UserCommand::R2;
    case '6': return UserCommand::L2;
    case '7': return UserCommand::R1;
    case '8': return UserCommand::L2_B;
    case '9': return UserCommand::R2_UP;
    case '0': return UserCommand::R2_DOWN;
    case '-': return UserCommand::R2_B;
    case '[': return UserCommand::R1_LEFT;
    case ']': return UserCommand::R1_RIGHT;
    default: return UserCommand::NONE;
    }
}

bool IOSDK::pollKeyboard(UserCommand &cmd, UserValue &value, bool &axis_active)
{
    cmd = UserCommand::NONE;
    axis_active = false;
    value.setZero();
    if (!keyboard_enabled_)
    {
        return false;
    }

    bool had_input = false;
    const auto now = std::chrono::steady_clock::now();
    char ch = 0;
    while (true)
    {
        ssize_t n = read(STDIN_FILENO, &ch, 1);
        if (n == 1)
        {
            had_input = true;
            unsigned char key = static_cast<unsigned char>(std::tolower(static_cast<unsigned char>(ch)));

            switch (key)
            {
            case 'a':
                axis_last_seen_[0] = now;
                break;
            case 'd':
                axis_last_seen_[1] = now;
                break;
            case 'w':
                axis_last_seen_[2] = now;
                break;
            case 's':
                axis_last_seen_[3] = now;
                break;
            case 'q':
                axis_last_seen_[4] = now;
                break;
            case 'e':
                axis_last_seen_[5] = now;
                break;
            default:
                break;
            }

            UserCommand mapped = mapKeyboardCommand(static_cast<char>(key));
            if (mapped != UserCommand::NONE)
            {
                const size_t idx = static_cast<size_t>(key);
                discrete_last_seen_[idx] = now;
                if (!discrete_pressed_[idx])
                {
                    discrete_pressed_[idx] = true;
                    cmd = mapped;
                }
            }
        }
        else
        {
            if (n == -1 && (errno == EAGAIN || errno == EWOULDBLOCK))
            {
                break;
            }
            if (n == 0)
            {
                break;
            }
            if (n == -1 && errno == EINTR)
            {
                continue;
            }
            break;
        }
    }

    for (size_t i = 0; i < discrete_pressed_.size(); ++i)
    {
        if (!discrete_pressed_[i])
        {
            continue;
        }
        if (now - discrete_last_seen_[i] > discrete_release_timeout_)
        {
            discrete_pressed_[i] = false;
        }
    }

    const bool a_active = (now - axis_last_seen_[0] <= axis_hold_timeout_);
    const bool d_active = (now - axis_last_seen_[1] <= axis_hold_timeout_);
    const bool w_active = (now - axis_last_seen_[2] <= axis_hold_timeout_);
    const bool s_active = (now - axis_last_seen_[3] <= axis_hold_timeout_);
    const bool q_active = (now - axis_last_seen_[4] <= axis_hold_timeout_);
    const bool e_active = (now - axis_last_seen_[5] <= axis_hold_timeout_);

    // Keyboard axis signs are tuned to match expected operator intuition:
    // W forward, S backward, A left, D right.
    value.lx = (a_active ? 1.0f : 0.0f) + (d_active ? -1.0f : 0.0f);
    value.ly = (s_active ? -1.0f : 0.0f) + (w_active ? 1.0f : 0.0f);
    value.rx = (e_active ? -1.0f : 0.0f) + (q_active ? 1.0f : 0.0f);
    value.ry = 0.0f;

    value.lx = std::clamp(value.lx, -1.0f, 1.0f);
    value.ly = std::clamp(value.ly, -1.0f, 1.0f);
    value.rx = std::clamp(value.rx, -1.0f, 1.0f);

    axis_active = a_active || d_active || w_active || s_active || q_active || e_active;
    return had_input || axis_active || cmd != UserCommand::NONE;
}

UserCommand IOSDK::getGamepadCommand() const
{
    if (gamepad_.start.pressed)
    {
        return UserCommand::START;
    }
    if (gamepad_.select.pressed)
    {
        return UserCommand::SELECT;
    }
    if (gamepad_.R2.pressed && gamepad_.A.pressed)
    {
        return UserCommand::R2_A;
    }
    if (gamepad_.L2.pressed && gamepad_.B.pressed)
    {
        return UserCommand::L2_B;
    }
    if (gamepad_.R1.pressed && gamepad_.up.pressed)
    {
        return UserCommand::R1_UP;
    }
    if (gamepad_.R1.pressed && gamepad_.left.pressed)
    {
        return UserCommand::R1_LEFT;
    }
    if (gamepad_.R1.pressed && gamepad_.right.pressed)
    {
        return UserCommand::R1_RIGHT;
    }
    if (gamepad_.R2.pressed && gamepad_.up.pressed)
    {
        return UserCommand::R2_UP;
    }
    if (gamepad_.R2.pressed && gamepad_.down.pressed)
    {
        return UserCommand::R2_DOWN;
    }
    if (gamepad_.R2.pressed && gamepad_.B.pressed)
    {
        return UserCommand::R2_B;
    }
    if (gamepad_.R2.pressed)
    {
        return UserCommand::R2;
    }
    if (gamepad_.L2.pressed)
    {
        return UserCommand::L2;
    }
    if (gamepad_.R1.pressed)
    {
        return UserCommand::R1;
    }
    return UserCommand::NONE;
}

UserValue IOSDK::getGamepadValue() const
{
    UserValue value;
    value.lx = -gamepad_.lx;
    value.ly = gamepad_.ly;
    value.rx = -gamepad_.rx;
    value.ry = gamepad_.ry;
    return value;
}
