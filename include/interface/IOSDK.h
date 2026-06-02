#ifndef IOSDK_H
#define IOSDK_H

#include "interface/IOInterface.h"
#include <array>
#include <chrono>
#include <mutex>
#include <string>
#include <termios.h>
#include "common/gamepad.hpp"
#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/idl/go2/SportModeState_.hpp>
#include <unitree/idl/ros2/Point_.hpp>

#include <unitree/idl/hg/IMUState_.hpp>
#include <unitree/idl/hg/LowCmd_.hpp>
#include <unitree/idl/hg/LowState_.hpp>
#include <unitree/robot/b2/motion_switcher/motion_switcher_client.hpp>

static const std::string HG_CMD_TOPIC = "rt/lowcmd";
static const std::string HG_IMU_TORSO = "rt/secondary_imu";
static const std::string HG_STATE_TOPIC = "rt/lowstate";
static const std::string HG_SPORT_STATE_TOPIC = "rt/sportmodestate";
static const std::string HG_SOCCER_BALL_POS_TOPIC = "rt/soccer/ball_pos";
static const std::string HG_SOCCER_TARGET_POS_TOPIC = "rt/soccer/target_pos";

using namespace unitree::common;
using namespace unitree::robot;
using namespace unitree_hg::msg::dds_;

const int G1_NUM_MOTOR = 29;
enum class Mode {
  PR = 0,  // Series Control for Ptich/Roll Joints
  AB = 1   // Parallel Control for A/B Joints
};

class IOSDK : public IOInterface
{
private:
    ChannelPublisherPtr<LowCmd_> lowcmd_publisher_;
    ChannelSubscriberPtr<LowState_> lowstate_subscriber_;
    ChannelSubscriberPtr<unitree_go::msg::dds_::SportModeState_> sportstate_subscriber_;
    ChannelSubscriberPtr<geometry_msgs::msg::dds_::Point_> soccer_ball_subscriber_;
    ChannelSubscriberPtr<geometry_msgs::msg::dds_::Point_> soccer_target_subscriber_;
    LowlevelCmd _lowCmd;
    LowlevelState _lowState;
    float soccer_ball_pos_[3];
    float soccer_target_pos_[3];
    REMOTE_DATA_RX rx_;
    Gamepad gamepad_;
    uint8_t mode_machine_;
    int counter_;
    UserCommand userCmd_;
    UserValue userValue_;
    bool keyboard_enabled_;
    bool terminal_configured_;
    int stdin_flags_backup_;
    struct termios terminal_orig_;
    std::array<std::chrono::steady_clock::time_point, 6> axis_last_seen_;
    std::array<std::chrono::steady_clock::time_point, 256> discrete_last_seen_;
    std::array<bool, 256> discrete_pressed_;
    std::chrono::milliseconds axis_hold_timeout_;
    std::chrono::milliseconds discrete_release_timeout_;
    UserCommand pending_keyboard_cmd_;
    std::mutex input_mutex_;
    std::mutex soccer_mutex_;

    void LowStateHandler(const void *message);
    void SportStateHandler(const void *message);
    void SoccerBallHandler(const void *message);
    void SoccerTargetHandler(const void *message);
    void initKeyboardInput();
    void restoreKeyboardInput();
    bool pollKeyboard(UserCommand &cmd, UserValue &value, bool &axis_active);
    UserCommand mapKeyboardCommand(char key) const;
    UserCommand getGamepadCommand() const;
    UserValue getGamepadValue() const;

public:
    IOSDK(/* args */);
    ~IOSDK();
    void sendRecv(const LowlevelCmd *cmd, LowlevelState *state);
};






#endif //IOSDK_H
