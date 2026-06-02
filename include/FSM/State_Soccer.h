#ifndef STATE_SOCCER_H
#define STATE_SOCCER_H

#include "FSM/FSMState.h"
#include "common/mathTools.h"
#include "utils/cnpy.h"
#include <onnxruntime_cxx_api.h>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/idl/ros2/Point_.hpp>
#include <array>
#include <vector>
#include <memory>
#include <mutex>

#define NUM_DOF 29

using namespace ArmatureConstants;

class State_Soccer : public FSMState
{
public:
    State_Soccer(CtrlComponents *ctrlComp);
    ~State_Soccer() = default;
    void enter() override;
    void run() override;
    void exit() override;
    FSMStateName checkChange() override;

private:
    void _loadPolicy();
    void _loadMotionReference();
    void _refresh_reference(int64_t time_step);
    void _observations_compute();
    void _action_compute();
    void _reset_lstm_state();
    std::array<float, 3> _world_to_base(const std::array<float, 3> &pos_w) const;
    void _onBallPos(const void *message);
    void _onTargetPos(const void *message);

    Ort::Env _env;
    Ort::SessionOptions _session_options;
    std::unique_ptr<Ort::Session> _session;
    Ort::AllocatorWithDefaultOptions _allocator;

    const std::vector<const char *> _input_names = {"obs", "h_in", "c_in", "time_step"};
    const std::vector<const char *> _output_names = {"actions", "h_out", "c_out"};

    std::vector<int64_t> _obs_shape;
    std::vector<int64_t> _h_shape;
    std::vector<int64_t> _c_shape;

    int64_t _obs_size_ = 0;
    int64_t _action_size_ = 0;
    int64_t _h_size_ = 0;
    int64_t _c_size_ = 0;

    bool _model_ready_ = false;
    bool _motion_ready_ = false;
    bool _terminate_flag = false;

    float _time_step_input_ = 0.0f;
    int64_t _time_step_index_ = 0;

    std::vector<float> _h_state_;
    std::vector<float> _c_state_;
    std::vector<float> _action;
    std::vector<float> _observation;

    std::vector<float> _ref_joint_pos;
    std::vector<float> _ref_joint_vel;
    std::array<float, 3> _ref_anchor_body_ang_vel = {0.0f, 0.0f, 0.0f};

    std::vector<float> _motion_joint_pos;      // [T, 29]
    std::vector<float> _motion_joint_vel;      // [T, 29]
    std::vector<float> _motion_body_ang_vel;   // [T, B, 3]
    int64_t _motion_steps_ = 0;
    int64_t _motion_body_count_ = 0;

    std::array<float, 3> _ball_pos_w = {0.7f, 0.0f, 0.11f};
    std::array<float, 3> _target_pos_w = {1.2f, 0.0f, 0.11f};
    std::mutex _soccer_mutex;
    std::shared_ptr<unitree::robot::ChannelSubscriber<geometry_msgs::msg::dds_::Point_>> _ball_subscriber;
    std::shared_ptr<unitree::robot::ChannelSubscriber<geometry_msgs::msg::dds_::Point_>> _target_subscriber;
    std::string _ball_topic = "rt/soccer/ball_pos";
    std::string _target_topic = "rt/soccer/target_pos";

    std::string _model_path;
    std::string _motion_file;
    float _anchor_terminate_thresh = 2.6f;
    float _clip_observations = 100.0f;
    float _clip_actions = 100.0f;
    int _anchor_body_index_ = 7;

    float _joint_q[NUM_DOF];
    float _targetPos_rl[NUM_DOF];
    float _last_targetPos_rl[NUM_DOF];

    const std::vector<float> _gravity_vec = {0.0f, 0.0f, -1.0f};
    // Soccer policy metadata order. These arrays must stay aligned with the ONNX metadata.
    const float _default_dof_pos[NUM_DOF] = {
        -0.312f, 0.0f, 0.0f, 0.669f, -0.363f, 0.0f,
        -0.312f, 0.0f, 0.0f, 0.669f, -0.363f, 0.0f,
        0.0f, 0.0f, 0.0f,
        0.2f, 0.2f, 0.0f, 0.6f, 0.0f, 0.0f, 0.0f,
        0.2f, -0.2f, 0.0f, 0.6f, 0.0f, 0.0f, 0.0f};

    // Map policy action index -> lowState/lowCmd motor index.
    const int dof_mapping_mj[NUM_DOF] = {
        0, 6, 12,
        1, 7, 13,
        2, 8, 14,
        3, 9, 15, 22,
        4, 10, 16, 23,
        5, 11, 17, 24,
        18, 25,
        19, 26,
        20, 27,
        21, 28};

    double dof_action_scale[NUM_DOF];
    const double limit_dof_tau[NUM_DOF] = {
        88.0, 139.0, 88.0, 139.0, 50.0, 50.0,
        88.0, 139.0, 88.0, 139.0, 50.0, 50.0,
        88.0, 50.0, 50.0,
        25.0, 25.0, 25.0, 25.0, 25.0, 5.0, 5.0,
        25.0, 25.0, 25.0, 25.0, 25.0, 5.0, 5.0};

    const double dof_Kps[NUM_DOF] = {
        40.179f, 99.098f, 40.179f, 99.098f, 28.501f, 28.501f,
        40.179f, 99.098f, 40.179f, 99.098f, 28.501f, 28.501f,
        40.179f, 28.501f, 28.501f,
        14.251f, 14.251f, 14.251f, 14.251f, 14.251f, 16.778f, 16.778f,
        14.251f, 14.251f, 14.251f, 14.251f, 14.251f, 16.778f, 16.778f};

    const double dof_Kds[NUM_DOF] = {
        2.558f, 6.309f, 2.558f, 6.309f, 1.814f, 1.814f,
        2.558f, 6.309f, 2.558f, 6.309f, 1.814f, 1.814f,
        2.558f, 1.814f, 1.814f,
        0.907f, 0.907f, 0.907f, 0.907f, 0.907f, 1.068f, 1.068f,
        0.907f, 0.907f, 0.907f, 0.907f, 0.907f, 1.068f, 1.068f};

    // const double dof_Kps[NUM_DOF] = {STIFFNESS_7520_22, STIFFNESS_7520_22, STIFFNESS_7520_14, STIFFNESS_7520_22, 2.0 * STIFFNESS_5020, 2.0 * STIFFNESS_5020,
    //                             STIFFNESS_7520_22, STIFFNESS_7520_22, STIFFNESS_7520_14, STIFFNESS_7520_22, 2.0 * STIFFNESS_5020, 2.0 * STIFFNESS_5020,
    //                             STIFFNESS_7520_14, 2.0 * STIFFNESS_5020, 2.0 * STIFFNESS_5020,
    //                             STIFFNESS_5020, STIFFNESS_5020, STIFFNESS_5020, STIFFNESS_5020, STIFFNESS_5020, STIFFNESS_5010_16, STIFFNESS_5010_16,
    //                             STIFFNESS_5020, STIFFNESS_5020, STIFFNESS_5020, STIFFNESS_5020, STIFFNESS_5020, STIFFNESS_5010_16, STIFFNESS_5010_16,}; // 电机Kp参数

    // const double dof_Kds[NUM_DOF] = {DAMPING_7520_22, DAMPING_7520_22, DAMPING_7520_14, DAMPING_7520_22, 2.0 * DAMPING_5020, 2.0 * DAMPING_5020,
    //                             DAMPING_7520_22, DAMPING_7520_22, DAMPING_7520_14, DAMPING_7520_22, 2.0 * DAMPING_5020, 2.0 * DAMPING_5020,
    //                             DAMPING_7520_14, 2.0 * DAMPING_5020, 2.0 * DAMPING_5020,
    //                             DAMPING_5020, DAMPING_5020, DAMPING_5020, DAMPING_5020, DAMPING_5020, DAMPING_5010_16, DAMPING_5010_16,
    //                             DAMPING_5020, DAMPING_5020, DAMPING_5020, DAMPING_5020, DAMPING_5020, DAMPING_5010_16, DAMPING_5010_16,};
};

#endif
