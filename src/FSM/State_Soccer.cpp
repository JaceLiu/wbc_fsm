#include "FSM/State_Soccer.h"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <fstream>
#include <iostream>
#include <nlohmann/json.hpp>

using json = nlohmann::json;
using geometry_msgs::msg::dds_::Point_;
using unitree::robot::ChannelSubscriber;

State_Soccer::State_Soccer(CtrlComponents *ctrlComp)
    : FSMState(ctrlComp, FSMStateName::SOCCER, "soccer")
{
    std::string config_path = std::string(PROJECT_ROOT_DIR) + "/config/soccer.json";
    std::ifstream config_file(config_path);
    if (!config_file.is_open()) {
        std::cerr << "[ERROR][State_Soccer] Failed to open config file: " << config_path << std::endl;
        return;
    }

    try {
        json config = json::parse(config_file);
        std::string base_path = std::string(PROJECT_ROOT_DIR) + "/";
        _model_path = base_path + config.at("model_path").get<std::string>();
        _motion_file = config.at("motion_file").get<std::string>();
        _anchor_terminate_thresh = config.value("safe_projgravity_threshold", 2.6f);
        _clip_observations = config.value("clip_observations", 100.0f);
        _clip_actions = config.value("clip_actions", 100.0f);
        _ball_topic = config.value("ball_topic", std::string("rt/soccer/ball_pos"));
        _target_topic = config.value("target_topic", std::string("rt/soccer/target_pos"));
        _anchor_body_index_ = config.value("anchor_body_index", 7);

        auto ball_init = config.value("ball_init", std::vector<float>{0.7f, 0.0f, 0.11f});
        auto target_init = config.value("target_init", std::vector<float>{1.2f, 0.0f, 0.11f});
        if (ball_init.size() == 3) {
            _ball_pos_w = {ball_init[0], ball_init[1], ball_init[2]};
        }
        if (target_init.size() == 3) {
            _target_pos_w = {target_init[0], target_init[1], target_init[2]};
        }
    } catch (const std::exception &e) {
        std::cerr << "[ERROR][State_Soccer] Failed to parse config file: " << e.what() << std::endl;
        return;
    }

    for (int i = 0; i < NUM_DOF; ++i) {
        dof_action_scale[i] = 0.25 * this->limit_dof_tau[i] / this->dof_Kps[i];
    }

    _ball_subscriber = std::make_shared<ChannelSubscriber<Point_>>(_ball_topic);
    _ball_subscriber->InitChannel(std::bind(&State_Soccer::_onBallPos, this, std::placeholders::_1), 1);
    _target_subscriber = std::make_shared<ChannelSubscriber<Point_>>(_target_topic);
    _target_subscriber->InitChannel(std::bind(&State_Soccer::_onTargetPos, this, std::placeholders::_1), 1);

    _loadPolicy();
    _loadMotionReference();
}

void State_Soccer::_loadPolicy()
{
    try {
        _session_options.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_ALL);
        _session = std::make_unique<Ort::Session>(_env, _model_path.c_str(), _session_options);

        _obs_shape = _session->GetInputTypeInfo(0).GetTensorTypeAndShapeInfo().GetShape();
        _h_shape = _session->GetInputTypeInfo(1).GetTensorTypeAndShapeInfo().GetShape();
        _c_shape = _session->GetInputTypeInfo(2).GetTensorTypeAndShapeInfo().GetShape();
        auto action_shape = _session->GetOutputTypeInfo(0).GetTensorTypeAndShapeInfo().GetShape();

        if (_obs_shape.size() != 2 || _h_shape.size() != 3 || _c_shape.size() != 3 || action_shape.size() != 2) {
            throw std::runtime_error("Unexpected ONNX tensor rank for soccer model.");
        }

        _obs_size_ = _obs_shape[1];
        _action_size_ = action_shape[1];
        _h_size_ = _h_shape[0] * _h_shape[1] * _h_shape[2];
        _c_size_ = _c_shape[0] * _c_shape[1] * _c_shape[2];

        _observation.assign(_obs_size_, 0.0f);
        _action.assign(_action_size_, 0.0f);
        _h_state_.assign(_h_size_, 0.0f);
        _c_state_.assign(_c_size_, 0.0f);
        _ref_joint_pos.assign(NUM_DOF, 0.0f);
        _ref_joint_vel.assign(NUM_DOF, 0.0f);
        _ref_anchor_body_ang_vel = {0.0f, 0.0f, 0.0f};

        _model_ready_ = (_obs_size_ == 160 && _action_size_ == NUM_DOF);
        if (!_model_ready_) {
            std::cerr << "[ERROR][State_Soccer] Unexpected model dimensions. obs=" << _obs_size_
                      << ", action=" << _action_size_ << " (expected 160/29)." << std::endl;
            return;
        }

        std::cout << "[State_Soccer] Loaded model: " << _model_path << std::endl;
        std::cout << "[State_Soccer] obs=" << _obs_size_ << ", action=" << _action_size_
                  << ", h_size=" << _h_size_ << ", c_size=" << _c_size_ << std::endl;
    } catch (const std::exception &e) {
        std::cerr << "[ERROR][State_Soccer] Failed to load ONNX model: " << e.what() << std::endl;
        _model_ready_ = false;
    }
}

void State_Soccer::_loadMotionReference()
{
    try {
        if (_motion_file.empty()) {
            throw std::runtime_error("motion_file is empty in soccer.json");
        }

        cnpy::npz_t npz = cnpy::npz_load(_motion_file);
        if (npz.find("joint_pos") == npz.end() || npz.find("joint_vel") == npz.end() || npz.find("body_ang_vel_w") == npz.end()) {
            throw std::runtime_error("motion npz missing required arrays: joint_pos/joint_vel/body_ang_vel_w");
        }

        const cnpy::NpyArray& jp = npz.at("joint_pos");
        const cnpy::NpyArray& jv = npz.at("joint_vel");
        const cnpy::NpyArray& bav = npz.at("body_ang_vel_w");

        if (jp.shape.size() != 2 || jv.shape.size() != 2 || bav.shape.size() != 3) {
            throw std::runtime_error("motion npz shape rank mismatch");
        }

        _motion_steps_ = static_cast<int64_t>(jp.shape[0]);
        _motion_body_count_ = static_cast<int64_t>(bav.shape[1]);

        if (jp.shape[1] != NUM_DOF || jv.shape[1] != NUM_DOF) {
            throw std::runtime_error("joint_pos/joint_vel second dim must be 29");
        }
        if (jv.shape[0] != static_cast<size_t>(_motion_steps_)) {
            throw std::runtime_error("joint_vel first dim mismatch with joint_pos");
        }
        if (bav.shape[0] != static_cast<size_t>(_motion_steps_) || bav.shape[2] != 3) {
            throw std::runtime_error("body_ang_vel_w shape must be [T, B, 3]");
        }
        if (_anchor_body_index_ < 0 || _anchor_body_index_ >= _motion_body_count_) {
            throw std::runtime_error("anchor_body_index out of range for body_ang_vel_w");
        }

        _motion_joint_pos = jp.as_vec<float>();
        _motion_joint_vel = jv.as_vec<float>();
        _motion_body_ang_vel = bav.as_vec<float>();

        _motion_ready_ = (_motion_steps_ > 0);
        std::cout << "[State_Soccer] Loaded motion reference: " << _motion_file
                  << " steps=" << _motion_steps_ << " bodies=" << _motion_body_count_ << std::endl;
    } catch (const std::exception& e) {
        std::cerr << "[ERROR][State_Soccer] Failed to load motion reference: " << e.what() << std::endl;
        _motion_ready_ = false;
    }
}

void State_Soccer::_reset_lstm_state()
{
    std::fill(_h_state_.begin(), _h_state_.end(), 0.0f);
    std::fill(_c_state_.begin(), _c_state_.end(), 0.0f);
    _time_step_input_ = 0.0f;
    _time_step_index_ = 0;
}

std::array<float, 3> State_Soccer::_world_to_base(const std::array<float, 3> &pos_w) const
{
    std::vector<float> base_quat = {
        _lowState->imu.quaternion[0],
        _lowState->imu.quaternion[1],
        _lowState->imu.quaternion[2],
        _lowState->imu.quaternion[3]};
    std::vector<float> delta_w = {
        pos_w[0] - _lowState->basePos[0],
        pos_w[1] - _lowState->basePos[1],
        pos_w[2] - _lowState->basePos[2]};
    auto delta_b = quat_apply_inverse(base_quat, delta_w);
    return {delta_b[0], delta_b[1], delta_b[2]};
}

void State_Soccer::_refresh_reference(int64_t time_step)
{
    if (!_motion_ready_) {
        _terminate_flag = true;
        return;
    }

    if (_motion_steps_ <= 0) {
        _terminate_flag = true;
        return;
    }

    const int64_t t = time_step % _motion_steps_;
    const size_t jt_off = static_cast<size_t>(t) * NUM_DOF;
    std::memcpy(_ref_joint_pos.data(), _motion_joint_pos.data() + jt_off, NUM_DOF * sizeof(float));
    std::memcpy(_ref_joint_vel.data(), _motion_joint_vel.data() + jt_off, NUM_DOF * sizeof(float));

    const size_t ang_off = (static_cast<size_t>(t) * static_cast<size_t>(_motion_body_count_) + static_cast<size_t>(_anchor_body_index_)) * 3;
    _ref_anchor_body_ang_vel[0] = _motion_body_ang_vel[ang_off + 0];
    _ref_anchor_body_ang_vel[1] = _motion_body_ang_vel[ang_off + 1];
    _ref_anchor_body_ang_vel[2] = _motion_body_ang_vel[ang_off + 2];
}

void State_Soccer::_observations_compute()
{
    if (!_model_ready_ || !_motion_ready_) {
        return;
    }

    std::vector<float> obs;
    obs.reserve(160);

    // 1) command: reference joint_pos(29) + joint_vel(29)
    obs.insert(obs.end(), _ref_joint_pos.begin(), _ref_joint_pos.end());
    obs.insert(obs.end(), _ref_joint_vel.begin(), _ref_joint_vel.end());

    // 2) projected_gravity(3)
    std::vector<float> base_quat = {
        _lowState->imu.quaternion[0],
        _lowState->imu.quaternion[1],
        _lowState->imu.quaternion[2],
        _lowState->imu.quaternion[3]};
    auto projected_gravity = QuatRotateInverse(base_quat, _gravity_vec);
    obs.insert(obs.end(), projected_gravity.begin(), projected_gravity.end());

    // 3) motion_ref_ang_vel(3), torso anchor index defaults to 7
    obs.push_back(_ref_anchor_body_ang_vel[0]);
    obs.push_back(_ref_anchor_body_ang_vel[1]);
    obs.push_back(_ref_anchor_body_ang_vel[2]);

    // 4) base_ang_vel(3)
    obs.push_back(static_cast<float>(_lowState->imu.gyroscope[0]));
    obs.push_back(static_cast<float>(_lowState->imu.gyroscope[1]));
    obs.push_back(static_cast<float>(_lowState->imu.gyroscope[2]));

    // 5) joint_pos_rel(29)
    for (int i = 0; i < NUM_DOF; ++i) {
        obs.push_back(_lowState->motorState[dof_mapping_mj[i]].q - _default_dof_pos[dof_mapping_mj[i]]);
    }

    // 6) joint_vel(29)
    for (int i = 0; i < NUM_DOF; ++i) {
        obs.push_back(_lowState->motorState[dof_mapping_mj[i]].dq);
    }

    // 7) actions(29), previous action
    obs.insert(obs.end(), _action.begin(), _action.end());

    // 8/9) ball and target positions in base frame
    std::array<float, 3> ball_w;
    std::array<float, 3> target_w;
    {
        std::lock_guard<std::mutex> lock(_soccer_mutex);
        ball_w = _ball_pos_w;
        target_w = _target_pos_w;
    }
    auto ball_b = _world_to_base(ball_w);
    auto target_b = _world_to_base(target_w);
    obs.push_back(ball_b[0]);
    obs.push_back(ball_b[1]);
    obs.push_back(ball_b[2]);
    obs.push_back(target_b[0]);
    obs.push_back(target_b[1]);
    obs.push_back(target_b[2]);

    if (obs.size() != static_cast<size_t>(_obs_size_)) {
        std::cerr << "[ERROR][State_Soccer] Observation size mismatch. got=" << obs.size()
                  << ", expected=" << _obs_size_ << std::endl;
        _terminate_flag = true;
        return;
    }

    _observation = obs;
    for (auto &v : _observation) {
        v = std::max(-_clip_observations, std::min(v, _clip_observations));
    }

    float gravity_err = std::abs(projected_gravity[2] - (-1.0f));
    if (gravity_err > _anchor_terminate_thresh) {
        _terminate_flag = true;
        std::cout << "[State_Soccer] Terminate by projected gravity error: " << gravity_err << std::endl;
    }
}

void State_Soccer::_action_compute()
{
    if (!_model_ready_ || _terminate_flag) {
        return;
    }

    try {
        auto memory_info = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeCPU);
        std::vector<Ort::Value> input_tensors;
        std::vector<int64_t> obs_shape = {1, _obs_size_};
        std::vector<int64_t> step_shape = {1, 1};

        input_tensors.push_back(Ort::Value::CreateTensor<float>(
            memory_info, _observation.data(), _observation.size(), obs_shape.data(), obs_shape.size()));
        input_tensors.push_back(Ort::Value::CreateTensor<float>(
            memory_info, _h_state_.data(), _h_state_.size(), _h_shape.data(), _h_shape.size()));
        input_tensors.push_back(Ort::Value::CreateTensor<float>(
            memory_info, _c_state_.data(), _c_state_.size(), _c_shape.data(), _c_shape.size()));
        input_tensors.push_back(Ort::Value::CreateTensor<float>(
            memory_info, &_time_step_input_, 1, step_shape.data(), step_shape.size()));

        auto output_tensors = _session->Run(
            Ort::RunOptions{nullptr},
            _input_names.data(),
            input_tensors.data(),
            input_tensors.size(),
            _output_names.data(),
            _output_names.size());

        float *actions = output_tensors[0].GetTensorMutableData<float>();
        float *h_out = output_tensors[1].GetTensorMutableData<float>();
        float *c_out = output_tensors[2].GetTensorMutableData<float>();

        std::memcpy(_action.data(), actions, _action.size() * sizeof(float));
        std::memcpy(_h_state_.data(), h_out, _h_state_.size() * sizeof(float));
        std::memcpy(_c_state_.data(), c_out, _c_state_.size() * sizeof(float));

        _time_step_index_ = (_time_step_index_ + 1) % std::max<int64_t>(1, _motion_steps_);
        _time_step_input_ = static_cast<float>(_time_step_index_);

        for (int i = 0; i < NUM_DOF; ++i) {
            _action[i] = std::max(-_clip_actions, std::min(_action[i], _clip_actions));
            _joint_q[dof_mapping_mj[i]] = _action[i] * dof_action_scale[dof_mapping_mj[i]] + _default_dof_pos[dof_mapping_mj[i]];
        }
    } catch (const std::exception &e) {
        std::cerr << "[ERROR][State_Soccer] ONNX inference failed: " << e.what() << std::endl;
        _terminate_flag = true;
    }
}

void State_Soccer::enter()
{
    _terminate_flag = !(_model_ready_ && _motion_ready_);
    _reset_lstm_state();
    std::fill(_action.begin(), _action.end(), 0.0f);
    std::fill(_ref_joint_pos.begin(), _ref_joint_pos.end(), 0.0f);
    std::fill(_ref_joint_vel.begin(), _ref_joint_vel.end(), 0.0f);
    _ref_anchor_body_ang_vel = {0.0f, 0.0f, 0.0f};

    for (int i = 0; i < NUM_DOF; ++i) {
        _lowCmd->motorCmd[i].mode = 10;
        _lowCmd->motorCmd[i].q = _lowState->motorState[i].q;
        _lowCmd->motorCmd[i].dq = 0.0f;
        _lowCmd->motorCmd[i].tau = 0.0f;
        _lowCmd->motorCmd[i].Kp = this->dof_Kps[i];
        _lowCmd->motorCmd[i].Kd = this->dof_Kds[i];
        _targetPos_rl[i] = _default_dof_pos[i];
        _last_targetPos_rl[i] = _lowState->motorState[i].q;
        _joint_q[i] = _default_dof_pos[i];
    }

    std::cout << "[State_Soccer] Entered SOCCER state." << std::endl;
}

void State_Soccer::run()
{
    if (!(_model_ready_ && _motion_ready_)) {
        _terminate_flag = true;
        return;
    }

    _refresh_reference(_time_step_index_);
    _observations_compute();
    _action_compute();
    if (_terminate_flag) {
        return;
    }

    std::memcpy(_targetPos_rl, _joint_q, sizeof(_joint_q));
    for (int j = 0; j < NUM_DOF; ++j) {
        _lowCmd->motorCmd[j].mode = 10;
        _lowCmd->motorCmd[j].q = _targetPos_rl[j];
        _lowCmd->motorCmd[j].dq = 0.0f;
        _lowCmd->motorCmd[j].tau = 0.0f;
        _lowCmd->motorCmd[j].Kp = this->dof_Kps[j];
        _lowCmd->motorCmd[j].Kd = this->dof_Kds[j];
        _last_targetPos_rl[j] = _targetPos_rl[j];
    }
}

void State_Soccer::exit()
{
    _reset_lstm_state();
    std::cout << "[State_Soccer] Exiting SOCCER state." << std::endl;
}

FSMStateName State_Soccer::checkChange()
{
    if (_lowState->userCmd == UserCommand::L2_B) {
        return FSMStateName::PASSIVE;
    }
    if (_terminate_flag) {
        std::cout << "[FSM] SOCCER terminate -> MJAMP" << std::endl;
        return FSMStateName::MJAMP;
    }
    if (_lowState->userCmd == UserCommand::R2_B) {
        std::cout << "[FSM] SOCCER -> MJAMP by R2_B" << std::endl;
        return FSMStateName::MJAMP;
    }
    if (_lowState->userCmd == UserCommand::SELECT) {
        throw std::runtime_error("exit..");
        return FSMStateName::PASSIVE;
    }
    return FSMStateName::SOCCER;
}

void State_Soccer::_onBallPos(const void *message)
{
    const auto msg = *(const Point_ *)message;
    std::lock_guard<std::mutex> lock(_soccer_mutex);
    _ball_pos_w[0] = static_cast<float>(msg.x());
    _ball_pos_w[1] = static_cast<float>(msg.y());
    _ball_pos_w[2] = static_cast<float>(msg.z());
}

void State_Soccer::_onTargetPos(const void *message)
{
    const auto msg = *(const Point_ *)message;
    std::lock_guard<std::mutex> lock(_soccer_mutex);
    _target_pos_w[0] = static_cast<float>(msg.x());
    _target_pos_w[1] = static_cast<float>(msg.y());
    _target_pos_w[2] = static_cast<float>(msg.z());
}
