#include "perception/G1SoccerPerceptionBridge.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <nlohmann/json.hpp>
#include <thread>

using json = nlohmann::json;
using geometry_msgs::msg::dds_::Point_;
using unitree::robot::ChannelPublisher;
using unitree::robot::ChannelSubscriber;

namespace {
constexpr const char* kDefaultNetworkInterface = "eth0";
constexpr const char* kEnvNetworkInterface = "G1_NETWORK_INTERFACE";
constexpr const char* kEnvBridgeEnabled = "WBC_G1_SOCCER_BRIDGE";

int64_t nowUs() {
    using namespace std::chrono;
    return duration_cast<microseconds>(steady_clock::now().time_since_epoch()).count();
}

bool envBool(const char* key, bool default_value) {
    const char* raw = std::getenv(key);
    if (raw == nullptr) {
        return default_value;
    }
    std::string value(raw);
    std::transform(value.begin(), value.end(), value.begin(),
                   [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    return value == "1" || value == "true" || value == "yes" || value == "on";
}

std::string envString(const char* key, const std::string& default_value) {
    const char* raw = std::getenv(key);
    return raw == nullptr || raw[0] == '\0' ? default_value : std::string(raw);
}

float envFloat(const char* key, float default_value) {
    const char* raw = std::getenv(key);
    if (raw == nullptr) {
        return default_value;
    }
    char* end = nullptr;
    const float value = std::strtof(raw, &end);
    if (end == raw || !std::isfinite(value)) {
        return default_value;
    }
    return value;
}
}  // namespace

G1SoccerPerceptionBridge::G1SoccerPerceptionBridge(const std::string& config_path) {
    enabled_ = envBool(kEnvBridgeEnabled, true);
    loadConfig(config_path);
    if (!enabled_) {
        std::cout << "[G1SoccerBridge] disabled" << std::endl;
        return;
    }

    detection_subscriber_ = std::make_shared<ChannelSubscriber<DetectionModule::DetectionResults>>(detection_topic_);
    detection_subscriber_->InitChannel([this](const void* msg) { handleDetection(msg); }, 1);

    location_subscriber_ = std::make_shared<ChannelSubscriber<LocationModule::LocationResult>>(location_topic_);
    location_subscriber_->InitChannel([this](const void* msg) { handleLocation(msg); }, 1);

    servo_state_subscriber_ = std::make_shared<ChannelSubscriber<unitree_go::msg::dds_::MotorStates_>>(servo_state_topic_);
    servo_state_subscriber_->InitChannel([this](const void* msg) { handleServoState(msg); }, 1);

    lowstate_subscriber_ = std::make_shared<ChannelSubscriber<unitree_hg::msg::dds_::LowState_>>(lowstate_topic_);
    lowstate_subscriber_->InitChannel([this](const void* msg) { handleLowState(msg); }, 1);

    sport_state_subscriber_ = std::make_shared<ChannelSubscriber<unitree_go::msg::dds_::SportModeState_>>(sport_state_topic_);
    sport_state_subscriber_->InitChannel([this](const void* msg) { handleSportState(msg); }, 1);

    ball_publisher_ = std::make_shared<ChannelPublisher<Point_>>(ball_topic_);
    ball_publisher_->InitChannel();
    target_publisher_ = std::make_shared<ChannelPublisher<Point_>>(target_topic_);
    target_publisher_->InitChannel();

    std::cout << "[G1SoccerBridge] ready"
              << " det=" << detection_topic_
              << " loc=" << location_topic_
              << " ball=" << ball_topic_
              << " target=" << target_topic_ << std::endl;
}

G1SoccerPerceptionBridge::~G1SoccerPerceptionBridge() {
    if (detection_subscriber_) detection_subscriber_->CloseChannel();
    if (location_subscriber_) location_subscriber_->CloseChannel();
    if (servo_state_subscriber_) servo_state_subscriber_->CloseChannel();
    if (lowstate_subscriber_) lowstate_subscriber_->CloseChannel();
    if (sport_state_subscriber_) sport_state_subscriber_->CloseChannel();
    if (ball_publisher_) ball_publisher_->CloseChannel();
    if (target_publisher_) target_publisher_->CloseChannel();
}

void G1SoccerPerceptionBridge::loadConfig(const std::string& config_path) {
    std::ifstream in(config_path);
    if (!in.is_open()) {
        std::cerr << "[G1SoccerBridge] failed to open config: " << config_path << std::endl;
        return;
    }

    try {
        const json config = json::parse(in);
        detection_topic_ = config.value("g1_detection_topic", detection_topic_);
        location_topic_ = config.value("g1_location_topic", location_topic_);
        servo_state_topic_ = config.value("g1_servo_state_topic", servo_state_topic_);
        lowstate_topic_ = config.value("g1_lowstate_topic", lowstate_topic_);
        sport_state_topic_ = config.value("g1_sport_state_topic", sport_state_topic_);
        ball_topic_ = config.value("ball_topic", ball_topic_);
        target_topic_ = config.value("target_topic", target_topic_);
        ball_min_score_ = config.value("g1_ball_min_score", ball_min_score_);
        goalpost_min_score_ = config.value("g1_goalpost_min_score", goalpost_min_score_);
        target_timeout_sec_ = config.value("g1_target_timeout_sec", target_timeout_sec_);
        field_length_m_ = config.value("g1_field_length_m", field_length_m_);
        opponent_goal_ = config.value("g1_opponent_goal", opponent_goal_);
        ball_z_ = config.value("ball_init", std::vector<float>{0.7f, 0.0f, ball_z_}).size() == 3
                          ? config.value("ball_init", std::vector<float>{0.7f, 0.0f, ball_z_})[2]
                          : ball_z_;
        target_z_ = config.value("target_init", std::vector<float>{1.2f, 0.0f, target_z_}).size() == 3
                            ? config.value("target_init", std::vector<float>{1.2f, 0.0f, target_z_})[2]
                            : target_z_;
    } catch (const std::exception& e) {
        std::cerr << "[G1SoccerBridge] config parse error: " << e.what() << std::endl;
    }
}

std::string G1SoccerPerceptionBridge::lower(std::string value) {
    std::transform(value.begin(), value.end(), value.begin(),
                   [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    return value;
}

bool G1SoccerPerceptionBridge::isBallClass(const std::string& class_name) {
    const auto name = lower(class_name);
    return name == "ball" || name.find("ball") != std::string::npos;
}

bool G1SoccerPerceptionBridge::isGoalpostClass(const std::string& class_name) {
    const auto name = lower(class_name);
    return name == "goalpost" || name.find("goalpost") != std::string::npos;
}

std::optional<G1SoccerPerceptionBridge::RelativePoint>
G1SoccerPerceptionBridge::detectionToRobotRelative(const DetectionModule::DetectionResult& result) const {
    const auto& xyz = result.xyz();
    if (!std::isfinite(xyz[0]) || !std::isfinite(xyz[1]) || !std::isfinite(xyz[2])) {
        return std::nullopt;
    }
    std::lock_guard<std::mutex> lock(state_mutex_);
    const float head_yaw = head_yaw_deg_ * static_cast<float>(M_PI) / 180.0f;
    const float head_pitch = head_pitch_deg_ * static_cast<float>(M_PI) / 180.0f;
    const float waist_yaw = waist_yaw_rad_;

    const float cy = std::cos(head_yaw);
    const float sy = std::sin(head_yaw);
    const float cp = std::cos(head_pitch);
    const float sp = std::sin(head_pitch);
    const float cw = std::cos(waist_yaw);
    const float sw = std::sin(waist_yaw);

    const float x1 = xyz[0];
    const float y1 = xyz[1] * cp - xyz[2] * sp;
    const float z1 = xyz[1] * sp + xyz[2] * cp;
    const float x2 = x1 * cy + z1 * sy;
    const float z2 = -x1 * sy + z1 * cy;
    const float x3 = x2 * cw - y1 * sw;
    const float y3 = x2 * sw + y1 * cw;

    if (!std::isfinite(x3) || !std::isfinite(y3)) {
        return std::nullopt;
    }

    return RelativePoint{x3, y3, result.score()};
}

std::array<float, 3> G1SoccerPerceptionBridge::robotRelativeToWorld(float rel_x, float rel_y, float z) const {
    std::lock_guard<std::mutex> lock(state_mutex_);
    const float c = std::cos(latest_field_pose_.theta);
    const float s = std::sin(latest_field_pose_.theta);
    return {
        latest_field_pose_.x + rel_x * c - rel_y * s,
        latest_field_pose_.y + rel_x * s + rel_y * c,
        z,
    };
}

std::array<float, 3> G1SoccerPerceptionBridge::opponentGoalFallbackWorld() const {
    const float x = opponent_goal_ ? field_length_m_ * 0.5f : -field_length_m_ * 0.5f;
    return {x, 0.0f, target_z_};
}

std::optional<G1SoccerPerceptionBridge::RelativePoint>
G1SoccerPerceptionBridge::chooseGoalTarget(const std::vector<RelativePoint>& goalposts) const {
    if (goalposts.empty()) {
        return std::nullopt;
    }
    if (goalposts.size() == 1) {
        return goalposts.front();
    }
    RelativePoint best = goalposts.front();
    RelativePoint second = goalposts[1];
    if (second.score > best.score) {
        std::swap(best, second);
    }
    return RelativePoint{
        0.5f * (best.x + second.x),
        0.5f * (best.y + second.y),
        std::max(best.score, second.score),
    };
}

void G1SoccerPerceptionBridge::publishPoint(
        unitree::robot::ChannelPublisherPtr<Point_>& publisher, const std::array<float, 3>& point) {
    if (!publisher) {
        return;
    }
    Point_ msg;
    msg.x(point[0]);
    msg.y(point[1]);
    msg.z(point[2]);
    publisher->Write(msg);
}

void G1SoccerPerceptionBridge::publishTargetFromGoalposts(
        const std::vector<RelativePoint>& goalposts, int64_t now_us) {
    const auto target_rel = chooseGoalTarget(goalposts);
    const std::array<float, 3> target_w = target_rel.has_value()
            ? robotRelativeToWorld(target_rel->x, target_rel->y, target_z_)
            : opponentGoalFallbackWorld();
    publishPoint(target_publisher_, target_w);
    last_target_us_.store(now_us);
}

void G1SoccerPerceptionBridge::publishFallbackTargetIfNeeded(int64_t now_us) {
    if (last_target_us_.load() != 0 &&
        now_us - last_target_us_.load() < static_cast<int64_t>(target_timeout_sec_ * 1e6f)) {
        return;
    }
    publishPoint(target_publisher_, opponentGoalFallbackWorld());
}

void G1SoccerPerceptionBridge::handleDetection(const void* message) {
    const auto* results = static_cast<const DetectionModule::DetectionResults*>(message);
    const int64_t now_us = nowUs();
    last_detection_us_.store(now_us);

    std::vector<RelativePoint> goalposts;
    RelativePoint best_ball;
    bool have_ball = false;

    for (const auto& result : results->results()) {
        if (isBallClass(result.class_name()) && result.score() >= ball_min_score_) {
            if (auto rel = detectionToRobotRelative(result)) {
                if (!have_ball || rel->score > best_ball.score) {
                    best_ball = *rel;
                    have_ball = true;
                }
            }
            continue;
        }
        if (isGoalpostClass(result.class_name()) && result.score() >= goalpost_min_score_) {
            if (auto rel = detectionToRobotRelative(result)) {
                goalposts.push_back(*rel);
            }
        }
    }

    if (have_ball) {
        publishPoint(ball_publisher_, robotRelativeToWorld(best_ball.x, best_ball.y, ball_z_));
        last_ball_us_.store(now_us);
    }

    publishTargetFromGoalposts(goalposts, now_us);
}

void G1SoccerPerceptionBridge::handleLocation(const void* message) {
    const auto* location = static_cast<const LocationModule::LocationResult*>(message);
    std::lock_guard<std::mutex> lock(state_mutex_);
    latest_field_pose_ = {location->robot2field_x(), location->robot2field_y(), location->robot2field_theta()};
    have_field_pose_ = true;
    last_location_us_.store(nowUs());
}

void G1SoccerPerceptionBridge::handleServoState(const void* message) {
    const auto* state = static_cast<const unitree_go::msg::dds_::MotorStates_*>(message);
    if (state->states().size() <= 1) {
        return;
    }
    std::lock_guard<std::mutex> lock(state_mutex_);
    head_yaw_deg_ = static_cast<float>(state->states()[0].q());
    head_pitch_deg_ = static_cast<float>(state->states()[1].q());
}

void G1SoccerPerceptionBridge::handleLowState(const void* message) {
    const auto* state = static_cast<const unitree_hg::msg::dds_::LowState_*>(message);
    if (state->motor_state().size() <= 12) {
        return;
    }
    std::lock_guard<std::mutex> lock(state_mutex_);
    waist_yaw_rad_ = static_cast<float>(state->motor_state()[12].q());
}

void G1SoccerPerceptionBridge::handleSportState(const void* message) {
    const auto* state = static_cast<const unitree_go::msg::dds_::SportModeState_*>(message);
    (void)state;
}

void G1SoccerPerceptionBridge::proceed() {
    if (!enabled_) {
        return;
    }
    publishFallbackTargetIfNeeded(nowUs());
}
