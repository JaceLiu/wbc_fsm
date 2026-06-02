#ifndef G1_SOCCER_PERCEPTION_BRIDGE_H
#define G1_SOCCER_PERCEPTION_BRIDGE_H

#include <array>
#include <atomic>
#include <cstdint>
#include <chrono>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

#include <unitree/idl/go2/MotorStates_.hpp>
#include <unitree/idl/go2/SportModeState_.hpp>
#include <unitree/idl/hg/LowState_.hpp>
#include <unitree/idl/ros2/Point_.hpp>
#include <unitree/robot/channel/channel_publisher.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>

#include "DetectionModule.hpp"
#include "LocationModule.hpp"

class G1SoccerPerceptionBridge {
public:
    explicit G1SoccerPerceptionBridge(const std::string& config_path);
    ~G1SoccerPerceptionBridge();

    void proceed();

private:
    struct Pose2D {
        float x = 0.0f;
        float y = 0.0f;
        float theta = 0.0f;
    };

    struct RelativePoint {
        float x = 0.0f;
        float y = 0.0f;
        float score = 0.0f;
    };

    void loadConfig(const std::string& config_path);
    void handleDetection(const void* message);
    void handleLocation(const void* message);
    void handleServoState(const void* message);
    void handleLowState(const void* message);
    void handleSportState(const void* message);

    std::optional<RelativePoint> detectionToRobotRelative(
            const DetectionModule::DetectionResult& result) const;
    std::array<float, 3> robotRelativeToWorld(float rel_x, float rel_y, float z) const;
    std::array<float, 3> opponentGoalFallbackWorld() const;
    void publishPoint(unitree::robot::ChannelPublisherPtr<geometry_msgs::msg::dds_::Point_>& publisher,
                      const std::array<float, 3>& point);
    void publishTargetFromGoalposts(const std::vector<RelativePoint>& goalposts, int64_t now_us);
    void publishFallbackTargetIfNeeded(int64_t now_us);
    std::optional<RelativePoint> chooseGoalTarget(const std::vector<RelativePoint>& goalposts) const;

    static bool isBallClass(const std::string& class_name);
    static bool isGoalpostClass(const std::string& class_name);
    static std::string lower(std::string value);

    std::string detection_topic_ = "detectionresults";
    std::string location_topic_ = "rt/locationresults";
    std::string servo_state_topic_ = "rt/g1_comp_servo/state";
    std::string lowstate_topic_ = "rt/lowstate";
    std::string sport_state_topic_ = "rt/sportmodestate";
    std::string ball_topic_ = "rt/soccer/ball_pos";
    std::string target_topic_ = "rt/soccer/target_pos";

    float ball_min_score_ = 0.70f;
    float goalpost_min_score_ = 0.35f;
    float ball_z_ = 0.11f;
    float target_z_ = 0.11f;
    float field_length_m_ = 14.0f;
    float target_timeout_sec_ = 1.0f;
    bool opponent_goal_ = true;
    bool enabled_ = true;

    unitree::robot::ChannelSubscriberPtr<DetectionModule::DetectionResults> detection_subscriber_;
    unitree::robot::ChannelSubscriberPtr<LocationModule::LocationResult> location_subscriber_;
    unitree::robot::ChannelSubscriberPtr<unitree_go::msg::dds_::MotorStates_> servo_state_subscriber_;
    unitree::robot::ChannelSubscriberPtr<unitree_hg::msg::dds_::LowState_> lowstate_subscriber_;
    unitree::robot::ChannelSubscriberPtr<unitree_go::msg::dds_::SportModeState_> sport_state_subscriber_;
    unitree::robot::ChannelPublisherPtr<geometry_msgs::msg::dds_::Point_> ball_publisher_;
    unitree::robot::ChannelPublisherPtr<geometry_msgs::msg::dds_::Point_> target_publisher_;

    mutable std::mutex state_mutex_;
    Pose2D latest_field_pose_;
    std::array<float, 3> latest_odom_position_ = {0.0f, 0.0f, 0.0f};
    float latest_odom_yaw_ = 0.0f;
    float head_yaw_deg_ = 0.0f;
    float head_pitch_deg_ = 0.0f;
    float waist_yaw_rad_ = 0.0f;
    bool have_field_pose_ = false;
    bool have_odom_pose_ = false;

    std::atomic<int64_t> last_detection_us_{0};
    std::atomic<int64_t> last_location_us_{0};
    std::atomic<int64_t> last_target_us_{0};
    std::atomic<int64_t> last_ball_us_{0};
    int64_t last_log_us_ = 0;
};

#endif
