#pragma once

#include "common/protocol.h"

struct ControlConfig {
    float max_linear_velocity = 0.65f;
    float max_angular_velocity = 2.4f;
    float angular_gain = 2.2f;
    float stop_distance = 3.0f;
    float slow_distance = 22.0f;
    float align_angle = 0.18f;
    float forward_angle = 1.05f;
    float linear_acceleration = 1.2f;
    float angular_acceleration = 5.0f;
};

struct ControlState {
    float last_linear_velocity = 0.0f;
    float last_angular_velocity = 0.0f;
};

struct ControlResult {
    TwistCmd command{};
    float distance = 0.0f;
    float target_angle = 0.0f;
    float angle_error = 0.0f;
    bool target_reached = false;
    bool valid = false;
};

ControlResult compute_twist_control(
    const RobotPose& robot,
    Vec2 ball,
    const ControlConfig& config,
    ControlState& state,
    float dt_seconds
);
