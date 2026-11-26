#include "strategy/Controller.hpp"

#include <algorithm>
#include <cmath>
#include <numbers>

namespace {

constexpr float PI = std::numbers::pi_v<float>;
constexpr float FIELD_HALF_WIDTH = 118.5f;   // cm, igual a vision/src/Calibration.cpp
constexpr float FIELD_HALF_HEIGHT = 82.5f;

bool finite(float value) {
    return std::isfinite(value);
}

bool valid_position(Vec2 position) {
    return position.x >= -FIELD_HALF_WIDTH &&
        position.x <= FIELD_HALF_WIDTH &&
        position.y >= -FIELD_HALF_HEIGHT &&
        position.y <= FIELD_HALF_HEIGHT;
}

float rate_limit(float desired, float previous, float maximum_delta) {
    return previous + std::clamp(desired - previous, -maximum_delta, maximum_delta);
}

} // namespace

ControlResult compute_twist_control(
    const RobotPose& robot,
    Vec2 ball,
    const ControlConfig& config,
    ControlState& state,
    float dt_seconds
) {
    ControlResult result{};
    const float dx = static_cast<float>(ball.x - robot.position.x);
    const float dy = static_cast<float>(ball.y - robot.position.y);
    result.distance = std::hypot(dx, dy);

    if (!finite(robot.angle) || !finite(result.distance) ||
        !valid_position(robot.position) || !valid_position(ball) ||
        config.max_linear_velocity <= 0.0f ||
        config.max_angular_velocity <= 0.0f) {
        state = {};
        return result;
    }

    result.valid = true;
    result.target_angle = std::atan2(dy, dx);
    result.angle_error = std::remainder(
        result.target_angle - robot.angle,
        2.0f * PI
    );

    float desired_linear = 0.0f;
    float desired_angular = config.angular_gain * result.angle_error;

    if (result.distance <= config.stop_distance) {
        result.target_reached = true;
        if (std::abs(result.angle_error) <= config.align_angle) {
            desired_angular = 0.0f;
        }
    } else {
        const float distance_factor = std::clamp(
            result.distance / config.slow_distance,
            0.0f,
            1.0f
        );
        const float heading_factor = std::max(0.0f, std::cos(result.angle_error));
        desired_linear = config.max_linear_velocity * distance_factor * heading_factor;

        // Quando a bola está muito fora da direção atual, gira primeiro.
        if (std::abs(result.angle_error) > config.forward_angle) {
            desired_linear *= 0.12f;
        }
    }

    const float dt = std::clamp(dt_seconds, 0.001f, 0.25f);
    state.last_linear_velocity = rate_limit(
        std::clamp(desired_linear, 0.0f, config.max_linear_velocity),
        state.last_linear_velocity,
        config.linear_acceleration * dt
    );
    state.last_angular_velocity = rate_limit(
        std::clamp(desired_angular, -config.max_angular_velocity, config.max_angular_velocity),
        state.last_angular_velocity,
        config.angular_acceleration * dt
    );

    result.command.v = state.last_linear_velocity;
    result.command.w = state.last_angular_velocity;
    return result;
}
