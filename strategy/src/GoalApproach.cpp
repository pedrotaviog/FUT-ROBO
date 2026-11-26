#include "strategy/GoalApproach.hpp"

#include <algorithm>
#include <cmath>
#include <numbers>
#include "strategy/ControlParams.hpp"

namespace {

constexpr float TWO_PI = 2.0f * std::numbers::pi_v<float>;

float wrap(float angle) {
    return std::remainder(angle, TWO_PI);
}

// Distância do ponto (px, py) ao segmento (ax, ay) -> (bx, by).
float distance_to_segment(float px, float py, float ax, float ay, float bx, float by) {
    const float sx = bx - ax;
    const float sy = by - ay;
    const float len2 = sx * sx + sy * sy;
    const float t = len2 > 0.0f ? std::clamp(((px - ax) * sx + (py - ay) * sy) / len2, 0.0f, 1.0f) : 0.0f;
    return std::hypot(px - (ax + t * sx), py - (ay + t * sy));
}

} // namespace

GoalApproachOutput compute_goal_approach(const GoalApproachInput& in, GoalApproachState& state) {
    GoalApproachOutput out;

    // Direção bola -> gol (u) e a perpendicular (n). Se a bola estiver em
    // cima do centro do gol, qualquer direção serve: usa +x.
    float ux = ATTACK_GOAL_X_M - in.ball_x;
    float uy = ATTACK_GOAL_Y_M - in.ball_y;
    const float to_goal = std::hypot(ux, uy);
    if (to_goal > 1e-3f) {
        ux /= to_goal;
        uy /= to_goal;
    } else {
        ux = 1.0f;
        uy = 0.0f;
    }
    const float nx = -uy;
    const float ny = ux;
    const float goal_heading = std::atan2(uy, ux);

    // Ponto alvo: atrás da bola na linha gol -> bola, dentro do campo.
    const float pose_x = std::clamp(in.ball_x - BEHIND_BALL_M * ux,
                                    -FIELD_HALF_X_M + FIELD_MARGIN_M, FIELD_HALF_X_M - FIELD_MARGIN_M);
    const float pose_y = std::clamp(in.ball_y - BEHIND_BALL_M * uy,
                                    -FIELD_HALF_Y_M + FIELD_MARGIN_M, FIELD_HALF_Y_M - FIELD_MARGIN_M);

    // Contornar a bola se o caminho reto até o ponto alvo passa perto dela.
    // O lado é escolhido uma vez (o do robô) e mantido até o caminho liberar,
    // para ele não ficar indeciso quando está bem na frente da bola.
    const float path_clearance = distance_to_segment(in.ball_x, in.ball_y,
                                                     in.robot_x, in.robot_y, pose_x, pose_y);
    if (state.avoid_side == 0 && path_clearance < BALL_CLEARANCE_M) {
        const float side = (in.robot_x - in.ball_x) * nx + (in.robot_y - in.ball_y) * ny;
        state.avoid_side = side >= 0.0f ? 1 : -1;
    }

    float target_x = pose_x;
    float target_y = pose_y;
    if (state.avoid_side != 0) {
        const float avoid_x = in.ball_x - AVOID_BEHIND_M * ux + state.avoid_side * AVOID_SIDE_M * nx;
        const float avoid_y = in.ball_y - AVOID_BEHIND_M * uy + state.avoid_side * AVOID_SIDE_M * ny;
        const bool reached = std::hypot(avoid_x - in.robot_x, avoid_y - in.robot_y) < POSE_LEFT_M;
        if (reached || path_clearance > BALL_CLEARANCE_EXIT_M) {
            state.avoid_side = 0;
        } else if (distance_to_segment(in.ball_x, in.ball_y, in.robot_x, in.robot_y, avoid_x, avoid_y)
                   < BALL_CLEARANCE_EXIT_M) {
            // A reta até o ponto de contorno também passa perto da bola
            // (robô na frente dela): vai antes para o lado da bola. Usa a folga
            // maior porque, com o atraso, o caminho real fica mais perto que a reta.
            target_x = in.ball_x + state.avoid_side * AVOID_SIDE_M * nx;
            target_y = in.ball_y + state.avoid_side * AVOID_SIDE_M * ny;
            state.at_pose = false;
        } else {
            target_x = avoid_x;
            target_y = avoid_y;
            state.at_pose = false;
        }
    }

    const float dx = target_x - in.robot_x;
    const float dy = target_y - in.robot_y;
    out.rho = std::hypot(dx, dy);
    const float target_dir = std::atan2(dy, dx);
    out.alpha = wrap(target_dir - in.robot_angle);
    // No ponto de contorno a orientação final não importa (beta = 0).
    out.beta = state.avoid_side != 0 ? 0.0f : wrap(goal_heading - target_dir);
    out.heading_error = wrap(goal_heading - in.robot_angle);

    // Histerese de "chegou no ponto alvo"
    if (state.avoid_side == 0) {
        if (state.at_pose) {
            if (out.rho > POSE_LEFT_M) state.at_pose = false;
        } else if (out.rho < POSE_REACHED_M) {
            state.at_pose = true;
        }
    }

    if (state.at_pose) {
        // No ponto alvo: só gira no lugar até ficar virado para o gol, com a
        // mesma zona morta com histerese do modo "ir até a bola".
        if (state.aligned) {
            if (std::abs(out.heading_error) > ALIGN_OUT) state.aligned = false;
        } else if (std::abs(out.heading_error) < ALIGN_IN) {
            state.aligned = true;
        }
        out.v = 0.0f;
        out.w = state.aligned ? 0.0f : turn_in_place_command(out.heading_error, in.angular_rate);
        out.mode = state.aligned ? "POSICIONADO" : "virando p/ gol";
    } else {
        state.aligned = false;
        // Lei polar. O D sobre o giro medido é o mesmo do modo "ir até a bola".
        out.w = K_ALPHA * out.alpha + K_BETA * out.beta - KD_ANGULAR * in.angular_rate;
        // Só anda para a frente e só quando já aponta razoavelmente para o
        // alvo; o cos() desacelera quando está torto.
        const float cone = state.avoid_side != 0 ? AVOID_ADVANCE_CONE : ADVANCE_CONE;
        out.v = std::abs(out.alpha) < cone
            ? std::min(K_RHO * out.rho, MAX_LINEAR_VEL) * std::cos(out.alpha)
            : 0.0f;
        out.mode = state.avoid_side != 0 ? "contornando" : "indo p/ trás";
    }

    saturate_twist(out.v, out.w);
    return out;
}
