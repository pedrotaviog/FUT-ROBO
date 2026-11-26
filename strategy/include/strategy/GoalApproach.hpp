#pragma once

/*
 * Leva o robô para trás da bola, na linha gol -> bola, e o deixa virado para
 * o gol de ataque (FUT_OBJETIVO=gol).
 *
 * O ponto alvo fica a BEHIND_BALL_M atrás da bola. Para chegar nele com a
 * orientação certa usa a lei de controle em coordenadas polares (Siegwart;
 * Astolfi). Se o caminho reto até o ponto alvo passaria perto demais da bola,
 * vai antes para um ponto ao lado dela, para contorná-la sem empurrar. Já no
 * ponto alvo, só gira no lugar até ficar virado para o gol.
 *
 * Recebe as posições e o ângulo já filtrados; não guarda nada da visão.
 */

// Tudo em metros e radianos, coordenadas do campo (origem no centro).
struct GoalApproachInput {
    float robot_x = 0.0f;
    float robot_y = 0.0f;
    float robot_angle = 0.0f;
    float angular_rate = 0.0f;   // velocidade de giro medida, filtrada
    float ball_x = 0.0f;
    float ball_y = 0.0f;
};

// Estado que precisa sobreviver entre frames (histereses).
struct GoalApproachState {
    int  avoid_side = 0;     // contornando a bola: +1 / -1 (lado), 0 = não
    bool at_pose = false;    // já está no ponto alvo, só ajustando o ângulo
    bool aligned = false;    // no ponto alvo e virado para o gol
};

struct GoalApproachOutput {
    float v = 0.0f;          // m/s
    float w = 0.0f;          // rad/s
    float rho = 0.0f;        // distância até o alvo atual, m
    float alpha = 0.0f;      // rad
    float beta = 0.0f;       // rad
    float heading_error = 0.0f;   // orientação final desejada - ângulo do robô
    const char* mode = "";
};

GoalApproachOutput compute_goal_approach(const GoalApproachInput& in, GoalApproachState& state);
