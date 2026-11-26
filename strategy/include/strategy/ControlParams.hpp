#pragma once

#include <algorithm>
#include <cmath>

// Parâmetros de controle compartilhados pelos modos do strategy. Os valores
// medidos e de onde vieram estão em docs/parametros_robo.md.

// Devem ser iguais ao TRACK_WIDTH e ao último ponto de CAL_SPEED do firmware
// (firmware/main/main.c): bitola medida e velocidade da roda em PWM 255.
inline constexpr float TRACK_WIDTH_M      = 0.19f;
inline constexpr float MAX_WHEEL_SPEED_MS = 0.354f;

// A visão publica centímetros de campo, não pixels: a homografia de
// `Calibration::calibrate` é ajustada contra XY_POINTS, os quatro cantos
// dados em cm reais (campo de 237 x 165), e Vec2 os transporta como cm inteiros.
inline constexpr float VISION_TO_M = 0.01f;

// ============================================================
//  GANHOS
// ============================================================
// Valores medidos (docs/parametros_robo.md) usados para escolher os ganhos:
// atraso efetivo comando -> visão (~0,35 s medido + ~0,05 s dos filtros, pela
// "meia regra" do SIMC) e menor velocidade de giro no lugar que o robô mantém.
inline constexpr float CONTROL_DELAY_S = 0.40f;   // s
inline constexpr float MIN_TURN_RATE   = 0.30f;   // rad/s

// O controle de orientação é um integrador com atraso (comando = velocidade de
// giro, saída = ângulo). Com controle P, a resposta não oscila enquanto
// Kp * atraso <= 1/e (~0,37); acima disso o robô passa do ponto. Usamos 0,28
// para ter margem para as não linearidades (velocidade mínima, tranco).
// Mesma ordem de grandeza da sintonia SIMC: Kc = 1 / (tau_c + atraso), tau_c >= atraso.
inline constexpr float KP_ANGULAR = 0.28f / CONTROL_DELAY_S;   // = 0,7 (rad/s) por rad de erro
// O termo D age sobre a velocidade de giro MEDIDA do robô, não sobre a
// derivada do erro: quando o ângulo alvo salta (ruído de 1 cm perto da bola),
// a derivada do erro geraria um pico falso ("derivative kick").
inline constexpr float KD_ANGULAR = 0.15f;  // (rad/s) por (rad/s) de giro medido

inline constexpr float MAX_ANGULAR_VEL = 2.0f;   // rad/s
inline constexpr float MAX_LINEAR_VEL  = 0.3f;   // m/s

// Zona morta angular com histerese, usada só quando o robô está parado na
// bola: é considerado alinhado quando o erro cai abaixo de ALIGN_IN e só volta
// a corrigir quando passa de ALIGN_OUT. Andando, a correção é sempre contínua.
// Mesmo girando na velocidade mínima, o robô ainda anda MIN_TURN_RATE *
// CONTROL_DELAY_S (~0,12 rad) depois do comando de parar; entrando em
// ALIGN_IN ele termina do outro lado em ~ALIGN_IN - 0,12. ALIGN_OUT precisa
// ficar acima disso com folga, senão ele recomeça a corrigir e oscila.
inline constexpr float ALIGN_IN  = 0.06f;   // rad (~3.4 graus)
inline constexpr float ALIGN_OUT = 2.0f * (MIN_TURN_RATE * CONTROL_DELAY_S - ALIGN_IN);   // = 0,12 rad (~6.9 graus)

// Compensação de zona morta: o firmware zera rodas abaixo de 0,01 m/s, o que
// no giro no lugar é w < 0,01 / (bitola / 2) ~ 0,105 rad/s. Com erro pequeno
// (entre ALIGN_IN e ~0,15 rad) o P puro pediria menos que isso e o robô ficaria
// parado fora do alinhamento. Girando no lugar, o comando nunca fica abaixo
// deste mínimo; o firmware o transforma na menor velocidade real (~0,3 rad/s).
inline constexpr float MIN_TURN_CMD = 0.15f;   // rad/s

// Filtros passa-baixa (média móvel exponencial).
// 1.0 = sem filtro; quanto menor, mais suave e mais atrasado.
inline constexpr float ANGLE_FILTER_ALPHA    = 0.5f;   // ângulo do robô
inline constexpr float RATE_FILTER_ALPHA     = 0.3f;   // velocidade de giro (termo D)
inline constexpr float POSITION_FILTER_ALPHA = 0.4f;   // posições da bola e do robô

// Um salto maior que isto entre frames é movimento real (bola chutada,
// robô empurrado), não ruído: o filtro de posição é reiniciado na hora.
inline constexpr float POSITION_JUMP_RESET_M = 0.03f;

// Só avança quando já estiver razoavelmente apontado para a bola.
inline constexpr float ADVANCE_CONE = 0.6f;   // rad

// Para de aproximar dentro deste raio da bola.
// Distância entre o centro da tag do robô e o centro da bola em que ele para.
// Encostar seria ~0,14 m: centro da tag até a frente do robô (~12,1 cm) + raio
// da bola (1,95 cm). 0,20 m (ajustado em teste) deixa ~6 cm de folga.
inline constexpr float STOP_DISTANCE_M = 0.2f;

// Rampa de aproximação: velocidade máxima a partir de SLOW_DISTANCE_M,
// caindo linearmente até MIN_APPROACH_VEL em STOP_DISTANCE_M. O mínimo existe
// para a roda não cair abaixo da zona morta do motor e parar antes da bola.
inline constexpr float SLOW_DISTANCE_M  = 0.40f;
inline constexpr float MIN_APPROACH_VEL = 0.08f;   // m/s

/*
 * Satura o par (v, w) respeitando o limite físico de cada roda, sem
 * distorcer a razão entre eles mais que o necessário.
 * A rotação tem prioridade: se estourar, reduz v primeiro.
 */
// Comando de giro no lugar até um erro de orientação: PD (D sobre o giro
// medido) com compensação de zona morta.
inline float turn_in_place_command(float error, float angular_rate) {
    const float w = KP_ANGULAR * error - KD_ANGULAR * angular_rate;
    if (std::abs(w) < MIN_TURN_CMD && std::abs(error) > ALIGN_IN) {
        return std::copysign(MIN_TURN_CMD, error);
    }
    return w;
}

inline void saturate_twist(float& v, float& w) {
    w = std::clamp(w, -MAX_ANGULAR_VEL, MAX_ANGULAR_VEL);

    const float w_component = std::abs(w) * TRACK_WIDTH_M / 2.0f;
    const float v_budget = MAX_WHEEL_SPEED_MS - w_component;

    if (v_budget <= 0.0f) {
        // A rotação sozinha já satura as rodas: sem margem para avançar.
        v = 0.0f;
        const float w_max = 2.0f * MAX_WHEEL_SPEED_MS / TRACK_WIDTH_M;
        w = std::clamp(w, -w_max, w_max);
        return;
    }

    v = std::clamp(v, -v_budget, v_budget);
    v = std::clamp(v, -MAX_LINEAR_VEL, MAX_LINEAR_VEL);
}

// ============================================================
//  IR PARA TRÁS DA BOLA, VIRADO PARA O GOL (FUT_OBJETIVO=gol)
// ============================================================
// Campo de 237 x 165 cm com origem no centro (vision/config.yaml). Os gols
// ficam no meio dos lados menores; o de ataque é o da direita (x positivo).
inline constexpr float FIELD_HALF_X_M  = 2.37f / 2.0f;
inline constexpr float FIELD_HALF_Y_M  = 1.65f / 2.0f;
inline constexpr float ATTACK_GOAL_X_M = FIELD_HALF_X_M;
inline constexpr float ATTACK_GOAL_Y_M = 0.0f;
inline constexpr float GOAL_WIDTH_M    = 0.52f;

// O robô para atrás da bola, na linha gol -> bola, a esta distância entre o
// centro da tag e o centro da bola (a mesma folga do modo "ir até a bola").
inline constexpr float BEHIND_BALL_M = STOP_DISTANCE_M;

// Margem até a borda do campo para o ponto alvo (meio robô, da tag à frente).
inline constexpr float FIELD_MARGIN_M = 0.12f;

// Folga mínima entre o centro da tag e o centro da bola ao passar por ela:
// meio robô (~12 cm) + raio da bola (~2 cm) + ~2 cm. Se o caminho reto até o
// ponto alvo passa mais perto que isso, o robô contorna a bola por um ponto
// lateral, para não empurrá-la.
inline constexpr float BALL_CLEARANCE_M = 0.16f;
// Contorno por dois pontos: primeiro bem ao lado da bola (AVOID_SIDE_M), se a
// reta até o segundo passar perto dela; depois ao lado e atrás
// (AVOID_BEHIND_M), para a chegada final no ponto alvo ser por trás.
inline constexpr float AVOID_SIDE_M   = 0.30f;
inline constexpr float AVOID_BEHIND_M = 1.5f * BEHIND_BALL_M;
// Para de contornar ao chegar no ponto de contorno, ou antes disso se a folga
// do caminho reto passar de BALL_CLEARANCE_EXIT_M (robô já longe da bola).
inline constexpr float BALL_CLEARANCE_EXIT_M = 0.22f;
// Contornando, só anda quando já aponta para o ponto de contorno: sair
// andando ainda virado para a bola faria a curva passar perto dela.
inline constexpr float AVOID_ADVANCE_CONE = 0.25f;   // rad

// Lei de controle em coordenadas polares (Siegwart; Astolfi) para chegar numa
// pose (posição + orientação):
//   rho   = distância até o ponto alvo
//   alpha = ângulo entre a frente do robô e a direção do ponto alvo
//   beta  = orientação final desejada menos a direção do ponto alvo
//   v = K_RHO * rho,   w = K_ALPHA * alpha + K_BETA * beta
// Estável (Lyapunov) com K_RHO > 0, K_BETA < 0, K_ALPHA - K_RHO > 0; a
// condição forte K_ALPHA + 5/3 K_BETA - 2/pi K_RHO > 0 garante que o robô
// chega sem precisar inverter o sentido de marcha no caminho.
// K_ALPHA é o mesmo ganho de orientação já sintonizado pelo atraso medido.
inline constexpr float K_RHO   = 0.4f;          // 1/s
inline constexpr float K_ALPHA = KP_ANGULAR;    // 1/s
inline constexpr float K_BETA  = -0.15f;        // 1/s
static_assert(K_RHO > 0.0f && K_BETA < 0.0f && K_ALPHA - K_RHO > 0.0f,
              "ganhos da lei polar fora da regiao de estabilidade");
static_assert(K_ALPHA + 5.0f / 3.0f * K_BETA - 2.0f / 3.14159265f * K_RHO > 0.0f,
              "ganhos da lei polar fora da condicao forte (robo inverteria a marcha)");

// Chegou na posição: abaixo de POSE_REACHED_M ele só gira no lugar até ficar
// virado para o gol; volta a andar se sair além de POSE_LEFT_M (histerese).
inline constexpr float POSE_REACHED_M = 0.04f;
inline constexpr float POSE_LEFT_M    = 0.08f;
