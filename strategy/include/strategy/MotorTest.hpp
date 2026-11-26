#pragma once

#include <array>
#include <optional>
#include <vector>
#include "common/protocol.h"

/*
 * Calibração dos motores com PWM direto (ativada com FUT_TESTE_MOTOR=1).
 *
 * Parte 1, giro no lugar: as duas rodas no mesmo PWM em sentidos opostos.
 * O movimento é simétrico (sem pivotar sobre uma roda travada, o que fazia
 * os rodízios e o atrito variarem). Nos dois sentidos de giro:
 *   - rampa de subida até girar de forma sustentada (PWM de partida);
 *   - rampa de descida até parar (PWM de parada);
 *   - curva: PWM em degraus subindo e descendo, medindo a velocidade média
 *     das rodas = |giro| * bitola / 2.
 *
 * Parte 2, linha reta: as duas rodas no mesmo PWM, para frente e para trás.
 * Pela velocidade e pela curvatura medidas separa a velocidade de cada roda
 * (qual é mais fraca e quanto), e pela direção do deslocamento em relação ao
 * ângulo da tag mede se a tag está torta em relação à frente do robô.
 */
class MotorTest {
public:
    explicit MotorTest(float track_width_m);

    // Devolve o comando do frame, ou nullopt quando o teste terminou.
    std::optional<MotorPwmCmd> step(const RobotPose& robot, float t);

    void print_summary() const;

private:
    enum class Kind { RampUp, RampDown, Rest, Hold, Straight };

    struct Step {
        Kind  kind;
        int   dir = 0;            // giro: +1 anti-horário, -1 horário; reta: ver STRAIGHT_*
        int   pwm = 0;
        bool  descending = false;
        float duration = 0.0f;    // só para Rest, Hold e Straight
    };

    struct RampResult {
        int start_pwm = -1;       // -1: não mediu
        int stop_pwm = -1;
        int measured_sign = 0;
    };

    struct HoldResult {
        int   dir = 0;
        int   pwm = 0;
        bool  descending = false;
        float rate = 0.0f;        // rad/s, com sinal
        float wheel_speed = 0.0f; // velocidade média das rodas, m/s
        int   frames = 0;
        int   frozen_frames = 0;
    };

    struct StraightResult {
        int   dir = 0;
        int   pwm = 0;
        float v = 0.0f;           // m/s ao longo da frente do robô (com sinal)
        float w = 0.0f;           // rad/s
        float side_angle = 0.0f;  // rad: direção do deslocamento menos o ângulo da tag
        int   frames = 0;
        bool  aborted = false;    // parou antes por chegar perto da borda
    };

    // Regressão linear incremental y = a*x + b.
    struct LineFit {
        float n = 0, sx = 0, sy = 0, sxx = 0, sxy = 0;
        void  add(float x, float y);
        float slope() const;
    };

    void  begin_step(float t);
    MotorPwmCmd command(int pwm) const;
    int   ramp_pwm(float t, int start, int sign) const;
    RampResult& ramp_result(int dir);

    float m_track_width;
    std::vector<Step> m_steps;
    size_t m_index = 0;
    bool   m_started = false;

    float m_prev_raw = 0.0f;
    float m_angle = 0.0f;          // ângulo desenrolado
    float m_step_start = 0.0f;
    float m_step_angle = 0.0f;
    int   m_last_printed_pwm = -1;

    std::array<RampResult, 2> m_ramps{};   // [0] anti-horário, [1] horário
    int   m_moving_frames = 0;
    int   m_moving_sign = 0;
    float m_moving_since = 0.0f;
    int   m_down_start_pwm = 0;
    float m_window_t = 0.0f;
    std::vector<float> m_window_angles;

    HoldResult m_hold;
    StraightResult m_straight;
    LineFit m_fit_angle, m_fit_x, m_fit_y;
    float   m_fit_t0 = -1.0f;
    float   m_heading_sin = 0.0f, m_heading_cos = 0.0f;
    float   m_last_x = 0.0f, m_last_y = 0.0f;   // última posição vista, cm
    int     m_straight_dir = 1;                 // sentido resolvido da reta atual
    float   m_straight_start_dist = 0.0f;       // distância ao centro no início, cm

    std::vector<HoldResult> m_holds;
    std::vector<StraightResult> m_straights;
    float m_total_frames = 0.0f;
    float m_total_time = 0.0f;
};
