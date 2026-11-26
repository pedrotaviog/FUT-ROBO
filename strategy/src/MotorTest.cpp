#include "strategy/MotorTest.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numbers>
#include <print>

namespace {

constexpr float PI = std::numbers::pi_v<float>;

// Rampa de subida/descida: degraus de RAMP_STEP a cada RAMP_STEP_S. Devagar
// o bastante para o atraso da visão (~0,35 s) errar só 1 ou 2 degraus.
constexpr int   RAMP_START  = 10;
constexpr int   RAMP_STEP   = 2;
constexpr float RAMP_STEP_S = 0.3f;

// "Começou a girar" = giro acumulado acima de MOVE_THRESHOLD, no mesmo
// sentido, por MOVE_FRAMES frames seguidos. Um erro isolado da visão dura 1
// ou 2 frames e não dispara a detecção.
constexpr float MOVE_THRESHOLD = 0.06f;   // rad
constexpr int   MOVE_FRAMES    = 5;

// Atraso medido no teste de giro (comando -> visão). O PWM que causou um
// evento é o que estava sendo enviado este tempo antes de a visão vê-lo.
constexpr float LATENCY_S = 0.35f;

// A descida começa um pouco acima do PWM de partida para garantir que as
// rodas estejam girando de verdade.
constexpr int DOWN_MARGIN = 15;

// Parado = medianas das metades da janela diferem menos que STOP_THRESHOLD.
constexpr float STOP_WINDOW_S  = 0.6f;
constexpr float STOP_THRESHOLD = 0.01f;   // rad

constexpr float REST_S = 1.5f;

// Curva do giro no lugar: sobe e depois desce, cada PWM medido duas vezes.
constexpr std::array<int, 9> CURVE_PWM = {40, 50, 60, 80, 100, 130, 160, 200, 255};
constexpr float HOLD_S    = 2.5f;
constexpr float MEASURE_S = 1.2f;   // janela final de cada degrau

// Linha reta: ida e volta em cada PWM, para o robô não sair do lugar.
constexpr std::array<int, 3> STRAIGHT_PWM = {80, 100, 120};
constexpr float STRAIGHT_S        = 2.5f;
constexpr float STRAIGHT_SETTLE_S = 0.5f;   // rodízios se reorientando
// Corta a reta antes de chegar na borda (campo de 237 x 165 cm, origem no centro).
constexpr float SAFE_HALF_X_CM = 90.0f;
constexpr float SAFE_HALF_Y_CM = 55.0f;
constexpr int   STRAIGHT_TO_CENTER = 0;   // Step::dir especial das retas
constexpr int   STRAIGHT_REVERSE   = 2;

// Ponto com mais que isso de frames repetidos fica fora da reta.
constexpr float MAX_FROZEN_RATIO = 0.25f;
// Subida e descida discordando mais que isso marca o PWM como suspeito.
constexpr float MAX_UP_DOWN_DIFF = 0.25f;

constexpr float NaN = std::numeric_limits<float>::quiet_NaN();

// Mediana de [first, last).
float median(auto first, auto last) {
    std::vector<float> values(first, last);
    if (values.empty()) return 0.0f;
    const auto mid = values.begin() + values.size() / 2;
    std::nth_element(values.begin(), mid, values.end());
    return *mid;
}

const char* spin_name(int dir) {
    return dir > 0 ? "anti-horario" : "horario";
}

} // namespace

void MotorTest::LineFit::add(float x, float y) {
    n += 1; sx += x; sy += y; sxx += x * x; sxy += x * y;
}

float MotorTest::LineFit::slope() const {
    const float den = n * sxx - sx * sx;
    return n >= 2 && den > 0.0f ? (n * sxy - sx * sy) / den : NaN;
}

MotorTest::MotorTest(float track_width_m) : m_track_width(track_width_m) {
    for (int dir : {1, -1}) {
        m_steps.push_back({Kind::RampUp, dir});
        m_steps.push_back({Kind::RampDown, dir});
        m_steps.push_back({Kind::Rest, dir, 0, false, REST_S});
        for (size_t i = 0; i < CURVE_PWM.size(); i++) {
            m_steps.push_back({Kind::Hold, dir, CURVE_PWM[i], false, HOLD_S});
        }
        for (size_t i = CURVE_PWM.size() - 1; i-- > 0;) {
            m_steps.push_back({Kind::Hold, dir, CURVE_PWM[i], true, HOLD_S});
        }
        m_steps.push_back({Kind::Rest, dir, 0, false, REST_S});
    }
    // Primeira reta de cada par vai em direção ao centro, a segunda volta.
    for (int pwm : STRAIGHT_PWM) {
        for (int dir : {STRAIGHT_TO_CENTER, STRAIGHT_REVERSE}) {
            m_steps.push_back({Kind::Straight, dir, pwm, false, STRAIGHT_S});
            m_steps.push_back({Kind::Rest, 0, 0, false, REST_S});
        }
    }
}

MotorTest::RampResult& MotorTest::ramp_result(int dir) {
    return m_ramps[dir > 0 ? 0 : 1];
}

int MotorTest::ramp_pwm(float t, int start, int sign) const {
    const int steps = static_cast<int>(std::max(0.0f, t - m_step_start) / RAMP_STEP_S);
    return start + sign * RAMP_STEP * steps;
}

MotorPwmCmd MotorTest::command(int pwm) const {
    const Step& s = m_steps[m_index];
    // Giro anti-horário (+): esquerda para trás, direita para frente.
    if (s.kind == Kind::Straight) {
        return MotorPwmCmd{static_cast<int16_t>(m_straight_dir * pwm), static_cast<int16_t>(m_straight_dir * pwm)};
    }
    const int left  = -s.dir * pwm;
    const int right = s.dir * pwm;
    return MotorPwmCmd{static_cast<int16_t>(left), static_cast<int16_t>(right)};
}

void MotorTest::begin_step(float t) {
    const Step& s = m_steps[m_index];
    m_step_start = t;
    m_step_angle = m_angle;
    m_last_printed_pwm = -1;
    m_moving_frames = 0;
    m_moving_sign = 0;
    m_window_t = t;
    m_window_angles.clear();
    m_fit_angle = {};
    m_fit_x = {};
    m_fit_y = {};
    m_fit_t0 = -1.0f;
    m_heading_sin = m_heading_cos = 0.0f;
    m_hold = {s.dir, s.pwm, s.descending};

    if (s.kind == Kind::Straight) {
        if (s.dir == STRAIGHT_TO_CENTER) {
            // Frente se a tag aponta para o lado do centro, senão ré.
            const float to_center = -m_last_x * std::cos(m_prev_raw) - m_last_y * std::sin(m_prev_raw);
            m_straight_dir = to_center >= 0.0f ? 1 : -1;
        } else {
            m_straight_dir = -m_straight_dir;
        }
        m_straight_start_dist = std::hypot(m_last_x, m_last_y);
    }
    m_straight = {m_straight_dir, s.pwm};

    switch (s.kind) {
    case Kind::RampUp:
        std::println("--- giro {}: rampa de subida ---", spin_name(s.dir));
        break;
    case Kind::RampDown:
        std::println("--- giro {}: rampa de descida ---", spin_name(s.dir));
        break;
    case Kind::Hold:
        if (s.pwm == CURVE_PWM.front() && !s.descending) {
            std::println("--- giro {}: curva ---", spin_name(s.dir));
        }
        break;
    case Kind::Straight:
        std::println("--- linha reta {} PWM {} ---", m_straight_dir > 0 ? "para frente" : "para tras", s.pwm);
        break;
    case Kind::Rest:
        break;
    }
}

std::optional<MotorPwmCmd> MotorTest::step(const RobotPose& robot, float t) {
    // Ângulo "desenrolado": soma as variações, sem o salto de ±pi.
    bool frozen = false;
    if (!m_started) {
        m_started = true;
        m_angle = robot.angle;
        m_total_time = -t;
        std::println("--- giro {}: rampa de subida ---", spin_name(m_steps[0].dir));
        m_step_start = t;
        m_step_angle = m_angle;
    } else {
        frozen = robot.angle == m_prev_raw;
        m_angle += std::remainder(robot.angle - m_prev_raw, 2.0f * PI);
    }
    m_prev_raw = robot.angle;
    m_last_x = robot.position.x;
    m_last_y = robot.position.y;
    m_total_frames += 1;

    const Step& s = m_steps[m_index];
    const float elapsed = t - m_step_start;
    bool done = false;   // passo atual terminou neste frame
    int pwm = 0;

    switch (s.kind) {
    case Kind::RampUp: {
        pwm = ramp_pwm(t, RAMP_START, 1);
        const float turned = m_angle - m_step_angle;
        const int sign = std::abs(turned) > MOVE_THRESHOLD ? (turned > 0 ? 1 : -1) : 0;
        if (sign != 0 && sign == m_moving_sign) {
            m_moving_frames++;
        } else {
            m_moving_sign = sign;
            m_moving_frames = sign != 0 ? 1 : 0;
            m_moving_since = t;
        }
        if (m_moving_frames >= MOVE_FRAMES) {
            RampResult& r = ramp_result(s.dir);
            r.start_pwm = ramp_pwm(m_moving_since - LATENCY_S, RAMP_START, 1);
            r.measured_sign = m_moving_sign;
            std::println("    girando de forma sustentada: PWM {} agora, ~{} descontando o atraso",
                         pwm, r.start_pwm);
            m_down_start_pwm = std::min(255, pwm + DOWN_MARGIN);
            done = true;
        } else if (pwm > 255) {
            std::println("    nao girou nem com PWM 255!");
            m_down_start_pwm = 0;
            done = true;
        }
        break;
    }
    case Kind::RampDown: {
        pwm = ramp_pwm(t, m_down_start_pwm, -1);
        m_window_angles.push_back(m_angle);
        if (t - m_window_t >= STOP_WINDOW_S) {
            // Medianas das duas metades da janela: um salto isolado da visão
            // não muda a mediana e não esconde a parada.
            const auto half = m_window_angles.begin() + m_window_angles.size() / 2;
            const float moved = median(half, m_window_angles.end()) - median(m_window_angles.begin(), half);
            if (std::abs(moved) < STOP_THRESHOLD) {
                RampResult& r = ramp_result(s.dir);
                r.stop_pwm = std::max(0, ramp_pwm(m_window_t - LATENCY_S, m_down_start_pwm, -1));
                std::println("    parou: ~PWM {}", r.stop_pwm);
                done = true;
            }
            m_window_t = t;
            m_window_angles.clear();
        }
        if (pwm <= 0) done = true;
        break;
    }
    case Kind::Rest:
        done = elapsed >= s.duration;
        break;
    case Kind::Hold: {
        pwm = s.pwm;
        m_hold.frames++;
        if (frozen) m_hold.frozen_frames++;
        // Regressão do ângulo no tempo na janela final: um frame errado da
        // visão pesa pouco, ao contrário de usar só o primeiro e o último.
        if (elapsed >= s.duration - MEASURE_S) {
            if (m_fit_t0 < 0.0f) m_fit_t0 = t;
            m_fit_angle.add(t - m_fit_t0, m_angle);
        }
        if (elapsed >= s.duration) {
            const float rate = m_fit_angle.slope();
            m_hold.rate = std::isnan(rate) ? 0.0f : rate;
            m_hold.wheel_speed = std::abs(m_hold.rate) * m_track_width / 2.0f;
            m_holds.push_back(m_hold);
            std::println("    PWM {:3} ({}): giro {:+.3f} rad/s -> roda {:.3f} m/s   repetidos {}/{}",
                         s.pwm, s.descending ? "descendo" : "subindo ", m_hold.rate,
                         m_hold.wheel_speed, m_hold.frozen_frames, m_hold.frames);
            done = true;
        }
        break;
    }
    case Kind::Straight: {
        pwm = s.pwm;
        const float x_cm = robot.position.x;
        const float y_cm = robot.position.y;
        if (elapsed >= STRAIGHT_SETTLE_S) {
            if (m_fit_t0 < 0.0f) m_fit_t0 = t;
            const float x = t - m_fit_t0;
            m_fit_angle.add(x, m_angle);
            m_fit_x.add(x, x_cm / 100.0f);
            m_fit_y.add(x, y_cm / 100.0f);
            m_heading_sin += std::sin(robot.angle);
            m_heading_cos += std::cos(robot.angle);
            m_straight.frames++;
        }
        // Só interrompe se estiver na borda E se afastando do centro.
        if ((std::abs(x_cm) > SAFE_HALF_X_CM || std::abs(y_cm) > SAFE_HALF_Y_CM) &&
            std::hypot(x_cm, y_cm) > m_straight_start_dist + 2.0f) {
            std::println("    perto da borda ({:.0f}, {:.0f}) cm: reta interrompida", x_cm, y_cm);
            m_straight.aborted = true;
        }
        if (elapsed >= s.duration || m_straight.aborted) {
            const float vx = m_fit_x.slope();
            const float vy = m_fit_y.slope();
            const float heading = std::atan2(m_heading_sin, m_heading_cos);
            const float travel = std::atan2(vy, vx) + (m_straight_dir < 0 ? PI : 0.0f);
            m_straight.v = m_straight_dir * std::hypot(vx, vy);
            m_straight.w = m_fit_angle.slope();
            // Quanto a tag está girada em relação à direção em que o robô anda.
            m_straight.side_angle = std::remainder(heading - travel, 2.0f * PI);
            m_straights.push_back(m_straight);
            std::println("    v {:+.3f} m/s   w {:+.3f} rad/s   deslocamento vs tag {:+.1f} graus   ({} frames)",
                         m_straight.v, m_straight.w, m_straight.side_angle * 180.0f / PI, m_straight.frames);
            done = true;
        }
        break;
    }
    }

    if ((s.kind == Kind::RampUp || s.kind == Kind::RampDown) && !done && pwm != m_last_printed_pwm) {
        std::println("    PWM {:3}  girado {:+.3f} rad", pwm, m_angle - m_step_angle);
        m_last_printed_pwm = pwm;
    }

    if (done) {
        if (++m_index == m_steps.size()) {
            m_total_time += t;
            return std::nullopt;
        }
        // Rampa de descida sem partida medida não tem de onde descer.
        if (m_steps[m_index].kind == Kind::RampDown && m_down_start_pwm == 0) m_index++;
        begin_step(t);
        const Step& next = m_steps[m_index];
        switch (next.kind) {
        case Kind::RampUp:   pwm = RAMP_START; break;
        case Kind::RampDown: pwm = m_down_start_pwm; break;
        case Kind::Rest:     pwm = 0; break;
        case Kind::Hold:
        case Kind::Straight: pwm = next.pwm; break;
        }
    }

    return command(pwm);
}

void MotorTest::print_summary() const {
    const float fps = m_total_time > 0.0f ? m_total_frames / m_total_time : NaN;
    std::println("\n================ CALIBRACAO DOS MOTORES (v3) ================");
    std::println("bitola usada: {:.3f} m (TRACK_WIDTH_M). Visao a {:.1f} fps.\n", m_track_width, fps);

    std::println("GIRO NO LUGAR (as duas rodas juntas, sentidos opostos)");
    std::println("sentido        conferido  partida  parada");
    for (int dir : {1, -1}) {
        const RampResult& r = m_ramps[dir > 0 ? 0 : 1];
        const char* check = r.measured_sign == 0 ? "?" : r.measured_sign == dir ? "OK" : "INVERTIDO";
        std::println("{:<13}  {:<9}  {:7}  {:6}", spin_name(dir), check, r.start_pwm, r.stop_pwm);
    }

    const auto usable = [](const HoldResult& h) {
        return h.frames > 0 && static_cast<float>(h.frozen_frames) / h.frames <= MAX_FROZEN_RATIO;
    };
    const auto find_hold = [&](int dir, int pwm, bool descending) -> const HoldResult* {
        for (const HoldResult& h : m_holds) {
            if (h.dir == dir && h.pwm == pwm && (h.descending == descending || pwm == CURVE_PWM.back())) return &h;
        }
        return nullptr;
    };

    std::println("\nvelocidade media das rodas (m/s). * = mais de {:.0f}% de frames repetidos (fora da reta)",
                 MAX_FROZEN_RATIO * 100);
    std::print("PWM                    ");
    for (int pwm : CURVE_PWM) std::print("{:8}", pwm);
    std::println("");
    for (int dir : {1, -1}) {
        for (bool descending : {false, true}) {
            std::print("{:<13} {:<8} ", spin_name(dir), descending ? "descida" : "subida");
            for (int pwm : CURVE_PWM) {
                const HoldResult* h = find_hold(dir, pwm, descending);
                if (!h) std::print("{:>8}", "-");
                else std::print("{:7.3f}{}", h->wheel_speed, usable(*h) ? " " : "*");
            }
            std::println("");
        }
    }

    std::println("\nPWMs acima da partida em que subida e descida discordam mais de {:.0f}%:", MAX_UP_DOWN_DIFF * 100);
    bool any_suspect = false;
    for (int dir : {1, -1}) {
        const int start = m_ramps[dir > 0 ? 0 : 1].start_pwm;
        for (int pwm : CURVE_PWM) {
            const HoldResult* up = find_hold(dir, pwm, false);
            const HoldResult* down = find_hold(dir, pwm, true);
            if (!up || !down || up == down || pwm < start) continue;
            const float ref = std::max(up->wheel_speed, down->wheel_speed);
            if (ref > 0.02f && std::abs(up->wheel_speed - down->wheel_speed) / ref > MAX_UP_DOWN_DIFF) {
                std::println("  {:<13} PWM {:3}: subida {:.3f}  descida {:.3f}",
                             spin_name(dir), pwm, up->wheel_speed, down->wheel_speed);
                any_suspect = true;
            }
        }
    }
    if (!any_suspect) std::println("  nenhum (medida consistente)");

    std::println("\nreta velocidade x PWM (subida + descida, pontos confiaveis em que as rodas giraram)");
    std::println("sentido        pontos  pwm_zero  v_max(PWM 255)");
    float worst_v_max = NaN;
    float worst_pwm_zero = NaN;
    for (int dir : {1, -1}) {
        LineFit fit;
        for (const HoldResult& h : m_holds) {
            if (h.dir == dir && h.wheel_speed >= 0.01f && usable(h)) fit.add(static_cast<float>(h.pwm), h.wheel_speed);
        }
        const float a = fit.slope();
        const float b = fit.n > 0 ? (fit.sy - a * fit.sx) / fit.n : NaN;
        const float pwm_zero = -b / a;
        const float v_max = a * 255.0f + b;
        std::println("{:<13}  {:6.0f}  {:8.1f}  {:8.3f} m/s", spin_name(dir), fit.n, pwm_zero, v_max);
        if (!std::isnan(v_max)) {
            worst_v_max = std::isnan(worst_v_max) ? v_max : std::min(worst_v_max, v_max);
            worst_pwm_zero = std::isnan(worst_pwm_zero) ? pwm_zero : std::max(worst_pwm_zero, pwm_zero);
        }
    }

    std::println("\nLINHA RETA (as duas rodas no mesmo PWM)");
    std::println("PWM  sentido   v(m/s)   w(rad/s)  roda_esq  roda_dir  dir/esq  desvio_tag(graus)");
    float ratio_sum = 0.0f, side_sum = 0.0f;
    int   ratio_n = 0;
    for (const StraightResult& r : m_straights) {
        const float v_left  = r.v - r.w * m_track_width / 2.0f;
        const float v_right = r.v + r.w * m_track_width / 2.0f;
        const float ratio = v_right / v_left;
        std::println("{:3}  {:<8}  {:+.3f}   {:+.3f}    {:+.3f}    {:+.3f}    {:5.2f}   {:+6.1f}{}",
                     r.pwm, r.dir > 0 ? "frente" : "tras", r.v, r.w, v_left, v_right, ratio,
                     r.side_angle * 180.0f / PI, r.aborted ? "   (interrompida)" : "");
        if (r.frames >= 8 && std::isfinite(ratio) && ratio > 0.0f) {
            ratio_sum += ratio;
            side_sum += r.side_angle;
            ratio_n++;
        }
    }
    const float ratio = ratio_n > 0 ? ratio_sum / ratio_n : NaN;
    const float side_deg = ratio_n > 0 ? side_sum / ratio_n * 180.0f / PI : NaN;

    std::println("\n================ RESUMO ================");
    const int stop_max = std::max(m_ramps[0].stop_pwm, m_ramps[1].stop_pwm);
    std::println("PWM_MIN       ~ {}   (maior PWM de parada no giro; a reta deu pwm_zero ~ {:.0f})",
                 stop_max, worst_pwm_zero);
    std::println("partida       ~ {} / {}   (PWM para sair do repouso, anti-horario / horario)",
                 m_ramps[0].start_pwm, m_ramps[1].start_pwm);
    std::println("roda dir/esq  ~ {:.2f}   (1.00 = iguais; < 1 a direita e mais fraca)", ratio);
    if (!std::isnan(ratio) && !std::isnan(worst_v_max)) {
        const float weaker = worst_v_max * 2.0f * std::min(ratio, 1.0f / ratio) / (1.0f + std::min(ratio, 1.0f / ratio));
        std::println("MAX_SPEED     ~ {:.3f} m/s (roda mais fraca em PWM 255; media das rodas {:.3f})", weaker, worst_v_max);
    } else {
        std::println("MAX_SPEED     ~ {:.3f} m/s (media das rodas em PWM 255)", worst_v_max);
    }
    std::println("desvio da tag ~ {:+.1f} graus  (quanto o angulo da tag esta girado em relacao a frente real", side_deg);
    std::println("                 do robo, anti-horario = +; perto de 180 = tag apontando para tras)");
}
