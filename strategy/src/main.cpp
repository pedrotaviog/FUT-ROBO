#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdlib>
#include <limits>
#include <numbers>
#include <optional>
#include <print>
#include <stdexcept>
#include <string_view>
#include <asio.hpp>
#include "common/protocol.h"
#include "strategy/Listener.hpp"
#include "strategy/MotorTest.hpp"
#include "strategy/UnicastSender.hpp"

constexpr char MULTICAST_ADDRESS[] = "224.1.1.1";
constexpr int  MULTICAST_PORT      = 5007;

constexpr char TARGET_HOSTNAME[] = "robot.local";
constexpr int  UNICAST_PORT      = 3333;


// Devem ser iguais ao TRACK_WIDTH e ao último ponto de CAL_SPEED do firmware
// (firmware/main/main.c): bitola medida e velocidade da roda em PWM 255.
constexpr float TRACK_WIDTH_M      = 0.19f;
constexpr float MAX_WHEEL_SPEED_MS = 0.354f;

// A visão publica centímetros de campo, não pixels: a homografia de
// `Calibration::calibrate` é ajustada contra XY_POINTS, os quatro cantos
// dados em cm reais (campo de 237 x 165), e Vec2 os transporta como cm inteiros.
constexpr float VISION_TO_M = 0.01f;

// ============================================================
//  GANHOS
// ============================================================
// Valores medidos (docs/parametros_robo.md) usados para escolher os ganhos:
// atraso efetivo comando -> visão (~0,35 s medido + ~0,05 s dos filtros, pela
// "meia regra" do SIMC) e menor velocidade de giro no lugar que o robô mantém.
constexpr float CONTROL_DELAY_S = 0.40f;   // s
constexpr float MIN_TURN_RATE   = 0.30f;   // rad/s

// O controle de orientação é um integrador com atraso (comando = velocidade de
// giro, saída = ângulo). Com controle P, a resposta não oscila enquanto
// Kp * atraso <= 1/e (~0,37); acima disso o robô passa do ponto. Usamos 0,28
// para ter margem para as não linearidades (velocidade mínima, tranco).
// Mesma ordem de grandeza da sintonia SIMC: Kc = 1 / (tau_c + atraso), tau_c >= atraso.
constexpr float KP_ANGULAR = 0.28f / CONTROL_DELAY_S;   // = 0,7 (rad/s) por rad de erro
// O termo D age sobre a velocidade de giro MEDIDA do robô, não sobre a
// derivada do erro: quando o ângulo alvo salta (ruído de 1 cm perto da bola),
// a derivada do erro geraria um pico falso ("derivative kick").
constexpr float KD_ANGULAR = 0.15f;  // (rad/s) por (rad/s) de giro medido

constexpr float MAX_ANGULAR_VEL = 2.0f;   // rad/s
constexpr float MAX_LINEAR_VEL  = 0.3f;   // m/s

// Zona morta angular com histerese, usada só quando o robô está parado na
// bola: é considerado alinhado quando o erro cai abaixo de ALIGN_IN e só volta
// a corrigir quando passa de ALIGN_OUT. Andando, a correção é sempre contínua.
// Mesmo girando na velocidade mínima, o robô ainda anda MIN_TURN_RATE *
// CONTROL_DELAY_S (~0,12 rad) depois do comando de parar; entrando em
// ALIGN_IN ele termina do outro lado em ~ALIGN_IN - 0,12. ALIGN_OUT precisa
// ficar acima disso com folga, senão ele recomeça a corrigir e oscila.
constexpr float ALIGN_IN  = 0.06f;   // rad (~3.4 graus)
constexpr float ALIGN_OUT = 2.0f * (MIN_TURN_RATE * CONTROL_DELAY_S - ALIGN_IN);   // = 0,12 rad (~6.9 graus)

// Filtros passa-baixa (média móvel exponencial).
// 1.0 = sem filtro; quanto menor, mais suave e mais atrasado.
constexpr float ANGLE_FILTER_ALPHA    = 0.5f;   // ângulo do robô
constexpr float RATE_FILTER_ALPHA     = 0.3f;   // velocidade de giro (termo D)
constexpr float POSITION_FILTER_ALPHA = 0.4f;   // posições da bola e do robô

// Um salto maior que isto entre frames é movimento real (bola chutada,
// robô empurrado), não ruído: o filtro de posição é reiniciado na hora.
constexpr float POSITION_JUMP_RESET_M = 0.03f;

// Só avança quando já estiver razoavelmente apontado para a bola.
constexpr float ADVANCE_CONE = 0.6f;   // rad

// Para de aproximar dentro deste raio da bola.
// Distância entre o centro da tag do robô e o centro da bola em que ele para.
// Encostar seria ~0,14 m: centro da tag até a frente do robô (~12,1 cm) + raio
// da bola (1,95 cm). 0,20 m (ajustado em teste) deixa ~6 cm de folga.
constexpr float STOP_DISTANCE_M = 0.2f;

// Rampa de aproximação: velocidade máxima a partir de SLOW_DISTANCE_M,
// caindo linearmente até MIN_APPROACH_VEL em STOP_DISTANCE_M. O mínimo existe
// para a roda não cair abaixo da zona morta do motor e parar antes da bola.
constexpr float SLOW_DISTANCE_M  = 0.40f;
constexpr float MIN_APPROACH_VEL = 0.08f;   // m/s

/*
 * Satura o par (v, w) respeitando o limite físico de cada roda, sem
 * distorcer a razão entre eles mais que o necessário.
 * A rotação tem prioridade: se estourar, reduz v primeiro.
 */
static void saturate_twist(float& v, float& w) {
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

namespace {

struct Point {
    float x = 0.0f;
    float y = 0.0f;
};

// Estado do controlador que precisa sobreviver entre frames.
struct TrackingState {
    bool  initialized   = false;
    Point ball;                   // posições filtradas, em metros
    Point robot;
    float filtered_sin  = 0.0f;   // filtro do ângulo feito em seno/cosseno
    float filtered_cos  = 1.0f;   // para não quebrar na volta de ±pi
    float prev_angle    = 0.0f;
    float angular_rate  = 0.0f;   // velocidade de giro medida, filtrada
    bool  aligned       = false;
};

/*
 * Filtra uma posição medida. A visão entrega cm inteiros, então de perto a
 * posição fica alternando entre dois valores vizinhos; o filtro tira essa
 * oscilação. Um salto grande é movimento real e reinicia o filtro, para o
 * robô não seguir uma posição antiga.
 */
void filter_position(Point& filtered, Point measured) {
    if (std::hypot(measured.x - filtered.x, measured.y - filtered.y) > POSITION_JUMP_RESET_M) {
        filtered = measured;
        return;
    }
    filtered.x += POSITION_FILTER_ALPHA * (measured.x - filtered.x);
    filtered.y += POSITION_FILTER_ALPHA * (measured.y - filtered.y);
}

Point to_meters(Vec2 v) {
    return {static_cast<float>(v.x) * VISION_TO_M, static_cast<float>(v.y) * VISION_TO_M};
}

} // namespace

TwistCmd compute_twist_control(const RobotPose& robot, Vec2 ball,
                               TrackingState& state, float dt) {
    constexpr float TWO_PI = 2.0f * std::numbers::pi_v<float>;
    TwistCmd cmd = {0.0f, 0.0f};

    if (!state.initialized) {
        state.ball  = to_meters(ball);
        state.robot = to_meters(robot.position);
        state.filtered_sin = std::sin(robot.angle);
        state.filtered_cos = std::cos(robot.angle);
    } else {
        filter_position(state.ball, to_meters(ball));
        filter_position(state.robot, to_meters(robot.position));
        // A média do ângulo é feita no seno e no cosseno porque o ângulo
        // "pula" de +pi para -pi e a média direta daria um valor errado.
        state.filtered_sin += ANGLE_FILTER_ALPHA * (std::sin(robot.angle) - state.filtered_sin);
        state.filtered_cos += ANGLE_FILTER_ALPHA * (std::cos(robot.angle) - state.filtered_cos);
    }
    const float robot_angle = std::atan2(state.filtered_sin, state.filtered_cos);

    const float dx = state.ball.x - state.robot.x;
    const float dy = state.ball.y - state.robot.y;
    const float distance = std::hypot(dx, dy);

    // Erro de orientação, normalizado para [-pi, pi]
    const float target_angle = std::atan2(dy, dx);
    const float error_theta = std::remainder(target_angle - robot_angle, TWO_PI);

    // Velocidade de giro medida (derivada do ângulo do robô, não do erro),
    // normalizada para não gerar pico na volta de ±pi e filtrada contra ruído.
    if (state.initialized) {
        const float raw_rate = std::remainder(robot_angle - state.prev_angle, TWO_PI) / dt;
        state.angular_rate += RATE_FILTER_ALPHA * (raw_rate - state.angular_rate);
    }
    state.prev_angle = robot_angle;
    state.initialized = true;

    const bool at_ball = distance <= STOP_DISTANCE_M;

    // Parado na bola, histerese: entra em "alinhado" abaixo de ALIGN_IN e sai
    // acima de ALIGN_OUT. Andando, nunca fica "alinhado": se ficasse, o robô
    // derivaria de lado sem corrigir e chegaria torto.
    if (!at_ball) {
        state.aligned = false;
    } else if (state.aligned) {
        if (std::abs(error_theta) > ALIGN_OUT) state.aligned = false;
    } else if (std::abs(error_theta) < ALIGN_IN) {
        state.aligned = true;
    }

    // Controle PD do ângulo; o D freia o giro medido. Alinhado, não gira.
    cmd.w = state.aligned ? 0.0f : KP_ANGULAR * error_theta - KD_ANGULAR * state.angular_rate;

    // Rampa de aproximação, atenuada pelo desalinhamento. O cos() faz o robô
    // desacelerar quando está torto, em vez de descrever arcos largos.
    if (!at_ball && std::abs(error_theta) < ADVANCE_CONE) {
        const float ramp = std::clamp(
            (distance - STOP_DISTANCE_M) / (SLOW_DISTANCE_M - STOP_DISTANCE_M), 0.0f, 1.0f);
        const float speed = MIN_APPROACH_VEL + ramp * (MAX_LINEAR_VEL - MIN_APPROACH_VEL);
        cmd.v = speed * std::cos(error_theta);
    } else {
        cmd.v = 0.0f;
    }

    saturate_twist(cmd.v, cmd.w);

    std::println("dist={:.3f}m  err={:+.3f}rad  giro={:+.3f}rad/s  {}  ->  v={:+.3f}m/s  w={:+.3f}rad/s",
                 distance, error_theta, state.angular_rate, state.aligned ? "ALINHADO" : "girando ",
                 cmd.v, cmd.w);

    return cmd;
}

// ============================================================
//  TESTE DE GIRO (ativado com FUT_TESTE_GIRO=1)
// ============================================================
// Gira o robô no lugar com comandos fixos de w e mede, pela visão, o que ele
// realmente fez: atraso até começar a girar, velocidade de giro em regime e
// quanto continua girando depois do comando de parar.

// Cópia da tabela de calibração do firmware (firmware/main/main.c), só para
// mostrar o PWM que cada comando gera. Atualize aqui se mudar lá.
constexpr std::array<float, 8> FW_CAL_SPEED = {0.048f, 0.076f, 0.124f, 0.155f, 0.201f, 0.240f, 0.291f, 0.354f};
constexpr std::array<int, 8>   FW_CAL_PWM   = {50, 60, 80, 100, 130, 160, 200, 255};
constexpr float FW_V_DEADZONE = 0.01f;

// Sinais alternados para o robô não acumular voltas.
constexpr std::array<float, 12> TEST_W = {
    0.2f, -0.2f, 0.4f, -0.4f, 0.6f, -0.6f, 0.8f, -0.8f, 1.0f, -1.0f, 1.5f, -1.5f
};
constexpr float TEST_SPIN_S   = 2.0f;   // tempo girando em cada comando
constexpr float TEST_REST_S   = 1.5f;   // tempo parado entre comandos
constexpr float TEST_STEADY_S = 1.0f;   // janela final do giro usada na média
// Giro acumulado que conta como "começou a girar"; acima do ruído do ângulo.
constexpr float TEST_MOVE_THRESHOLD = 0.08f;   // rad

namespace {

int firmware_pwm(float wheel_speed) {
    const float abs_v = std::abs(wheel_speed);
    if (abs_v < FW_V_DEADZONE) return 0;
    if (abs_v <= FW_CAL_SPEED.front()) return FW_CAL_PWM.front();
    for (size_t i = 1; i < FW_CAL_SPEED.size(); i++) {
        if (abs_v <= FW_CAL_SPEED[i]) {
            const float t = (abs_v - FW_CAL_SPEED[i - 1]) / (FW_CAL_SPEED[i] - FW_CAL_SPEED[i - 1]);
            return FW_CAL_PWM[i - 1] + static_cast<int>(std::lround(t * (FW_CAL_PWM[i] - FW_CAL_PWM[i - 1])));
        }
    }
    return FW_CAL_PWM.back();
}

class SpinTest {
public:
    // Devolve o comando do frame, ou nullopt quando o teste terminou.
    std::optional<TwistCmd> step(const RobotPose& robot, float t) {
        // Ângulo "desenrolado": soma as variações, sem o salto de ±pi.
        if (!m_started) {
            m_started = true;
            m_angle = robot.angle;
            begin_phase(t);
        } else {
            const float delta = std::remainder(robot.angle - m_prev_raw, 2.0f * std::numbers::pi_v<float>);
            // Só conta depois que o robô começou a girar: parado, a visão
            // pode repetir o mesmo ângulo legitimamente.
            if (m_spinning && !std::isnan(m_results[m_index].delay) && robot.angle == m_prev_raw) {
                m_results[m_index].frozen_frames++;
            }
            m_angle += delta;
        }
        m_prev_raw = robot.angle;

        Result& r = m_results[m_index];
        const float elapsed = t - m_phase_start;

        if (m_spinning) {
            if (std::isnan(r.delay) && std::abs(m_angle - m_phase_angle) > TEST_MOVE_THRESHOLD) {
                r.delay = elapsed;
            }
            if (!m_steady_started && elapsed >= TEST_SPIN_S - TEST_STEADY_S) {
                m_steady_started = true;
                m_steady_t = t;
                m_steady_angle = m_angle;
            }
            if (elapsed >= TEST_SPIN_S) {
                if (m_steady_started && t > m_steady_t) {
                    r.rate = (m_angle - m_steady_angle) / (t - m_steady_t);
                }
                m_spinning = false;
                m_phase_start = t;
                m_phase_angle = m_angle;
            }
        } else if (elapsed >= TEST_REST_S) {
            r.coast = m_angle - m_phase_angle;
            if (++m_index == TEST_W.size()) return std::nullopt;
            begin_phase(t);
        }

        const TwistCmd cmd{0.0f, m_spinning ? TEST_W[m_index] : 0.0f};
        std::println("t={:6.2f}s  w_cmd={:+.2f}  angulo={:+.3f}rad  girado={:+.3f}rad",
                     t, cmd.w, robot.angle, m_angle - m_phase_angle);
        return cmd;
    }

    void print_summary() const {
        std::println("\n================ RESULTADO DO TESTE DE GIRO ================");
        std::println("w_cmd    PWM  roda_cmd   atraso   giro_medido  medido/cmd  roda_real  girou_apos_parar  frames_repetidos");
        for (const Result& r : m_results) {
            const float wheel_cmd  = std::abs(r.w_cmd) * TRACK_WIDTH_M / 2.0f;
            const float wheel_real = std::abs(r.rate) * TRACK_WIDTH_M / 2.0f;
            std::println("{:+.2f}  {:5}  {:.3f}m/s  {:6.2f}s  {:+8.3f}rad/s  {:9.2f}  {:.3f}m/s  {:+13.3f}rad  {:10}",
                         r.w_cmd, firmware_pwm(wheel_cmd), wheel_cmd, r.delay, r.rate,
                         r.rate / r.w_cmd, wheel_real, r.coast, r.frozen_frames);
        }
        std::println("atraso = nan: nao chegou a girar {:.2f} rad em {:.1f}s.", TEST_MOVE_THRESHOLD, TEST_SPIN_S);
        std::println("frames_repetidos (so com o robo ja girando) alto: a visao provavelmente perdeu o robo e repetiu a pose antiga.");
    }

private:
    struct Result {
        float w_cmd = 0.0f;
        float delay = std::numeric_limits<float>::quiet_NaN();
        float rate  = std::numeric_limits<float>::quiet_NaN();
        float coast = std::numeric_limits<float>::quiet_NaN();
        int   frozen_frames = 0;
    };

    void begin_phase(float t) {
        m_spinning = true;
        m_steady_started = false;
        m_phase_start = t;
        m_phase_angle = m_angle;
        m_results[m_index].w_cmd = TEST_W[m_index];
        std::println("--- comando w={:+.2f} rad/s ---", TEST_W[m_index]);
    }

    std::array<Result, TEST_W.size()> m_results{};
    size_t m_index = 0;
    bool   m_started = false;
    bool   m_spinning = false;
    bool   m_steady_started = false;
    float  m_prev_raw = 0.0f;
    float  m_angle = 0.0f;         // ângulo desenrolado
    float  m_phase_start = 0.0f;
    float  m_phase_angle = 0.0f;
    float  m_steady_t = 0.0f;
    float  m_steady_angle = 0.0f;
};

// Ctrl+C: o firmware não tem timeout e manteria o último comando, então o
// laço principal manda parar antes de sair.
std::atomic<bool> g_interrupted{false};

void on_sigint(int) {
    g_interrupted = true;
}

} // namespace

int main() {
    try {
        std::println("Starting Strategy Listener Node...");
        Listener listener(MULTICAST_ADDRESS, MULTICAST_PORT);

        std::string target_ip;
        try {
            asio::io_context io_context;
            asio::ip::udp::resolver resolver(io_context);
            auto endpoints = resolver.resolve(asio::ip::udp::v4(), TARGET_HOSTNAME, "");
            if (endpoints.empty()) {
                throw std::runtime_error("No endpoints resolved");
            }
            target_ip = endpoints.begin()->endpoint().address().to_string();
            std::println("Found '{}' at IP: {}", TARGET_HOSTNAME, target_ip);
        } catch (const std::exception&) {
            std::println("Could not resolve the hostname {}.", TARGET_HOSTNAME);
            return 1;
        }

        UnicastSender sender(target_ip, UNICAST_PORT);

        FieldPoses field{};
        const size_t expected_size = sizeof(FieldPoses);
        auto buffer = reinterpret_cast<uint8_t*>(&field);

        const auto env_enabled = [](const char* name) {
            const char* value = std::getenv(name);
            return value != nullptr && std::string_view(value) == "1";
        };
        const bool spin_test_mode = env_enabled("FUT_TESTE_GIRO");
        const bool motor_test_mode = env_enabled("FUT_TESTE_MOTOR");
        SpinTest spin_test;
        MotorTest motor_test(TRACK_WIDTH_M);
        if (spin_test_mode) {
            std::println("MODO TESTE DE GIRO: deixe o robo livre no campo, longe da bola e das bordas.");
        }
        if (motor_test_mode) {
            std::println("MODO CALIBRACAO DOS MOTORES: robo no centro do campo, longe da bola e das bordas.");
            std::println("Ele vai girar no lugar e depois andar em linha reta (ida e volta). Duracao: ~3 minutos.");
        }
        std::signal(SIGINT, on_sigint);

        const auto start = std::chrono::steady_clock::now();
        auto last_frame = start;
        TrackingState tracking_state{};

        while (true) {
            const size_t bytes_read = listener.receive(buffer, expected_size);

            if (g_interrupted) {
                sender.send(TwistCmd{0.0f, 0.0f});
                std::println("\nInterrompido — comando de parada enviado.");
                if (spin_test_mode) spin_test.print_summary();
                if (motor_test_mode) motor_test.print_summary();
                return 0;
            }

            const auto now = std::chrono::steady_clock::now();
            const float dt = std::chrono::duration<float>(now - last_frame).count();
            last_frame = now;

            if (bytes_read != expected_size) {
                if (bytes_read > 0) {
                    std::println("Warning: packet size mismatch ({} vs {})",
                                 bytes_read, expected_size);
                }
                continue;
            }

            // Se a visão engasgou, não vale confiar na pose: manda parar.
            if (dt > 0.5f) {
                std::println("Vision gap of {:.2f}s — sending stop", dt);
                sender.send(TwistCmd{0.0f, 0.0f});
                tracking_state = {};
                continue;
            }

            // Limite inferior evita um pico no termo D quando dois pacotes
            // chegam quase juntos.
            const float control_dt = std::max(dt, 0.01f);

            const RobotPose& robot = field.robots[0];

            if (motor_test_mode) {
                const float t = std::chrono::duration<float>(now - start).count();
                const auto cmd = motor_test.step(robot, t);
                if (!cmd) {
                    sender.send(TwistCmd{0.0f, 0.0f});
                    motor_test.print_summary();
                    return 0;
                }
                sender.send(*cmd);
                continue;
            }

            if (spin_test_mode) {
                const float t = std::chrono::duration<float>(now - start).count();
                const auto cmd = spin_test.step(robot, t);
                if (!cmd) {
                    sender.send(TwistCmd{0.0f, 0.0f});
                    spin_test.print_summary();
                    return 0;
                }
                sender.send(*cmd);
                continue;
            }

            sender.send(compute_twist_control(robot, field.ball, tracking_state, control_dt));
        }
    } catch (const std::exception& e) {
        std::println("Exception in listener loop: {}", e.what());
        return 1;
    }

    return 0;
}
