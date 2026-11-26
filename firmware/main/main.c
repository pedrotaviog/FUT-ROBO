/* BSD Socket API Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdbool.h>
#include <string.h>
#include <sys/param.h>
#include <stdlib.h>
#include <math.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "mdns.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "protocol_examples_common.h"
#include "driver/gpio.h"
#include "driver/ledc.h"

#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include <lwip/netdb.h>

#include "common/protocol.h"

#ifdef CONFIG_EXAMPLE_SOCKET_IP_INPUT_STDIN
#include "addr_from_stdin.h"
#endif

#if defined(CONFIG_EXAMPLE_IPV4)
#define HOST_IP_ADDR CONFIG_EXAMPLE_IPV4_ADDR
#elif defined(CONFIG_EXAMPLE_IPV6)
#define HOST_IP_ADDR CONFIG_EXAMPLE_IPV6_ADDR
#else
#define HOST_IP_ADDR ""
#endif

#define PORT CONFIG_EXAMPLE_PORT

// --- HARDWARE DEFINITIONS & KINEMATICS ---
#define IN1 4
#define IN2 5
#define IN3 6
#define IN4 7

#define SOLENOIDE_PIN 10

#define TRACK_WIDTH 0.19f  // Track width (L) in meters (keep equal to TRACK_WIDTH_M in strategy)

// Calibração velocidade da roda -> PWM, medida com FUT_TESTE_MOTOR=1 (giro no
// lugar, média dos dois sentidos, bateria carregada; as duas rodas deram a
// mesma curva). A curva não é reta, então o PWM é interpolado entre os pontos.
// Refaça o teste e troque a tabela se mudar motor, roda, driver ou bateria.
// O último ponto é a velocidade máxima (keep equal to MAX_WHEEL_SPEED_MS in strategy).
static const float CAL_SPEED[] = {0.048f, 0.076f, 0.124f, 0.155f, 0.201f, 0.240f, 0.291f, 0.354f};
static const int   CAL_PWM[]   = {84,     92,     109,    126,    151,    176,    209,    255};
#define CAL_POINTS ((int)(sizeof(CAL_PWM) / sizeof(CAL_PWM[0])))

// Abaixo de V_DEADZONE (m/s) a roda fica parada. Entre V_DEADZONE e a menor
// velocidade da tabela, ela anda nessa menor velocidade: abaixo de PWM ~40 a
// roda já girando para, então não existe velocidade contínua menor.
#define V_DEADZONE 0.01f

// Tranco de partida: com a roda parada (ou invertendo o sentido), o atrito
// estático pede PWM acima da zona morta para começar.
// Por KICK_MS a roda recebe pelo menos KICK_PWM e depois volta ao da tabela.
#define KICK_PWM 100
#define KICK_MS 60

// Failsafe: sem nenhum comando por este tempo, os motores param. Sem isso o
// robô manteria o último comando para sempre se o strategy caísse.
#define CMD_TIMEOUT_MS 300

// Estado de cada roda para o tranco de partida.
typedef struct {
    int     last_pwm;        // último PWM pedido (sem o tranco)
    int64_t kick_until_us;   // até quando o tranco vale
} wheel_state_t;

static wheel_state_t wheel_left = {0, 0};
static wheel_state_t wheel_right = {0, 0};

typedef struct {
    int pin1;
    int pin2;
    ledc_channel_t channel1;
    ledc_channel_t channel2;
} motor_t;

static motor_t ME = { .pin1 = IN1, .pin2 = IN2, .channel1 = LEDC_CHANNEL_0, .channel2 = LEDC_CHANNEL_1 };
static motor_t MD = { .pin1 = IN3, .pin2 = IN4, .channel1 = LEDC_CHANNEL_2, .channel2 = LEDC_CHANNEL_3 };

static const char *TAG = "robot_main";

void set_PWM(motor_t *motor, int velocity)
{
    int duty = abs(velocity) * 4095 / 255;
    duty = duty > 4095 ? 4095 : duty;
    if (velocity > 0) {
        ledc_set_duty(LEDC_LOW_SPEED_MODE, motor->channel1, duty);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, motor->channel1);
        ledc_set_duty(LEDC_LOW_SPEED_MODE, motor->channel2, 0);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, motor->channel2);
    } else if (velocity < 0) {
        ledc_set_duty(LEDC_LOW_SPEED_MODE, motor->channel1, 0);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, motor->channel1);
        ledc_set_duty(LEDC_LOW_SPEED_MODE, motor->channel2, duty);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, motor->channel2);
    } else {
        ledc_set_duty(LEDC_LOW_SPEED_MODE, motor->channel1, 0);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, motor->channel1);
        ledc_set_duty(LEDC_LOW_SPEED_MODE, motor->channel2, 0);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, motor->channel2);
    }
}

void init_motor_pins(void)
{
    gpio_set_direction(IN1, GPIO_MODE_OUTPUT);
    gpio_set_direction(IN2, GPIO_MODE_OUTPUT);
    gpio_set_direction(IN3, GPIO_MODE_OUTPUT);
    gpio_set_direction(IN4, GPIO_MODE_OUTPUT);

    ledc_timer_config_t t = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .timer_num = LEDC_TIMER_0,
        .duty_resolution = LEDC_TIMER_12_BIT,
        .freq_hz = 1000,
        .clk_cfg = LEDC_AUTO_CLK
    };
    ledc_timer_config(&t);

    ledc_channel_config_t cs[] = {
        { .gpio_num = IN1, .speed_mode = LEDC_LOW_SPEED_MODE, .channel = LEDC_CHANNEL_0, .intr_type = LEDC_INTR_DISABLE, .timer_sel = LEDC_TIMER_0, .duty = 0, .hpoint = 0 },
        { .gpio_num = IN2, .speed_mode = LEDC_LOW_SPEED_MODE, .channel = LEDC_CHANNEL_1, .intr_type = LEDC_INTR_DISABLE, .timer_sel = LEDC_TIMER_0, .duty = 0, .hpoint = 0 },
        { .gpio_num = IN3, .speed_mode = LEDC_LOW_SPEED_MODE, .channel = LEDC_CHANNEL_2, .intr_type = LEDC_INTR_DISABLE, .timer_sel = LEDC_TIMER_0, .duty = 0, .hpoint = 0 },
        { .gpio_num = IN4, .speed_mode = LEDC_LOW_SPEED_MODE, .channel = LEDC_CHANNEL_3, .intr_type = LEDC_INTR_DISABLE, .timer_sel = LEDC_TIMER_0, .duty = 0, .hpoint = 0 }
    };
    for (int i = 0; i < 4; i++) {
        ledc_channel_config(&cs[i]);
    }
    ESP_LOGI(TAG, "Motores init");
}

int scale_velocity_to_pwm(float velocity)
{
    if (fabsf(velocity) < V_DEADZONE) {
        return 0;
    }
    int sign = (velocity > 0.0f) ? 1 : -1;
    float abs_v = fabsf(velocity);

    if (abs_v <= CAL_SPEED[0]) {
        return CAL_PWM[0] * sign;
    }
    for (int i = 1; i < CAL_POINTS; i++) {
        if (abs_v <= CAL_SPEED[i]) {
            // Interpolação linear entre os dois pontos medidos vizinhos
            float t = (abs_v - CAL_SPEED[i - 1]) / (CAL_SPEED[i] - CAL_SPEED[i - 1]);
            int pwm = CAL_PWM[i - 1] + (int)lroundf(t * (CAL_PWM[i] - CAL_PWM[i - 1]));
            return pwm * sign;
        }
    }
    return CAL_PWM[CAL_POINTS - 1] * sign;
}

// Aplica o tranco de partida: devolve o PWM a usar agora para a roda.
int apply_kick(wheel_state_t *wheel, int pwm)
{
    int64_t now = esp_timer_get_time();
    if (pwm == 0) {
        wheel->kick_until_us = 0;
    } else if (wheel->last_pwm == 0 || (wheel->last_pwm > 0) != (pwm > 0)) {
        // Saindo do repouso ou invertendo o sentido
        wheel->kick_until_us = now + KICK_MS * 1000;
    }
    wheel->last_pwm = pwm;

    if (pwm != 0 && now < wheel->kick_until_us && abs(pwm) < KICK_PWM) {
        return pwm > 0 ? KICK_PWM : -KICK_PWM;
    }
    return pwm;
}

static void run_robot(void *pvParameters)
{
    int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    if (sock < 0) {
        ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
        vTaskDelete(NULL);
        return;
    }

    esp_netif_ip_info_t ip_info;
    esp_netif_get_ip_info(esp_netif_get_handle_from_ifkey("WIFI_STA_DEF"), &ip_info);

    struct sockaddr_in local_addr;
    local_addr.sin_addr.s_addr = ip_info.ip.addr;
    local_addr.sin_family = AF_INET;
    local_addr.sin_port = htons(PORT);

    if (bind(sock, (struct sockaddr *)&local_addr, sizeof(local_addr)) < 0) {
        ESP_LOGE(TAG, "Socket bind failed: errno %d", errno);
        close(sock);
        vTaskDelete(NULL);
        return;
    }

    struct timeval timeout = { .tv_sec = 0, .tv_usec = CMD_TIMEOUT_MS * 1000 };
    setsockopt(sock, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));

    ESP_LOGI(TAG, "UDP receiver listening on port %d...", PORT);

    bool motors_on = false;

    while (1) {
        struct sockaddr_in source_addr;
        socklen_t socklen = sizeof(source_addr);
        union {
            struct TwistCmd twist;
            struct MotorPwmCmd pwm;
            uint8_t raw[16];
        } packet;
        int len = recvfrom(sock, &packet, sizeof(packet), 0, (struct sockaddr *)&source_addr, &socklen);

        if (len < 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK) {
                if (motors_on) {
                    ESP_LOGW(TAG, "No command for %d ms, stopping motors", CMD_TIMEOUT_MS);
                    set_PWM(&ME, 0);
                    set_PWM(&MD, 0);
                    apply_kick(&wheel_left, 0);
                    apply_kick(&wheel_right, 0);
                    motors_on = false;
                }
                continue;
            }
            ESP_LOGE(TAG, "recvfrom failed: errno %d", errno);
            break;
        }

        char sender_ip[32];
        inet_ntoa_r(source_addr.sin_addr, sender_ip, sizeof(sender_ip) - 1);

        // Log por pacote em nível DEBUG: em INFO, duas linhas por comando a
        // ~30 Hz saturam a serial e atrasam o laço.
        if (len == sizeof(struct TwistCmd)) {
            ESP_LOGD(TAG, "Received TwistCmd from %s: v=%.2f, w=%.2f", sender_ip, packet.twist.v, packet.twist.w);

            // Kinematic translation from linear and angular velocities to individual wheel velocities
            float v_l = packet.twist.v - (packet.twist.w * TRACK_WIDTH / 2.0f);
            float v_r = packet.twist.v + (packet.twist.w * TRACK_WIDTH / 2.0f);

            // Convert velocity (m/s) to PWM with the measured calibration table,
            // plus the start-up kick when a wheel leaves rest
            int pwm_l = apply_kick(&wheel_left, scale_velocity_to_pwm(v_l));
            int pwm_r = apply_kick(&wheel_right, scale_velocity_to_pwm(v_r));

            ESP_LOGD(TAG, "Setting PWM - Left (ME): %d, Right (MD): %d (from pwm_l=%d, pwm_r=%d)", pwm_l, pwm_r, pwm_l, pwm_r);

            // Apply motor directions (both motors move forward with positive PWM on this hardware)
            set_PWM(&ME, pwm_l);
            set_PWM(&MD, pwm_r);
            motors_on = pwm_l != 0 || pwm_r != 0;
        } else if (len == sizeof(struct MotorPwmCmd)) {
            // Calibração: PWM direto, sem tabela nem tranco, mesma convenção
            // de sentido do TwistCmd.
            int pwm_l = MAX(-255, MIN(255, packet.pwm.left));
            int pwm_r = MAX(-255, MIN(255, packet.pwm.right));
            wheel_left.last_pwm = pwm_l;
            wheel_right.last_pwm = pwm_r;
            ESP_LOGD(TAG, "Received MotorPwmCmd from %s: left=%d, right=%d", sender_ip, pwm_l, pwm_r);

            set_PWM(&ME, pwm_l);
            set_PWM(&MD, pwm_r);
            motors_on = pwm_l != 0 || pwm_r != 0;
        } else {
            ESP_LOGW(TAG, "Received %d bytes (expected %d or %d) from %s", len,
                     (int)sizeof(struct TwistCmd), (int)sizeof(struct MotorPwmCmd), sender_ip);
        }
    }

    // Garante motores parados caso o loop encerre por erro de socket
    set_PWM(&ME, 0);
    set_PWM(&MD, 0);
    shutdown(sock, 0);
    close(sock);
    vTaskDelete(NULL);
}

esp_err_t start_mdns_service()
{
    //initialize mDNS service
    esp_err_t err = mdns_init();
    if (err) {
        ESP_LOGE(TAG, "MDNS Init failed: %d\n", err);
        return err;
    }

    //set hostname
    return mdns_hostname_set("robot");
}

// void run_test(void *pvParameters) {
//     int w = 50;

//     while (1) {
//         set_PWM(&ME, w);
//         set_PWM(&MD, 0);
//         vTaskDelay(pdMS_TO_TICKS(2000));

//         set_PWM(&ME, 0);
//         set_PWM(&MD, w);
//         vTaskDelay(pdMS_TO_TICKS(2000));

//         set_PWM(&ME, 0);
//         set_PWM(&MD, 0);
//         vTaskDelay(pdMS_TO_TICKS(2000));
//     }
// }

void app_main(void)
{
    // Mantém a solenoide travada em nível baixo com pull-down desde o início
    gpio_set_level(SOLENOIDE_PIN, 0);
    gpio_config_t sol_cfg = {
        .pin_bit_mask = (1ULL << SOLENOIDE_PIN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&sol_cfg);

    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    ESP_ERROR_CHECK(start_mdns_service());

    // Initialize motor driver pins and LEDC PWM channels
    init_motor_pins();

    /* This helper function configures Wi-Fi or Ethernet, as selected in menuconfig.
- Read "Establishing Wi-Fi or Ethernet Connection" section in
- examples/protocols/README.md for more information about this function.
     */
    ESP_ERROR_CHECK(example_connect());

    xTaskCreate(run_robot, "run_robot", 4096, NULL, 5, NULL);
    // xTaskCreate(run_robot, "robot", 4096, NULL, 5, NULL);
}