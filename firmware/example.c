#include <stdint.h>
#include <string.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_bt.h"
#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_bt_defs.h"
#include "esp_bt_main.h"
#include "esp_gatt_common_api.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_timer.h"
#include "inttypes.h"

static const char *TAG = "ROBO_BLE_PERC"; // Nova Tag

// --- DEFINIÇÕES HARDWARE ---
#define IN1 16
#define IN2 17
#define ENCODER_CLK_MD 34
#define ENCODER_DT_MD 35
#define IN3 18
#define IN4 19
#define ENCODER_CLK_ME 36
#define ENCODER_DT_ME 39
#define CHUTADOR 21
#define VELOCIDADE_BASE_DIREITA 255 // <<-- Velocidade Base
#define VELOCIDADE_BASE_ESQUERDA 225 // <<-- Velocidade Base
#define ENCODER_DEBOUNCE_TIME_US 1000

// --- VARIÁVEIS GLOBAIS ---
volatile int contador_pulsos_MD = 0;
volatile int contador_pulsos_ME = 0;
//static char comando_direcao_atual[10] = "S";
//static uint8_t velocidade_percentual = 100;
static uint32_t encoder_task_delay_ms = 100;
static int64_t last_md_interrupt_time = 0;
static int64_t last_me_interrupt_time = 0;
static portMUX_TYPE my_spinlock = portMUX_INITIALIZER_UNLOCKED;

static int g_velocidade_md = 0; // Velocidade global para Motor Direito
static int g_velocidade_me = 0; // Velocidade global para Motor Esquerdo

// Estrutura Motor
typedef struct { int pin1; int pin2; ledc_channel_t channel1; ledc_channel_t channel2; } motor_t;
motor_t MD = { .pin1 = IN1, .pin2 = IN2, .channel1 = LEDC_CHANNEL_0, .channel2 = LEDC_CHANNEL_1 };
motor_t ME = { .pin1 = IN3, .pin2 = IN4, .channel1 = LEDC_CHANNEL_2, .channel2 = LEDC_CHANNEL_3 };

// --- DEFINIÇÕES BLE ---
#define GATTS_TAG TAG
#define PROFILE_NUM                 1
#define PROFILE_APP_ID              0
#define ESP_APP_ID                  0x55
#define DEVICE_NAME                 "Robo_BLE"
#define SVC_INST_ID                 0
#define GATTS_SERVICE_UUID_ROBOT    0xABF0
#define GATTS_CHAR_UUID_COMMAND     0xABF1
#define GATTS_CHAR_UUID_ENCODER     0xABF2
#define GATTS_DESCR_UUID_CCC        0x2902
#define GATTS_NUM_HANDLE_ROBO       6

// Advertising Data
static uint8_t adv_config_done = 0;
#define adv_config_flag      (1 << 0)
static esp_ble_adv_params_t adv_params = {
    .adv_int_min=0x20,.adv_int_max=0x40,.adv_type=ADV_TYPE_IND,.own_addr_type=BLE_ADDR_TYPE_PUBLIC,
    .channel_map=ADV_CHNL_ALL,.adv_filter_policy=ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};
static esp_ble_adv_data_t adv_data = {
    .set_scan_rsp=false,.include_name=true,.include_txpower=false,.appearance=0x00,
    .manufacturer_len=0,.p_manufacturer_data=NULL,.service_data_len=0,.p_service_data=NULL,
    .service_uuid_len=0,.p_service_uuid=NULL,.flag=(ESP_BLE_ADV_FLAG_GEN_DISC|ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),
};

// Estrutura Perfil
struct gatts_profile_inst {
    esp_gatts_cb_t gatts_cb; uint16_t gatts_if; uint16_t app_id; uint16_t conn_id;
    uint16_t service_handle; esp_gatt_srvc_id_t service_id;
    uint16_t char_command_handle; uint16_t char_encoder_handle; uint16_t descr_encoder_handle;
    bool is_connected; bool notify_enabled;
};

// Handlers
static void gatts_profile_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);
static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param);

// Instância Perfil
static struct gatts_profile_inst robo_profile_tab[PROFILE_NUM] = {
    [PROFILE_APP_ID] = { .gatts_cb = gatts_profile_event_handler, .gatts_if = ESP_GATT_IF_NONE, .is_connected = false, .notify_enabled = false, },
};

// --- FUNÇÕES HARDWARE (sem alterações) ---
static void IRAM_ATTR encoder_md_isr_handler(void* arg){
    int64_t now=esp_timer_get_time(); if(now-last_md_interrupt_time < ENCODER_DEBOUNCE_TIME_US) return; last_md_interrupt_time=now;
    portENTER_CRITICAL_ISR(&my_spinlock); int clk=gpio_get_level(ENCODER_CLK_MD), dt=gpio_get_level(ENCODER_DT_MD); contador_pulsos_MD += (clk != dt) ? 1 : -1; portEXIT_CRITICAL_ISR(&my_spinlock);
}
static void IRAM_ATTR encoder_me_isr_handler(void* arg){
    int64_t now=esp_timer_get_time(); if(now-last_me_interrupt_time < ENCODER_DEBOUNCE_TIME_US) return; last_me_interrupt_time=now;
    portENTER_CRITICAL_ISR(&my_spinlock); int clk=gpio_get_level(ENCODER_CLK_ME), dt=gpio_get_level(ENCODER_DT_ME); contador_pulsos_ME += (clk != dt) ? 1 : -1; portEXIT_CRITICAL_ISR(&my_spinlock);
}
void set_PWM(motor_t motor, int velocidade){
    int duty=abs(velocidade)*4095/255; duty=duty>4095?4095:duty;
    if(velocidade>0){ledc_set_duty(LEDC_HIGH_SPEED_MODE, motor.channel1, duty); ledc_update_duty(LEDC_HIGH_SPEED_MODE, motor.channel1); ledc_set_duty(LEDC_HIGH_SPEED_MODE, motor.channel2, 0); ledc_update_duty(LEDC_HIGH_SPEED_MODE, motor.channel2);}
    else if(velocidade<0){ledc_set_duty(LEDC_HIGH_SPEED_MODE, motor.channel1, 0); ledc_update_duty(LEDC_HIGH_SPEED_MODE, motor.channel1); ledc_set_duty(LEDC_HIGH_SPEED_MODE, motor.channel2, duty); ledc_update_duty(LEDC_HIGH_SPEED_MODE, motor.channel2);}
    else{ledc_set_duty(LEDC_HIGH_SPEED_MODE, motor.channel1, 0); ledc_update_duty(LEDC_HIGH_SPEED_MODE, motor.channel1); ledc_set_duty(LEDC_HIGH_SPEED_MODE, motor.channel2, 0); ledc_update_duty(LEDC_HIGH_SPEED_MODE, motor.channel2);}
}
void init_encoders(void){
    gpio_config_t c = {.intr_type=GPIO_INTR_ANYEDGE, .mode=GPIO_MODE_INPUT, .pin_bit_mask=((1ULL<<ENCODER_CLK_MD)|(1ULL<<ENCODER_DT_MD)|(1ULL<<ENCODER_CLK_ME)|(1ULL<<ENCODER_DT_ME)), .pull_up_en=1}; gpio_config(&c);
    gpio_install_isr_service(0); gpio_isr_handler_add(ENCODER_CLK_MD, encoder_md_isr_handler, NULL); gpio_isr_handler_add(ENCODER_CLK_ME, encoder_me_isr_handler, NULL);
    gpio_isr_handler_add(ENCODER_DT_MD, encoder_md_isr_handler, NULL); gpio_isr_handler_add(ENCODER_DT_ME, encoder_me_isr_handler, NULL); ESP_LOGI(TAG, "Encoders init");
}
void init_motor_pins(void){
    gpio_set_direction(IN1,GPIO_MODE_OUTPUT); gpio_set_direction(IN2,GPIO_MODE_OUTPUT); gpio_set_direction(IN3,GPIO_MODE_OUTPUT); gpio_set_direction(IN4,GPIO_MODE_OUTPUT);
    ledc_timer_config_t t = {.speed_mode=LEDC_HIGH_SPEED_MODE, .timer_num=LEDC_TIMER_0, .duty_resolution=LEDC_TIMER_12_BIT, .freq_hz=1000, .clk_cfg=LEDC_AUTO_CLK}; ledc_timer_config(&t);
    // --- CORREÇÃO INICIALIZAÇÃO LEDC ---
    ledc_channel_config_t cs[] = {
        { .gpio_num = IN1, .speed_mode = LEDC_HIGH_SPEED_MODE, .channel = LEDC_CHANNEL_0, .intr_type = LEDC_INTR_DISABLE, .timer_sel = LEDC_TIMER_0, .duty = 0, .hpoint = 0 },
        { .gpio_num = IN2, .speed_mode = LEDC_HIGH_SPEED_MODE, .channel = LEDC_CHANNEL_1, .intr_type = LEDC_INTR_DISABLE, .timer_sel = LEDC_TIMER_0, .duty = 0, .hpoint = 0 },
        { .gpio_num = IN3, .speed_mode = LEDC_HIGH_SPEED_MODE, .channel = LEDC_CHANNEL_2, .intr_type = LEDC_INTR_DISABLE, .timer_sel = LEDC_TIMER_0, .duty = 0, .hpoint = 0 },
        { .gpio_num = IN4, .speed_mode = LEDC_HIGH_SPEED_MODE, .channel = LEDC_CHANNEL_3, .intr_type = LEDC_INTR_DISABLE, .timer_sel = LEDC_TIMER_0, .duty = 0, .hpoint = 0 }
    };
    for(int i=0;i<4;i++){ ledc_channel_config(&cs[i]); }
    // --- FIM CORREÇÃO ---
    ESP_LOGI(TAG, "Motores init");
}
static void kick(){gpio_set_level(CHUTADOR, 1); vTaskDelay(pdMS_TO_TICKS(500)); gpio_set_level(CHUTADOR, 0); vTaskDelay(pdMS_TO_TICKS(200)); ESP_LOGI(TAG,"Chute!");}

// --- HANDLER GAP ---
static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param) {
    switch (event) {
        case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT:
            adv_config_done |= adv_config_flag; ESP_LOGI(TAG,"GAP: ADV_DATA_SET_COMPLETE, status=%d", param->adv_data_cmpl.status);
            if (adv_config_done == adv_config_flag) { esp_ble_gap_start_advertising(&adv_params); } break;
        case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
            if (param->adv_start_cmpl.status==ESP_BT_STATUS_SUCCESS) ESP_LOGI(TAG,"Advertising iniciado"); else ESP_LOGE(TAG,"Adv start failed: %d",param->adv_start_cmpl.status); break;
        case ESP_GAP_BLE_ADV_STOP_COMPLETE_EVT: ESP_LOGI(TAG,"Advertising parado, status=%d", param->adv_stop_cmpl.status); break;
        case ESP_GAP_BLE_UPDATE_CONN_PARAMS_EVT: ESP_LOGI(TAG,"update conn params status=%d, min_int=%d...",param->update_conn_params.status,param->update_conn_params.min_int); break;
        default: ESP_LOGD(TAG,"GAP Evento: %d", event); break;
    }
}

// --- HANDLER GATTS (com parse de comandos + percentagem) ---
static void gatts_profile_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param) {
    ESP_LOGD(TAG, "GATTS Evento: %d, gatts_if %d", event, gatts_if);

    if (event == ESP_GATTS_CONNECT_EVT || event == ESP_GATTS_DISCONNECT_EVT) {
        if(event == ESP_GATTS_CONNECT_EVT){ ESP_LOGI(TAG,"CONNECT: conn %d",param->connect.conn_id); robo_profile_tab[0].conn_id=param->connect.conn_id; robo_profile_tab[0].gatts_if=gatts_if; robo_profile_tab[0].is_connected=true; esp_ble_gap_stop_advertising();}
        else { ESP_LOGI(TAG,"DISCONNECT: conn %d, reason %x",param->disconnect.conn_id,param->disconnect.reason); robo_profile_tab[0].is_connected=false; robo_profile_tab[0].notify_enabled=false; esp_ble_gap_start_advertising(&adv_params);} return;
    }
    if (robo_profile_tab[0].gatts_if!=ESP_GATT_IF_NONE && gatts_if!=robo_profile_tab[0].gatts_if) return;

    switch (event) {
        case ESP_GATTS_REG_EVT:
            ESP_LOGI(TAG,"REG_EVT, status %d",param->reg.status); if(param->reg.status!=ESP_GATT_OK)return; robo_profile_tab[0].gatts_if=gatts_if;
            esp_ble_gap_set_device_name(DEVICE_NAME); esp_ble_gap_config_adv_data(&adv_data); adv_config_done|=adv_config_flag;
            robo_profile_tab[0].service_id.is_primary=true; robo_profile_tab[0].service_id.id.inst_id=0; robo_profile_tab[0].service_id.id.uuid.len=ESP_UUID_LEN_16; robo_profile_tab[0].service_id.id.uuid.uuid.uuid16=GATTS_SERVICE_UUID_ROBOT;
            esp_ble_gatts_create_service(gatts_if,&robo_profile_tab[0].service_id,GATTS_NUM_HANDLE_ROBO); break;
        case ESP_GATTS_CREATE_EVT:
            ESP_LOGI(TAG,"CREATE_SVC, status %d, handle %d",param->create.status,param->create.service_handle); if(param->create.status!=ESP_GATT_OK)return; robo_profile_tab[0].service_handle=param->create.service_handle;
            esp_ble_gatts_start_service(robo_profile_tab[0].service_handle); break;
        case ESP_GATTS_START_EVT:
             ESP_LOGI(TAG,"START_SVC, status %d, handle %d", param->start.status, param->start.service_handle); if(param->start.status!=ESP_GATT_OK)return;
             ESP_LOGI(TAG, "Adicionando char Comando...");
             // --- CORREÇÃO UUID CHAR ---
             esp_bt_uuid_t char_uuid_cmd = {.len = ESP_UUID_LEN_16, .uuid.uuid16 = GATTS_CHAR_UUID_COMMAND};
             esp_ble_gatts_add_char(param->start.service_handle, &char_uuid_cmd,
                                    ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
                                    ESP_GATT_CHAR_PROP_BIT_WRITE,
                                    NULL, NULL);
             // --- FIM CORREÇÃO ---
             break;
        case ESP_GATTS_ADD_CHAR_EVT: {
            uint16_t char_uuid = param->add_char.char_uuid.uuid.uuid16;
            ESP_LOGI(TAG,"ADD_CHAR, status %d, handle %d, uuid %x",param->add_char.status,param->add_char.attr_handle,char_uuid);
            if(param->add_char.status!=ESP_GATT_OK)break;

            if(char_uuid==GATTS_CHAR_UUID_COMMAND){
                robo_profile_tab[0].char_command_handle=param->add_char.attr_handle;
                ESP_LOGI(TAG, "Adicionando char Encoder...");
                // --- CORREÇÃO UUID CHAR ---
                esp_bt_uuid_t char_uuid_enc = {.len = ESP_UUID_LEN_16, .uuid.uuid16 = GATTS_CHAR_UUID_ENCODER};
                esp_ble_gatts_add_char(param->add_char.service_handle, &char_uuid_enc,
                                       ESP_GATT_PERM_READ,
                                       ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_NOTIFY,
                                       NULL, NULL);
                // --- FIM CORREÇÃO ---
            }
            else if(char_uuid==GATTS_CHAR_UUID_ENCODER){
                robo_profile_tab[0].char_encoder_handle=param->add_char.attr_handle;
                ESP_LOGI(TAG, "Adicionando desc CCCD...");
                // --- CORREÇÃO UUID DESCR ---
                esp_bt_uuid_t descr_uuid_ccc = {.len = ESP_UUID_LEN_16, .uuid.uuid16 = GATTS_DESCR_UUID_CCC};
                esp_ble_gatts_add_char_descr(param->add_char.service_handle, &descr_uuid_ccc,
                                             ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
                                             NULL, NULL);
                // --- FIM CORREÇÃO ---
            }
            break; // <<<--- CORREÇÃO: BREAK estava no sítio errado
        } // <<<--- CORREÇÃO: Fechar chaveta do case ADD_CHAR_EVT
        case ESP_GATTS_ADD_CHAR_DESCR_EVT:
             ESP_LOGI(TAG,"ADD_DESCR, status %d, handle %d",param->add_char_descr.status,param->add_char_descr.attr_handle);
             if(param->add_char_descr.status==ESP_GATT_OK && param->add_char_descr.descr_uuid.uuid.uuid16==GATTS_DESCR_UUID_CCC){
                 robo_profile_tab[0].descr_encoder_handle=param->add_char_descr.attr_handle;
                 ESP_LOGI(TAG,"--- Config GATT OK ---");
             } else {
                 ESP_LOGE(TAG,"Falha desc CCCD");
             }
             break; // <<<--- CORREÇÃO: Adicionar BREAK

        case ESP_GATTS_WRITE_EVT:
            ESP_LOGI(TAG, "WRITE_EVT, conn %d, handle %d, len %d", param->write.conn_id, param->write.handle, param->write.len);
            if (!param->write.is_prep){
                ESP_LOG_BUFFER_HEX(TAG, param->write.value, param->write.len);
                // Escrita no CCCD? (Lógica do Encoder, pode manter)
                if (robo_profile_tab[0].descr_encoder_handle == param->write.handle && param->write.len == 2){
                    uint16_t v = param->write.value[1]<<8 | param->write.value[0];
                    if (v == 0x0001){ ESP_LOGI(TAG, "Notificações ATIVADAS"); robo_profile_tab[0].notify_enabled = true; }
                    else if (v == 0x0000){ ESP_LOGI(TAG, "Notificações DESATIVADAS"); robo_profile_tab[0].notify_enabled = false; }
                    else { ESP_LOGE(TAG, "Valor CCCD incorreto: 0x%04x", v); }
                // Escrita no Comando?
                } else if (robo_profile_tab[0].char_command_handle == param->write.handle) {
                    char recv_buf[32];
                    int len = param->write.len < sizeof(recv_buf) - 1 ? param->write.len : sizeof(recv_buf) - 1;
                    memcpy(recv_buf, param->write.value, len);
                    recv_buf[len] = '\0';
                    ESP_LOGI(TAG, "Comando Recebido Raw: '%s'", recv_buf);

                    // --- INÍCIO DA MODIFICAÇÃO ---

                    uint32_t new_interval;
                    // Variáveis para o comando de movimento
                    char direcao_str[10]; 
                    int vel_d_recv, vel_e_recv;

                    if (sscanf(recv_buf, "I %" PRIu32, &new_interval) == 1) {
                        if (new_interval >= 50 && new_interval <= 2000) {
                            encoder_task_delay_ms = new_interval;
                            ESP_LOGI(TAG, "Intervalo Encoder -> %" PRIu32 " ms", encoder_task_delay_ms);
                    } else { ESP_LOGW(TAG, "Intervalo inválido: %" PRIu32, new_interval); }

                    // REMOVIDO: O sscanf de "P %" (percentagem) não é mais necessário

                    } else if (strchr(recv_buf, 'K') != NULL) {
                        ESP_LOGI(TAG, "Comando Chute"); 
                        kick(); 
                        g_velocidade_md = 0; // Para o robô após o chute
                        g_velocidade_me = 0;

                    // ADICIONADO: Tenta ler o comando de movimento (ex: "F 250 255")
                    } else if (sscanf(recv_buf, "%s %d %d", direcao_str, &vel_d_recv, &vel_e_recv) == 3) {
                        g_velocidade_md = vel_d_recv;
                        g_velocidade_me = vel_e_recv;
                        ESP_LOGI(TAG, "Movimento: %s, MD: %d, ME: %d", direcao_str, g_velocidade_md, g_velocidade_me);
                    
                    } else { // Comando não reconhecido
                        ESP_LOGW(TAG, "Comando não reconhecido: %s", recv_buf);
                        // Para o robô por segurança se o comando for inválido
                        g_velocidade_md = 0;
                        g_velocidade_me = 0;
                    }
                    // --- FIM DA MODIFICAÇÃO ---
            ESP_LOGI(TAG, "Comando Recebido Raw: '%s'", recv_buf);

            } else { ESP_LOGW(TAG, "Escrita em handle desconhecido."); }

                if (param->write.need_rsp) esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, ESP_GATT_OK, NULL);
            } else { if (param->write.need_rsp) esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, ESP_GATT_REQ_NOT_SUPPORTED, NULL); }
            break;

        // Outros eventos...
        case ESP_GATTS_MTU_EVT: ESP_LOGI(TAG,"MTU %d", param->mtu.mtu); break;
        case ESP_GATTS_STOP_EVT: ESP_LOGI(TAG,"STOP_SVC, status %d",param->stop.status); break;
        case ESP_GATTS_DELETE_EVT: ESP_LOGI(TAG,"DEL_SVC, status %d",param->del.status); break;
        case ESP_GATTS_UNREG_EVT: ESP_LOGI(TAG,"UNREG_EVT"); break;
        case ESP_GATTS_READ_EVT: ESP_LOGI(TAG,"READ_EVT, handle %d",param->read.handle); esp_ble_gatts_send_response(gatts_if, param->read.conn_id, param->read.trans_id, ESP_GATT_READ_NOT_PERMIT, NULL); break;
        case ESP_GATTS_CONF_EVT: ESP_LOGI(TAG,"CONF_EVT, status %d",param->conf.status); break;
        default: ESP_LOGW(TAG, "GATTS Evento %d não tratado", event); break;
    }
}


// --- TASKS (com intervalo e velocidade ajustáveis) ---
static void encoder_monitor_task(void *arg) {
    while (1) {
        int pulsos_MD, pulsos_ME;
        taskENTER_CRITICAL(&my_spinlock);
        pulsos_MD = contador_pulsos_MD; contador_pulsos_MD = 0;
        pulsos_ME = contador_pulsos_ME; contador_pulsos_ME = 0;
        taskEXIT_CRITICAL(&my_spinlock);

        if (robo_profile_tab[0].is_connected && robo_profile_tab[0].notify_enabled) {
            char buffer[64];
            snprintf(buffer, sizeof(buffer), "ENC %d %d\n", pulsos_MD, pulsos_ME);
            esp_ble_gatts_send_indicate(robo_profile_tab[0].gatts_if, robo_profile_tab[0].conn_id,
                                        robo_profile_tab[0].char_encoder_handle, strlen(buffer), (uint8_t *)buffer, false);
        }
        vTaskDelay(pdMS_TO_TICKS(encoder_task_delay_ms));
    }
}

static void movimento_task(void *arg) {
    while (true) {
        /*int base_vel_md = 0;
        int base_vel_me = 0;
        float fator = (float)velocidade_percentual / 100.0;

        // Determina velocidades base (igual anterior)
        if (strstr(comando_direcao_atual, "FR"))      { base_vel_md = VELOCIDADE_BASE_DIREITA/2; base_vel_me = VELOCIDADE_BASE_ESQUERDA;  }
        else if (strstr(comando_direcao_atual, "FL")) { base_vel_md = VELOCIDADE_BASE_DIREITA;  base_vel_me = VELOCIDADE_BASE_ESQUERDA/2;   }
        else if (strstr(comando_direcao_atual, "BR")) { base_vel_md = -VELOCIDADE_BASE_DIREITA/2; base_vel_me = -VELOCIDADE_BASE_ESQUERDA;  }
        else if (strstr(comando_direcao_atual, "BL")) { base_vel_md = -VELOCIDADE_BASE_DIREITA;   base_vel_me = -VELOCIDADE_BASE_ESQUERDA/2;}
        else if (strchr(comando_direcao_atual, 'F'))  { base_vel_md = -VELOCIDADE_BASE_DIREITA;   base_vel_me = VELOCIDADE_BASE_ESQUERDA;   }
        else if (strchr(comando_direcao_atual, 'B'))  { base_vel_md = VELOCIDADE_BASE_DIREITA;    base_vel_me = -VELOCIDADE_BASE_ESQUERDA;  }
        else if (strchr(comando_direcao_atual, 'R'))  { base_vel_md = VELOCIDADE_BASE_DIREITA;    base_vel_me = VELOCIDADE_BASE_ESQUERDA;  }
        else if (strchr(comando_direcao_atual, 'L'))  { base_vel_md = -VELOCIDADE_BASE_DIREITA;   base_vel_me = -VELOCIDADE_BASE_ESQUERDA;   }
        */
        int final_vel_md = -g_velocidade_md;
        int final_vel_me = g_velocidade_me;

        set_PWM(MD, final_vel_md);
        set_PWM(ME, final_vel_me);

        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

// HACK Bluedroid
esp_err_t esp_bluedroid_init(void);
esp_err_t esp_bluedroid_enable(void);

// --- APP MAIN ---
void app_main(void) {
    esp_err_t ret;
    ESP_LOGI(TAG, "Iniciando app_main...");
    ret = nvs_flash_init(); if (ret==ESP_ERR_NVS_NO_FREE_PAGES||ret==ESP_ERR_NVS_NEW_VERSION_FOUND){ESP_ERROR_CHECK(nvs_flash_erase()); ret=nvs_flash_init();} ESP_ERROR_CHECK(ret);
    ESP_LOGI(TAG, "Inicializando Hardware...");
    init_motor_pins(); init_encoders(); gpio_set_direction(CHUTADOR,GPIO_MODE_OUTPUT); gpio_set_level(CHUTADOR,0);
    ESP_LOGI(TAG, "Inicializando BT Controller...");
    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));
    esp_bt_controller_config_t cfg=BT_CONTROLLER_INIT_CONFIG_DEFAULT(); ret=esp_bt_controller_init(&cfg); if(ret){ESP_LOGE(TAG,"init controller failed: %s",esp_err_to_name(ret));return;}
    ret=esp_bt_controller_enable(ESP_BT_MODE_BLE); if(ret){ESP_LOGE(TAG,"enable controller failed: %s",esp_err_to_name(ret));return;}
    ESP_LOGI(TAG, "Inicializando Bluedroid Stack...");
    ret=esp_bluedroid_init(); if(ret){ESP_LOGE(TAG,"init bluedroid failed: %s",esp_err_to_name(ret));return;}
    ret=esp_bluedroid_enable(); if(ret){ESP_LOGE(TAG,"enable bluedroid failed: %s",esp_err_to_name(ret));return;}
    esp_bt_sleep_disable();
    ESP_LOGI(TAG, "Registrando Callbacks e App BLE...");
    ret=esp_ble_gatts_register_callback(gatts_profile_event_handler); if(ret){ESP_LOGE(TAG,"gatts register error: %x",ret);return;}
    ret=esp_ble_gap_register_callback(gap_event_handler); if(ret){ESP_LOGE(TAG,"gap register error: %x",ret);return;}
    ret=esp_ble_gatts_app_register(ESP_APP_ID); if(ret){ESP_LOGE(TAG,"gatts app register error: %x",ret);return;}
    esp_err_t mtu_ret=esp_ble_gatt_set_local_mtu(500); if(mtu_ret){ESP_LOGE(TAG,"set local MTU failed: %x",mtu_ret);}
    ESP_LOGI(TAG, "Criando Tasks...");
    xTaskCreate(movimento_task,"movimento_task",2048,NULL,5,NULL);
    xTaskCreate(encoder_monitor_task,"encoder_monitor_task",4096,NULL,5,NULL);
    ESP_LOGI(TAG, "Inicialização concluída. Entrando no loop principal.");
    while(1){vTaskDelay(pdMS_TO_TICKS(10000));}
}