#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <cstring>
#include <math.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/i2c.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_err.h"
#include "solicare_device.hpp"

#define TAG               "MAX30102_HR"
// I2C
#define I2C_NUM           I2C_NUM_0
#define SDA_PIN           18
#define SCL_PIN           20
#define I2C_FREQ          400000
#define I2C_TIMEOUT_MS    1000
// MAX30102
#define ADDR              0x57
#define REG_INTR_STATUS_1 0x00
#define REG_INTR_STATUS_2 0x01
#define REG_INTR_ENABLE_1 0x02
#define REG_INTR_ENABLE_2 0x03
#define REG_FIFO_WR_PTR   0x04
#define REG_OVF_COUNTER   0x05
#define REG_FIFO_RD_PTR   0x06
#define REG_FIFO_DATA     0x07
#define REG_FIFO_CONF     0x08
#define REG_MODE          0x09
#define REG_SPO2          0x0A
#define REG_LED1          0x0C
#define REG_LED2          0x0D
// 설정
// FIFO_CONF: [7:5]=SMP_AVE(8x=011), [4]=ROLLOVER(1), [3:0]=A_FULL(7 → 여유 8샘플 시 인터럽트)
#define FIFO_CONF_INIT    0xDF  // 1101 1111? -> (011<<5)=0x60, ROLLOVER=0x10, A_FULL=0x0F 아님 주의
// 위 한줄 혼란 방지: 직접 값 지정
// SMP_AVE=8x(0b011<<5=0x60) | ROLLOVER=0x10 | A_FULL=7(0x07) = 0x60+0x10+0x07 = 0x77
#undef  FIFO_CONF_INIT
#define FIFO_CONF_INIT    0x77  // 최종: 0b0111 0111 = 0x77
// 모드/샘플
#define MODE_HEART        0x02         // HR-only(IR만)
#define SPO2_STD          0x27         // SR=100Hz(001), LED_PW=411us(11), (상위 SMP_AVE는 FIFO에서 설정)
#define LED_STD           0x22         // ~7.6mA (필요시 조정)
// GPIO/INT
#define MAX30102_INT_GPIO   GPIO_NUM_8
#define INT_PIN_SEL         (1ULL << MAX30102_INT_GPIO)
// 버스트/포맷
#define BURST_SZ              8
#define BYTES_PER_SAMPLE      3   // HR-only → IR 18bit = 3 bytes
// HR 파라미터
#define SR_HZ             100.0f      // SPO2_STD의 SR과 일치
#define MIN_BPM           40.0f
#define MAX_BPM           200.0f
#define IR_ON_THRESHOLD   50000U
#define IR_OFF_THRESHOLD  35000U
#define STABLE_COUNT      12
#define MIN_RR_MS         340
#define HR_SMOOTH_N       5
#define MA_N              100
// 밴드패스(0.7–3.5Hz @100Hz), a0=1
static const float a_bp[3] = { 1.0f, -1.5610180758007182f, 0.6413515380575631f };
static const float b_bp[3] = { 0.020083365564211235f, 0.0f, -0.020083365564211235f };
// 인터럽트 큐
static QueueHandle_t s_int_queue = NULL;
static volatile uint32_t s_isr_cnt = 0;
// 필터/상태
static float z1 = 0.0f, z2 = 0.0f;
static float ma_buf[MA_N] = { 0 };
static int ma_i = 0, ma_cnt = 0;
static float ma_sum = 0.0f;
static float hrBuf[HR_SMOOTH_N] = { 0 };
static int hrCount = 0, hrIdx = 0;
static bool worn = false;
static int on_cnt = 0, off_cnt = 0;
static float prev_y = 0.0f, last_y = 0.0f;
static float last_peak_ms = -1.0f;
static float noise_ma = 0.0f;
static uint32_t sample_idx = 0;  // 샘플 시간축(버스트 영향 제거)


static std::function<void(float, const char*, uint32_t)> g_emit_bpm;

// I2C 유틸

bool SolicareDevice::initialize()
{
    ESP_LOGI(TAG, "=== Initializing Solicare Device ===");

    // Initialize WiFi
    ESP_LOGI(TAG, "📶 Initializing WiFi Client...");
    WiFiClient::Config wifi_config(config_.wifi_ssid, config_.wifi_password);
    wifi_client_ = std::make_unique<WiFiClient>(wifi_config);
    if (!wifi_client_->initialize())
    {
        ESP_LOGE(TAG, "❌ Failed to initialize WiFi Client");
        return false;
    }
    ESP_LOGI(TAG, "✅ WiFi Client initialized");

    // Initialize WebSocket client
    ESP_LOGI(TAG, "🌐 Initializing WebSocket client...");
    WebSocketClient::Config ws_config(config_.device_name, config_.socket_server_ip, config_.socket_server_port);
    socket_client_ = std::make_unique<WebSocketClient>(ws_config, [this]() { return wifi_client_->is_available(); });
    if (!socket_client_->initialize())
    {
        ESP_LOGE(TAG, "❌ Failed to initialize WebSocket client");
        return false;
    }
    ESP_LOGI(TAG, "✅ WebSocket client initialized and ready to connect as '%s'", config_.device_name.c_str());

    ESP_LOGI(TAG, "=== Solicare Device initialized successfully ===");
    return true;
}

void SolicareDevice::run()
{
    ESP_LOGI(TAG, "=== Starting Solicare Device Runtime ===");

    ESP_LOGI(TAG, "📡 Preparing network connection...");
    wifi_client_->try_connect();

    ESP_LOGI(TAG, "🌐 Attempting to connect to WebSocket...");
    socket_client_->async_connect();
    socket_client_->start_sender_task();
    g_emit_bpm = [this](float bpm, const char* status, uint32_t t_ms) {
        this->send_bpm_json(bpm, status, t_ms);
        };


}void SolicareDevice::send_bpm_json(float bpm, const char* status, uint32_t t_ms) {
    if (!socket_client_) {
        ESP_LOGW(TAG, "WebSocket client not ready");
        return;
    }
    if (!socket_client_->is_available()) {
        ESP_LOGW(TAG, "WebSocket not connected, skip send");
        return;
    }

        // 반드시 배열로 선언
    char json[192];

    // JSON 문자열에서 쌍따옴표는 \" 로 이스케이프
    // 공백이나 % s 같은 오탈자 없이 정확히 작성
    int n = snprintf(json, sizeof(json),
        "{\"device\":\"%s\",\"bpm\":%.1f,\"timestamp_ms\":%u,\"status\":\"%s\"}",
        config_.device_name.c_str(),
        bpm,
        (unsigned)t_ms,
        status);
    if (n < 0 || n >= (int)sizeof(json)) {
        ESP_LOGW(TAG, "JSON truncated or format error");
        return;
    }

    // std::string(json)은 char*를 받으므로 위처럼 배열로 선언한 것이 필수
    if (socket_client_->send_text_now(std::string(json))) {
        ESP_LOGI(TAG, "Sent JSON: %s", json);
    }
    else {
        ESP_LOGW(TAG, "Failed to send JSON");
    }
}

esp_err_t i2c_init() {
    i2c_config_t cfg;
    memset(&cfg, 0, sizeof(cfg));
    cfg.mode = I2C_MODE_MASTER;
    cfg.sda_io_num = SDA_PIN;
    cfg.scl_io_num = SCL_PIN;
    cfg.sda_pullup_en = GPIO_PULLUP_ENABLE;
    cfg.scl_pullup_en = GPIO_PULLUP_ENABLE;
    cfg.master.clk_speed = I2C_FREQ;

    esp_err_t err = i2c_param_config(I2C_NUM, &cfg);
    if (err != ESP_OK) return err;
    err = i2c_driver_install(I2C_NUM, cfg.mode, 0, 0, 0);
    if (err != ESP_OK) return err;
    return ESP_OK;
}

static esp_err_t write_reg(uint8_t reg, uint8_t val) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_write_byte(cmd, val, true);
    i2c_master_stop(cmd);
    esp_err_t r = i2c_master_cmd_begin(I2C_NUM, cmd, pdMS_TO_TICKS(I2C_TIMEOUT_MS));
    i2c_cmd_link_delete(cmd);
    if (r != ESP_OK) ESP_LOGE(TAG, "WRITE reg 0x%02X val 0x%02X fail=%s", reg, val, esp_err_to_name(r));
    return r;
}
static esp_err_t read_regs(uint8_t reg, uint8_t* buf, size_t len) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (ADDR << 1) | I2C_MASTER_READ, true);
    for (size_t i = 0; i < len; i++) {
        i2c_master_read_byte(cmd, &buf[i], (i < len - 1) ? I2C_MASTER_ACK : I2C_MASTER_NACK);
    }
    i2c_master_stop(cmd);
    esp_err_t r = i2c_master_cmd_begin(I2C_NUM, cmd, pdMS_TO_TICKS(I2C_TIMEOUT_MS));
    i2c_cmd_link_delete(cmd);
    if (r != ESP_OK) ESP_LOGE(TAG, "READ reg 0x%02X len %u fail=%s", reg, (unsigned)len, esp_err_to_name(r));
    return r;
}
// 상태 클리어 & FIFO 리셋
static void clear_interrupt_status(void) {
    uint8_t s1 = 0, s2 = 0;
    if (read_regs(REG_INTR_STATUS_1, &s1, 1) == ESP_OK &&
        read_regs(REG_INTR_STATUS_2, &s2, 1) == ESP_OK) {
        ESP_LOGI(TAG, "Cleared INT status: s1=0x%02X s2=0x%02X", s1, s2);
    }
    else {
        ESP_LOGE(TAG, "Failed to read INT status for clear");
    }
}
static void fifo_reset(void) {
    write_reg(REG_FIFO_WR_PTR, 0x00);
    write_reg(REG_OVF_COUNTER, 0x00);
    write_reg(REG_FIFO_RD_PTR, 0x00);
}
// FIFO 포인터/카운트
static int fifo_count_debug(void) {
    uint8_t wr = 0, rd = 0, ovf = 0;
    if (read_regs(REG_FIFO_WR_PTR, &wr, 1) != ESP_OK) { ESP_LOGE(TAG, "read WR_PTR fail"); return -1; }
    if (read_regs(REG_OVF_COUNTER, &ovf, 1) != ESP_OK) { ESP_LOGW(TAG, "read OVF fail"); }
    if (read_regs(REG_FIFO_RD_PTR, &rd, 1) != ESP_OK) { ESP_LOGE(TAG, "read RD_PTR fail"); return -1; }
    int cnt = (int)((wr - rd) & 0x1F);
    ESP_LOGD(TAG, "FIFO ptr wr=%u rd=%u ovf=%u cnt=%d", wr, rd, ovf, cnt);
    return cnt;
}
// 버스트 IR 읽기
static esp_err_t fifo_read_n_ir(int n, uint32_t* out_ir) {
    if (n <= 0 || n > BURST_SZ) return ESP_ERR_INVALID_ARG;
    uint8_t buf[BURST_SZ * BYTES_PER_SAMPLE] = { 0 };
    int bytes = n * BYTES_PER_SAMPLE;
    esp_err_t r = read_regs(REG_FIFO_DATA, buf, bytes);
    if (r != ESP_OK) return r;
    for (int i = 0; i < n; i++) {
        const uint8_t* p = &buf[i * 3];
        out_ir[i] = (((uint32_t)p[0] & 0x03) << 16) | ((uint32_t)p[1] << 8) | p[2];
    }
    return ESP_OK;
}
// 신호 처리
static float dc_remove(float x) {
    ma_sum += x - ma_buf[ma_i];
    ma_buf[ma_i] = x;
    ma_i = (ma_i + 1) % MA_N;
    if (ma_cnt < MA_N) ma_cnt++;
    float dc = ma_sum / (float)ma_cnt;
    return x - dc;
}
static float biquad_bp(float x) {
    float y = b_bp[0] * x + z1;
    z1 = b_bp[1] * x - a_bp[1] * y + z2;
    z2 = b_bp[2] * x - a_bp[2] * y;
    return y;
}
static float smooth_hr(float hr) {
    hrBuf[hrIdx] = hr;
    hrIdx = (hrIdx + 1) % HR_SMOOTH_N;
    if (hrCount < HR_SMOOTH_N) hrCount++;
    float s = 0.0f;
    for (int i = 0; i < hrCount; i++) s += hrBuf[i];
    return s / (float)hrCount;
}
static void reset_filter_states(void) {
    z1 = z2 = 0.0f;
    prev_y = last_y = 0.0f;
    noise_ma = 0.0f;
    for (int i = 0; i < MA_N; i++) ma_buf[i] = 0.0f;
    ma_i = 0; ma_cnt = 0; ma_sum = 0.0f;
    hrCount = 0; hrIdx = 0;
    for (int i = 0; i < HR_SMOOTH_N; i++) hrBuf[i] = 0.0f;
    last_peak_ms = -1.0f;
}
// 샘플 처리: 샘플 인덱스 기반 시간(ms) 전달
static void process_sample(uint32_t ir_raw, float t_ms) {
    // 착용 판정 로직
    if (!worn) {
        if (ir_raw > IR_ON_THRESHOLD) {
            if (++on_cnt > STABLE_COUNT) {
                worn = true; on_cnt = 0; off_cnt = 0;
                reset_filter_states();
                ESP_LOGI(TAG, "Finger ON");
                // 필요하면 ON 상태 알림만 보낼 수도 있음:
                // if (g_emit_bpm) g_emit_bpm(0.0f, "ON", (uint32_t)t_ms);
            }
        }
        else {
            on_cnt = 0;
        }
        return;
    }
    else {
        if (ir_raw < IR_OFF_THRESHOLD) {
            if (++off_cnt > STABLE_COUNT) {
                worn = false; off_cnt = 0; on_cnt = 0;
                ESP_LOGW(TAG, "Finger OFF");
                // OFF면 0 BPM 전송
                if (g_emit_bpm) g_emit_bpm(0.0f, "OFF", (uint32_t)t_ms);
                return;
            }
        }
        else {
            off_cnt = 0;
        }
    }

    // 신호 처리
    float x = (float)ir_raw;
    x = dc_remove(x);
    float y = biquad_bp(x);
    float abs_y = fabsf(y);

    // 적응 임계값
    noise_ma = 0.97f * noise_ma + 0.03f * abs_y;
    float thr = fmaxf(0.05f, 1.0f * noise_ma);

    // 피크 검출
    if (prev_y < last_y && last_y >= y && last_y > thr) {
        if (last_peak_ms < 0.0f) {
            last_peak_ms = t_ms;
        }
        else {
            float rr = t_ms - last_peak_ms;
            if (rr > MIN_RR_MS) {
                last_peak_ms = t_ms;
                float hr = 60000.0f / rr;
                float hr_s = smooth_hr(hr);
                if (hr_s >= MIN_BPM && hr_s <= MAX_BPM) {
                    ESP_LOGI(TAG, "Heart Rate: %.1f BPM", hr_s);
                    // 유효 HR 전송 (착용 중이므로 "ON")
                    if (g_emit_bpm) g_emit_bpm(hr_s, "ON", (uint32_t)t_ms);
                }
                else {
                    ESP_LOGW(TAG, "HR out of range: %.1f BPM (raw=%.1f)", hr_s, hr);
                }
            }
        }
    }

    prev_y = last_y;
    last_y = y;
}
// ISR
static portMUX_TYPE s_spinlock = portMUX_INITIALIZER_UNLOCKED;
static void IRAM_ATTR int_isr(void* arg) {
    portENTER_CRITICAL_ISR(&s_spinlock);
    s_isr_cnt++;
    portEXIT_CRITICAL_ISR(&s_spinlock);
    uint32_t gpio_num = (uint32_t)arg;
    BaseType_t hpw = pdFALSE;
    xQueueSendFromISR(s_int_queue, &gpio_num, &hpw);
    if (hpw) portYIELD_FROM_ISR();
}
// 인터럽트 태스크
static void int_task(void* arg) {
    uint32_t io_num;
    uint32_t ir_buf[BURST_SZ];
    const float Ts_ms = 1000.0f / SR_HZ;
    while (1) {
        if (!xQueueReceive(s_int_queue, &io_num, portMAX_DELAY)) continue;
        // 상태 읽어 클리어
        uint8_t int1 = 0, int2 = 0;
        if (read_regs(REG_INTR_STATUS_1, &int1, 1) != ESP_OK ||
            read_regs(REG_INTR_STATUS_2, &int2, 1) != ESP_OK) {
            ESP_LOGE(TAG, "INT_STATUS read fail");
            continue;
        }
        int cnt = fifo_count_debug();
        if (cnt <= 0) continue;
        int left = cnt;
        while (left > 0) {
            int n = (left >= BURST_SZ) ? BURST_SZ : left;
            if (fifo_read_n_ir(n, ir_buf) != ESP_OK) {
                ESP_LOGE(TAG, "fifo_read_n_ir fail");
                break;
            }
            for (int i = 0; i < n; i++) {
                float t_ms = (sample_idx * Ts_ms);
                process_sample(ir_buf[i], t_ms);
                sample_idx++;
            }
            left -= n;
        }
    }
}
extern "C" void app_main(void) {

    ESP_LOGI(TAG, "🚀 Starting Solicare Camera Application");
    ESP_LOGI(TAG, "═══════════════════════════════════════");

    SolicareDevice::Config config;
    ESP_LOGI(TAG, "📋 Configuration:");
    ESP_LOGI(TAG, "   📶 WiFi SSID: %s", config.wifi_ssid.c_str());
    ESP_LOGI(TAG, "   🌐 Server: %s:%d", config.socket_server_ip.c_str(), config.socket_server_port);
    ESP_LOGI(TAG, "═══════════════════════════════════════");

    SolicareDevice device(config);
    if (!device.initialize())
    {
        ESP_LOGE(TAG, "❌ Failed to initialize application");
        return;
    }
    device.run();
    ESP_LOGI(TAG, "💤 Main loop started - system running...");

    if (i2c_init() != ESP_OK) {
        ESP_LOGE(TAG, "I2C init failed");
        return;
    }
    vTaskDelay(pdMS_TO_TICKS(10));
    clear_interrupt_status();
    fifo_reset();
    vTaskDelay(pdMS_TO_TICKS(5));
    // 설정
    write_reg(REG_FIFO_CONF, FIFO_CONF_INIT); // SMP_AVE=8, ROLLOVER=1, A_FULL=7
    write_reg(REG_SPO2, SPO2_STD);       // SR=100Hz, PW=411us
    write_reg(REG_LED1, LED_STD);
    write_reg(REG_LED2, LED_STD);
    write_reg(REG_MODE, MODE_HEART);     // HR-only(IR)
    // 인터럽트: A_FULL만
    write_reg(REG_INTR_ENABLE_1, 0x80);
    write_reg(REG_INTR_ENABLE_2, 0x00);
    vTaskDelay(pdMS_TO_TICKS(50));
    // GPIO8(INT)
    gpio_config_t io_conf;
    memset(&io_conf, 0, sizeof(io_conf));
    io_conf.pin_bit_mask = INT_PIN_SEL;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE; // 외부 풀업이면 DISABLE
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.intr_type = GPIO_INTR_NEGEDGE;
    ESP_ERROR_CHECK(gpio_config(&io_conf));
    ESP_ERROR_CHECK(gpio_config(&io_conf));
    int init_lvl = gpio_get_level(MAX30102_INT_GPIO);
    ESP_LOGI(TAG, "INT GPIO%d level=%d (idle should be 1)", (int)MAX30102_INT_GPIO, init_lvl);
    // 큐/태스크/ISR
    s_int_queue = xQueueCreate(16, sizeof(uint32_t));
    if (!s_int_queue) { ESP_LOGE(TAG, "xQueueCreate failed"); return; }
    if (xTaskCreate(int_task, "max30102_int_task", 4096, NULL, 10, NULL) != pdPASS) {
        ESP_LOGE(TAG, "xTaskCreate failed"); return;
    }
    ESP_ERROR_CHECK(gpio_install_isr_service(0));
    ESP_ERROR_CHECK(gpio_isr_handler_add(MAX30102_INT_GPIO, int_isr, (void*)MAX30102_INT_GPIO));
    ESP_LOGI(TAG, "Ready (BURST=%d, SR=%.0f sps, SMP_AVE=8, A_FULL=7)", BURST_SZ, SR_HZ);
    while (1) vTaskDelay(pdMS_TO_TICKS(1000));
}


