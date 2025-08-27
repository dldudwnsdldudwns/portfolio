// main.cpp — HR + Battery voltage JSON together

#include <cmath>
#include <cstring>
#include <functional>
#include <memory>
#include <string>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/i2c.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_err.h"

#include "solicare_device.hpp"

// ========= MAX30102 defines (existing) =========
#define TAG "MAX30102_HR"
#define I2C_NUM I2C_NUM_0
#define SDA_PIN 18
#define SCL_PIN 20
#define I2C_FREQ 400000
#define I2C_TIMEOUT_MS 1000
#define ADDR 0x57
#define REG_INTR_STATUS_1 0x00
#define REG_INTR_STATUS_2 0x01
#define REG_INTR_ENABLE_1 0x02
#define REG_INTR_ENABLE_2 0x03
#define REG_FIFO_WR_PTR 0x04
#define REG_OVF_COUNTER 0x05
#define REG_FIFO_RD_PTR 0x06
#define REG_FIFO_DATA 0x07
#define REG_FIFO_CONF 0x08
#define REG_MODE 0x09
#define REG_SPO2 0x0A
#define REG_LED1 0x0C
#define REG_LED2 0x0D
#undef FIFO_CONF_INIT
#define FIFO_CONF_INIT 0x77 // SMP_AVE=8, ROLLOVER=1, A_FULL=7
#define MODE_HEART 0x02
#define SPO2_STD 0x27
#define LED_STD 0x22
#define MAX30102_INT_GPIO GPIO_NUM_8
#define INT_PIN_SEL (1ULL << MAX30102_INT_GPIO)
#define BURST_SZ 8
#define BYTES_PER_SAMPLE 3
#define SR_HZ 100.0f
#define MIN_BPM 40.0f
#define MAX_BPM 200.0f
#define IR_ON_THRESHOLD 50000U
#define IR_OFF_THRESHOLD 35000U
#define STABLE_COUNT 12
#define MIN_RR_MS 340
#define HR_SMOOTH_N 5
#define MA_N 100

static const float a_bp[9] = { 1.0f, -1.5610180758007182f, 0.6413515380575631f };
static const float b_bp[9] = { 0.020083365564211235f, 0.0f, -0.020083365564211235f };

static QueueHandle_t s_int_queue = NULL;
static volatile uint32_t s_isr_cnt = 0;

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
static uint32_t sample_idx = 0;
static std::function<void(float, const char*, uint32_t)> g_emit_bpm;

// ========= Battery ADC (integrated from voltagesen.cpp) =========
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"

static constexpr int BAT_ADC_GPIO = 0;                 // same as TARGET_GPIO
static constexpr auto BAT_USE_ATTEN = ADC_ATTEN_DB_2_5;
static constexpr int BAT_ADC_SAMPLES = 32;
static constexpr float R1_OHMS = 30000.0f;
static constexpr float R2_OHMS = 7500.0f;
static constexpr float DIV_GAIN_INV = (R1_OHMS + R2_OHMS) / R2_OHMS;

static adc_oneshot_unit_handle_t s_bat_adc = nullptr;
static adc_cali_handle_t s_bat_cali = nullptr;
static adc_unit_t s_bat_unit;
static adc_channel_t s_bat_channel;

static bool battery_adc_init_once() {
    if (s_bat_adc) return true;
    if (adc_oneshot_io_to_channel(BAT_ADC_GPIO, &s_bat_unit, &s_bat_channel) != ESP_OK) {
        ESP_LOGE(TAG, "Battery adc_oneshot_io_to_channel failed");
        return false;
    }
    adc_oneshot_unit_init_cfg_t unit_cfg = { .unit_id = s_bat_unit, .ulp_mode = ADC_ULP_MODE_DISABLE };
    if (adc_oneshot_new_unit(&unit_cfg, &s_bat_adc) != ESP_OK) {
        ESP_LOGE(TAG, "Battery adc_oneshot_new_unit failed");
        return false;
    }
    adc_oneshot_chan_cfg_t chan_cfg = { .atten = BAT_USE_ATTEN, .bitwidth = ADC_BITWIDTH_DEFAULT };
    if (adc_oneshot_config_channel(s_bat_adc, s_bat_channel, &chan_cfg) != ESP_OK) {
        ESP_LOGE(TAG, "Battery adc_oneshot_config_channel failed");
        return false;
    }
    adc_cali_curve_fitting_config_t cali_cfg = {
        .unit_id = s_bat_unit, .chan = s_bat_channel, .atten = BAT_USE_ATTEN, .bitwidth = ADC_BITWIDTH_DEFAULT
    };
    if (adc_cali_create_scheme_curve_fitting(&cali_cfg, &s_bat_cali) != ESP_OK) {
        ESP_LOGE(TAG, "Battery adc_cali_create_scheme_curve_fitting failed");
        return false;
    }
    ESP_LOGI(TAG, "Battery calibration ready");
    return true;
}

static bool read_battery_voltage(float& v_batt_out) {
    if (!battery_adc_init_once()) return false;
    int64_t sum_mv = 0; int valid = 0;
    for (int i = 0; i < BAT_ADC_SAMPLES; ++i) {
        int mv = 0;
        esp_err_t r = adc_oneshot_get_calibrated_result(s_bat_adc, s_bat_cali, s_bat_channel, &mv);
        if (r == ESP_OK) { sum_mv += mv; valid++; }
        else { ESP_LOGW(TAG, "Battery read failed: %d", r); }
    }
    if (valid == 0) return false;
    float v_out = (float)sum_mv / valid / 1000.0f;
    v_batt_out = v_out * DIV_GAIN_INV;
    return true;
}

// ========= I2C utils (existing) =========
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

static int fifo_count_debug(void) {
    uint8_t wr = 0, rd = 0, ovf = 0;
    if (read_regs(REG_FIFO_WR_PTR, &wr, 1) != ESP_OK) { ESP_LOGE(TAG, "read WR_PTR fail"); return -1; }
    if (read_regs(REG_OVF_COUNTER, &ovf, 1) != ESP_OK) { ESP_LOGW(TAG, "read OVF fail"); }
    if (read_regs(REG_FIFO_RD_PTR, &rd, 1) != ESP_OK) { ESP_LOGE(TAG, "read RD_PTR fail"); return -1; }
    int cnt = (int)((wr - rd) & 0x1F);
    ESP_LOGD(TAG, "FIFO ptr wr=%u rd=%u ovf=%u cnt=%d", wr, rd, ovf, cnt);
    return cnt;
}

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

// ========= Signal processing (existing) =========
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

// 샘플 처리
static void process_sample(uint32_t ir_raw, float t_ms) {
    if (!worn) {
        if (ir_raw > IR_ON_THRESHOLD) {
            if (++on_cnt > STABLE_COUNT) {
                worn = true; on_cnt = 0; off_cnt = 0;
                reset_filter_states();
                ESP_LOGI(TAG, "Finger ON");
                // 필요시 ON 상태 알림만 별도 전송 가능
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
                if (g_emit_bpm) g_emit_bpm(0.0f, "OFF", (uint32_t)t_ms);
                return;
            }
        }
        else {
            off_cnt = 0;
        }
    }

    float x = (float)ir_raw;
    x = dc_remove(x);
    float y = biquad_bp(x);
    float abs_y = fabsf(y);

    noise_ma = 0.97f * noise_ma + 0.03f * abs_y;
    float thr = fmaxf(0.05f, 1.0f * noise_ma);

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

// ========= ISR / INT task (existing) =========
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

static void int_task(void* arg) {
    uint32_t io_num;
    uint32_t ir_buf[BURST_SZ];
    const float Ts_ms = 1000.0f / SR_HZ;
    while (1) {
        if (!xQueueReceive(s_int_queue, &io_num, portMAX_DELAY)) continue;
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

// ========= SolicareDevice changes: send JSON with voltage =========
void SolicareDevice::send_bpm_json(float bpm, float voltage, const char* status, uint32_t t_ms) {
    if (!socket_client_) { ESP_LOGW(TAG, "WebSocket client not ready"); return; } // OK
    if (!socket_client_->is_available()) { ESP_LOGW(TAG, "WebSocket not connected, skip send"); return; } // OK

    char json[192];

    int n = snprintf(json, sizeof(json),
        "{\"device\":\"%s\",\"bpm\":%.1f,\"voltage\":%.3f,\"timestamp_ms\":%u,\"status\":\"%s\"}",
        config_.device_name.c_str(),
        bpm,
        voltage,
        (unsigned)t_ms,
        status);
    if (n < 0 || n >= (int)sizeof(json)) {
        ESP_LOGW(TAG, "JSON truncated or format error");
        return;
    }
    std::string json_str(json, n); // 안전하게 길이 지정 생성
    if (socket_client_->send_text_now(json_str)) {
        ESP_LOGI(TAG, "Sent JSON: %s", json_str.c_str());
    }
    else {
        ESP_LOGW(TAG, "Failed to send JSON");
    }
}



extern "C" void app_main(void) {
    ESP_LOGI(TAG, "🚀 Starting Solicare Application");
    ESP_LOGI(TAG, "═══════════════════════════════════════");

    SolicareDevice::Config config;
    ESP_LOGI(TAG, "📋 WiFi SSID: %s", config.wifi_ssid.c_str());
    ESP_LOGI(TAG, " 🌐 Server: %s:%d", config.socket_server_ip.c_str(), config.socket_server_port);

    SolicareDevice device(config);
    if (!device.initialize()) {
        ESP_LOGE(TAG, "❌ Failed to initialize application");
        return;
    }
    device.run();

    if (i2c_init() != ESP_OK) {
        ESP_LOGE(TAG, "I2C init failed");
        return;
    }
    vTaskDelay(pdMS_TO_TICKS(10));
    clear_interrupt_status();
    fifo_reset();
    vTaskDelay(pdMS_TO_TICKS(5));

    write_reg(REG_FIFO_CONF, FIFO_CONF_INIT);
    write_reg(REG_SPO2, SPO2_STD);
    write_reg(REG_LED1, LED_STD);
    write_reg(REG_LED2, LED_STD);
    write_reg(REG_MODE, MODE_HEART);

    write_reg(REG_INTR_ENABLE_1, 0x80);
    write_reg(REG_INTR_ENABLE_2, 0x00);
    vTaskDelay(pdMS_TO_TICKS(50));

    gpio_config_t io_conf;
    memset(&io_conf, 0, sizeof(io_conf));
    io_conf.pin_bit_mask = INT_PIN_SEL;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    io_conf.intr_type = GPIO_INTR_NEGEDGE;
    ESP_ERROR_CHECK(gpio_config(&io_conf));
    int init_lvl = gpio_get_level(MAX30102_INT_GPIO);
    ESP_LOGI(TAG, "INT GPIO%d level=%d (idle should be 1)", (int)MAX30102_INT_GPIO, init_lvl);

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

// ========= SolicareDevice::initialize / run definitions =========
bool SolicareDevice::initialize() {
    ESP_LOGI(TAG, "=== Initializing Solicare Device ===");
    ESP_LOGI(TAG, "📶 Initializing WiFi Client...");
    WiFiClient::Config wifi_config(config_.wifi_ssid, config_.wifi_password);
    wifi_client_ = std::make_unique<WiFiClient>(wifi_config);
    if (!wifi_client_->initialize()) {
        ESP_LOGE(TAG, "❌ Failed to initialize WiFi Client");
        return false;
    }
    ESP_LOGI(TAG, "✅ WiFi Client initialized");

    ESP_LOGI(TAG, "🌐 Initializing WebSocket client...");
    WebSocketClient::Config ws_config(config_.device_name, config_.socket_server_ip, config_.socket_server_port);
    socket_client_ = std::make_unique<WebSocketClient>(ws_config, [this]() { return wifi_client_->is_available(); });
    if (!socket_client_->initialize()) {
        ESP_LOGE(TAG, "❌ Failed to initialize WebSocket client");
        return false;
    }
    ESP_LOGI(TAG, "✅ WebSocket client initialized and ready to connect as '%s'", config_.device_name.c_str());
    ESP_LOGI(TAG, "=== Solicare Device initialized successfully ===");
    return true;
}

void SolicareDevice::run() {
    ESP_LOGI(TAG, "=== Starting Solicare Device Runtime ==="); // [2]
    wifi_client_->try_connect();                                // [3]
    socket_client_->async_connect();                            // [1]
    socket_client_->start_sender_task();                        // [1]

    g_emit_bpm = [this](float bpm, const char* status, uint32_t t_ms) {
        float vbatt = 0.0f;
        bool ok = read_battery_voltage(vbatt);                  // [4]
        if (!ok) {
            vbatt = 0.0f;
            ESP_LOGW(TAG, "Battery read failed, sending voltage=0.0V"); // [2]
        }
        this->send_bpm_json(bpm, vbatt, status, t_ms); // 변경된 시그니처 호출 [2]
        };
}

