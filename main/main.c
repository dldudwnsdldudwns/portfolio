#include <stdio.h>
#include "esp_log.h"
#include "esp_check.h"
#include "driver/i2c_master.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_rom/esp_rom_sys.h"

static const char* TAG = "i2c_scan";

// ===== 설정값 =====
#define I2C_SDA_PIN        19
#define I2C_SCL_PIN        20
#define I2C_PORT           0
#define I2C_FREQ_HZ        50000   // 내부 풀업이면 50kHz 정도로 낮춰 안정성 확보
#define PROBE_TIMEOUT_MS   2000
#define PROBE_RETRY        3
#define EXPECT_ADDR_1      0x57    // MAX30102 주소

// ===== 버스 리커버리(라인 stuck 해제) =====
static void i2c_bus_recovery_gpio(void) {
    gpio_config_t io = {
        .pin_bit_mask = (1ULL << I2C_SCL_PIN) | (1ULL << I2C_SDA_PIN),
        .mode = GPIO_MODE_INPUT_OUTPUT_OD,
        .pull_up_en = 1,
        .pull_down_en = 0,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io);

    if (gpio_get_level(I2C_SDA_PIN) == 0) {
        for (int i = 0; i < 9; i++) {
            gpio_set_level(I2C_SCL_PIN, 1);
            esp_rom_delay_us(5);
            gpio_set_level(I2C_SCL_PIN, 0);
            esp_rom_delay_us(5);
        }
    }
    gpio_set_level(I2C_SDA_PIN, 0);
    esp_rom_delay_us(5);
    gpio_set_level(I2C_SCL_PIN, 1);
    esp_rom_delay_us(5);
    gpio_set_level(I2C_SDA_PIN, 1);
    esp_rom_delay_us(5);
}

static bool i2c_lines_idle_check(void) {
    gpio_config_t io = {
        .pin_bit_mask = (1ULL << I2C_SCL_PIN) | (1ULL << I2C_SDA_PIN),
        .mode = GPIO_MODE_INPUT_OUTPUT_OD,
        .pull_up_en = 1,
        .pull_down_en = 0,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&io);

    for (int i = 0; i < 1000; i++) {
        if (gpio_get_level(I2C_SDA_PIN) == 1 && gpio_get_level(I2C_SCL_PIN) == 1)
            return true;
        esp_rom_delay_us(1);
    }
    return false;
}

static esp_err_t new_bus(i2c_master_bus_handle_t* out) {
    i2c_master_bus_config_t bus_cfg = {
        .i2c_port = I2C_PORT,
        .sda_io_num = I2C_SDA_PIN,
        .scl_io_num = I2C_SCL_PIN,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags = {
            .enable_internal_pullup = true
        },
    };
    return i2c_new_master_bus(&bus_cfg, out);
}

static esp_err_t probe_with_retry(i2c_master_bus_handle_t bus, uint16_t addr, uint32_t timeout_ms) {
    for (int i = 0; i < PROBE_RETRY; i++) {
        esp_err_t err = i2c_master_probe(bus, addr, timeout_ms);
        if (err == ESP_OK) return ESP_OK;
        if (err == ESP_ERR_NOT_FOUND) return err;
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    return ESP_FAIL;
}

static void prioritize_expected(i2c_master_bus_handle_t bus) {
    uint16_t a = EXPECT_ADDR_1;
    esp_err_t err = probe_with_retry(bus, a, PROBE_TIMEOUT_MS);
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "Found expected device at 0x%02X", a);
    }
    else if (err == ESP_ERR_NOT_FOUND) {
        ESP_LOGI(TAG, "No device at expected 0x%02X (NACK)", a);
    }
    else if (err == ESP_ERR_TIMEOUT) {
        ESP_LOGW(TAG, "TIMEOUT at expected 0x%02X (라인 약함/배선 의심)", a);
    }
    else {
        ESP_LOGE(TAG, "Probe 0x%02X error: %s", a, esp_err_to_name(err));
    }
}

static void scan_all(i2c_master_bus_handle_t bus) {
    int found = 0;
    for (uint16_t addr = 0x03; addr <= 0x77; addr++) {
        esp_err_t err = i2c_master_probe(bus, addr, PROBE_TIMEOUT_MS);
        if (err == ESP_OK) {
            ESP_LOGI(TAG, "Found device at 0x%02X", addr);
            found++;
        }
        else if (err == ESP_ERR_TIMEOUT) {
            ESP_LOGW(TAG, "TIMEOUT at 0x%02X (배선/풀업 약함)", addr);
        }
    }
    if (found == 0) {
        ESP_LOGW(TAG, "No I2C devices found");
    }
}

void app_main(void) {
    ESP_LOGI(TAG, "Checking I2C line idle status...");
    if (!i2c_lines_idle_check()) {
        ESP_LOGW(TAG, "I2C lines not idle. Trying recovery...");
        i2c_bus_recovery_gpio();
        if (!i2c_lines_idle_check()) {
            ESP_LOGE(TAG, "Still not idle. 배선/풀업/합선 점검 필요.");
        }
        else {
            ESP_LOGI(TAG, "Bus recovery succeeded.");
        }
    }
    else {
        ESP_LOGI(TAG, "Lines idle.");
    }

    i2c_master_bus_handle_t bus = NULL;
    ESP_ERROR_CHECK(new_bus(&bus));

    ESP_LOGI(TAG, "Prioritized probe...");
    prioritize_expected(bus);

    ESP_LOGI(TAG, "Full I2C scan at %dkHz", I2C_FREQ_HZ / 1000);
    scan_all(bus);

    ESP_ERROR_CHECK(i2c_del_master_bus(bus));
}
