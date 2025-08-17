#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/i2c.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_check.h"

#define TAG               "MAX30102_DBG


// I2C
#define I2C_NUM           I2C_NUM_0
#define SDA_PIN           21
#define SCL_PIN           23
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

// 설정(테스트용 안전값)
#define MODE_HEART        0x02          // HR only
// SPO2: [7:5]=SMP_AVE(4=8x), [4:2]=SR(100Hz=001), [1:0]=LED_PW(18bit=11(0x3))
// 여기선 100Hz, 411us(18bit)
#define SPO2_STD          0x27
// FIFO_CONF: [7:5]=SMP_AVE(4x=010), [4]=ROLLOVER(1), [3:0]=A_FULL
// BURST를 8 기준으로 빠르게 테스트하려고 A_FULL을 낮춤(예: 8 비워지면 인터럽트)
// 데이터시트에 따라 A_FULL은 (임계=값) 정의가 보드별 라이브러리와 다를 수 있어, 작은 값으로 자주 인터럽트 유도
#define FIFO_CONF_INIT    0b10110000    // SMP_AVE=4x(010), ROLLOVER=1, A_FULL=0
#define LED_STD           0x1F          // ~7.6mA

// GPIO/INT
#define MAX30102_INT_GPIO   GPIO_NUM_8
#define INT_PIN_SEL         (1ULL << MAX30102_INT_GPIO)

// 버스트
#define BURST_SZ              8
#define BYTES_PER_IR          3
#define BURST_BYTES           (BURST_SZ * BYTES_PER_IR)

// 큐
static QueueHandle_t s_int_queue = NULL;

// 디버그 카운터
static volatile uint32_t s_isr_cnt = 0;

// ---------------- I2C 유틸 ----------------
static esp_err_t i2c_init(void) {
    i2c_config_t cfg = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = SDA_PIN,
        .scl_io_num = SCL_PIN,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_FREQ,
    };
    ESP_RETURN_ON_ERROR(i2c_param_config(I2C_NUM, &cfg), TAG, "i2c_param_config failed");
    ESP_RETURN_ON_ERROR(i2c_driver_install(I2C_NUM, cfg.mode, 0, 0, 0), TAG, "i2c_driver_install failed");
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

// FIFO 포인터 초기화
static void fifo_reset(void) {
    ESP_ERROR_CHECK_WITHOUT_ABORT(write_reg(REG_FIFO_WR_PTR, 0x00));
    ESP_ERROR_CHECK_WITHOUT_ABORT(write_reg(REG_OVF_COUNTER, 0x00));
    ESP_ERROR_CHECK_WITHOUT_ABORT(write_reg(REG_FIFO_RD_PTR, 0x00));
}

// 포인터/카운트 로그 포함
static int fifo_count_debug(void) {
    uint8_t wr = 0, rd = 0, ovf = 0;
    if (read_regs(REG_FIFO_WR_PTR, &wr, 1) != ESP_OK) { ESP_LOGE(TAG, "read WR_PTR fail"); return -1; }
    if (read_regs(REG_OVF_COUNTER, &ovf, 1) != ESP_OK) { ESP_LOGW(TAG, "read OVF fail"); }
    if (read_regs(REG_FIFO_RD_PTR, &rd, 1) != ESP_OK) { ESP_LOGE(TAG, "read RD_PTR fail"); return -1; }
    int cnt = (int)((wr - rd) & 0x1F);
    ESP_LOGI(TAG, "FIFO ptr wr=%u rd=%u ovf=%u cnt=%d", wr, rd, ovf, cnt);
    return cnt;
}

// IR n샘플 연속 읽기
static esp_err_t fifo_read_n_ir(int n, uint32_t* out_ir) {
    if (n <= 0 || n > BURST_SZ) return ESP_ERR_INVALID_ARG;
    uint8_t buf[BURST_BYTES] = { 0 };
    int bytes = n * BYTES_PER_IR;
    esp_err_t r = read_regs(REG_FIFO_DATA, buf, bytes);
    if (r != ESP_OK) return r;
    for (int i = 0; i < n; i++) {
        const uint8_t* p = &buf[i * 3];
        uint32_t ir = (((uint32_t)p[0] & 0x03) << 16) | ((uint32_t)p[1] << 8) | p[2];
        out_ir[i] = ir;
    }
    return ESP_OK;
}

// ---------------- GPIO ISR ----------------
static void IRAM_ATTR int_isr(void* arg) {
    s_isr_cnt++;
    uint32_t gpio_num = (uint32_t)arg;
    BaseType_t hpw = pdFALSE;
    xQueueSendFromISR(s_int_queue, &gpio_num, &hpw);
    if (hpw) portYIELD_FROM_ISR();
}

// ---------------- 인터럽트 처리 태스크 ----------------
static void int_task(void* arg) {
    uint32_t io_num;
    uint32_t ir_buf[BURST_SZ];

    while (1) {
        if (!xQueueReceive(s_int_queue, &io_num, portMAX_DELAY)) continue;

        int level_now = gpio_get_level(io_num);
        ESP_LOGI(TAG, "INT received (isr_cnt=%" PRIu32 "), GPIO%d level=%d", s_isr_cnt, (int)io_num, level_now);

        // 1) 인터럽트 상태 읽기(클리어)
        uint8_t int1 = 0, int2 = 0;
        if (read_regs(REG_INTR_STATUS_1, &int1, 1) == ESP_OK &&
            read_regs(REG_INTR_STATUS_2, &int2, 1) == ESP_OK) {
            ESP_LOGI(TAG, "INT_STATUS: int1=0x%02X int2=0x%02X", int1, int2);
        }
        else {
            ESP_LOGE(TAG, "read INT_STATUS failed");
            continue;
        }

        // 2) FIFO 포인터/카운트
        int cnt = fifo_count_debug();
        if (cnt <= 0) {
            ESP_LOGW(TAG, "No samples to read (cnt=%d)", cnt);
            continue;
        }

        // 3) BURST 읽기 반복
        int left = cnt;
        while (left > 0) {
            int n = (left >= BURST_SZ) ? BURST_SZ : left;
            esp_err_t r = fifo_read_n_ir(n, ir_buf);
            if (r != ESP_OK) {
                ESP_LOGE(TAG, "fifo_read_n_ir(n=%d) fail=%s", n, esp_err_to_name(r));
                break;
            }
            for (int i = 0; i < n; i++) {
                ESP_LOGD(TAG, "IR[%d/%d]=%" PRIu32, i + 1, n, ir_buf[i]);
            }
            left -= n;
        }

        // 4) 읽은 뒤 라인 레벨 확인(Idle=1로 복귀하는지)
        int lvl_after = gpio_get_level(io_num);
        ESP_LOGI(TAG, "After read: GPIO%d level=%d", (int)io_num, lvl_after);
    }
}

// ---------------- 초기화 및 main ----------------
void app_main(void) {
    ESP_ERROR_CHECK(i2c_init());

    ESP_LOGI(TAG, "Configuring MAX30102...");

    // FIFO 초기화
    fifo_reset();
    vTaskDelay(pdMS_TO_TICKS(10));

    // 기본 설정
    ESP_ERROR_CHECK_WITHOUT_ABORT(write_reg(REG_FIFO_CONF, FIFO_CONF_INIT));
    ESP_ERROR_CHECK_WITHOUT_ABORT(write_reg(REG_SPO2, SPO2_STD));
    ESP_ERROR_CHECK_WITHOUT_ABORT(write_reg(REG_LED1, LED_STD));
    ESP_ERROR_CHECK_WITHOUT_ABORT(write_reg(REG_LED2, LED_STD));
    ESP_ERROR_CHECK_WITHOUT_ABORT(write_reg(REG_MODE, MODE_HEART));

    // 인터럽트 enable: PPG_RDY + A_FULL 둘 다 켜서 어떤 신호든 들어오게
    // int_enable_1: bit7 A_FULL_EN, bit6 PPG_RDY_EN
    ESP_ERROR_CHECK_WITHOUT_ABORT(write_reg(REG_INTR_ENABLE_1, 0xC0));
    ESP_ERROR_CHECK_WITHOUT_ABORT(write_reg(REG_INTR_ENABLE_2, 0x00));

    vTaskDelay(pdMS_TO_TICKS(50));

    // GPIO8 설정 (내부 풀업, 하강엣지)
    gpio_config_t io_conf = {
        .pin_bit_mask = INT_PIN_SEL,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = 1,    // 외부 풀업 있으면 0
        .pull_down_en = 0,
        .intr_type = GPIO_INTR_NEGEDGE
    };
    ESP_ERROR_CHECK(gpio_config(&io_conf));

    int init_lvl = gpio_get_level(MAX30102_INT_GPIO);
    ESP_LOGI(TAG, "INT GPIO%d initial level=%d (expect 1 idle with pull-up)", (int)MAX30102_INT_GPIO, init_lvl);

    // 큐/태스크
    s_int_queue = xQueueCreate(16, sizeof(uint32_t));
    if (!s_int_queue) { ESP_LOGE(TAG, "xQueueCreate failed"); return; }
    if (xTaskCreate(int_task, "max30102_int_task", 4096, NULL, 10, NULL) != pdPASS) {
        ESP_LOGE(TAG, "xTaskCreate failed");
        return;
    }

    // ISR 서비스/핸들러
    ESP_ERROR_CHECK(gpio_install_isr_service(0));
    ESP_ERROR_CHECK(gpio_isr_handler_add(MAX30102_INT_GPIO, int_isr, (void*)MAX30102_INT_GPIO));

    ESP_LOGI(TAG, "Ready (INT on GPIO%d, BURST=%d)", (int)MAX30102_INT_GPIO, BURST_SZ);

    // 주기적으로 라인 레벨 모니터(디버그)
    while (1) {
        int lvl = gpio_get_level(MAX30102_INT_GPIO);
        ESP_LOGD(TAG, "POLL level GPIO%d=%d, isr_cnt=%" PRIu32, (int)MAX30102_INT_GPIO, lvl, s_isr_cnt);
        vTaskDelay(pdMS_TO_TICKS(200));
    }
}