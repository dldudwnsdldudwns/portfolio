#include "websocket_client.hpp"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char* TAG = "WebSocketClient";

static const char* ws_error_type_str(int t) {
    switch (t) {
    case WEBSOCKET_ERROR_TYPE_NONE:          return "NONE";
    case WEBSOCKET_ERROR_TYPE_TCP_TRANSPORT: return "TCP_TRANSPORT";
    case WEBSOCKET_ERROR_TYPE_PONG_TIMEOUT:  return "PONG_TIMEOUT";
    case WEBSOCKET_ERROR_TYPE_HANDSHAKE:     return "HANDSHAKE";
    case WEBSOCKET_ERROR_TYPE_SERVER_CLOSE:  return "SERVER_CLOSE";
    default: return "UNKNOWN";
    }
}

WebSocketClient::WebSocketClient(const Config& config, std::function<bool()> is_wifi_available)
    : config_(config), is_wifi_available_(std::move(is_wifi_available))
{
    send_queue_ = xQueueCreate(config_.max_queue_size, sizeof(BinaryData*));
}

WebSocketClient::~WebSocketClient()
{
    if (send_queue_) {
        vQueueDelete(send_queue_);
        send_queue_ = nullptr;
    }
    if (client_) {
        esp_websocket_client_stop(client_);
        esp_websocket_client_destroy(client_);
        client_ = nullptr;
    }
}

bool WebSocketClient::initialize()
{
    esp_websocket_client_config_t ws_cfg = {};
    ws_cfg.uri = config_.uri.c_str();
    ws_cfg.buffer_size = 65536; // 64KB
    ws_cfg.task_prio = 5;
    ws_cfg.disable_auto_reconnect = true;
    ws_cfg.reconnect_timeout_ms = config_.retry_interval_ms;
    ws_cfg.network_timeout_ms = config_.connect_timeout_ms;
    // NOTE: ping_interval_sec는 "초" 단위 필드입니다. 필요 시 ms -> sec로 변환해 사용하세요.
    // ws_cfg.ping_interval_sec   = config_.ping_interval_sec;

    client_ = esp_websocket_client_init(&ws_cfg);
    if (!client_) {
        ESP_LOGE(TAG, "Failed to init websocket client");
        return false;
    }

    esp_websocket_register_events(client_, WEBSOCKET_EVENT_ANY, websocket_event_handler, this);
    return true;
}

void WebSocketClient::async_connect()
{
    if (!client_) {
        ESP_LOGE(TAG, "Critical error: Tried to connect before WebSocket client was initialized");
        return;
    }
    if (trying_to_connect_) return;

    trying_to_connect_ = true;

    auto connect_task = [](void* pvParameter)
        {
            auto* self = static_cast<WebSocketClient*>(pvParameter);
            auto* cfg = &self->config_;

            // Wait until wifi is available
            while (!self->is_wifi_available_()) {
                ESP_LOGI(TAG, "Waiting for WiFi to be available before connecting to WebSocket...");
                vTaskDelay(pdMS_TO_TICKS(cfg->retry_interval_ms));
            }

            ESP_LOGI(TAG, "Connecting to WebSocket server: %s", cfg->uri.c_str());
            esp_websocket_client_start(self->client_);
            vTaskDelete(nullptr);
        };

    xTaskCreatePinnedToCore(connect_task, "ws_connect_task", 4096, this, 5, nullptr, 0);
}

void WebSocketClient::start_sender_task()
{
    if (sender_task_handle_ == nullptr) {
        ESP_LOGI(TAG, "Starting data sender task");
        xTaskCreatePinnedToCore(&WebSocketClient::sender_task_wrapper, "data_sender", 4096, this, 5,
            &sender_task_handle_, 0);
    }
}

void WebSocketClient::stop_sender_task()
{
    if (sender_task_handle_ != nullptr) {
        vTaskDelete(sender_task_handle_);
        sender_task_handle_ = nullptr;
    }
    ESP_LOGI(TAG, "Image sender task stopped");
}

bool WebSocketClient::enqueue_binary_data(std::unique_ptr<uint8_t[]> data, size_t size)
{
    if (!send_queue_) return false;

    auto* packet = new BinaryData(size, std::move(data));

    if (uxQueueMessagesWaiting(send_queue_) >= config_.max_queue_size) {
        ESP_LOGW(TAG, "Data queue full, dropping oldest data");
        BinaryData* old = nullptr;
        if (xQueueReceive(send_queue_, &old, 0) == pdTRUE && old) {
            delete old;
        }
    }

    if (xQueueSend(send_queue_, &packet, 0) != pdTRUE) {
        delete packet;
        return false;
    }
    return true;
}

void WebSocketClient::websocket_event_handler(void* handler_args,
    esp_event_base_t wrap_base,
    int32_t wrap_event_id,
    void* wrap_event_data)
{
    auto handle_ws_event = [](void* pvParameter)
        {
            auto* tuple = static_cast<std::tuple<WebSocketClient*, esp_event_base_t, int32_t, void*>*>(pvParameter);
            auto* client = static_cast<WebSocketClient*>(std::get<0>(*tuple));
            auto* config = &client->config_;
            // auto base     = static_cast<esp_event_base_t>(std::get<1>(*tuple));
            auto event_id = static_cast<int32_t>(std::get<2>(*tuple));
            auto* event_data = static_cast<esp_websocket_event_data_t*>(std::get<3>(*tuple));

            bool do_reconnect = false;

            switch (event_id) {
            case WEBSOCKET_EVENT_CONNECTED:
                client->trying_to_connect_ = false;
                ESP_LOGI(TAG, "WebSocket connected to server: %s", config->uri.c_str());
                ESP_LOGI(TAG, "Sending device name to server: %s", config->device_name.c_str());
                if (client->send_text(config->device_name)) {
                    ESP_LOGI(TAG, "Device name '%s' sent successfully", config->device_name.c_str());
                }
                else {
                    ESP_LOGE(TAG, "Connection established, but failed to send device name to server");
                    esp_websocket_client_stop(client->client_);
                }
                break;

            case WEBSOCKET_EVENT_DISCONNECTED:
                if (!client->trying_to_connect_) {
                    do_reconnect = true;
                    const auto& eh = event_data->error_handle;
                    ESP_LOGI(TAG,
                        "WebSocket disconnected (type:%s/%d, esp_err:0x%08x, stack:0x%08x, cert_flags:0x%08x)",
                        ws_error_type_str(eh.error_type),
                        eh.error_type,
                        static_cast<unsigned int>(eh.esp_tls_last_esp_err),
                        static_cast<unsigned int>(eh.esp_tls_stack_err),
                        static_cast<unsigned int>(eh.esp_tls_cert_verify_flags));
                }
                break;

            case WEBSOCKET_EVENT_DATA:
                ESP_LOGI(TAG,
                    "WebSocket data received: %zu bytes (payload_len: %zu)",
                    static_cast<size_t>(event_data->data_len),
                    static_cast<size_t>(event_data->payload_len));
                break;

            case WEBSOCKET_EVENT_ERROR:
            {
                std::string error_message;
                const esp_websocket_error_codes_t& err = event_data->error_handle;

                switch (err.error_type) {
                case WEBSOCKET_ERROR_TYPE_NONE:
                    error_message = "No error";
                    break;
                case WEBSOCKET_ERROR_TYPE_TCP_TRANSPORT:
                    error_message = "TCP transport error";
                    break;
                case WEBSOCKET_ERROR_TYPE_PONG_TIMEOUT:
                    error_message = "PONG timeout (no response from server)";
                    break;
                case WEBSOCKET_ERROR_TYPE_HANDSHAKE:
                    error_message = "Handshake error";
                    break;
                case WEBSOCKET_ERROR_TYPE_SERVER_CLOSE:
                    error_message = "Server closed connection";
                    break;
                default:
                    error_message = "Unknown WebSocket error type";
                    break;
                }

                if (client->trying_to_connect_) {
                    do_reconnect = true;
                }

                ESP_LOGE(TAG,
                    "WebSocket error: %s (type:%s/%d, esp_err:0x%08x, stack:0x%08x, cert_flags:0x%08x)",
                    error_message.c_str(),
                    ws_error_type_str(err.error_type),
                    err.error_type,
                    static_cast<unsigned int>(err.esp_tls_last_esp_err),
                    static_cast<unsigned int>(err.esp_tls_stack_err),
                    static_cast<unsigned int>(err.esp_tls_cert_verify_flags));
                break;
            }

            default:
                // ESP_LOGW(TAG, "Unhandled WebSocket event (event_id: %d)", event_id);
                break;
            }

            if (do_reconnect) {
                ESP_LOGI(TAG,
                    "Attempting to reconnect WebSocket in %.1f seconds...",
                    static_cast<float>(config->retry_interval_ms) / 1000.0f);
                vTaskDelay(pdMS_TO_TICKS(config->retry_interval_ms));
                client->trying_to_connect_ = false;
                client->async_connect();
            }

            delete tuple;
            vTaskDelete(nullptr);
        };

    auto* wrap_self = static_cast<WebSocketClient*>(handler_args);
    auto* param = new std::tuple<WebSocketClient*, esp_event_base_t, int32_t, void*>(
        wrap_self, wrap_base, wrap_event_id, wrap_event_data);

    xTaskCreatePinnedToCore(handle_ws_event, "ws_event_handler", 4096, param, 5, nullptr, 0);
}

void WebSocketClient::sender_task_wrapper(void* param)
{
    auto* self = static_cast<WebSocketClient*>(param);
    auto* config = &self->config_;

    ESP_LOGI(TAG, "Data sender task started");

    while (true) {
        if (!self->is_available()) {
            vTaskDelay(pdMS_TO_TICKS(config->data_wait_timeout_ms));
            continue;
        }

        BinaryData* binary_data = nullptr;
        if (xQueueReceive(self->send_queue_, &binary_data, pdMS_TO_TICKS(config->data_wait_timeout_ms)) == pdTRUE) {
            if (binary_data && self->is_available()) {
                self->async_send_binary(std::move(binary_data->second), binary_data->first);
            }
            delete binary_data;
            vTaskDelay(pdMS_TO_TICKS(config->send_interval_ms));
            continue;
        }

        ESP_LOGI(TAG, "No data to send, waiting for new data...");
        vTaskDelay(pdMS_TO_TICKS(config->data_wait_timeout_ms));
    }
}

bool WebSocketClient::send_text(std::string text)
{
    return esp_websocket_client_send_text(client_, text.c_str(), text.size(), portMAX_DELAY) > 0;
}

void WebSocketClient::async_send_binary(std::unique_ptr<uint8_t[]> data, size_t size)
{
    if (!esp_websocket_client_is_connected(client_)) {
        ESP_LOGI(TAG, "Failed to send data - WebSocket not connected");
        return;
    }

    auto send_task = [](void* pvParameter)
        {
            auto* param = static_cast<std::pair<WebSocketClient*, std::vector<uint8_t>>*>(pvParameter);
            WebSocketClient* self = param->first;
            const std::vector<uint8_t>& buf = param->second;

            int sent = esp_websocket_client_send_bin(self->client_,
                reinterpret_cast<const char*>(buf.data()),
                buf.size(),
                portMAX_DELAY);
            if (sent != static_cast<int>(buf.size())) {
                ESP_LOGW(TAG, "Failed to send data via WebSocket (sent: %d, size: %zu)",
                    sent, static_cast<size_t>(buf.size()));
                esp_websocket_client_stop(self->client_);
            }
            delete param;
            vTaskDelete(nullptr);
        };

    auto* param = new std::pair<WebSocketClient*, std::vector<uint8_t>>(
        this, std::vector<uint8_t>(data.get(), data.get() + size));

    xTaskCreatePinnedToCore(send_task, "ws_send_task", 8192, param, 5, nullptr, 0);
}
