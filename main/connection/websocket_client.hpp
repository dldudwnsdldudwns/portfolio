#pragma once

// WebSocket client related basic configuration constants
#define DEFAULT_WS_RETRY_INTERVAL_MS 4000  // 4 seconds
#define DEFAULT_WS_CONNECT_TIMEOUT_MS 3000 // 3 seconds
#define DEFAULT_WS_SEND_TIMEOUT_MS 1500    // 1.5 seconds
#define DEFAULT_WS_SEND_INTERVAL_MS 5      // 5 milliseconds
#define DEFAULT_WS_QUEUE_TIMEOUT_MS 1000   // 1 second
#define DEFAULT_WS_MAX_QUEUE_SIZE 10       // 10 images

#include <functional>
#include <memory>
#include <string>
#include <unordered_map>

extern "C"
{
#include "esp_event.h"
#include "esp_websocket_client.h"
}

class WebSocketClient
{
  public:
	struct Config
	{
		std::string device_name;
		std::string uri;

		size_t max_queue_size;
		uint32_t connect_timeout_ms;
		uint32_t send_timeout_ms;
		uint32_t send_interval_ms;
		uint32_t data_wait_timeout_ms;
		uint32_t retry_interval_ms;

		Config(const std::string& device_name, const std::string& server_ip, uint16_t server_port)
		    : device_name(device_name), uri("ws://" + server_ip + ":" + std::to_string(server_port) + "/"),
		      max_queue_size(DEFAULT_WS_MAX_QUEUE_SIZE), connect_timeout_ms(DEFAULT_WS_CONNECT_TIMEOUT_MS),
		      send_timeout_ms(DEFAULT_WS_SEND_TIMEOUT_MS), send_interval_ms(DEFAULT_WS_SEND_INTERVAL_MS),
		      data_wait_timeout_ms(DEFAULT_WS_QUEUE_TIMEOUT_MS), retry_interval_ms(DEFAULT_WS_RETRY_INTERVAL_MS)
		{
		}
	};
	bool send_text_now(const std::string& text);

	explicit WebSocketClient(const Config& config, std::function<bool()> is_wifi_available);
	~WebSocketClient();

	bool initialize();
	void async_connect();
	void start_sender_task();
	void stop_sender_task();
	bool enqueue_binary_data(std::unique_ptr<uint8_t[]> data, size_t size);

	bool is_available() const
	{
		return client_ && esp_websocket_client_is_connected(client_);
	}

  private:
	static constexpr const char* TAG = "Solicare.WebSocket";

	using BinaryData = std::pair<size_t, std::unique_ptr<uint8_t[]>>;

	Config config_;
	bool trying_to_connect_ = false;

	esp_websocket_client_handle_t client_ = nullptr;
	QueueHandle_t send_queue_             = nullptr;
	TaskHandle_t sender_task_handle_      = nullptr;

	std::function<bool()> is_wifi_available_;

	static void websocket_event_handler(void* handler_args, esp_event_base_t base, int32_t event_id, void* event_data);
	static void sender_task_wrapper(void* param);

	bool send_text(std::string text);
	void async_send_binary(std::unique_ptr<uint8_t[]> data, size_t size);


};
