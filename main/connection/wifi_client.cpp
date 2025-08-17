#include "wifi_client.hpp"
#include <cstring>

extern "C"
{
#include "esp_netif.h"
#include "lwip/ip_addr.h"
}

WiFiClient::WiFiClient(const Config& config) : config_(config)
{
}

WiFiClient::~WiFiClient()
{
	disconnect();
	if (initialized_)
	{
		esp_event_handler_instance_unregister(WIFI_EVENT, ESP_EVENT_ANY_ID, wifi_handler_instance_);
		esp_event_handler_instance_unregister(IP_EVENT, IP_EVENT_STA_GOT_IP, ip_handler_instance_);
		esp_wifi_deinit();
	}
}

bool WiFiClient::initialize()
{
	if (initialized_)
	{
		return true;
	}

	esp_err_t ret = nvs_flash_init();
	if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
	{
		ESP_ERROR_CHECK(nvs_flash_erase());
		ret = nvs_flash_init();
	}
	ESP_ERROR_CHECK(ret);

	ESP_ERROR_CHECK(esp_netif_init());
	ESP_ERROR_CHECK(esp_event_loop_create_default());
	esp_netif_create_default_wifi_sta();

	wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
	ESP_ERROR_CHECK(esp_wifi_init(&cfg));

	ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &event_handler, this,
	                                                    &wifi_handler_instance_));
	ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &event_handler, this,
	                                                    &ip_handler_instance_));

	ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));

	wifi_config_t wifi_config = {};
	std::strncpy(reinterpret_cast<char*>(wifi_config.sta.ssid), config_.ssid.c_str(), sizeof(wifi_config.sta.ssid) - 1);
	std::strncpy(reinterpret_cast<char*>(wifi_config.sta.password), config_.password.c_str(),
	             sizeof(wifi_config.sta.password) - 1);
	wifi_config.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;
	wifi_config.sta.pmf_cfg.capable    = true;
	wifi_config.sta.pmf_cfg.required   = false;

	ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
	ESP_ERROR_CHECK(esp_wifi_start());
	initialized_ = true;
	return initialized_;
}

void WiFiClient::try_connect()
{
	if (!initialized_)
	{
		ESP_LOGE(TAG, "WiFi connect requested before initialization");
		return;
	}
	if (!trying_to_connect_)
	{
		trying_to_connect_ = true;
		auto connect_task  = [](void* pvParameter)
		{
			WiFiClient* client = static_cast<WiFiClient*>(pvParameter);
			ESP_LOGI(TAG, "📶 Attempting to connect to WiFi network: %s", client->config_.ssid.c_str());
			esp_err_t connect_ret = esp_wifi_connect();
			ESP_LOGI(TAG, "esp_wifi_connect() returned: %d", connect_ret);
			client->trying_to_connect_ = false;
			vTaskDelete(NULL);
		};
		xTaskCreatePinnedToCore(connect_task, "wifi_connect_task", 4096, this, 5, NULL, 0);
	}
}

void WiFiClient::disconnect()
{
	if (connected_)
	{
		esp_wifi_disconnect();
		connected_ = false;
		ip_address_.clear();
		ESP_LOGI(TAG, "WiFi disconnected");
	}
}

void WiFiClient::event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data)
{
	auto* manager = static_cast<WiFiClient*>(arg);

	if (event_base == WIFI_EVENT)
	{
		manager->handle_wifi_event(event_id, event_data);
	}
	else if (event_base == IP_EVENT)
	{
		manager->handle_ip_event(event_id, event_data);
	}
	else
	{
		ESP_LOGW(TAG, "Unhandled event: base=%s, id=%ld", event_base, event_id);
	}
}

void WiFiClient::handle_wifi_event(int32_t event_id, void* event_data)
{
	switch (event_id)
	{
	case WIFI_EVENT_STA_START:
		ESP_LOGI(TAG, "WiFi STA started, waiting for connect request");
		return;

	case WIFI_EVENT_STA_CONNECTED:
	{
		connected_  = true;
		auto* event = static_cast<wifi_event_sta_connected_t*>(event_data);
		ESP_LOGI(TAG, "📶 Connected to WiFi network: SSID: %s", event->ssid);
		ESP_LOGI(TAG, "⏳ Waiting for IP assignment... Trying to get IP from AP");
		return;
	}
	case WIFI_EVENT_STA_DISCONNECTED:
	{
		connected_ = false;
		ip_address_.clear();
		if (event_data == nullptr)
		{
			ESP_LOGE(TAG, "event_data is NULL for WIFI_EVENT_STA_DISCONNECTED");
			ESP_LOGI(TAG, "🔄 Will retry WiFi connection in %.1f seconds...", config_.retry_interval_ms / 1000.0f);
			vTaskDelay(pdMS_TO_TICKS(config_.retry_interval_ms));
			try_connect();
			break;
		}
		auto* event             = static_cast<wifi_event_sta_disconnected_t*>(event_data);
		const char* log_message = nullptr;
		uint32_t retry_interval = config_.retry_interval_ms;
		switch (reasonToCategory.find(event->reason) != reasonToCategory.end() ? reasonToCategory.at(event->reason)
		                                                                       : ReasonCategory::ETC)
		{
		case ReasonCategory::AP:
			log_message    = "❓ SSID not found - check if the WiFi ssid is correct";
			retry_interval = 2500; // No delay for AP not found, retry as soon as possible
			break;
		case ReasonCategory::AUTH:
			log_message = "🔑 Authentication failed during authentification - check SSID and password";
			break;
		case ReasonCategory::ENCRYPTION:
			log_message = "🔑 Authentication failed during encryption - check your configuration";
			break;
		case ReasonCategory::ASSOC:
			log_message    = "📡 Association failed - check AP and network status";
			retry_interval = 10000; // Longer delay for association issues
			break;
		case ReasonCategory::ETC:
		default:
			log_message    = (event->reason == WIFI_REASON_CONNECTION_FAIL) ? "❌ Temporary connection failure"
			                                                                : "❔ Unspecified disconnect reason";
			retry_interval = 1000; // retry as soon as possible
			break;
		}
		ESP_LOGI(TAG, "📴 WiFi disconnected - Reason: %d (%s)", event->reason, log_message);
		ESP_LOGI(TAG, "🔄 Will retry WiFi connection in %.1f seconds...", retry_interval / 1000.0f);
		vTaskDelay(pdMS_TO_TICKS(retry_interval));
		try_connect();
		break;
	}
	case WIFI_EVENT_STA_BSS_RSSI_LOW:
	{
		ESP_LOGD(TAG, "📶 WiFi signal strength is low (RSSI low event)");
		break;
	}
	default:
		ESP_LOGW(TAG, "event_data exists but not handled for event_id %ld", event_id);
		break;
	}
}

void WiFiClient::handle_ip_event(int32_t event_id, void* event_data)
{
	if (event_id == IP_EVENT_STA_GOT_IP)
	{
		if (event_data == nullptr)
		{
			ESP_LOGE(TAG, "event_data is NULL for IP_EVENT_STA_GOT_IP");
			ip_address_.clear();
			ESP_LOGW(TAG, "❌ Failed to retrieve IP address - event: %ld", event_id);
			ESP_LOGI(TAG, "🔄 Retrying to connect...");
			disconnect();
			try_connect();
			return;
		}
		auto* event = static_cast<ip_event_got_ip_t*>(event_data);
		char ip_str[DEFAULT_WIFI_IP_STRING_SIZE];
		char netmask_str[DEFAULT_WIFI_IP_STRING_SIZE];
		char gw_str[DEFAULT_WIFI_IP_STRING_SIZE];

		esp_ip4addr_ntoa(&event->ip_info.ip, ip_str, sizeof(ip_str));
		esp_ip4addr_ntoa(&event->ip_info.netmask, netmask_str, sizeof(netmask_str));
		esp_ip4addr_ntoa(&event->ip_info.gw, gw_str, sizeof(gw_str));

		ip_address_ = ip_str;

		ESP_LOGI(TAG, "✅ Sucessfully received IP address");
		ESP_LOGI(TAG, "📡 IP Address: %s", ip_str);
		ESP_LOGI(TAG, "🖧 Subnet Mask: %s", netmask_str);
		ESP_LOGI(TAG, "🚪 Gateway: %s", gw_str);
	}
	else
	{
		ip_address_.clear();
		ESP_LOGW(TAG, "❌ Failed to retrieve IP address - event: %ld", event_id);
		ESP_LOGI(TAG, "🔄 Retrying to connect...");
		disconnect();
		try_connect();
	}
}