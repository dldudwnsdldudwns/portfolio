#pragma once

// WiFi related basic configuration constants
#define DEFAULT_WIFI_IP_STRING_SIZE 16
#define DEFAULT_WIFI_TIMEOUT_MS 10000       // 10 seconds
#define DEFAULT_WIFI_RETRY_INTERVAL_MS 5000 // 5 seconds

#include <functional>
#include <memory>
#include <string>
#include <unordered_map>

extern "C" {
#include "esp_event.h"
#include "esp_log.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "nvs_flash.h"
}

class WiFiClient {
public:
  struct Config {
    std::string ssid;
    std::string password;
    uint32_t timeout_ms;
    uint32_t retry_interval_ms;

    Config(const std::string &ssid, const std::string &password)
        : ssid(ssid), password(password), timeout_ms(DEFAULT_WIFI_TIMEOUT_MS),
          retry_interval_ms(DEFAULT_WIFI_RETRY_INTERVAL_MS) {}
  };

  explicit WiFiClient(const Config &config);
  ~WiFiClient();

  bool initialize();
  void try_connect();
  void disconnect();

  bool is_available() const { return connected_ && is_ip_assigned(); }
  std::string get_ip_address() const { return ip_address_; }

private:
  static constexpr const char *TAG = "Solicare.WiFi";

  enum class ReasonCategory { AP, AUTH, ENCRYPTION, ASSOC, ETC };
  static inline const std::unordered_map<int, ReasonCategory> reasonToCategory =
      {
          {WIFI_REASON_NO_AP_FOUND, ReasonCategory::AP},
          {WIFI_REASON_BEACON_TIMEOUT, ReasonCategory::AP},
          {WIFI_REASON_AUTH_FAIL, ReasonCategory::AUTH},
          {WIFI_REASON_AUTH_EXPIRE, ReasonCategory::AUTH},
          {WIFI_REASON_AUTH_LEAVE, ReasonCategory::AUTH},
          {WIFI_REASON_802_1X_AUTH_FAILED, ReasonCategory::AUTH},
          {WIFI_REASON_MIC_FAILURE, ReasonCategory::ENCRYPTION},
          {WIFI_REASON_4WAY_HANDSHAKE_TIMEOUT, ReasonCategory::ENCRYPTION},
          {WIFI_REASON_GROUP_KEY_UPDATE_TIMEOUT, ReasonCategory::ENCRYPTION},
          {WIFI_REASON_PAIRWISE_CIPHER_INVALID, ReasonCategory::ENCRYPTION},
          {WIFI_REASON_GROUP_CIPHER_INVALID, ReasonCategory::ENCRYPTION},
          {WIFI_REASON_AKMP_INVALID, ReasonCategory::ENCRYPTION},
          {WIFI_REASON_IE_IN_4WAY_DIFFERS, ReasonCategory::ENCRYPTION},
          {WIFI_REASON_CIPHER_SUITE_REJECTED, ReasonCategory::ENCRYPTION},
          {WIFI_REASON_ASSOC_FAIL, ReasonCategory::ASSOC},
          {WIFI_REASON_ASSOC_EXPIRE, ReasonCategory::ASSOC},
          {WIFI_REASON_ASSOC_LEAVE, ReasonCategory::ASSOC},
          {WIFI_REASON_ASSOC_TOOMANY, ReasonCategory::ASSOC},
          {WIFI_REASON_ASSOC_NOT_AUTHED, ReasonCategory::ASSOC},
          {WIFI_REASON_NOT_AUTHED, ReasonCategory::ASSOC},
          {WIFI_REASON_NOT_ASSOCED, ReasonCategory::ASSOC},
          {WIFI_REASON_UNSPECIFIED, ReasonCategory::ETC},
          {WIFI_REASON_CONNECTION_FAIL, ReasonCategory::ETC},
  };

  Config config_;
  bool initialized_ = false;
  bool connected_ = false;
  bool trying_to_connect_ = false;

  std::string ip_address_;

  esp_event_handler_instance_t wifi_handler_instance_;
  esp_event_handler_instance_t ip_handler_instance_;

  static void event_handler(void *arg, esp_event_base_t event_base,
                            int32_t event_id, void *event_data);

  bool is_ip_assigned() const { return !ip_address_.empty(); }
  void handle_wifi_event(int32_t event_id, void *event_data);
  void handle_ip_event(int32_t event_id, void *event_data);
};
