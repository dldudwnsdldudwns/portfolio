idf_component_register(SRCS "main.cpp"
"connection/wifi_client.cpp"
"connection/websocket_client.cpp"
PRIV_REQUIRES esp_wifi esp_event esp_netif nvs_flash lwip driver spi_flash
esp_timer esp_system heap esp_hw_support efuse
esp_adc
INCLUDE_DIRS "." "connection")