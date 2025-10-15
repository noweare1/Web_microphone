idf_component_register(SRCS "cJSON.c" "wifi_apsta_main.c"
                  PRIV_REQUIRES esp_driver_ledc
                  PRIV_REQUIRES esp_driver_i2s
                  PRIV_REQUIRES esp_driver_gpio
                  PRIV_REQUIRES esp_driver_ledc
                  PRIV_REQUIRES esp_partition
                  PRIV_REQUIRES spiffs
                  PRIV_REQUIRES esp_wifi
                  PRIV_REQUIRES esp_timer
                  PRIV_REQUIRES nvs_flash
									PRIV_REQUIRES esp_http_server
                  INCLUDE_DIRS "../include"
                  INCLUDE_DIRS "."
								  INCLUDE_DIRS "../spiffs")

spiffs_create_partition_image(storage ../spiffs SPIFFS_IMAGE_FLASH_IN_PROJECT)
                   
  