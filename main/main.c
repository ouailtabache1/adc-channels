#include "freertos/FreeRTOS.h"
#include "esp_wifi.h"
#include "esp_system.h"
#include "esp_event.h"
#include "esp_event_loop.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "driver/gpio.h"
#include "ad.h"
#include "console.h"

void app_main(void)
{
	esp_err_t res=nvs_flash_init();
	if (res==ESP_ERR_NVS_NO_FREE_PAGES || res==ESP_ERR_NVS_NEW_VERSION_FOUND) {
		ESP_ERROR_CHECK(nvs_flash_erase());
		res=nvs_flash_init();
	}
	ESP_ERROR_CHECK(res);
	//init the esp console
	console_init();
	configASSERT(ad_init());
}

