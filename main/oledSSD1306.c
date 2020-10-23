#include <driver/gpio.h>
#include <driver/spi_master.h>
#include <esp_log.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <time.h>
#include <u8g2.h>
#include <icons.h>

#include "sdkconfig.h"
#include "u8g2_esp32_hal.h"
#include "headermain.h"

#define PIN_SDA 21 //D2
#define PIN_SCL 22 //D1
#define maxbpm 150
#define minbpm 40

static const char *TAGE = "ssd1306";
void task_SSD1306(char *beatstep) {

	char steps[stepdigs];
	char bpm[bpmdigs];
	char time[timedigs];
	char mode[modedigs];

	int i;

	u8g2_esp32_hal_t u8g2_esp32_hal = U8G2_ESP32_HAL_DEFAULT;
	u8g2_esp32_hal.sda   = PIN_SDA;
	u8g2_esp32_hal.scl  = PIN_SCL;
	u8g2_esp32_hal_init(u8g2_esp32_hal);

	u8g2_t u8g2;
	u8g2_Setup_ssd1306_64x48_er_f(
		&u8g2,
		U8G2_R0,
		u8g2_esp32_msg_i2c_cb,
		u8g2_esp32_msg_i2c_and_delay_cb);
	u8x8_SetI2CAddress(&u8g2.u8x8,0x78);

	ESP_LOGI(TAGE, "u8g2_InitDisplay");
	u8g2_InitDisplay(&u8g2);

	ESP_LOGI(TAGE, "u8g2_ClearDisplay");
	u8g2_ClearDisplay(&u8g2);

	ESP_LOGI(TAGE, "u8g2_SetPowerSave");
	u8g2_SetPowerSave(&u8g2, 0);
	
	ESP_LOGI(TAGE, "u8g2_ClearBuffer");
	u8g2_ClearBuffer(&u8g2);

	ESP_LOGI(TAGE, "u8g2_SendBuffer");
	u8g2_SendBuffer(&u8g2);

	while (true)
	{
		for(i = 0; i < beatstep_bits; i++){
			if(i<(stepdigs)){
				steps[i] = beatstep[i];
			}else if(i>= stepdigs && i < (stepdigs+bpmdigs)){
				bpm[i-stepdigs] = beatstep[i];
			}else if(i>=(stepdigs+bpmdigs) && i < (stepdigs+bpmdigs+timedigs)){
				time[i-(stepdigs+bpmdigs)] = beatstep[i];
			}else{
				mode[i-(stepdigs+bpmdigs+timedigs)] = beatstep[i];
			}
		}

	    if(atoi(mode)==0)
		{
			//tela 0 (desligada)

			u8g2_ClearBuffer(&u8g2);
			
			u8g2_SetPowerSave(&u8g2, 1);

		}else if(atoi(mode)==1)
		{
			//tela 1 (time)

			u8g2_SetPowerSave(&u8g2, 0);

			u8g2_ClearBuffer(&u8g2);

			u8g2_SetFont(&u8g2, u8g2_font_pxplusibmcgathin_8f);

			u8g2_DrawStr(&u8g2, 1, 30, time);

			u8g2_SendBuffer(&u8g2);

		}else if(atoi(mode)==2){
			//tela 2 (bpm+steps)

			u8g2_SetPowerSave(&u8g2, 0);

			u8g2_ClearBuffer(&u8g2);
			
			u8g2_SetFont(&u8g2, u8g2_font_pxplusibmcgathin_8f);

			u8g2_DrawStr(&u8g2, 5, 18, bpm);

			u8g2_DrawStr(&u8g2, 5, 40, steps);

			u8g2_DrawXBM(&u8g2, 38, 5, heart_img_width, heart_img_height, heart_img_bitmap);
			
			u8g2_DrawXBM(&u8g2, 38, 26, step_img_width, step_img_height, step_img_bitmap);

			u8g2_SendBuffer(&u8g2);

			vTaskDelay(1000 / portTICK_PERIOD_MS);

			u8g2_ClearBuffer(&u8g2);

			u8g2_SetPowerSave(&u8g2, 1);

			vTaskDelay(250 / portTICK_PERIOD_MS);
		}
	}

	ESP_LOGI(TAGE, "All done!");

	vTaskDelete(NULL);
}