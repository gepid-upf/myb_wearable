#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <ctype.h>
#include <math.h>
#include <string.h>
#include <limits.h>
#include <driver/i2c.h>
#include <freertos/task.h>
#include "esp_log.h"
#include "esp_err.h"
#include "esp_spiffs.h"
#include "mpu6050/mpu6050.h"
#include "esp_sleep.h"
#include "oledSSD1306.c"
#include "gatts.c"
#include <time.h>
#include <sys/time.h>
#include "driver/periph_ctrl.h"
#include "driver/timer.h"
#include "headermain.h"

#include "nvs.h"
#include "nvs_flash.h"
#include "soc/rtc_cntl_reg.h"
#include "soc/sens_reg.h"
#include "driver/gpio.h"
#include "driver/rtc_io.h"
#include "driver/adc.h"
#include "driver/dac.h"
#include "esp32/ulp.h"
#include "ulp_main.h"
#include "esp_timer.h"

#define TIMER_DIVIDER 16                             //  Hardware timer clock divider
#define TIMER_SCALE (TIMER_BASE_CLK / TIMER_DIVIDER) // convert counter value to seconds
#define TIMER_INTERVAL0_SEC (3.4179)                 // sample test interval for the first timer
#define TIMER_INTERVAL1_SEC (5.78)                   // sample test interval for the second timer
#define TEST_WITHOUT_RELOAD 0                        // testing will be done without auto reload
#define TEST_WITH_RELOAD 1

#define PI 3.14159265358979323846f
#define AVG_BUFF_SIZE 20
#define SAMPLE_SIZE 2000
#define I2C_SDA 26
#define I2C_SCL 25
#define I2C_FREQ 100000
#define I2C_PORT I2C_NUM_0
#define SIZING 1024
#define threshold 10

#ifndef TP_INTERRUPT_MAIN

#define TP_INTERRUPT_MAIN

#include "freertos/FreeRTOS.h"
#include "driver/touch_pad.h"
#include "soc/rtc_periph.h"
#include "soc/sens_periph.h"

#define TOUCH_THRESH_NO_USE (0)
#define TOUCH_THRESH_PERCENT (80)
#define TOUCHPAD_FILTER_TOUCH_PERIOD (10)

static bool s_pad_activated[TOUCH_PAD_MAX];
static uint32_t s_pad_init_val[TOUCH_PAD_MAX];

#endif

//include do sen0203
#include "adc1_bpm_counter.c"

static const char *TAG_TOUCH = "Touch pad";

float self_test[6] = {0, 0, 0, 0, 0, 0};
float accel_bias[3] = {0, 0, 0};
float gyro_bias[3] = {0, 0, 0};

uint16_t teste_count = 0;
uint16_t teste_bpm = 0;

uint16_t step_count = 0;

char beatstep[beatstep_bits+1] = {"0"};

bool jafoi = 0;

//i2c ds1307
//i2c_dev_t dev;

extern const uint8_t ulp_main_bin_start[] asm("_binary_ulp_main_bin_start");
extern const uint8_t ulp_main_bin_end[] asm("_binary_ulp_main_bin_end");

/* This function is called once after power-on reset, to load ULP program into
 * RTC memory and configure the ADC.
 */

static void init_ulp_program(void);

/* This function is called every time before going into deep sleep.
 * It starts the ULP program and resets measurement counter.
 */
static void start_ulp_program(void);

RTC_DATA_ATTR static time_t last_date;

esp_err_t i2c_master_init()
{
    i2c_config_t conf = {};
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = I2C_SDA;
    conf.scl_io_num = I2C_SCL;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = I2C_FREQ;
    i2c_param_config(I2C_PORT, &conf);

    return (i2c_driver_install(I2C_PORT, I2C_MODE_MASTER, 0, 0, 0));
}

/*
 * file: endereço do arquivo a ser lido/escrito
 * rw: wa ou e, sendo 'escrever', ou 'ler' no arquivo
 * buffer: buffer para leitura ou escrita
*/
void read_write_file(char *file, char *rw, void *buffer)
{
    FILE *file_r = fopen(file, rw);

    if (file_r == NULL)
    {
        printf("Fail to open %s :C\n", file);
    }

    if (strcmp(rw, "wa") == 0)
    {
        fprintf(file_r, "%s", (char *)buffer);
    }
    else if (strcmp(rw, "r") == 0)
    {
        char line[128];

        if (fgets(line, sizeof(line), file_r) != NULL)
        {
            strcpy((char *)buffer, line);
        }
    }

    fclose(file_r);
}

void step_counter()
{
    mpu6050_rotation_t gyro;
    uint64_t current = 0, last = 0;
    int8_t range = 0;
    float gyro_ds_x, gyro_ds_y, gyro_ds_z, gyro_res;
    int lim_sup = 80, lim_inf = -80;
    int step_changed = 0;
    int count_seq_act = 0;
    bool first = true;
    int cheats = 0;
    bool stop_cheating = 0;

    while (true)
    {
        if (!mpu6050_get_int_dmp_status())
        {

            range = mpu6050_get_full_scale_accel_range();

            mpu6050_get_rotation(&gyro);

            range = mpu6050_get_full_scale_gyro_range();

            gyro_res = mpu6050_get_gyro_res(range);

            gyro_ds_x = (float)gyro.gyro_x * gyro_res - gyro_bias[0];
            gyro_ds_y = (float)gyro.gyro_y * gyro_res - gyro_bias[1];
            gyro_ds_z = (float)gyro.gyro_z * gyro_res - gyro_bias[2];

            mpu6050_madgwick_quaternion_update(
                (float)0,
                (float)0,
                (float)0,
                gyro_ds_x * PI / 180.0f,
                gyro_ds_y * PI / 180.0f,
                gyro_ds_z * PI / 180.0f);

            // printf("%f %f %f\n", gyro_ds_x, gyro_ds_y, gyro_ds_z);
            // printf("%d\n", count_seq_act);

            current = (uint32_t)(xTaskGetTickCount());

            if (gyro_ds_z >= lim_sup && first)
            {
                first = false;
                last = (uint32_t)(xTaskGetTickCount());
            }
            else if (gyro_ds_z <= lim_inf)
            {
                if (!first)
                {
                    uint32_t time = current - last;

                    if (time > 13)
                    {
                        if (time == 15)
                        {
                            cheats++;
                        }
                        else
                        {
                            if (stop_cheating == 1)
                            {
                                cheats = 0;
                                stop_cheating = 0;
                            }
                            stop_cheating = 1;
                        }
                        //printf("time: %d", time);
                        count_seq_act++;
                        teste_count += 2;
                        //printf("\n");
                    }
                    else
                    {
                        count_seq_act = 0;
                    }
                }
                first = true;
            }
            else if (current - last > 150)
            {
                first = true;
                count_seq_act = 0;
            }

            if (count_seq_act >= 5 && cheats < 5)
            {
                step_count += (count_seq_act * 2);
                step_changed = 1;
                count_seq_act = 0;
            }

            if (beatstep[beatstep_bits]=='0')
            {
                if (step_changed)
                {
                    beatstep[beatstep_bits] = '1';
                    ESP_LOGI(mpu6050_get_tag(), "Step Counter: %d", step_count);
                    {
                        ble_step_count = step_count;
                        char buffer_steps[6];
                        sprintf(buffer_steps, "%d", step_count);
                        sprintf(beatstep, "%d", step_count);
                        read_write_file("/spiffs/stepcount.csv", "wa", buffer_steps);
                    }
                    step_changed = 0;
                    beatstep[beatstep_bits] = '0';
                }
            }
            vTaskDelay(10 / portTICK_PERIOD_MS);
        }
    }
}

void unix(void *param)
{
    // int unix_time = 1571776920;

    struct tm data;
    char buffer_all[sizeof(beatstep)];
    int s;
    struct timeval now;

    while (1)
    {
        while (lock_unix_time);
        lock_unix_time = 1;
        time_t tt = time(NULL) + unix_time;
        lock_unix_time = 0;
        data = *gmtime(&tt);

        char data_formatada[64];
        // strftime(data_formatada, 64, "%d/%m/%Y %H:%M:%S", &data); //Cria uma String formatada da estrutura "data"
        strftime(data_formatada, 64, "%H:%M:%S", &data); //Cria uma String formatada da estrutura "data"

        for (int j = (stepdigs + bpmdigs); j < (stepdigs + bpmdigs + timedigs); j++)
        {
            if (beatstep[beatstep_bits] == '0')
            {
                beatstep[beatstep_bits] = '1';
                beatstep[j] = data_formatada[j - (stepdigs + bpmdigs)];
                beatstep[beatstep_bits] = '0';
            }
        }
        

        //aqui verifica o buffer para enviar ao ble, contando com bpm, hora de registro do
        //primeiro bpm, e passos

        if (beatstep[beatstep_bits] == '0')
        {
            beatstep[beatstep_bits] = '1';

            if(!jafoi)
            {
                if(idx_def_bpm_array==1)
                {
                    for (int l = (stepdigs + bpmdigs), s = (BPM_DEF_SIZE+4); l < (stepdigs + bpmdigs + timedigs); l++, s++)
                    {    
                        if(isdigit(beatstep[l])){
                            buf_bpm[s] = (uint8_t)beatstep[l] - '0';
                        }else{
                            s--;
                        }
                    }

                    jafoi = 1;
                }
            }

            beatstep[beatstep_bits] = '0';
        }
        
        if(jafoi)
        {
            if(idx_def_bpm_array >= BPM_DEF_SIZE)
            {
                
                for (int j = 0; j < BPM_DEF_SIZE; j++)
                {
                    buf_bpm[j] = def_bpm_array[j];
                    def_bpm_array[j] = 0;
                }
                
                buf_bpm[BPM_DEF_SIZE] = 0;
                buf_bpm[BPM_DEF_SIZE+1] = step_count & 0xFF; //lower byte
                buf_bpm[BPM_DEF_SIZE+2] = step_count >> 8; //higher byte
                buf_bpm[BPM_DEF_SIZE+3] = 0;
                idx_def_bpm_array = 0;
                first_time = false;

                jafoi = 0;
            }
        }

        // printf("Data formatada: %s\n", data_formatada);
        // printf("segundos: %d\n", (int32_t)data.tm_sec);
        // printf("minutos: %d\n", (int32_t)data.tm_min);
        // printf("hora: %d\n", (int32_t)data.tm_hour);

        last_date = tt;

        // if (time22 > 10)
        // {
        //     printf("Entering deep sleep\n\n");

        //     esp_deep_sleep_start();
        // }

        

        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

//daquiiiiiii

/*
  Lê valores recebidos em todos os touch pads.
  Usa 2/3 do valor lido como threshold
  para engatilhar a interrupção quando o pad é tocado.

  Nota: não se deve tocar nenhum pad no início da aplicação.
 */
static void tp_example_set_thresholds(void)
{
    uint16_t touch_value;
    for (int i = 0; i < TOUCH_PAD_MAX; i++)
    {
        //lê valor filtrado
        touch_pad_read_filtered(i, &touch_value);
        s_pad_init_val[i] = touch_value;
        ESP_LOGI(TAG_TOUCH, "test init: touch pad [%d] val is %d", i, touch_value);
        //seta threshold de interrupção.
        ESP_ERROR_CHECK(touch_pad_set_thresh(i, touch_value * 2 / 3));
    }
}

/*
  Checa se quaisquer dos pads foram ativados
  lendo uma tabela atualizada por rtc_intr()
  Se sim, printa no monitor.
  Após, limpa entradas relacionadas na tabela.
 */
static void tp_example_read_task(void *pvParameter)
{
    touch_pad_intr_enable();
    while (1)
    {
        //obs.: dá pra trocar o pad, tbm é possível fazer um switch case
        for (int i = 0; i < TOUCH_PAD_MAX; i++)
        {
            if (s_pad_activated[i] == true && i == 3)
            {
                if(beatstep[beatstep_bits] == '0')
                {
                    beatstep[beatstep_bits] = '1';
                    
                    ESP_LOGI(TAG_TOUCH, "T%d activated!", i);

                    switch (beatstep[stepdigs + bpmdigs + timedigs])
                    {
                    case '2':
                        beatstep[stepdigs + bpmdigs + timedigs] = '0';
                        break;

                    case '1':
                        beatstep[stepdigs + bpmdigs + timedigs] = '2';
                        break;

                    case '0':
                        beatstep[stepdigs + bpmdigs + timedigs] = '1';
                        break;
                    }

                    beatstep[beatstep_bits] = '0';
                }
                

                // Espera um pouco para o pad ser liberado
                vTaskDelay(500 / portTICK_PERIOD_MS);
                // Limpa informação da ativação do pad
                s_pad_activated[i] = false;
            }
        }
        vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}

/*
  Lida com uma interrupção engatilhada quando um pad é tocado.
  Reconhece qual pad é tocado e salva em uma tabela.
 */
static void tp_example_rtc_intr(void *arg)
{
    uint32_t pad_intr = touch_pad_get_status();
    //clear interrupt
    touch_pad_clear_status();
    for (int i = 0; i < TOUCH_PAD_MAX; i++)
    {
        if ((pad_intr >> i) & 0x01)
        {
            s_pad_activated[i] = true;
        }
    }
}

/*
 * Antes de ler o touch pad, é preciso inicializar as entradas do RTC.
 */
static void tp_example_touch_pad_init()
{
    for (int i = 0; i < TOUCH_PAD_MAX; i++)
    {
        //init RTC IO and mode for touch pad.
        touch_pad_config(i, TOUCH_THRESH_NO_USE);
    }
}
//até aquiiiiiii

void app_main()
{

    esp_sleep_wakeup_cause_t cause = esp_sleep_get_wakeup_cause();
    switch (cause)
    {
    case ESP_SLEEP_WAKEUP_TIMER:
    {
        printf("Wake up timer\n");
        break;
    }
    case ESP_SLEEP_WAKEUP_ULP:
    {
        printf("Wake up from ulp\n");
        break;
    }

    case ESP_SLEEP_WAKEUP_TOUCHPAD:
    {
        printf("Wake up from touch on pad %d\n", esp_sleep_get_touchpad_wakeup_status());
        // printf("ULP_TIME: %d\n", ulp_time & UINT16_MAX);
        // printf("ULP_TEST: %d\n", ulp_test & UINT16_MAX);
        break;
    }

    default:
        printf("wake up cause default\n");
        init_ulp_program();
        break;
    }

    //aqui
    start_ulp_program();
    
    //inicializa modo tela: dormindo
    //usar o touch pad para alterar o modo
    beatstep[beatstep_bits] = '0'; //lock = 0
    beatstep[stepdigs + bpmdigs + timedigs] = '0'; // modo = 0

    esp_err_t ret;

    //aqui: cuidar o MAXFILES
    esp_vfs_spiffs_conf_t spiffs_config = {
        .base_path = "/spiffs",
        .partition_label = NULL,
        .max_files = 4,
        .format_if_mount_failed = true};

    ret = esp_vfs_spiffs_register(&spiffs_config);
    if (ret != ESP_OK)
    {
        if (ret == ESP_FAIL)
            ESP_LOGE("SPIFFS", "Failed to mount or format filesystem.");
        else if (ret == ESP_ERR_NOT_FOUND)
            ESP_LOGE("SPIFFS", "Failed to find SPIFFS partition.");
        else
            ESP_LOGE("SPIFFS", "Failed to initialize SPIFFS (%s).",
                     esp_err_to_name(ret));
    }
    else
        ESP_LOGI("SPIFFS", "Initialized.");

    size_t total = 0, used = 0;
    ret = esp_spiffs_info(NULL, &total, &used);
    if (ret != ESP_OK)
    {
        ESP_LOGE("SPIFFS", "Failed to get SPIFFS partition information (%s).",
                 esp_err_to_name(ret));
    }
    else
    {
        ESP_LOGI("SPIFFS", "Partition size: total: %d, used: %d.", total, used);
    }

    ESP_ERROR_CHECK(i2c_master_init());

    // config ble
    {
        ESP_ERROR_CHECK(nvs_flash_init());
        ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));
        esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
        esp_bt_controller_init(&bt_cfg);
        esp_bt_controller_enable(ESP_BT_MODE_BLE);

        ret = esp_bluedroid_init();
        if (ret)
        {
            ESP_LOGE(GATTS_TAG, "%s init bluetooth failed: %s\n", __func__, esp_err_to_name(ret));
            return;
        }
        ret = esp_bluedroid_enable();
        if (ret)
        {
            ESP_LOGE(GATTS_TAG, "%s enable bluetooth failed: %s\n", __func__, esp_err_to_name(ret));
            return;
        }

        ret = esp_ble_gatts_register_callback(gatts_event_handler);
        if (ret)
        {
            ESP_LOGE(GATTS_TAG, "gatts register error, error code = %x", ret);
            return;
        }
        ret = esp_ble_gap_register_callback(gap_event_handler);
        if (ret)
        {
            ESP_LOGE(GATTS_TAG, "gap register error, error code = %x", ret);
            return;
        }
        // ret = esp_ble_gatts_app_register(PROFILE_A_APP_ID);
        // if (ret)
        // {
        //     ESP_LOGE(GATTS_TAG, "gatts app register error, error code = %x", ret);
        //     return;
        // }
        // ret = esp_ble_gatts_app_register(PROFILE_B_APP_ID);
        // if (ret)
        // {
        //     ESP_LOGE(GATTS_TAG, "gatts app register error, error code = %x", ret);
        //     return;
        // }
        ret = esp_ble_gatts_app_register(PROFILE_SANDRO_APP_ID);
        if (ret)
        {
            ESP_LOGE(GATTS_TAG, "gatts app register error, error code = %x", ret);
            return;
        }
        esp_err_t local_mtu_ret = esp_ble_gatt_set_local_mtu(512);
        if (local_mtu_ret)
        {
            ESP_LOGE(GATTS_TAG, "set local  MTU failed, error code = %x", local_mtu_ret);
        }
    }

    xTaskCreatePinnedToCore(unix, "unix", 8192, NULL, 1, NULL, 1);

    //inicializa e configura mpu6050
    ESP_LOGI(mpu6050_get_tag(), "Device ID: %d.", mpu6050_get_device_id());

    mpu6050_self_test(self_test);
    ESP_LOGI(mpu6050_get_tag(), "Device performing self-test.");

    if (self_test[0] < 1.0f && self_test[1] < 1.0f && self_test[2] < 1.0f &&
        self_test[3] < 1.0f && self_test[4] < 1.0f && self_test[5] < 1.0f)
    {
        mpu6050_reset();

        mpu6050_calibrate(accel_bias, gyro_bias);
        ESP_LOGI(mpu6050_get_tag(), "Device being calibrated.");

        mpu6050_init();
        ESP_LOGI(mpu6050_get_tag(), "Device initialized.");
        xTaskCreatePinnedToCore(step_counter, "StepCounter", 8192, NULL, 5, NULL, 1);
    }
    else
        ESP_LOGI(mpu6050_get_tag(), "Device did not pass self-test.");

    xTaskCreatePinnedToCore(&task_SSD1306, "task_run_SSD1306", 8192, (char *)&beatstep, 1, NULL, 1); //vetorstep

    //aquiii
    xTaskCreatePinnedToCore(&task_sen0203, "adc_reading_task", 8192, (char *)&beatstep, 5, NULL, 0);

    // Inicializa o touch pad, irá iniciar um temporizador para executar um filtro
    ESP_LOGI(TAG_TOUCH, "Initializing touch pad");
    touch_pad_init();
    // Se usar moto de interrupção, deve-se setar o modo FSM para 'TOUCH_FSM_MODE_TIMER'.
    touch_pad_set_fsm_mode(TOUCH_FSM_MODE_TIMER);
    // Seta tensão de referência para carga/descarga
    // Para a maioria dos senários, é recomendado o uso da seguinte combinação:
    // a high reference voltage será 2.7V - 1V = 1.7V, e a low reference voltage será 0.5V.
    touch_pad_set_voltage(TOUCH_HVOLT_2V7, TOUCH_LVOLT_0V5, TOUCH_HVOLT_ATTEN_1V);
    // Inicializa IO do touch pad
    tp_example_touch_pad_init();
    // Inicializa e começa um filtro por software para detectar leves diferenças de capacitância
    touch_pad_filter_start(TOUCHPAD_FILTER_TOUCH_PERIOD);
    // Seta thresholds
    tp_example_set_thresholds();
    // Define rotina de interrupção
    touch_pad_isr_register(tp_example_rtc_intr, NULL);
    // Inicializa uma task que permite verificar quais pads foram tocados
    xTaskCreatePinnedToCore(&tp_example_read_task, "touch_pad_read_task", 2048, (char *)&beatstep, 3, NULL, 0);
}

static void init_ulp_program(void)
{
    esp_timer_init();

    esp_err_t err = ulp_load_binary(0, ulp_main_bin_start,
                                    (ulp_main_bin_end - ulp_main_bin_start) / sizeof(uint32_t));
    ESP_ERROR_CHECK(err);

    /* Configure ADC channel */
    /* Note: when changing channel here, also change 'adc_channel' constant
       in adc.S */
    adc1_config_channel_atten(ADC1_CHANNEL_6, ADC_ATTEN_DB_11);
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_ulp_enable();

    /* Set low and high thresholds, approx. 1.35V - 1.75V*/
    // ulp_low_thr = 0;
    // ulp_time = 0;
    // ulp_high_thr = 2000;

    /* Set ULP wake up period to 20ms */
    ulp_set_wakeup_period(0, 20000);

    /* Disconnect GPIO12 and GPIO15 to remove current drain through
     * pullup/pulldown resistors.
     * GPIO12 may be pulled high to select flash voltage.
     */
    rtc_gpio_isolate(GPIO_NUM_12);
    rtc_gpio_isolate(GPIO_NUM_15);
    esp_deep_sleep_disable_rom_logging(); // suppress boot messages
}

static void start_ulp_program(void)
{
    /* Reset sample counter */
    // ulp_sample_counter = 0;

    /* Start the program */
    esp_err_t err = ulp_run(&ulp_entry - RTC_SLOW_MEM);
    ESP_ERROR_CHECK(err);
}