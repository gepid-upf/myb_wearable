// /**
//  * @file main.c
//  *
//  * @author
//  * Angelo Elias Dalzotto (150633@upf.br)
//  * Gabriel Boni Vicari (133192@upf.br)
//  * Samuel Antunes Vieira (158820@upf.br)
//  * Sandro Motter (166400@upf.br)
//  * GEPID - Grupo de Pesquisa em Cultura Digital (http://gepid.upf.br/)
//  * Universidade de Passo Fundo (http://www.upf.br/)
//  *
//  * @copyright 2018-2019 Angelo Elias Dalzotto, Gabriel Boni Vicari, Samuel Antunes Vieira, Sandro Motter
//  *
//  * @brief Main file for the MYB project for the ESP-IDF.
//  */

// #include <stdio.h>
// #include <stdlib.h>
// #include <stdbool.h>
// #include <math.h>
// #include <string.h>
// #include <limits.h>
// #include <driver/i2c.h>
// #include <freertos/task.h>
// #include "esp_log.h"
// #include "esp_err.h"
// #include "esp_spiffs.h"
// #include "mpu6050/mpu6050.h"
// #include "max30100/max30100.h"
// #include "tp_interrupt_main.c"
// #include "ibeacon.c"
// #include "esp_sleep.h"
// #include "gatts_demo.c"
// #include <time.h>
// #include <sys/time.h>
// #include "driver/periph_ctrl.h"
// #include "driver/timer.h"

// #define TIMER_DIVIDER 16                             //  Hardware timer clock divider
// #define TIMER_SCALE (TIMER_BASE_CLK / TIMER_DIVIDER) // convert counter value to seconds
// #define TIMER_INTERVAL0_SEC (3.4179)                 // sample test interval for the first timer
// #define TIMER_INTERVAL1_SEC (5.78)                   // sample test interval for the second timer
// #define TEST_WITHOUT_RELOAD 0                        // testing will be done without auto reload
// #define TEST_WITH_RELOAD 1

// //inclui bibliotecas e de comunicação entre os dispositivos

// #define PI 3.14159265358979323846f
// #define AVG_BUFF_SIZE 20
// #define SAMPLE_SIZE 2000
// #define I2C_SDA 26
// #define I2C_SCL 25
// #define I2C_FREQ 100000
// #define I2C_PORT I2C_NUM_0
// #define SIZING 1024

// //define pinos de comunicação I2C, limites e PI
// float self_test[6] = {0, 0, 0, 0, 0, 0};
// float accel_bias[3] = {0, 0, 0};
// float gyro_bias[3] = {0, 0, 0};
// //variaveis globais de teste, aceleração, posição, flags

// uint16_t teste_count = 0;
// uint16_t teste_bpm = 0;

// int step_count = 0;

// //sinaleira kk
// int lock = 0;

// max30100_config_t max30100 = {};

// //configura I2C
// esp_err_t i2c_master_init()
// {
//     i2c_config_t conf = {};
//     conf.mode = I2C_MODE_MASTER;
//     conf.sda_io_num = I2C_SDA;
//     conf.scl_io_num = I2C_SCL;
//     conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
//     conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
//     conf.master.clk_speed = I2C_FREQ;
//     i2c_param_config(I2C_PORT, &conf);

//     return (i2c_driver_install(I2C_PORT, I2C_MODE_MASTER, 0, 0, 0));
// }

// /*
//  * file: endereço do arquivo a ser lido/escrito
//  * rw: wa ou e, sendo 'escrever', ou 'ler' no arquivo
//  * buffer: buffer para leitura ou escrita
// */
// void read_write_file(char *file, char *rw, void *buffer)
// {
//     FILE *file_r = fopen(file, rw);

//     if (file_r == NULL)
//     {
//         printf("Fail to open %s :C\n", file);
//         // return;
//     }

//     if (strcmp(rw, "wa") == 0) //escrita
//     {
//         fprintf(file_r, "%s", (char *)buffer);
//     }
//     else if (strcmp(rw, "r") == 0) //leitura
//     {
//         char line[128];

//         if (fgets(line, sizeof(line), file_r) != NULL)
//         {
//             strcpy((char *)buffer, line);
//         }
//     }

//     fclose(file_r);
// }

// void set_data(char option)
// {

//     //grava no arquivo
//     if (option == 'p')
//     {
//         /////////GERENCIA DE ENERGIA (sleep)////////

//         //Tudo ligado: 69 mA
//         //com accel em standby: 60 mA
//         //com accel em e gyro em standby: 58 mA
//         // printf("deep sleep enable\n");
//         // esp_err_t ret;

//         // max30100_set_shtd();

//         // bool x = false;

//         // x = mpu6050_get_standby_x_accel_enabled();
//         // printf("mpu6050_get_standby_x_accel_enabled: %d\n", x );

//         // mpu6050_set_standby_x_accel_enabled(!x);
//         // mpu6050_set_standby_y_accel_enabled(!x);
//         // mpu6050_set_standby_z_accel_enabled(!x);

//         // x = mpu6050_get_standby_x_gyro_enabled();
//         // printf("mpu6050_get_standby_x_gyro_enabled: %d\n", x );

//         // mpu6050_get_standby_x_gyro_enabled(!x);
//         // mpu6050_get_standby_y_gyro_enabled(!x);
//         // mpu6050_get_standby_z_gyro_enabled(!x);

//         // printf("get_sleep_enabled: %d\n", mpu6050_get_sleep_enabled());

//         // x = mpu6050_get_temp_sensor_enabled();
//         // mpu6050_set_temp_sensor_enabled(!x);
//         // (
//         //     uint8_t device_address,
//         //     uint8_t register_address,
//         //     uint8_t bit_number,
//         //     uint8_t data
//         // );

//         // mpu6050_set_sleep_enabled(1);
//         // // esp_err_t ret = esp_sleep_enable_timer_wakeup(10000000);
//         // if(ret != ESP_OK){
//         //     printf("erro 1\n");
//         // }
//         // printf("desligando...");
//         // esp_deep_sleep_start();

//         // mpu6050_set_sleep_enabled(1);
//     }
//     else if (option == 'r') //'r'
//     {
//         if (lock == 0)
//         {
//             lock = 1;

//             printf("entrou... \n");
//             //reseta passos
//             {
//                 char buffer_steps[6];
//                 sprintf(buffer_steps, "%d", 0);
//                 read_write_file("/spiffs/stepcount.csv", "wa", buffer_steps);
//             }
//             step_count = 0;

//             //reseta bpm
//             read_write_file("/spiffs/bpm.csv", "wa", " ");
//             teste_bpm = 0;

//             lock = 0;
//             printf("saiu... \n");
//         }
//     }
// }

// void fred()
// {
//     uint32_t current = 0, last = 0;
//     uint16_t steps = 0;
//     char buffer_bpm[SIZING] = "";
//     int begin = 0;

//     while (true)
//     {
//         current = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
//         //nota: deu-se um pequeno tempo para o sistema terminar de processar

//         //sugestão: não realizar essa tarefa em tal frequência,
//         //          continuar de 1 em 1 segundo...
//         //          usar opção de touch pad para enviar bpm em momento desejado (sync)

//         if (((current - last) >= 150) && lock == 0 && current > 1500)
//         {

//             lock = 1;
//             //lê passos
//             {
//                 char buffer[6];
//                 read_write_file("/spiffs/stepcount.csv", "r", buffer);
//                 steps = atoi(buffer);
//             }

//             if (strlen(buffer_bpm) <= 1)
//                 begin = 0;
//             // printf("len: %d\n", strlen(buffer_bpm));

//             read_write_file("/spiffs/bpm.csv", "r", buffer_bpm);

//             lock = 0;

//             char aux_c[128] = "";
//             for (int i = begin + 1; i < strlen(buffer_bpm); i++)
//             {
//                 begin++;

//                 // printf("i: %d\n", i);

//                 if (buffer_bpm[i] == ' ')
//                     break;
//                 size_t size = strlen(aux_c);
//                 aux_c[size] = buffer_bpm[i];
//                 if (i >= (strlen(buffer_bpm) - 1))
//                     begin = 0;
//             }
//             // printf("%s\n", aux_c);

//             teste_bpm = atoi(aux_c);

//             // printf("%s\n", buffer_bpm);
//             // printf("teste_bpm: %d\n", teste_bpm);
//             // printf("steps: %d\n", steps);

//             vendor_config.major = ENDIAN_CHANGE_U16(teste_bpm);
//             vendor_config.minor = ENDIAN_CHANGE_U16(steps);
//             // esp_ble_gap_start_advertising(&ble_adv_params);

//             /* code */
//             esp_ble_ibeacon_t ibeacon_adv_data;
//             esp_err_t status = esp_ble_config_ibeacon_data(&vendor_config, &ibeacon_adv_data);
//             if (status == ESP_OK)
//             {
//                 esp_ble_gap_config_adv_data_raw((uint8_t *)&ibeacon_adv_data, sizeof(ibeacon_adv_data));
//             }
//             else
//             {
//                 ESP_LOGE(DEMO_TAG, "Config iBeacon data failed: %s\n", esp_err_to_name(status));
//             }
//             last = current;
//         }
//         vTaskDelay(20 / portTICK_PERIOD_MS);
//     }
// }
// /*
// * função que conta passos conforme dados de aceleração, posição gerenciados na mpu6050
// * e seu cálculo no tempo.
// * armazena na memória o número atual de passos, dados calculados em tempo real
// */
// void step_counter()
// {
//     mpu6050_rotation_t gyro;
//     uint64_t current = 0, last = 0;
//     int8_t range = 0;
//     float gyro_ds_x, gyro_ds_y, gyro_ds_z, gyro_res;
//     int lim_sup = 80, lim_inf = -80;
//     int step_changed = 0;
//     int count_seq_act = 0;
//     bool first = true;
//     int cheats = 0;
//     bool stop_cheating = 0;

//     while (true)
//     {
//         if (!mpu6050_get_int_dmp_status())
//         {

//             //recebe a faixa de escala completa das saídas do acelerômetro
//             //range == 0;
//             //table:
//             //      0 ± 2g
//             //      1 ± 4g
//             //      2 ± 8g
//             //      3 ± 16g
//             range = mpu6050_get_full_scale_accel_range();

//             mpu6050_get_rotation(&gyro);

//             //recebe o intervalo de escala completa das saídas do giroscópio
//             // de acordo com a tabela:
//             //      0 ± 250  °/s
//             //      1 ± 500  °/s
//             //      2 ± 1000 °/s
//             //      3 ± 2000 °/s
//             range = mpu6050_get_full_scale_gyro_range();

//             //retorna resolução dos giros
//             //Ex: range == 0 então gyro_res = 250.0 / 32768.0;
//             gyro_res = mpu6050_get_gyro_res(range);

//             gyro_ds_x = (float)gyro.gyro_x * gyro_res - gyro_bias[0];
//             gyro_ds_y = (float)gyro.gyro_y * gyro_res - gyro_bias[1];
//             gyro_ds_z = (float)gyro.gyro_z * gyro_res - gyro_bias[2];

//             //filtros
//             mpu6050_madgwick_quaternion_update(
//                 (float)0,
//                 (float)0,
//                 (float)0,
//                 gyro_ds_x * PI / 180.0f,
//                 gyro_ds_y * PI / 180.0f,
//                 gyro_ds_z * PI / 180.0f);

//             // printf("%f %f %f\n", gyro_ds_x, gyro_ds_y, gyro_ds_z);
//             // printf("%d\n", count_seq_act);

//             current = (uint32_t)(xTaskGetTickCount());

//             if (gyro_ds_z >= lim_sup && first) //se passar pelo superior e se já passou pelo limite inferior
//             {
//                 first = false;
//                 last = (uint32_t)(xTaskGetTickCount());
//             }
//             else if (gyro_ds_z <= lim_inf)
//             {
//                 if (!first)
//                 {
//                     uint32_t time = current - last;

//                     if (time > 13) //!Ninguem corre tão rápido...
//                     {
//                         if (time == 15) //15
//                         {
//                             cheats++;
//                         }
//                         else
//                         {
//                             if (stop_cheating == 1)
//                             {
//                                 cheats = 0;
//                                 stop_cheating = 0;
//                             }
//                             stop_cheating = 1;
//                         }
//                         printf("time: %d", time);
//                         count_seq_act++;
//                         teste_count += 2;
//                         printf("\n"); //apenas para piscar led do esp para indicar movimento
//                     }
//                     else
//                     {
//                         count_seq_act = 0; //quebra sequencia de ações
//                     }
//                 }
//                 first = true;
//             }
//             else if (current - last > 150) //ninguem caminha tão lento, ou seja 150 depois de passar pelo limite superior
//             {
//                 first = true;      //habilita nova busca por limite superior
//                 count_seq_act = 0; //quebra sequencia de ações
//             }

//             if (count_seq_act >= 5 && cheats < 5) //se houve uma sequencia de 5 passos, incrementa o contador de passos
//             {
//                 step_count += (count_seq_act * 2); //multiplica por dois, pois cada ação representa dois passos
//                 step_changed = 1;
//                 count_seq_act = 0; //apenas reinicia contador de senquencias
//             }

//             //se houve um passo grava no arquivo stepcount.csv
//             if (lock == 0)
//             {
//                 if (step_changed)
//                 {
//                     lock = 1;
//                     ESP_LOGI(mpu6050_get_tag(), "Step Counter: %d", step_count);
//                     //escreve passos
//                     {
//                         char buffer_steps[6];
//                         sprintf(buffer_steps, "%d", step_count);
//                         read_write_file("/spiffs/stepcount.csv", "wa", buffer_steps);
//                     }
//                     step_changed = 0;
//                     lock = 0;
//                 }
//             }
//             vTaskDelay(10 / portTICK_PERIOD_MS);
//         }
//         //read_step_count(&step_count);
//     }
// }
// /*
// * Função que coleta dados do max30100 e realiza cálculo de pulsação,
// * num intervalo de milisegundos, guardando o valor de pulso atual na memória
// */
// void bpm_counter(void *param)
// {
//     char buffer_bpm[SIZING] = "";

//     max30100_data_t result = {};

//     while (true)
//     {
//         max30100_update(&max30100, &result);
//         if (lock == 0)
//         {
//             if (result.pulse_detected)
//             {
//                 lock = 1;
//                 read_write_file("/spiffs/bpm.csv", "r", buffer_bpm);
//                 teste_bpm = (uint16_t)result.heart_bpm;
//                 // ESP_LOGI("MAX30100", "BPM: %f", result.heart_bpm);

//                 //converte BPM para string
//                 char c_bpm[10];
//                 sprintf(c_bpm, "%d", (uint8_t)result.heart_bpm);

//                 //concatena buffer
//                 strcat(buffer_bpm, " ");
//                 strcat(buffer_bpm, c_bpm);

//                 //grava buffer no arquivo
//                 read_write_file("/spiffs/bpm.csv", "wa", buffer_bpm);
//                 lock = 0;
//             }
//         }
//         vTaskDelay(10 / portTICK_PERIOD_MS);
//     }
// }

// void unix(void *param)
// {
//     char buffer_unix_time[6];

//     char buffer[6];
//     int aux = 0;

//     // while (lock_unix_time)
//     //     ;
//     // lock_unix_time = 1;
//     // read_write_file("/spiffs/unixtime.csv", "r", buffer);
//     // lock_unix_time = 0;

//     aux = atoi(buffer);
//     printf("aux: %d\n", aux);

//     struct tm data;

//     struct timeval tv; //Cria a estrutura temporaria para funcao abaixo.
//     tv.tv_sec = aux;   //Atribui minha data atual. Voce pode usar o NTP para isso ou o site citado no artigo!
//     settimeofday(&tv, NULL);
//     int x = 0;
//     x = aux;
//     while (true)
//     {

//         time_t tt = time(NULL);
//         data = *gmtime(&tt);

//         char data_formatada[64];
//         strftime(data_formatada, 64, "%d/%m/%Y %H:%M:%S", &data);
//         printf("\nUnix Time: %d\n", (int32_t)tt); //Mostra na Serial o Unix time
//         printf("Data formatada: %s\n", data_formatada);
//         aux = (int32_t)tt;
//         printf("AUXXXX: %d", aux);

//         sprintf(buffer_unix_time, "%d", aux);
//         read_write_file("/spiffs/unixtime.csv", "wa", buffer_unix_time);

//         if (!lock_unix_time)
//         {
//             lock_unix_time = 1;
//             if (unix_time)
//             {
//                 sprintf(buffer_unix_time, "%d", unix_time);
//                 read_write_file("/spiffs/unixtime.csv", "wa", buffer_unix_time);

//                 tv.tv_sec = unix_time; //Atribui minha data atual. Voce pode usar o NTP para isso ou o site citado no artigo!
//                 settimeofday(&tv, NULL);
//             }
//             unix_time = 0;

//             lock_unix_time = 0;
//         }

//         // if (aux >= x + 5)
//         // {
//         //     esp_err_t ret = esp_sleep_enable_timer_wakeup(5000000);
//         //     if (ret != ESP_OK)
//         //     {
//         //         printf("erro 1\n");
//         //     }
//         //     printf("desligando...");
//         //     esp_deep_sleep_start();
//         // }

//         vTaskDelay(1000 / portTICK_PERIOD_MS);
//     }
// }

// void app_main()
// {
//     esp_err_t ret;

//     esp_vfs_spiffs_conf_t spiffs_config = {
//         .base_path = "/spiffs",
//         .partition_label = NULL,
//         .max_files = 3,
//         .format_if_mount_failed = true};
//     //configura diretório padrão de memória
//     ret = esp_vfs_spiffs_register(&spiffs_config);
//     if (ret != ESP_OK)
//     {
//         if (ret == ESP_FAIL)
//             ESP_LOGE("SPIFFS", "Failed to mount or format filesystem.");
//         else if (ret == ESP_ERR_NOT_FOUND)
//             ESP_LOGE("SPIFFS", "Failed to find SPIFFS partition.");
//         else
//             ESP_LOGE("SPIFFS", "Failed to initialize SPIFFS (%s).",
//                      esp_err_to_name(ret));
//     }
//     else
//         ESP_LOGI("SPIFFS", "Initialized.");
//     //retorna mensagem de sucesso ou falha de comunicação I2C

//     size_t total = 0, used = 0;
//     ret = esp_spiffs_info(NULL, &total, &used);
//     if (ret != ESP_OK)
//     {
//         ESP_LOGE("SPIFFS", "Failed to get SPIFFS partition information (%s).",
//                  esp_err_to_name(ret));
//     }
//     else
//     {
//         ESP_LOGI("SPIFFS", "Partition size: total: %d, used: %d.", total, used);
//     }

//     ESP_ERROR_CHECK(i2c_master_init());
//     //realiza teste de erro de comunicação entre componentes
//     {
//         //config ble
//         ESP_ERROR_CHECK(nvs_flash_init());
//         ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));
//         esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
//         esp_bt_controller_init(&bt_cfg);
//         esp_bt_controller_enable(ESP_BT_MODE_BLE);

//         ret = esp_bluedroid_init();
//         if (ret)
//         {
//             ESP_LOGE(GATTS_TAG, "%s init bluetooth failed: %s\n", __func__, esp_err_to_name(ret));
//             return;
//         }
//         ret = esp_bluedroid_enable();
//         if (ret)
//         {
//             ESP_LOGE(GATTS_TAG, "%s enable bluetooth failed: %s\n", __func__, esp_err_to_name(ret));
//             return;
//         }

//         ret = esp_ble_gatts_register_callback(gatts_event_handler);
//         if (ret)
//         {
//             ESP_LOGE(GATTS_TAG, "gatts register error, error code = %x", ret);
//             return;
//         }
//         ret = esp_ble_gap_register_callback(gap_event_handler);
//         if (ret)
//         {
//             ESP_LOGE(GATTS_TAG, "gap register error, error code = %x", ret);
//             return;
//         }
//         ret = esp_ble_gatts_app_register(PROFILE_A_APP_ID);
//         if (ret)
//         {
//             ESP_LOGE(GATTS_TAG, "gatts app register error, error code = %x", ret);
//             return;
//         }
//         ret = esp_ble_gatts_app_register(PROFILE_B_APP_ID);
//         if (ret)
//         {
//             ESP_LOGE(GATTS_TAG, "gatts app register error, error code = %x", ret);
//             return;
//         }
//         esp_err_t local_mtu_ret = esp_ble_gatt_set_local_mtu(500);
//         if (local_mtu_ret)
//         {
//             ESP_LOGE(GATTS_TAG, "set local  MTU failed, error code = %x", local_mtu_ret);
//         }
//     }

//     xTaskCreate(unix, "unix", 8192, NULL, 5, NULL);

//     //retorna mensagem de sucesso ou falha de atribuição de valores de tamanho
//     //de partição de memória usada/total

//     // ESP_LOGI(mpu6050_get_tag(), "Device ID: %d.", mpu6050_get_device_id());
//     // //inicializa e configura mpu6050

//     // mpu6050_self_test(self_test);
//     // ESP_LOGI(mpu6050_get_tag(), "Device performing self-test.");

//     // //self_test:
//     // //indices acelerometro : 0, 1, 2 == respectivamente eixos x, y, z
//     // //indices giroscópio : 3, 4, 5 == respectivamente eixos x, y, z

//     // if (self_test[0] < 1.0f && self_test[1] < 1.0f && self_test[2] < 1.0f &&
//     //     self_test[3] < 1.0f && self_test[4] < 1.0f && self_test[5] < 1.0f)
//     // {
//     //     //reseta todos os registradores para seus valores padrões
//     //     //A princio ñ precisa ser executada pois também é chamada em "mpu6050_calibrate()"
//     //     mpu6050_reset();

//     //     //configura dispositivo para calibração
//     //     mpu6050_calibrate(accel_bias, gyro_bias);
//     //     ESP_LOGI(mpu6050_get_tag(), "Device being calibrated.");

//     //     //inicia mpu com modo sleep desligado
//     //     // mpu6050_init();
//     //     ESP_LOGI(mpu6050_get_tag(), "Device initialized.");
//     //     xTaskCreatePinnedToCore(step_counter, "StepCounter", 8192, NULL, 3, NULL, 0);
//     // }
//     // else
//     //     ESP_LOGI(mpu6050_get_tag(), "Device did not pass self-test.");
//     // //verifica erros na inicialização de mpu6050

//     // ESP_ERROR_CHECK(max30100_init(
//     //     &max30100,
//     //     I2C_PORT,
//     //     MAX30100_DEFAULT_OPERATING_MODE,
//     //     MAX30100_DEFAULT_SAMPLING_RATE,
//     //     MAX30100_DEFAULT_LED_PULSE_WIDTH,
//     //     MAX30100_DEFAULT_IR_LED_CURRENT,
//     //     MAX30100_DEFAULT_START_RED_LED_CURRENT,
//     //     MAX30100_DEFAULT_MEAN_FILTER_SIZE,
//     //     1,
//     //     true,
//     //     false));
//     // ESP_LOGI("MAX30100", "Device initialized.");
//     // //inicializa e verifica erros no dispositivo max30100
//     // xTaskCreatePinnedToCore(bpm_counter, "BPMCounter", 8192, NULL, 3, NULL, 0);
//     //Cria processo com acessos irrestrito ao mapa de memória completo do microcontrolador
//     // 1 - ponteiro para a função do processo
//     // 2 - nome descritivo do processo
//     // 3 - tamanho da pilha do processo especificada como o número de variáveis que a pilha pode suportar.
//     //  Se a pilha for 16bits e este valor for definido como 100,
//     //  200 bytes serão alocados para armazenamento da pilha.
//     // 4 - ponteiro que será usado como parâmetro para o processo criado
//     // 5 - prioridade a qual o processo deverá rodar
//     // 6 - usado para retornar um identificador pelo qual o processo pode ser referenciado

//     // Inicializa periférico touch pad, que inicializará um temporizador para executar um filtro
//     ESP_LOGI(TAG, "Initializing touch pad");
//     touch_pad_init();
//     // Se usado o modo de gatilho de interrupção, deve ser setado o sensor de toque modo FSM em 'TOUCH_FSM_MODE_TIMER'.
//     touch_pad_set_fsm_mode(TOUCH_FSM_MODE_TIMER);
//     // Seta tensão de referência para carga/descarga
//     // Para vários cenários, é recomendado usar a seguinte combinação:
//     // a tensão de referência alta será de 2.7v - 1v = 1.7v, a tensão de referência baixa será de 0.5v.
//     touch_pad_set_voltage(TOUCH_HVOLT_2V7, TOUCH_LVOLT_0V5, TOUCH_HVOLT_ATTEN_1V);
//     // Inicia E/S do touch pad
//     tp_example_touch_pad_init();
//     // Inicializa e inicia um software de filtro que detecta leve mudança de capacitância.
//     touch_pad_filter_start(TOUCHPAD_FILTER_TOUCH_PERIOD);
//     // Define limiar
//     tp_example_set_thresholds();
//     // Rotina de interrupção de toque é registrada
//     touch_pad_isr_register(tp_example_rtc_intr, NULL);
//     // Inicia uma tarefa que mostra quais pinos foram tocados

//     xTaskCreatePinnedToCore(&tp_example_read_task,
//                             "touch_pad_read_task", 4096, (void *)&set_data, 1, NULL, 1);
//     //cria processo para leitura de touch pad
// }
