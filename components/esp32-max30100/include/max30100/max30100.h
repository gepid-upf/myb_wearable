/**
 * @file max30100.h
 * 
 * @author
 * Angelo Elias Dalzotto (150633@upf.br)
 * GEPID - Grupo de Pesquisa em Cultura Digital (http://gepid.upf.br/)
 * Universidade de Passo Fundo (http://www.upf.br/)
 * 
 * @copyright
 * Copyright (c) 2017 Raivis Strogonovs (https://morf.lv)
 * 
 * @brief This library was created to interface the MAX30100 pulse and oxymeter
 * sensor with ESP32 using the IDF-SDK. It includes functions to initialize with
 * the programmer's desired parameters and to update the readings, detecting pulse
 * and having the pulse saturation O2. It is based on Strogonovs Arduino library.
 * 
 * Essa biblioteca foi criada para interfaciar o sensor de pulso e oxímetro MAX30100
 * com ESP32 usando o IDF-SDK. Inclui funções para inicializar com os parametros
 * desejados pelo programador e para atualizar leituras, detectando pulso e tendo
 * saturação de pulso 02. É baseada na biblioteca Strongonovs do Arduino.
*/
#ifndef MAX30100_H
#define MAX30100_H

#include <math.h>
#include "driver/i2c.h"
#include "freertos/task.h"
#include "esp_err.h"
#include "esp32_i2c_rw/esp32_i2c_rw.h"

/**
 * Param padrões de inicialização.
 */
#define MAX30100_DEFAULT_OPERATING_MODE         MAX30100_MODE_HR_ONLY
#define MAX30100_DEFAULT_IR_LED_CURRENT         MAX30100_LED_CURRENT_50MA 
#define MAX30100_DEFAULT_START_RED_LED_CURRENT  MAX30100_LED_CURRENT_0MA
#define MAX30100_DEFAULT_SAMPLING_RATE          MAX30100_SAMPLING_RATE_100HZ
#define MAX30100_DEFAULT_LED_PULSE_WIDTH        MAX30100_PULSE_WIDTH_1600US_ADC_16
#define MAX30100_DEFAULT_ACCEPTABLE_INTENSITY_DIFF      65000
#define MAX30100_DEFAULT_RED_LED_CURRENT_ADJUSTMENT_MS    500
#define MAX30100_DEFAULT_RESET_SPO2_EVERY_N_PULSES          4
#define MAX30100_DEFAULT_ALPHA                              0.95
#define MAX30100_DEFAULT_MEAN_FILTER_SIZE                  5 //15
#define MAX30100_DEFAULT_PULSE_MIN_THRESHOLD              10    //100
#define MAX30100_DEFAULT_PULSE_MAX_THRESHOLD              1000      //2000
#define MAX30100_DEFAULT_PULSE_BPM_SAMPLE_SIZE             10

/**
 * Operação modo Enum.
 * Pulsação apenas ou saturação de oxigênio + pulsação
 */
typedef enum _max30100_mode_t {
    MAX30100_MODE_HR_ONLY                 = 0x02,
    MAX30100_MODE_SPO2_HR                 = 0x03
} max30100_mode_t;

/**
 * enum. taxa de amostragem
 * Taxa de amostragem interna de 50Hz à 1KHz.
 */
typedef enum SamplingRate {
    MAX30100_SAMPLING_RATE_50HZ           = 0x00,
    MAX30100_SAMPLING_RATE_100HZ          = 0x01,
    MAX30100_SAMPLING_RATE_167HZ          = 0x02,
    MAX30100_SAMPLING_RATE_200HZ          = 0x03,
    MAX30100_SAMPLING_RATE_400HZ          = 0x04,
    MAX30100_SAMPLING_RATE_600HZ          = 0x05,
    MAX30100_SAMPLING_RATE_800HZ          = 0x06,
    MAX30100_SAMPLING_RATE_1000HZ         = 0x07
} max30100_sampling_rate_t;

/**
 * enum. largura de pulso de led
 * seta de 200us à 1600us.
 */
typedef enum LEDPulseWidth {
    MAX30100_PULSE_WIDTH_200US_ADC_13     = 0x00,
    MAX30100_PULSE_WIDTH_400US_ADC_14     = 0x01,
    MAX30100_PULSE_WIDTH_800US_ADC_15     = 0x02,
    MAX30100_PULSE_WIDTH_1600US_ADC_16    = 0x03,
} max30100_pulse_width_t;

/**
 * enum. corrente do Led
 * Seta corrente para quaiquer leds. de 0mA à 50mA.
 */
typedef enum LEDCurrent {
    MAX30100_LED_CURRENT_0MA              = 0x00,
    MAX30100_LED_CURRENT_4_4MA            = 0x01,
    MAX30100_LED_CURRENT_7_6MA            = 0x02,
    MAX30100_LED_CURRENT_11MA             = 0x03,
    MAX30100_LED_CURRENT_14_2MA           = 0x04,
    MAX30100_LED_CURRENT_17_4MA           = 0x05, 
    MAX30100_LED_CURRENT_20_8MA           = 0x06,
    MAX30100_LED_CURRENT_24MA             = 0x07,
    MAX30100_LED_CURRENT_27_1MA           = 0x08,
    MAX30100_LED_CURRENT_30_6MA           = 0x09,
    MAX30100_LED_CURRENT_33_8MA           = 0x0A,
    MAX30100_LED_CURRENT_37MA             = 0x0B,
    MAX30100_LED_CURRENT_40_2MA           = 0x0C,
    MAX30100_LED_CURRENT_43_6MA           = 0x0D,
    MAX30100_LED_CURRENT_46_8MA           = 0x0E,
    MAX30100_LED_CURRENT_50MA             = 0x0F
} max30100_current_t;

/**
 * estrutura de configuração FIFO.
 * NÃO DECLARAR.
 */
typedef struct _max30100_fifo_t {
    uint16_t raw_ir;
    uint16_t raw_red;
} max30100_fifo_t;

/**
 * estrutura de filtro DC.
 * NÃO DECLARAR.
 */
typedef struct _max30100_dc_filter_t {
    float w;
    float result;
} max30100_dc_filter_t;

/**
 * estrutura de filtro Butterworth.
 * NÃO DECLARAR.
 */
typedef struct _max30100_butterworth_filter_t
{
    float v[2];
    float result;
} max30100_butterworth_filter_t;

/**
 * estrutura do filtro de diferenciação médio.
 * NÃO DECLARAR.
 */
typedef struct _max30100_mean_diff_filter_t
{
  float* values;
  uint8_t index;
  float sum;
  uint8_t count;
} max30100_mean_diff_filter_t;

/**
 * estrutura de dados.
 * Você precisa disso para acompanhar os valores.
 */
typedef struct _max30100_data_t {
    bool pulse_detected; 
    float heart_bpm;

    float ir_cardiogram;

    float ir_dc_value;
    float red_dc_value;

    float spO2;

    uint32_t last_beat_threshold;

    float dc_filtered_red;
    float dc_filtered_ir;
} max30100_data_t;

/**
 * a estrutura 'objeto' do sensor.
 * Use-a para manter toda informação de configuração
 * Não mude diretamente, use funções para fazê-lo.
 */
typedef struct _max30100_config_t {

    i2c_port_t i2c_num;

    bool debug;

    uint8_t red_current; 
    uint32_t last_red_current_check;

    uint8_t current_pulse_detector_state;
    float current_bpm;
    float* values_bpm;
    float values_bpm_sum;
    uint8_t values_bpm_count;
    uint8_t bpm_index;
    uint32_t last_beat_threshold;

    uint32_t acceptable_intense_diff;
    uint32_t red_current_adj_ms;
    uint8_t reset_spo2_pulse_n;
    float dc_alpha;
    uint16_t pulse_min_threshold;
    uint16_t pulse_max_threshold;

    uint8_t mean_filter_size;
    uint8_t pulse_bpm_sample_size;

    max30100_fifo_t prev_fifo;

    max30100_dc_filter_t dc_filter_ir;
    max30100_dc_filter_t dc_filter_red;
    max30100_butterworth_filter_t lpb_filter_ir;
    max30100_mean_diff_filter_t mean_diff_ir;

    float ir_ac_sq_sum;
    float red_ac_sq_sum;
    uint16_t samples_recorded;
    uint16_t pulses_detected;
    float current_spO2;

    max30100_current_t ir_current;

} max30100_config_t;

/**
 * @brief Inicializa o sensor com o I2C desejado e parametros dados.
 * Você pode tambér utilizar o modo debug.
 * 
 * @details Essas funções incluem alguns parametros internos que inicializam
 * com valores padrão. Você mode mudá-los depois com funções max30100_set_*.
 * 
 * @param this é o endereço da estrutura de configuração.
 * @param i2c_num é a porta I2C do ESP32 conectada com MAX30100.
 * @param mode é o modo de funcionamento do sensor (HR somente ou HR+SPO2)
 * @param sampling_rate é a frequência em que amostras são tiradas internamente.
 * @param pulse_width é a largura de pulso de led. 
 * @param ir_current é a corrente do IR Led.
 * @param start_red_current é o valor inicial para corrente do led vermelho.
 * @param mean_filter_size é o tamanho do vetor de amostras para o filtro.
 * @param pulse_bpm_sample_size é o tamanho do vetor de amostragem de frequência cardíaca.
 * @param high_res_mode Habilita ou não modo de alta resolução.
 * @param debug Habilita se saída de registradores será usada para monitoramento serial.
 * 
 * @retorna estado de execução.
 */

esp_err_t max30100_init( max30100_config_t* this, i2c_port_t i2c_num, 
                         max30100_mode_t mode, max30100_sampling_rate_t sampling_rate, 
                         max30100_pulse_width_t pulse_width, max30100_current_t ir_current,
                         max30100_current_t start_red_current, uint8_t mean_filter_size, 
                         uint8_t pulse_bpm_sample_size, bool high_res_mode, bool debug );

/**
 * @brief Lê dos registradores internos MAX30100 e atualiza estruturas internas.
 * Os resultados são escritos numa estrutura de dados.
 * 
 * @details Essas funções devem ser chamadas numa taxa de 100Hz.
 * 
 * @param this é o endereço da estrutura de configuração.
 * @param data é o endereço da estrutura de dados que salva os resultados.
 * 
 * @retorna estado de execução.
 */
esp_err_t max30100_update(max30100_config_t* this, max30100_data_t* data);

/**
 * @brief Lẽ temperatura interna do MAX30100.
 * 
 * @param this é o endereço da estrutura de configuração.
 * @param temperature é o endereço da variável que armazena a temperatura.
 * 
 * @retorna estado de execução.
 */
esp_err_t max30100_read_temperature(max30100_config_t* this, float* temperature);

/**
 * @brief Printa registro para fins de depuração.
 * 
 * @param this é o endereço da estrutura de configuração.
 * 
 * @retorna estado de execução.
 */
esp_err_t max30100_print_registers(max30100_config_t* this);

/**
 * @brief Seta o modo de operação. HR somente ou HR+SPO2.
 * 
 * @details This é automaticamente chamada pela função de inicialização.
 * 
 * @param this é o endereço da estrutura de configuração.
 * @param mode é o modo de operação desejado.
 * 
 * @retorna estado de execução.
 */
esp_err_t max30100_set_mode(max30100_config_t* this, max30100_mode_t mode);

/**
 * @brief Seta a resolução. Alta ou padrão.
 * 
 * @details This é automaticamente chamada pela função de inicialização.
 * 
 * @param this é o endereço da estrutura de configuração.
 * @param enable é se alta resolução será habilitada.
 * 
 * @retorna estado de execução.
 */
esp_err_t max30100_set_high_res(max30100_config_t* this, bool enabled);

/**
 * @brief Seta correntes de Led.
 * 
 * @details This é automaticamente chamada pela função de inicialização
 * ou quando necessária para bilbioteca.
 * 
 * @param this é o enndereço da estrutura de configuração.
 * @param red_current é o valor de corrente do led vermelho.
 * @param ir_current é o valor de corrente do led IR.
 * 
 * @retorna estado de execução.
 */
esp_err_t max30100_set_led_current( max30100_config_t* this, 
                                    max30100_current_t red_current, 
                                    max30100_current_t ir_current );

/**
 * @brief Seta a largura de pulso.
 * 
 * @details This é automaticamente chamada pela função de inicialização.
 * 
 * @param this é o endereço da estrutura de configuração.
 * @param pw é a largura de pulso desejada.
 * 
 * @retorna estado de execução.
 */                                    
esp_err_t max30100_set_pulse_width(max30100_config_t* this, max30100_pulse_width_t pw);

/**
 * @brief Seta o valor de taxa de amostragem interno.
 * 
 * @details This é automaticamente chamado pela função de inicialização.
 * 
 * @param this é o endereço da estrutura de configuração.
 * @param rate é a taxa de amostragem desejada.
 * 
 * @retorda estado de execução.
 */
esp_err_t max30100_set_sampling_rate( max30100_config_t* this, 
                                      max30100_sampling_rate_t rate );

/**
 * @brief Seta a diferença de intensidade aceitável.
 * 
 * @details This é setado para valor padrão na inicialização. (recomendado)
 * a única maneira de mudar é chamando esta função.
 * 
 * @param this é o endereço da estrutura de configuração.
 * @param acceptable_intense_diff é o valor de diferença aceito.
 * 
 * @retorna estado de execução.
 */
esp_err_t max30100_set_acceptable_intense_diff( max30100_config_t* this,
                                                 uint32_t acceptable_intense_diff );

/**
 * @brief Seta a taxa em que a corrende do led vermelho pode ser ajustada.
 * 
 * @details This é setado para valor padão na inicialização. (recomendado)
 * A única maneira de mudar é chamando esta função.
 * 
 * @param this é o endereço da estrutura de configuração
 * @param red_current_adj_ms é o valor em milisegundos a ser setado.
 * 
 * @retorna estado de execução.
 */
esp_err_t max30100_set_red_current_adj_ms( max30100_config_t* this, 
                                           uint32_t red_current_adj_ms );

/**
 * @brief seta o número de pulsos em que os valores do spo2 são resetados.
 * 
 * @details This é setado para valor padão na inicialização. (recomendado)
 * A única maneira de mudar é chamando esta função.
 * 
 * @param this é o endereço da estrutura de configuração.
 * @param reset_spo2_pulse_n é o número de pulsos.
 * 
 * @retorna estado de execução.
 */
esp_err_t max30100_set_reset_spo2_pulse_n( max30100_config_t* this, 
                                           uint8_t reset_spo2_pulse_n);

/**
 * @brief Seta o valor alpha do filtro DC.
 * 
 * @details This é setado para valor padão na inicialização. (recomendado)
 * A única maneira de mudar é chamando esta função.
 * 
 * @param this é o endereço da estrutura de configuração
 * @param dc_alpha é valor alpha do filtro.
 * 
 * @retorna estado de execução.
 */                                           
esp_err_t max30100_set_dc_alpha(max30100_config_t* this, float dc_alpha );

/**
 * @brief Seta o limite mínimo para o bpm.
 * 
  * @details This é setado para valor padão na inicialização. (recomendado)
 * A única maneira de mudar é chamando esta função.
 * 
 * @param this é o endereço da estrutura de configuração
 * Você não pode simplesmente por valores aleatórios nesses 2.
 * Checar tabela 8 na datasheet página 19.
 * você pode setar 300 para o dedo ou 20 para pulso (com muito ruído).
 * 
 * @param pulse_min_threshold é o valor.
 * 
 * @retorna estado de execução.
 */
esp_err_t max30100_set_pulse_min_threshold( max30100_config_t* this, 
                                            uint16_t pulse_min_threshold );

/**
 * @brief Seta limite máximo para bpm.
 * 
   * @details This é setado para valor padão na inicialização. (recomendado)
 * A única maneira de mudar é chamando esta função.
 * 
 * @param this é o endereço da estrutura de configuração
 * Você não pode simplesmente por valores aleatórios nesses 2.
 * Checar tabela 8 na datasheet página 19.
 * @param pulse_max_threshold é o valor.
 * 
 * @retorna estado de execução.
 */
esp_err_t max30100_set_pulse_max_threshold( max30100_config_t* this, 
                                            uint16_t pulse_max_threshold );

#endif

/**
 * MIT License
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:

 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.

 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
*/
uint8_t max30100_device_address;
uint8_t buffer[14];

void max30100_set_shtd();
uint8_t max30100_get_shtd(); 