/**
 * @file registers.h
 * 
 * @author
 * Angelo Elias Dalzotto (150633@upf.br)
 * GEPID - Grupo de Pesquisa em Cultura Digital (http://gepid.upf.br/)
 * Universidade de Passo Fundo (http://www.upf.br/)
 * 
 * @copyright
 * Copyright (c) 2017 Raivis Strogonovs (https://morf.lv)
 * 
 * @brief This is the "private" headers file for the MAX30100 ESP32 library.
 * Please do NOT include in your code.
*/

#ifndef MAX30100_REGISTERS_H
#define MAX30100_REGISTERS_H

#include "max30100.h"

/**
 * MAX30100 definições de pinos internos
 */
#define MAX30100_DEVICE                   0x57
#define MAX30100_REV_ID                   0xFE
#define MAX30100_PART_ID                  0xFF
#define MAX30100_INT_STATUS               0x00
#define MAX30100_INT_ENABLE               0x01
#define MAX30100_FIFO_WRITE               0x02
#define MAX30100_FIFO_OVERFLOW_COUNTER    0x03
#define MAX30100_FIFO_READ                0x04
#define MAX30100_FIFO_DATA                0x05
#define MAX30100_MODE_CONF                0x06
#define MAX30100_SPO2_CONF                0x07
#define MAX30100_LED_CONF                 0x09
#define MAX30100_TEMP_INT                 0x16
#define MAX30100_TEMP_FRACTION            0x17

/**
 * definições de pinos para configurações de modo
 */
#define MAX30100_MODE_SHDN                (1<<7) 
#define MAX30100_MODE_RESET               (1<<6)
#define MAX30100_MODE_TEMP_EN             (1<<3)
#define MAX30100_SPO2_HI_RES_EN           (1<<6)

/**
 * máquina de estado de pulso
 */
typedef enum _pulse_state_machine {
    MAX30100_PULSE_IDLE,
    MAX30100_PULSE_TRACE_UP,
    MAX30100_PULSE_TRACE_DOWN
} pulse_state_machine;

/**
 * declarações de funções privadas
 * Estas funções irão ser chamadas somente pela bilbioteca, não pelo usuário
 */

/**
 * @algoritmo de detecção de pulso
 * 
 * @detalhes: chamadas na função de atualização
 * 
 * @param este é o endereço da estrutura de configuração.
 * @param sensor_value é o valor lido do sensor depois dos filtros
 * 
 * @retorna verdadeiro se detectado.
 */
bool max30100_detect_pulse(max30100_config_t* this, float sensor_value);

/**
 * @brief filtro de equilíbrio de intensidade.
 * 
 * @details filtro DC para valores brutos.
 * 
 * @param este é o endereço da estrutura de configuração
 * @param red_dc é o w na estrutura do filtro de led vermelho
 * @param é o w na estrutura do filtro de led infra-vermelho
 * 
 * @restorna estado de execução.
 */
esp_err_t max30100_balance_intensities(max30100_config_t* this, float red_dc, float ir_dc);

/**
 * @brief Escrita no registrador MAX30100.
 * 
 * @param este é o endereço da estrutura de configuração.
 * @param address é o endereço do registrador.
 * @param val é o byte a ser escrito.
 * 
 * @retorna estado de excução.
 */
esp_err_t max30100_write_register(max30100_config_t* this, uint8_t address, uint8_t val);

/**
 * @brief Leitura no registrador MAX30100.
 * 
 * @param este é o endereço da estrutura de configuração.
 * @param address é o endereço do registrador.
 * @param reg é o endereço do byte a ser lido.
 * 
 * @retorna estado de execução.
 */
esp_err_t max30100_read_register(max30100_config_t* this, uint8_t address, uint8_t* reg);

/**
 * @brief Read set of MAX30100 registers.
 * 
 * @param este é o endereço da estrutura de configuração.
 * @param address é endereço do registrador.
 * @param data é o endereço inicial a ser salvo.
 * @param size é o tamanho do registrador a ser lido.
 * 
 * @returns status of execution.
 */
esp_err_t max30100_read_from( max30100_config_t* this, uint8_t address, 
                              uint8_t* data, uint8_t size );

/**
 * @brief Read from MAX30100 FIFO.
 * 
 * @param este é o endereço da estrutura de configuração.
 * @param fifo é o endereço de uma fifo.
 * 
 * @retorna estado de execução.
 */
esp_err_t max30100_read_fifo(max30100_config_t* this, max30100_fifo_t* fifo);

/**
 * @brief remove o deslocamento DC do valor do sensor
 * 
 * @param este é o endereço da estrutura de configuração.
 * @param x é o valor bruto lido do led.
 * @param prev_w é o w filtrado anteriormente da estrutura de filtro dc.
 * @param alpha é o parametro alpha do filtro dc.
 * 
 * @retorna uma estrutura de filtro dc.
 */
max30100_dc_filter_t max30100_dc_removal(float x, float prev_w, float alpha);

/**
 * @brief aplica o filtro diferencial médio
 * 
 * @param este é o endereço da estrutura de configuração.
 * @param M é o resultado DC filtrado da estrutura de filtro DC
 * 
 * @retorna um valor filtrado.
 */
float max30100_mean_diff(max30100_config_t* this, float M);

/**
 * @brief aplica um filtro butterworth passa-baixas
 * 
 * @param this é o endereço da estrutura de configuração
 * @param x é o valor do filtro diferencial médio.
 */
void max30100_lpb_filter(max30100_config_t* this, float x);

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
 * 

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
