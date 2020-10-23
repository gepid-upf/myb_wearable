#include <stdio.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/adc.h"
#include "esp_adc_cal.h"
#include "headermain.h"

static esp_adc_cal_characteristics_t *adc_chars;
static const adc_channel_t channel = ADC_CHANNEL_6; //GPIO34 if ADC1, GPIO14 if ADC2
static const adc_atten_t atten = ADC_ATTEN_DB_11;
static const adc_unit_t unit = ADC_UNIT_1;

uint8_t def_bpm_array[BPM_DEF_SIZE];
uint16_t idx_def_bpm_array = 0;
bool first_time = false;

int32_t module(int32_t value)
{
    return value > 0 ? value : value * -1;
}

void task_sen0203(char *beatstep)
{
    //Configure ADC
    if (unit == ADC_UNIT_1)
    {
        adc1_config_width(ADC_WIDTH_BIT_12);
        adc1_config_channel_atten(channel, atten);
    }

    //Characterize ADC
    adc_chars = calloc(1, sizeof(esp_adc_cal_characteristics_t));
    esp_adc_cal_value_t val_type = esp_adc_cal_characterize(unit, atten, ADC_WIDTH_BIT_12, DEFAULT_VREF, adc_chars);

    float bpm_med_array[BPM_MED_SIZE];
    char aux_beats[bpmdigs];

    float bpm = 0.0;
    float last_bpm = 0.0;
    //float soma_beats = 0.0;
    float bpm_med = 0;
    float aux = 0;
    char temp1;
    char temp2;
    
    uint32_t current_time = 0;
    uint32_t up_time = 0;
    uint32_t diff = 0;
    uint32_t time_now = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);

    uint8_t conta_desceu = 0;
    uint8_t cont_med = 0;
    uint8_t cont_val = 0;
    uint8_t samples_bpm = 0;
    uint8_t bpm_counter = 0;

    uint32_t of_all_time = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);

    int32_t adc_reading = 0;
    int32_t last_adc = 0;
    int32_t variation = 0;
    int32_t last_variation = 0;
    int32_t diff_variation = 0;
    int32_t last_diff_variation = 0;

    bool beet = false;
    bool last_beet = false;
    bool went_up = false;
    bool last_went_up = false;
    bool unlock_beet = true;
    bool flag1 = 0;
    bool flag2 = 0;

    while (1)
    {
        //coleta amostras e faz média
        for (int i = 0; i < N_SAMPLES_ADC; i++)
        {
            adc_reading += adc1_get_raw((adc1_channel_t)channel);
            vTaskDelay(0);
        }
        adc_reading /= N_SAMPLES_ADC;

        //variation = erro entre amostra atual e anterior
        variation = adc_reading - last_adc;
        //diff_variation = erro do erro 
        diff_variation = variation - last_variation;

        //up
        if (adc_reading > (last_adc + last_adc * ERROR))
        {
            //se subir uma vez, então subiu, se subir duas vezes ou mais, desceu,
            //a menos que tenha descido na ultima leitura
            went_up = true;

            if(flag2){
                flag1 = 0;
                flag2 = 0;
                conta_desceu = 0;
            }

            if(flag1)
                flag2=1;

            if (last_went_up)
                went_up = false;
        }
        //down
        if (adc_reading < (last_adc - last_adc * ERROR))
        {
            //se descer uma vez, desceu
            unlock_beet = true;
            went_up = false;
            conta_desceu++;
            flag1=1;
        }

        if (went_up && unlock_beet)
        {
            //se desceu e depois subiu, habilita o beet
            unlock_beet = false;
            beet = true;
        }

        if (module(diff_variation) > 36)//desabilita beet se variação entre últimos 3 valores for maior q 25
            beet = false;

        if (variation > 36)//desabilitia beet se variação entre últimos 2 valores for maior que 25
            beet = false;

        if (adc_reading < 330)//desabilita beet se a média de sinal bruto for menor que 330
            beet = false;

        if (adc_reading > 1900)//desabilita beet se a média de sinal bruto for maior que 1900
            beet = false;

        if (module(diff_variation - last_diff_variation) > 50)//desabilita beet se variação entre últimos valores for maior que 30
            beet = false;//talvez aqui já seja um exagero...

        // if(conta_desceu < 3)//desabilita beet se desceu menos de três vezes antes de subir
        //     beet = false;

        if (!last_beet && beet)//se a última leitura não for um batimento, e a atual for, detecta pulso
        {
            flag1 = 0;
            flag2 = 0;
            conta_desceu = 0;

            printf("###################DETECTED###################\n");
            bpm_counter++;
            //up_time = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
            if (!first_time) //se for primeira vez detectando beat
            {
                first_time = true;

                for(int i=0; i<BPM_MED_SIZE; i++)
                {
                    bpm_med_array[i]=0;
                }
            }
        }

        //10 segundos...
        if((uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS) >= (time_now + PERIOD_CALC_MED))
        {
            if(bpm_counter > 0){
                bpm = (bpm_counter*(60.0/(PERIOD_CALC_MED/1000.0)));
                // Quantas vezes em X segundos?
                // 60 Dividido por X multiplicando esse número...
                // ...igual quantas vez por minuto!
            }
            bpm_counter = 0;

            if (bpm < 50.0 || bpm > 170.0)
                    bpm = 0;
            
            if(bpm != 0)//daria pra testar só receber valor se for diferente do último, por exemplo...
            {
                bpm_med_array[cont_med] = bpm;

                sprintf(aux_beats, "%d", ((int)bpm));
                
                //adiciona um '0' se for menor q 100
                if(bpm < 100){
                    aux_beats[2] = aux_beats[1];
                    aux_beats[1] = aux_beats[0];
                    aux_beats[0] = '0';
                    
                }
                aux_beats[3] = ' ';

                //printf("%s %f\n", aux_beats, bpm);
                int idx;
                if(beatstep[beatstep_bits]=='0')
                {
                    beatstep[beatstep_bits]='1';
                    for(int x = stepdigs, idx = 0; x < (stepdigs+bpmdigs-1); x++, idx++)
                    {
                        //if(atoi(aux_beats[idx]))
                        beatstep[x] = aux_beats[idx];
                    }
                    beatstep[beatstep_bits]='0';
                }

            }
            
            cont_med++;
            if(cont_med >= BPM_MED_SIZE) cont_med = 0;

            time_now = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
        }

        //2 minutos...
        if((uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS) >= (of_all_time + PERIOD_SEND_ARRAY))
        {
            
            //ve quantos valores tem
            for(int i=0; i<BPM_MED_SIZE; i++)
            {
                if(bpm_med_array[i])
                {                 
                    cont_val++;
                    //soma_beats += bpm_med_array[i];
                }
                //bpm_med_array[i] = 0;
            }

            if(cont_val != 0)
            {
                //ordena o vetor
                for(int j=0; j<cont_val; j++)
                {
                    if(cont_val != 1)
                    {
                        for(int k=j+1; k<cont_val; k++)
                        {
                            if(bpm_med_array[j] > bpm_med_array[k])
                            {
                                aux = bpm_med_array[j];
                                bpm_med_array[j] = bpm_med_array[k];
                                bpm_med_array[k] = aux;
                            }
                        }

                    }                    
                }

                //faz mediana
                if(cont_val%2==0)//par ou impar
                {
                    bpm_med = (bpm_med_array[cont_val/2 - 1] + bpm_med_array[cont_val/2])/2.0;
                }else{
                    bpm_med = bpm_med_array[cont_val/2];
                }
            

                //bpm_med = (float)(soma_beats/cont_val);
                //printf("%d %d\n", (int)bpm_med, cont_val);
                //se valor válido, registra no array
                if(idx_def_bpm_array < BPM_DEF_SIZE)
                {
                    def_bpm_array[idx_def_bpm_array] = (int)bpm_med;
                    idx_def_bpm_array++;
                }

                //reseta o vetor de médias
                for(int z=0; z<BPM_MED_SIZE; z++)
                {
                    bpm_med_array[z] = 0;
                }

                cont_med = 0;
                cont_val = 0;
                //soma_beats = 0;
            }
            of_all_time = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
        }

        //mostra amostra atual, detecção de pulso no plotter, e heart rate (ou bpm)
        printf("%d %d %f\n", adc_reading, (beet && !last_beet) ? 670 : 570, bpm);

        //atualiza valores
        last_went_up = went_up;
        last_diff_variation = diff_variation;
        last_variation = variation;
        last_beet = beet;
        beet = 0;
        last_adc = adc_reading;

        if (bpm != 0 && bpm != last_bpm)
        {
            last_bpm = bpm;
        }

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}
