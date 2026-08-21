#include "voltage.h"
#include "main.h"
#include "adc.h"
#include "dma.h"
#include <stdio.h>

#define size 100

extern DMA_HandleTypeDef hdma_adc1;

static uint32_t adc1_in17 = 0;
static uint32_t adc1_in19 = 0;
static uint32_t adc1_buf[size] = {0};

void adc_average(void)
{
    uint32_t _adc1_in17 = 0;
    uint32_t _adc1_in19 = 0;

    uint32_t i=0;

    for(i=0;i<size;i++){
        if(i%2){
            _adc1_in17 += adc1_buf[i];
        }else{
            _adc1_in19 += adc1_buf[i];
        }
    }

    i = size/2;
    adc1_in17 = _adc1_in17/i;
    adc1_in19 = _adc1_in19/i;
}

uint32_t count;
/*ADC DMA传输回调函数*/
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    if(hadc==&hadc1){
        // count++;
        // printf("count: %d\r\n", count);
       adc_average();
    }
}


/*电压采集初始化*/
void Voltage_Init(void)
{
    HAL_Delay(100);

    /*必须在停止状态进行校准*/
    HAL_ADCEx_Calibration_Start(&hadc1, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED);


    if(HAL_OK != HAL_ADC_Start_DMA(&hadc1, adc1_buf, size)){
        printf("HAL_ADC_Start_DMA: NO\r\n");
        while(1);
    }




}

static uint32_t get_adc_average(ADC_HandleTypeDef* adc, uint8_t times)
{
    uint8_t i;
    uint32_t temp_val = 0;

    if(times>=size){
        times = size;
    }
    for(i=0;i<times;i++){
        temp_val += adc1_buf[i];
        //HAL_Delay(5);
    }
    return temp_val/times;
}

float get_voltage(void)
{
    float temp= get_adc_average(&hadc1, 100);

    temp = (temp/65536.0f)*3.3f;

	return temp;
}

void print_adc(void)
{
    printf("adc1_in17: %.4f    ", ((float)adc1_in17/65536.0f)*3.3f);
    printf("adc1_in19: %.4f\r\n", ((float)adc1_in19/65536.0f)*3.3f);
}



