#include "temperature.h"
#include "main.h"
#include "adc.h"

uint32_t buf[100] = {0};

void Temperature_Init(void)
{
    HAL_Delay(100);

    HAL_ADCEx_Calibration_Start(&hadc3, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED); //ADC校准

    //HAL_ADC_Start(&hadc3);

    //HAL_ADC_PollForConversion(&hadc3,10);                		//轮询转换

    HAL_ADC_Start_DMA(&hadc3, buf, 100);

}

uint32_t Get_Adc(ADC_HandleTypeDef *adc)
{

    // HAL_ADC_Start(adc);
    // HAL_ADC_PollForConversion(adc,10);                		//轮询转换
    return HAL_ADC_GetValue(adc);
}

uint32_t get_adc_average(ADC_HandleTypeDef* adc, uint8_t times)
{
    uint8_t i;
    uint32_t temp_val = 0;

    for(i=0;i<times;i++){
        temp_val += buf[i];
        //HAL_Delay(5);
    }

    return temp_val/times;
}

float get_mcu_temp_sensor(void)
{
    uint32_t adcx;
	  float result;

    float temp=0;
    uint16_t ts_cal1,ts_cal2;
    
    ts_cal1=*(volatile uint16_t*)(0X1FF1E820);
    ts_cal2=*(volatile uint16_t*)(0X1FF1E840);
    temp=(float)((110.0f-30.0f)/(ts_cal2-ts_cal1));

    adcx = get_adc_average(&hadc3, 100);
    result = (float)(temp*(adcx-ts_cal1)+30);
	//result=temperate*=100;					//扩大100倍.
	return result;
}























