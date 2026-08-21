#ifndef _ADC_DMA_HPP
#define _ADC_DMA_HPP





#include <stdint.h>
#include <adc.h>

class adc_dma
{
private:
    /* data */
    uint32_t  dma_cn;
    uint32_t cn_len;

    void * dma_buf;
    uint32_t *dma_val;

    float ad_max;
    ADC_HandleTypeDef *hadc;

    

public:
    adc_dma(ADC_HandleTypeDef *hadc, void *dma_buf, uint32_t dma_cn, uint32_t cn_len);
    ~adc_dma();

    void start(void);
    void stop(void);
    void calculate(void);
    
    
    float ibus;
    float vbus;
    float temp;                 /*温度*/

    float R1,R2;
    float adc_pot;

    void get_ibus(void);        /*获取母线电流*/
    void get_vbus(void);        /*获取母线电压*/
    void get_temp(void);        /*获取温度*/
    void get_pot(void);         /*获取电位*/

};






































#endif






