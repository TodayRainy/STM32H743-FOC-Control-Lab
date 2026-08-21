#include "adc_dma.hpp"

#include <stdio.h>


adc_dma::adc_dma(ADC_HandleTypeDef *hadc, void *dma_buf, uint32_t dma_cn, uint32_t cn_len)
{
    this->hadc      = hadc;
    this->dma_buf   = dma_buf;
    this->dma_cn    = dma_cn;
    this->cn_len    = cn_len;

    switch (hadc->Init.Resolution){
        case ADC_RESOLUTION_8B:
            ad_max = (1UL<<8)-1;
            break;
        case ADC_RESOLUTION_10B:
            ad_max = (1UL<<10)-1;
            break;
        case ADC_RESOLUTION_12B:
            ad_max = (1UL<<12)-1;
            break;
        case ADC_RESOLUTION_14B:
            ad_max = (1UL<<14)-1;
            break;
        case ADC_RESOLUTION_16B:
            ad_max = (1UL<<16)-1;
            break;
        default:
            ad_max = 1;
            break;
    }

    dma_val = new uint32_t[dma_cn];

    HAL_ADCEx_Calibration_Start(hadc, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED);

    start();
}

adc_dma::~adc_dma()
{
    stop();
    delete(dma_val);
}

void adc_dma::start(void)
{
    /* 启动DMA转换 */
    HAL_ADC_Start_DMA(hadc, (uint32_t *)dma_buf, dma_cn * cn_len);
    // HAL_ADC_Start_IT(&hadc3);    /* 启动DMA转换 */
}

void adc_dma::stop(void)
{
    HAL_ADC_Stop_DMA(hadc);
}

void adc_dma::calculate(void)
{
    uint16_t *p = (uint16_t*)dma_buf;

    for(int i=0; i<dma_cn; i++){
        uint32_t tmp = 0;
        for(int j=0; j<cn_len; j++){
            tmp += p[j*dma_cn+i];
        }
        dma_val[i] = tmp/cn_len;
    }

    get_vbus();
    get_ibus();
    get_temp();
    get_pot();
}


void adc_dma::get_ibus(void)
{

}

void adc_dma::get_vbus(void)
{
    uint32_t offset = -0;       /*手动测试的零偏*/

    vbus = (float)(dma_val[0]+offset)/ad_max*3.3f*11.0f;
}

void adc_dma::get_temp(void)
{

}



void adc_dma::get_pot(void)
{

    adc_pot = dma_val[1]/ad_max*100.0f;
}















