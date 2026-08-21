#include "icm42688p.h"

#include <spi.h>



static void cs(ICM42688Dev_t *dev, uint8_t sta)
{
    if(NULL==dev)return;

    if(sta){
        HAL_GPIO_WritePin(dev->cs_port, dev->cs_pin, GPIO_PIN_SET);
    }else{
        HAL_GPIO_WritePin(dev->cs_port, dev->cs_pin, GPIO_PIN_RESET);
    }
}

static int read_reg(uint8_t reg, uint8_t *data, uint32_t timeout)
{
    void *func_addr = (void*)read_reg;
    ICM42688Dev_t *dev = container_of()
}
















