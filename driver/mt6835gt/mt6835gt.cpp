#include "mt6835gt.hpp"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#pragma arm section code  = "RAMCODE"

// #pragma arm section

#define _2PI   6.283185307179586f      /*2x圆周率*/

#define SPI_DMA_ENABLE          0

mt6835gt::mt6835gt(SPI_HandleTypeDef*p_hspi, GPIO_TypeDef *csPort, uint16_t csPin)
{
    /*补齐漏初始化的成员: 对象由 new 分配, C++ 不会自动清零*/
    this->dir_count    = 0;
    this->angle        = 0;
    this->last_angle   = 0;
    this->OverSpeed    = 0;
    this->WeakField    = 0;
    this->UnderVoltage = 0;

    this->p_hspi    = p_hspi;
    this->csPort    = csPort;
    this->csPin     = csPin;

    memset(tx_burst, 0x00, sizeof(tx_burst));
    uint16_t reg = MT6835_REG_ANGLE3;
    reg &= 0xFFF; 
    reg |= MT6835_CMD_BURST;
    tx_burst[0] = (reg>>8)&0xFF;
    tx_burst[1] = (reg>>0)&0xFF;

    // write_reg(MT6835_REG_ID, 0x18);
    // write_zero();
    // write_eeprom();
    // HAL_Delay(500);

    

    uint8_t rx = 0;
    read_reg(MT6835_REG_ID, rx);

    printf("MT6835_REG_ID: 0X%X\r\n", rx);    
}

mt6835gt::~mt6835gt()
{
}

void mt6835gt::Angle_Deal(uint8_t rx[6])
{
    uint32_t code = 0;
    code = (rx[2]<<16|rx[3]<<8|rx[4])>>3;

    OverSpeed = rx[4]&0x01;
    WeakField = rx[4]&0x02;
    UnderVoltage = rx[4]&0x04;

    angle = (float)code /2097152.0f *_2PI;
    
    /*计算方向*/
    if(last_angle<=angle){
        dir_count++;
    }else{
        dir_count--;
    }
    if(dir_count>5)dir_count=5;
    if(dir_count<-5)dir_count=-5;
    last_angle = angle;
}



int mt6835gt::read_reg(uint32_t reg, uint8_t &data, uint32_t timeout)
{
    int ret = 0;
    uint8_t tx[3] = {0};
    uint8_t rx[3] = {0};

    cs(1);
    reg &= 0xFFF; 
    reg |= MT6835_CMD_RD;

    tx[0] = (reg>>8)&0xFF;
    tx[1] = (reg>>0)&0xFF;

    cs(0);
    ret = HAL_SPI_TransmitReceive(p_hspi, tx, rx, 3, timeout);
    cs(1);

    data = rx[2];

    return ret;
}

int mt6835gt::write_reg(uint32_t reg, uint8_t data, uint32_t timeout)
{
    int ret = 0;
    uint8_t tx[3] = {0};
    uint8_t rx[3] = {0};

    cs(1);
    reg &= 0xFFF; 
    reg |= MT6835_CMD_WR;

    tx[0] = (reg>>8)&0xFF;
    tx[1] = (reg>>0)&0xFF;
    tx[2] = data;

    cs(0);
    ret = HAL_SPI_TransmitReceive(p_hspi, tx, rx, 3, timeout);
    cs(1);
    return ret;
}

int mt6835gt::write_zero(void)
{
    int ret = 0;
    uint16_t reg = 0;
    uint8_t tx[3] = {0};
    uint8_t rx[3] = {0};

    printf("Writing zero position... ");

    cs(1);
    reg &= 0xFFF; 
    reg |= MT6835_CMD_ZERO;

    tx[0] = (reg>>8)&0xFF;
    tx[1] = (reg>>0)&0xFF;

    cs(0);
    ret = HAL_SPI_TransmitReceive(p_hspi, tx, rx, 3, HAL_MAX_DELAY);
    cs(1);

    if(0x55==rx[2]){
        printf("完成\r\n");
    }else{
        printf("失败\r\n");
        ret = HAL_ERROR;
    }

    return ret;
}

int mt6835gt::write_eeprom(void)
{
    int ret = 0;
    uint16_t reg = 0;
    uint8_t tx[3] = {0};
    uint8_t rx[3] = {0};

    printf("Writing EEPROM... ");
    HAL_Delay(100);

    cs(1);
    reg &= 0xFFF; 
    reg |= MT6835_CMD_EEPROM;

    tx[0] = (reg>>8)&0xFF;
    tx[1] = (reg>>0)&0xFF;

    cs(0);
    ret = HAL_SPI_TransmitReceive(p_hspi, tx, rx, 3, HAL_MAX_DELAY);
    cs(1);

    HAL_Delay(10000);
    if(0x55==rx[2]){
        printf("完成\r\n");
    }else{
        printf("失败\r\n");
        ret = HAL_ERROR;
    }
    
    return ret;
}

int mt6835gt::read_angle(float &angle)
{
    #if SPI_DMA_ENABLE
    
    Angle_Deal(rx_burst);
    angle = this->angle;
    read_angle_dma_tx();
    return 0;

    #else

    int ret = 0;
    cs(1);
    cs(0);
    ret = HAL_SPI_TransmitReceive(p_hspi, tx_burst, rx_burst, 6, HAL_MAX_DELAY);
    cs(1);
    Angle_Deal(rx_burst);
    angle = this->angle;

    return ret;
    #endif
}

bool mt6835gt::read_dir(void)
{
    return dir_count>0?true:false;
}


#if SPI_DMA_ENABLE

int mt6835gt::read_angle_dma_tx(void)
{
    int ret = 0;
    cs(1);
    cs(0);
    ret = HAL_SPI_TransmitReceive_DMA(p_hspi, tx_burst, rx_burst, 6);

    return ret;
}
#endif


void mt6835gt::read_angle_dma_rx(void)
{
    cs(1);
}


// #pragma arm section code  = "RAMCODE"

#pragma arm section
