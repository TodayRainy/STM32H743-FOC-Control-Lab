#ifndef _MT6701_ACD_HPP
#define _MT6701_ACD_HPP


#include "i2c.h"
#include <stdio.h>


#define MT6701ACD_I2CADDR       0x06
#define MT6701ACD_ANGLE0        0x03
#define MT6701ACD_ANGLE1        0x04

class mt6701acd
{
private:
    /* data */

    uint8_t addr;
    I2C_HandleTypeDef *p_hi2c;

    float   angle;
    uint8_t rx_burst[2];                     /*连续读取角度的接受缓冲区*/

    void Angle_Deal(uint8_t rx[2]);

public:
    mt6701acd(I2C_HandleTypeDef *p_hi2c, uint8_t addr);
    ~mt6701acd();


    int read_reg(uint8_t reg, uint8_t &data, uint32_t timeout = HAL_MAX_DELAY);

    int read_angle(float &angle);

};


































#endif
