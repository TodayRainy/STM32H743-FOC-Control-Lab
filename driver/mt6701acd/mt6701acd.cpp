#include "mt6701acd.hpp"


#define _2PI   6.283185307179586f      /*2x圆周率*/

mt6701acd::mt6701acd(I2C_HandleTypeDef *p_hi2c, uint8_t addr)
{
    /*补齐漏初始化的成员: 对象由 new 分配, C++ 不会自动清零*/
    this->angle = 0;

    this->p_hi2c    = p_hi2c;
    this->addr      = addr;

}

mt6701acd::~mt6701acd()
{
}

int mt6701acd::read_reg(uint8_t reg, uint8_t &data, uint32_t timeout)
{
    int ret = 0;
    ret = HAL_I2C_Mem_Read(p_hi2c, (addr<<1)|0x01, reg, I2C_MEMADD_SIZE_8BIT, &data, 1, timeout);
    if(ret){
        printf("read_reg %d: NO: %d\r\n", reg, ret);
    }
    return ret;
}

int mt6701acd::read_angle(float &angle)
{
    int ret = 0;

    ret |= read_reg(MT6701ACD_ANGLE0, rx_burst[0]);
    ret |= read_reg(MT6701ACD_ANGLE1, rx_burst[1]);

    Angle_Deal(rx_burst);

    angle = this->angle;

    return ret;
}

void mt6701acd::Angle_Deal(uint8_t rx[2])
{
    uint16_t code = 0;
    code = (rx[0]<<6) | (rx[1]>>2);

    angle = (float)code /16384.0f *_2PI;
}












