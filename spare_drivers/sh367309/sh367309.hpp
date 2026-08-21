#ifndef _SH367309_HPP
#define _SH367309_HPP


#ifdef __cplusplus
extern "C" {  // 告诉 C++ 编译器：这部分用 C 的方式编译
#endif

#include <stdint.h>
#include <gpio.h>
#include <sys.h>
#include <i2c.h>



#ifdef __cplusplus
}
#endif



#ifdef __cplusplus


#include "iic/soft_i2c.hpp"

class sh367309
{
private:
    /* data */
    bool flag;
    uint8_t addr;                       /* 0x1A */
    I2C_HandleTypeDef *p_hi2c;

    soft_i2c *soft_i2c_0;

    // int read_reg(uint8_t addr, uint8_t reg, uint8_t &data, uint32_t timeout = HAL_MAX_DELAY);
    // int read_reg(uint8_t addr, uint8_t reg, uint8_t *data, uint32_t len, uint32_t timeout = HAL_MAX_DELAY);
    // int read_reg(uint8_t reg, uint8_t &data, uint32_t timeout = HAL_MAX_DELAY);
    // int read_reg(uint8_t reg, uint8_t *data, uint32_t len, uint32_t timeout = HAL_MAX_DELAY);

    int write_reg_crc8(uint8_t reg, uint8_t data, uint32_t timeout = HAL_MAX_DELAY);
    int read_reg(uint8_t reg, uint8_t &data, uint32_t timeout = HAL_MAX_DELAY);
    // int write_reg_crc8(uint8_t addr, uint8_t reg, uint8_t data, uint32_t timeout = HAL_MAX_DELAY);

    int soft_write_reg_crc8(uint8_t reg, uint8_t data, uint32_t timeout = HAL_MAX_DELAY);

    int soft_read_reg(uint8_t reg, uint8_t &data, uint32_t timeout = HAL_MAX_DELAY);

    void config(void);

public:
    sh367309(I2C_HandleTypeDef *p_hi2c, uint8_t addr);
    ~sh367309();




};


























#endif




















#endif












