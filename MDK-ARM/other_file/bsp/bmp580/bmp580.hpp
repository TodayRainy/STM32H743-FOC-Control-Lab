#ifndef _BMP580_HPP
#define _BMP580_HPP

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



class bmp580
{
private:
    /* data */

    bool flag;
    uint8_t state;                      /*状态机: 0 开始校准； 1 校准中； 2 校准完成*/

    uint8_t addr;                       /* 0x47 */
    I2C_HandleTypeDef *p_hi2c;

    /*校准相关*/
    int16_t air_raw_bias;
    float air_scale;
    int32_t air_raw_sum;
    int32_t sample_count;

    void config(void);

    int read_reg(uint8_t addr, uint8_t reg, uint8_t &data, uint32_t timeout = HAL_MAX_DELAY);
    int read_reg(uint8_t addr, uint8_t reg, uint8_t *data, uint32_t len, uint32_t timeout = HAL_MAX_DELAY);
    int read_reg(uint8_t reg, uint8_t &data, uint32_t timeout = HAL_MAX_DELAY);
    int read_reg(uint8_t reg, uint8_t *data, uint32_t len, uint32_t timeout = HAL_MAX_DELAY);

    int write_reg(uint8_t reg, uint8_t data, uint32_t timeout = HAL_MAX_DELAY);
    int write_reg(uint8_t addr, uint8_t reg, uint8_t data, uint32_t timeout = HAL_MAX_DELAY);

public:
    bmp580(I2C_HandleTypeDef *p_hi2c, uint8_t addr);
    ~bmp580();


    

};
















































#endif


#endif


