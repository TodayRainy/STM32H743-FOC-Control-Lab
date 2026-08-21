#ifndef _ICM20948_HPP
#define _ICM20948_HPP


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

/*icm20948 i2c地址 0x69  ak09916地址 0x0c*/

#define ICM20948_CALIB_COUNT            1000    /* 零偏校准次数 */

class icm20948
{
private:
    /* data */
    bool flag;
    uint8_t state;                      /*状态机: 0 开始校准； 1 校准中； 2 校准完成*/
    uint8_t addr;
    I2C_HandleTypeDef *p_hi2c;

    /* 加速度计和陀螺仪原始值偏移量 */
    int16_t acc_raw_bias[3];
    int16_t gyro_raw_bias[3];
    float gyro_scale;
    float acc_scale;

    /*校准相关*/
    int32_t gyro_raw_sum[3];      /*原始值累计*/
    int32_t acc_raw_sum[3];
    int32_t sample_count;

    
    /*通信协议读写接口*/
    int read_reg(uint8_t addr, uint8_t reg, uint8_t &data, uint32_t timeout = HAL_MAX_DELAY);
    int read_reg(uint8_t addr, uint8_t reg, uint8_t *data, uint32_t len, uint32_t timeout = HAL_MAX_DELAY);
    int read_reg(uint8_t reg, uint8_t &data, uint32_t timeout = HAL_MAX_DELAY);
    int read_reg(uint8_t reg, uint8_t *data, uint32_t len, uint32_t timeout = HAL_MAX_DELAY);

    int write_reg(uint8_t reg, uint8_t data, uint32_t timeout = HAL_MAX_DELAY);
    int write_reg(uint8_t addr, uint8_t reg, uint8_t data, uint32_t timeout = HAL_MAX_DELAY);

    void config(void);

    uint8_t read_int_status(void);        /*获取中断状态*/
    
public:
    icm20948(I2C_HandleTypeDef *p_hi2c, uint8_t addr);
    ~icm20948();

    /*读取陀螺仪 加速度计数据*/
    int read_gyro_accel(float gyro[3], float accel[3]);

    int read_mag(int16_t mag[3]);

    

};


























#endif

#endif
