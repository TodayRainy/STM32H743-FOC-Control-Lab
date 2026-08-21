#ifndef _QMC5883P_HPP
#define _QMC5883P_HPP


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

/*芯片寄存器列表*/
#define QMC5883P_CHIPID                     0x00        /*芯片ID*/
#define QMC5883P_X_LSB                      0x01        /*X轴数据*/
#define QMC5883P_X_MSB                      0x02        
#define QMC5883P_Y_LSB                      0x03        /*Y轴数据*/
#define QMC5883P_Y_MSB                      0x04
#define QMC5883P_Z_LSB                      0x05        /*Z轴数据*/
#define QMC5883P_Z_MSB                      0x06
#define QMC5883P_STATE                      0x09        /*状态寄存器*/
#define QMC5883P_CONTROL0                   0x0A        /*控制寄存器*/
#define QMC5883P_CONTROL1                   0x0B



class qmc5883p
{
private:
    /* data */
    bool flag;
    uint8_t addr;
    I2C_HandleTypeDef *p_hi2c;



    void config(void);



public:
    qmc5883p(I2C_HandleTypeDef *p_hi2c, uint8_t addr);
    ~qmc5883p();

    int read_reg(uint8_t reg, uint8_t &data, uint32_t timeout = HAL_MAX_DELAY);

    int read_reg(uint8_t reg, uint8_t *data, uint32_t len, uint32_t timeout = HAL_MAX_DELAY);

    int read_id(uint8_t &data, uint32_t timeout = HAL_MAX_DELAY);

    int write_reg(uint8_t reg, uint8_t data, uint32_t timeout = HAL_MAX_DELAY);

    int get_mag(float *mag_x, float *mag_y, float *mag_z);

    /* 获取磁力计原始值 */
    int get_mag(int16_t *mag_x, int16_t *mag_y, int16_t *mag_z);


    void init_reference_heading();
    float get_relative_yaw();

    int compassUpdate(float *mag_x, float *mag_y, float *mag_z);


};


#endif



























#endif
