#include "qmc5883p.hpp"

#include <bsp.hpp>

/*传感器方向*/
typedef enum 
{
    ALIGN_DEFAULT = 0, 
    CW0_DEG = 1,
    CW90_DEG = 2,
    CW180_DEG = 3,
    CW270_DEG = 4,
    CW0_DEG_FLIP = 5,
    CW90_DEG_FLIP = 6,
    CW180_DEG_FLIP = 7,
    CW270_DEG_FLIP = 8
} sensor_align_e;



qmc5883p::qmc5883p(I2C_HandleTypeDef *p_hi2c, uint8_t addr)
{
    this->flag = false;

    if(NULL==p_hi2c)return;

    this->p_hi2c    = p_hi2c;
    this->addr      = addr;
    this->flag = true;

    uint8_t id = 1;
    read_id(id);
    printf("qmc5883p id: 0x%02X\r\n", id);

    config();

    init_reference_heading();

}

qmc5883p::~qmc5883p()
{
}

/* 寄存器配置 */
void qmc5883p::config(void)
{
    int ret = 0;

    // ret |= write_reg(0x0d, 0x40);
    // HAL_Delay(50);

    // ret |= write_reg(0x29, 0x06);
    // HAL_Delay(50);

    /*连续模式 200Hz*/
    // ret |= write_reg(QMC5883P_CONTROL0, 0xCD);
    // HAL_Delay(50);


    // ret |= write_reg(QMC5883P_CONTROL1, 0x0C);
    // HAL_Delay(50);

    // ret |= write_reg(0x0d, 0x40);
    // HAL_Delay(50);

    /*Y Z轴方向取反*/
    ret |= write_reg(0x29, 0x06);
    HAL_Delay(50);

    /*设置量程 30Guass*/
    ret |= write_reg(QMC5883P_CONTROL1, 0x08);
    HAL_Delay(50);

    /*正常模式 ODR=200Hz*/
    ret |= write_reg(QMC5883P_CONTROL0, 0xCD);
    HAL_Delay(50);


    #if 1
    uint8_t data = 0;
    read_reg(0x0d, data);                       printf("0x0d:       0x%02X\r\n", data);
    read_reg(0x0e, data);                       printf("0x0e:       0x%02X\r\n", data);
    read_reg(0x0F, data);                       printf("0x0F:       0x%02X\r\n", data);

    read_reg(0x0d, data);                       printf("0x0d:       0x%02X\r\n", data);
    read_reg(0x29, data);                       printf("0x29:       0x%02X\r\n", data);
    read_reg(QMC5883P_CONTROL0, data);          printf("QMC5883P_CONTROL0:          0x%02X\r\n", data);
    read_reg(QMC5883P_CONTROL1, data);          printf("QMC5883P_CONTROL1:          0x%02X\r\n", data);
    #endif

    if(ret){
        printf("qmc5883p 配置: NO\r\n");
        while(1);
    }else{
        printf("qmc5883p 配置: OK\r\n");
    }

}

int qmc5883p::write_reg(uint8_t reg, uint8_t data, uint32_t timeout)
{
    int ret = 0;
    ret = HAL_I2C_Mem_Write(p_hi2c, (addr<<1), reg, I2C_MEMADD_SIZE_8BIT, &data, 1, timeout);
    return ret;
}

int qmc5883p::read_reg(uint8_t reg, uint8_t *data, uint32_t len, uint32_t timeout)
{
    int ret = 0;
    ret = HAL_I2C_Mem_Read(p_hi2c, (addr<<1)|0x01, reg, I2C_MEMADD_SIZE_8BIT, data, len, timeout);
    if(ret){
        printf("read_reg %d: NO: %d\r\n", reg, ret);
    }
    return ret;
}

int qmc5883p::read_reg(uint8_t reg, uint8_t &data, uint32_t timeout)
{
    int ret = 0;
    ret = HAL_I2C_Mem_Read(p_hi2c, (addr<<1)|0x01, reg, I2C_MEMADD_SIZE_8BIT, &data, 1, timeout);
    if(ret){
        printf("read_reg %d: NO: %d\r\n", reg, ret);
    }
    return ret;
}

int qmc5883p::read_id(uint8_t &data, uint32_t timeout)
{
    int ret = 0;

    if (HAL_I2C_IsDeviceReady(p_hi2c, (addr<<1), 2, 1000) == HAL_OK) {
        printf("qmc5883p Device found at i2c address: 0x%02X\n", addr);
    }

    ret = read_reg(0, data, timeout);
    if(ret){
        printf("read_id: NO: %d\r\n", ret);
    }
    return ret;
}

int qmc5883p::get_mag(float *mag_x, float *mag_y, float *mag_z)
{
    int ret;
    uint8_t raw[6]  = {};
    int16_t mag[3] = {};
    float mag_f[3]  = {};
    
    ret = read_reg(QMC5883P_STATE, raw, 1);
    if(ret)return ret;

    if(raw[0]&0x01==0)return -1;




    ret = read_reg(QMC5883P_X_LSB, raw, 6);
    if(ret==0){
        mag[0] = (raw[0]<<0) | (raw[1]<<8);
        mag[1] = (raw[2]<<0) | (raw[3]<<8);
        mag[2] = (raw[4]<<0) | (raw[5]<<8);

        mag_f[0] = mag[0];
        mag_f[1] = mag[1];
        mag_f[2] = mag[2];

        *mag_x = mag_f[0];
        *mag_y = mag_f[1];
        *mag_z = mag_f[2];
        // *mag_x = mag_f[1];
        // *mag_y = -mag_f[0];
        // *mag_z = mag_f[2];
    }else{


    }
    return ret;
}

int qmc5883p::get_mag(int16_t *mag_x, int16_t *mag_y, int16_t *mag_z)
{
    int ret;
    uint8_t raw[6]  = {};
    int16_t mag[3] = {};
    
    ret = read_reg(QMC5883P_STATE, raw, 1);
    if(ret)return ret;

    if((raw[0]&0x01)==0)return -1;

    ret = read_reg(QMC5883P_X_LSB, raw, 6);
    if(ret==0){
        mag[0] = (raw[0]<<0) | (raw[1]<<8);
        mag[1] = (raw[2]<<0) | (raw[3]<<8);
        mag[2] = (raw[4]<<0) | (raw[5]<<8);

        *mag_x = mag[0];
        *mag_y = mag[1];
        *mag_z = mag[2];
    }

    return ret;
}



float heading_0 = 0.0f;

// 返回单位：度，范围：0~360
float get_heading(float mag_x, float mag_y)
{
    float heading = atan2(mag_y, mag_x) * 180.0f / PI;
    if (heading < 0)
        heading += 360.0f;
    return heading;
}

void qmc5883p::init_reference_heading()
{
    float mag_x, mag_y, mag_z;

    if(get_mag(&mag_x, &mag_y, &mag_z) == 0){
        heading_0 = get_heading(mag_x, mag_y);
        // heading_0 = get_heading(mag_y, mag_x);
    }
}

float qmc5883p::get_relative_yaw()
{
    // float mag_x, mag_y, mag_z;
    // if(get_mag(&mag_x, &mag_y, &mag_z) == 0){
    //     float heading_now = get_heading(mag_x, mag_y);
    //     float yaw = heading_now - heading_0;

    //     // 归一化到 -180 ~ +180
    //     if(yaw > 180.0f)
    //         yaw -= 360.0f;
    //     else if(yaw < -180.0f)
    //         yaw += 360.0f;

    //     return yaw;
    // }
    // return 0.0f; // 读取失败返回0，可根据需求修改
    float mag_x, mag_y, mag_z;
    if (get_mag(&mag_x, &mag_y, &mag_z) == 0) {
        float heading_now = get_heading(mag_x, mag_y);
        float yaw = heading_now - heading_0;

        // 归一化到 0 ~ 360
        if (yaw < 0.0f)
            yaw += 360.0f;
        else if (yaw >= 360.0f)
            yaw -= 360.0f;

        return yaw;
    }
    return 0.0f;
}



































