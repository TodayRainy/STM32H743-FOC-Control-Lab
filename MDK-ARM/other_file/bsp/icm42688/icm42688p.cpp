#include "icm42688p.hpp"

#include "bsp.hpp"





icm42688p::icm42688p(SPI_HandleTypeDef*p_hspi, GPIO_TypeDef *csPort, uint16_t csPin)
{
    this->flag = false;

    if(NULL==p_hspi || NULL==csPort)return;

    this->p_hspi    = p_hspi;
    this->csPort    = csPort;
    this->csPin     = csPin;
    this->flag = true;


    uint8_t id = 0;
    read_id(id);
    printf("icm42688p id: 0x%2X\r\n", id);

    gyro_fs_sel = GYRO_FS_2000DPS;
    acc_full = ACCEL_FS_16G;

    /*缩放系数计算*/
    scale_calculation();

    config();

    calibrate_bias(1000);

}

icm42688p::~icm42688p()
{
}

/* 缩放计算 */
void icm42688p::scale_calculation(void)
{
    switch (gyro_fs_sel){
        case GYRO_FS_2000DPS:
            gyro_scale = 2000.0f / 32768.0f;
            break;
        case GYRO_FS_1000DPS:
            gyro_scale = 1000.0f / 32768.0f;
            break;
        case GYRO_FS_500DPS:
            gyro_scale = 500.0f / 32768.0f;
            break;
        case GYRO_FS_250DPS:
            gyro_scale = 250.0f / 32768.0f;
            break;
        case GYRO_FS_125DPS:
            gyro_scale = 125.0f / 32768.0f;
            break;
        case GYRO_FS_62_5DPS:
            gyro_scale = 62.5f / 32768.0f;
            break;
        case GYRO_FS_31_25DPS:
            gyro_scale = 31.25f / 32768.0f;
            break;
        case GYRO_FS_15_625DPS:
            gyro_scale = 15.625f / 32768.0f;
            break;
        default:
            // 默认安全值，比如2000dps的缩放
            gyro_scale = 2000.0f / 32768.0f;
            break;
    }

    switch (acc_full) {
        case ACCEL_FS_16G:
            acc_scale = 16.0f / 32768.0f;
            break;
        case ACCEL_FS_8G:
            acc_scale = 8.0f / 32768.0f;
            break;
        case ACCEL_FS_4G:
            acc_scale = 4.0f / 32768.0f;
            break;
        case ACCEL_FS_2G:
            acc_scale = 2.0f / 32768.0f;
            break;
        default:
            acc_scale = 16.0f / 32768.0f;       // 默认16g
            break;
    }

}

/* 寄存器配置 */
void icm42688p::config(void)
{
    int ret = 0;
    uint8_t config_data = 0;

    /*系统自检*/
    // ret |= write_reg(ICM_SELF_TEST_CONFIG, 0xFF);    
    // HAL_Delay(50);

    config_data = 0; config_data = 0x00;
    ret |= write_reg(ICM_REG_BANK_SEL, config_data);           /* 切换回 Bank 0 */
    HAL_Delay(50);

    config_data = 0; config_data = 0x01;
    ret |= write_reg(ICM_DEVICE_CONFIG, config_data);          /* SPI模式0 软复位*/
    HAL_Delay(50);

    config_data = 0; config_data = 0x05;
    ret |= write_reg(ICM_DRIVE_CONFIG, config_data);           /* SPI_SLEW_RATE:101  I2C_SLEW_RATE:000 */
    HAL_Delay(50);

    config_data = 0; config_data = 0x0F;
    ret |= write_reg(ICM_PWR_MGMT0, config_data);              /* 功耗配置 加速度计:低噪声模式。陀螺仪:低噪声模式 温度传感器启用 */
    HAL_Delay(50);

    config_data = 0; config_data = 0x00;
    ret |= write_reg(ICM_INT_CONFIG1, config_data);            /* 中断1配置 */
    HAL_Delay(50);

    /*陀螺仪初始化*/
    config_data = 0; config_data = gyro_fs_sel|GYRO_ODR_32000HZ;
    ret |= write_reg(ICM_GYRO_CONFIG0, config_data);           /* 陀螺仪初始化 1khz ±2000dps*/
    HAL_Delay(50);

    config_data = 0; config_data = TEMP_FILT_BW_4000HZ | GYRO_UI_FILT_ORDER_1 | GYRO_DEC2_M2_RESERVED_0;
    ret |= write_reg(ICM_GYRO_CONFIG1, config_data);           /* 陀螺仪初始化 调整带宽和滤波次数  低延迟实时控制:0x22  噪声敏感、数据平滑优先: 0x6A   通用平衡:0x16*/
    HAL_Delay(50);

    config_data = 0; config_data = ACCEL_UI_FILT_BW_ODR_DIV_2 | GYRO_UI_FILT_BW_ODR_DIV_2;;
    ret |= write_reg(ICM_GYRO_ACCEL_CONFIG0, config_data);     /* 陀螺仪初始化 调整陀螺仪和加速度计的低通滤波器带宽 0x11:400HZ的带宽*/
    HAL_Delay(50);

    /*加速度计初始化*/
    config_data = 0; config_data = acc_full | ACCEL_ODR_1KHZ;
    ret |= write_reg(ICM_ACCEL_CONFIG0, config_data);          /* 加速度计初始化 500hz ±16g*/
    HAL_Delay(50);

    config_data = 0; config_data = ACCEL_UI_FILT_ORDER_1 | ACCEL_DEC2_M2_RESERVED_0;;
    ret |= write_reg(ICM_ACCEL_CONFIG1, config_data);          /* 加速度计初始化 调整带宽和滤波次数 0x0C*/
    HAL_Delay(50);

    config_data = 0; config_data = 0x00;
    ret |= write_reg(ICM_FIFO_CONFIG, config_data);            /* 禁用所有FIFO（因用户要求） */
    HAL_Delay(50);

    config_data = 0; config_data = 0x00;
    ret |= write_reg(ICM_INT_CONFIG, config_data);             /* 禁用所有中断*/
    HAL_Delay(50);
    
    config_data = 0; config_data = 0x08;
    ret |= write_reg(ICM_INT_SOURCE0, config_data);            /* 中断源配置 数据就绪中断*/
    HAL_Delay(50);


    #if 0
    uint8_t data = 0;
    read_reg(ICM_DRIVE_CONFIG, data);       printf("ICM_DRIVE_CONFIG:       0x%02X\r\n", data);
    read_reg(ICM_INT_CONFIG, data);         printf("ICM_INT_CONFIG:         0x%02X\r\n", data);
    read_reg(ICM_PWR_MGMT0, data);          printf("ICM_PWR_MGMT0:          0x%02X\r\n", data);
    read_reg(ICM_INT_CONFIG1, data);        printf("ICM_INT_CONFIG1:        0x%02X\r\n", data);
    read_reg(ICM_INT_SOURCE0, data);        printf("ICM_INT_SOURCE0:        0x%02X\r\n", data);
    read_reg(ICM_GYRO_CONFIG0, data);       printf("ICM_GYRO_CONFIG0:       0x%02X\r\n", data);
    read_reg(ICM_GYRO_CONFIG1, data);       printf("ICM_GYRO_CONFIG1:       0x%02X\r\n", data);
    read_reg(ICM_GYRO_ACCEL_CONFIG0, data); printf("ICM_GYRO_ACCEL_CONFIG0: 0x%02X\r\n", data);
    read_reg(ICM_ACCEL_CONFIG0, data);      printf("ICM_ACCEL_CONFIG0:      0x%02X\r\n", data);
    read_reg(ICM_ACCEL_CONFIG1, data);      printf("ICM_ACCEL_CONFIG1:      0x%02X\r\n", data);
    #endif

    if(ret){
        printf("ICM42688P 配置: NO\r\n");
        while(1);
    }else{
        printf("ICM42688P 配置: OK\r\n");
    }

}

/**
 * @brief 校准
 * 
 * @param sample_count 
 */
void icm42688p::calibrate_bias(int sample_count)
{

#if 0
    printf("正在进行传感器零偏校准...\r\n");

    float acc_sum[3] = {0};
    float gyro_sum[3] = {0};

    for (int i = 0; i < sample_count; ++i)
    {
        uint8_t rx[12] = {};
        int16_t gyro_raw[3] = {};
        int16_t accel_raw[3] = {};

        read_reg(ICM_ACCEL_DATA_X1, rx, 12);

        accel_raw[0] = (int16_t)(rx[0]<<8 | rx[1]);
        accel_raw[1] = (int16_t)(rx[2]<<8 | rx[3]);
        accel_raw[2] = (int16_t)(rx[4]<<8 | rx[5]);

        gyro_raw[0]  = (int16_t)(rx[6]<<8 | rx[7]);
        gyro_raw[1]  = (int16_t)(rx[8]<<8 | rx[9]);
        gyro_raw[2]  = (int16_t)(rx[10]<<8 | rx[11]);

        for (int j = 0; j < 3; ++j)
        {
            gyro_sum[j] += (float)gyro_raw[j]/32768.0f * 2000.0f;
            acc_sum[j]  += (float)accel_raw[j]/32768.0f * 16.0f * 9.8f;
        }

        HAL_Delay(1);  // 每次采样间隔 10ms，共约 1s
    }

    for (int i = 0; i < 3; ++i)
    {
        gyro_bias[i] = gyro_sum[i] / sample_count;
        acc_bias[i]  = acc_sum[i] / sample_count;
    }

    // Z 轴加速度应为 9.8，去除它的影响，只保留 X、Y 偏置
    acc_bias[2] -= 9.8f;

    printf("陀螺仪偏置: x: %.4f y: %.4f z: %.4f\r\n", gyro_bias[0], gyro_bias[1], gyro_bias[2]);
    printf("加速度偏置: x: %.4f y: %.4f z: %.4f\r\n", acc_bias[0], acc_bias[1], acc_bias[2]);
#else


    printf("正在进行传感器零偏校准...\r\n");

    int32_t gyro_raw_sum[3] = {0};
    int32_t acc_raw_sum[3]  = {0};

    int16_t gyro_raw[3] = {};
    int16_t accel_raw[3] = {};

    for (int i = 0; i < sample_count; ++i)
    {
        uint8_t rx[12] = {};
        

        read_reg(ICM_ACCEL_DATA_X1, rx, 12);

        accel_raw[0] = (int16_t)(rx[0]<<8 | rx[1]);
        accel_raw[1] = (int16_t)(rx[2]<<8 | rx[3]);
        accel_raw[2] = (int16_t)(rx[4]<<8 | rx[5]);

        gyro_raw[0]  = (int16_t)(rx[6]<<8 | rx[7]);
        gyro_raw[1]  = (int16_t)(rx[8]<<8 | rx[9]);
        gyro_raw[2]  = (int16_t)(rx[10]<<8 | rx[11]);

        for (int j = 0; j < 3; ++j)
        {
            gyro_raw_sum[j] += gyro_raw[j];
            acc_raw_sum[j]  += accel_raw[j];
        }
        HAL_Delay(1);  // 每次采样间隔 1ms，共约 1s
    }

    for (int i = 0; i < 3; ++i)
    {
        gyro_raw_bias[i] = gyro_raw_sum[i] / sample_count;
        acc_raw_bias[i]  = acc_raw_sum[i] / sample_count;
    }

    // Z 轴加速度应为 9.8，去除它的影响，只保留 X、Y 偏置
    acc_raw_bias[2] -=  1/acc_scale;

    printf("陀螺仪偏置: x: %d y: %d z: %d\r\n", gyro_raw_bias[0], gyro_raw_bias[1], gyro_raw_bias[2]);
    printf("加速度偏置: x: %d y: %d z: %d\r\n", acc_raw_bias[0], acc_raw_bias[1], acc_raw_bias[2]);


#endif

}




/**
 * @brief 读取寄存器
 * 
 * @param reg 
 * @param data 
 * @param timeout 
 * @return int 
 */
int icm42688p::read_reg(uint8_t reg, uint8_t &data, uint32_t timeout)
{
    if(false==flag)return -10;

    int ret = 0;

    uint8_t tx = reg | ICM42688P_READ_FLAG;

    cs(0);

    /*发送地址*/
    ret |= HAL_SPI_Transmit(p_hspi, &tx, 1, timeout);

    /*读取数据*/
    ret |= HAL_SPI_Receive(p_hspi, &data, 1, timeout);

    if(ret){
        printf("read_reg: NO\r\n");
    }
    cs(1);

    return ret;
}

/**
 * @brief 读取多个寄存器
 * 
 * @param reg 
 * @param data 
 * @param len 
 * @param timeout 
 * @return int 
 */
int icm42688p::read_reg(uint8_t reg, uint8_t *data, uint32_t len, uint32_t timeout)
{
    if(false==flag)return -10;

    int ret = 0;

    uint8_t tx = reg | ICM42688P_READ_FLAG;

    cs(0);

    /*发送地址*/
    ret |= HAL_SPI_Transmit(p_hspi, &tx, 1, timeout);

    /*读取数据*/
    ret |= HAL_SPI_Receive(p_hspi, data, len, timeout);

    if(ret){
        printf("read_reg: NO\r\n");
    }
    cs(1);

    return ret;
}

/**
 * @brief 读取芯片id
 * 
 * @param data 
 * @param timeout 
 * @return int 
 */
int icm42688p::read_id(uint8_t &data, uint32_t timeout)
{
    int ret = read_reg(WHO_AM_I_REG, data, timeout);

    return ret;
}

/**
 * @brief 写入寄存器
 * 
 * @param reg 
 * @param data 
 * @param timeout 
 * @return int 
 */
int icm42688p::write_reg(uint8_t reg, uint8_t data, uint32_t timeout)
{
    int ret = 0;
    if(false==flag)return -11;
    uint8_t tx[2] = {};

    tx[0] = reg;
    tx[1] = data;

    cs(0);
    ret = HAL_SPI_Transmit(p_hspi, tx, 2, timeout);
    cs(1);

    return ret;
}

/**
 * @brief 获取温度
 * 
 * @return float 
 */
float icm42688p::get_temp(void)
{   
    uint8_t rx[2] = {};
    uint16_t temp = 0;

    // read_reg(ICM_TEMP_DATA1, data);
    // temp |= (data<<8);

    // read_reg(ICM_TEMP_DATA0, data);
    // temp |= (data<<0);


    read_reg(ICM_TEMP_DATA1, rx, 2);

    temp = rx[0]<<8 | rx[1];

    float temp_f = ((float)temp/132.48f)+25.0f;

    return temp_f;
}

/**
 * @brief 获取角速度和加速度
 * 
 * @param Axis3f_gyro 
 * @param Axis3f_accel 
 * @return int 
 */
int icm42688p::get_acc_gyro(float *gyro_x, float *gyro_y, float *gyro_z, float *accel_x, float *accel_y, float *accel_z)
{
    uint8_t rx[12]      = {};
    int16_t gyro[3]     = {};
    int16_t accel[3]    = {};   
    float gyro_f[3]     = {};
    float accel_f[3]    = {};

    // uint8_t status = 0;
    // read_reg(ICM_INT_STATUS, &status, 1); 
    // if (!(status & 0x08)){
    //     return 0;
    // }

    /* 读取六轴寄存器 */
    read_reg(ICM_ACCEL_DATA_X1, rx, 12);

    accel[0] = (int16_t)(rx[0]<<8 | rx[1]<<0)   - acc_raw_bias[0];
    accel[1] = (int16_t)(rx[2]<<8 | rx[3]<<0)   - acc_raw_bias[0];
    accel[2] = (int16_t)(rx[4]<<8 | rx[5]<<0)   - acc_raw_bias[0];
    gyro[0]  = (int16_t)(rx[6]<<8  | rx[7]<<0)  - gyro_raw_bias[0];
    gyro[1]  = (int16_t)(rx[8]<<8  | rx[9]<<0)  - gyro_raw_bias[1];
    gyro[2]  = (int16_t)(rx[10]<<8 | rx[11]<<0) - gyro_raw_bias[2];

    /* 转换乘浮点 */
    // gyro_f[0] = (float)gyro[0]/32768.0f *2000.0f - gyro_bias[0];
    // gyro_f[1] = (float)gyro[1]/32768.0f *2000.0f - gyro_bias[1];
    // gyro_f[2] = (float)gyro[2]/32768.0f *2000.0f - gyro_bias[2];

    // accel_f[0] = (float)accel[0]/32768.0f *16.0f*9.8f - acc_bias[0];
    // accel_f[1] = (float)accel[1]/32768.0f *16.0f*9.8f - acc_bias[1];
    // accel_f[2] = (float)accel[2]/32768.0f *16.0f*9.8f - acc_bias[2];
    
    gyro_f[0] = (float)gyro[0] *gyro_scale;
    gyro_f[1] = (float)gyro[1] *gyro_scale;
    gyro_f[2] = (float)gyro[2] *gyro_scale;

    accel_f[0] = (float)accel[0] *acc_scale;
    accel_f[1] = (float)accel[1] *acc_scale;
    accel_f[2] = (float)accel[2] *acc_scale;


    /* 保存数据 */
    *gyro_x = gyro_f[0];
    *gyro_y = gyro_f[1];
    *gyro_z = gyro_f[2];

    *accel_x = accel_f[0];
    *accel_y = accel_f[1];
    // *accel_z = accel_f[2];
    *accel_z = 0;


    // printf("accel: x: %d  y: %d  z:%d\r\n", accel[0], accel[1], accel[2]);
    // printf("gyro: x: %d  y: %d  z:%d\r\n\r\n", gyro[0], gyro[1], gyro[2]);

    // printf("加速度: x: %.4f  y: %.4f  z:%.4f  总体值: %.4f\r\n", accel_f[0], accel_f[1], accel_f[2], 
    //         sqrtf(accel_f[0]*accel_f[0]+ accel_f[1]*accel_f[1]+accel_f[2]*accel_f[2]));
    // printf("角速度:  x: %.4f  y: %.4f  z:%.4f\r\n\r\n", gyro_f[0], gyro_f[1], gyro_f[2]);

    return 0;
}


/**
 * @brief  获取加速度计
 * 
 * @param accel_x 
 * @param accel_y 
 * @param accel_z 
 * @return int 
 */
int icm42688p::get_acc(float *accel_x, float *accel_y, float *accel_z)
{
    int ret = -1;
    uint8_t rx[6]      = {};
    int16_t accel[3]    = {};   
    float accel_f[3]    = {};

    /* 读取六轴寄存器 */
    ret = read_reg(ICM_ACCEL_DATA_X1, rx, 6);

    /* 转换乘浮点 */
    accel_f[0] = (float)accel[0]/32768.0f *16.0f*9.8f - acc_bias[0];
    accel_f[1] = (float)accel[1]/32768.0f *16.0f*9.8f - acc_bias[1];
    accel_f[2] = (float)accel[2]/32768.0f *16.0f*9.8f - acc_bias[2];

    /* 保存数据 */

    *accel_x = accel_f[0];
    *accel_y = accel_f[1];
    *accel_z = accel_f[2];

    return ret;
}

/**
 * @brief 获取角速度
 * 
 * @param gyro_x 
 * @param gyro_y 
 * @param gyro_z 
 * @return int 
 */
int icm42688p::get_gyro(float *gyro_x, float *gyro_y, float *gyro_z)
{
    int ret = -1;
    uint8_t rx[6]      = {};
    int16_t gyro[3]    = {};   
    float gyro_f[3]    = {};

    /* 读取六轴寄存器 */
    ret = read_reg(ICM_GYRO_DATA_X1, rx, 6);

    /* 转换乘浮点 */
    gyro_f[0] = (float)gyro[0]/32768.0f *2000.0f - gyro_bias[0];
    gyro_f[1] = (float)gyro[1]/32768.0f *2000.0f - gyro_bias[1];
    gyro_f[2] = (float)gyro[2]/32768.0f *2000.0f - gyro_bias[2];

    /* 保存数据 */ 
    *gyro_x = gyro_f[0];
    *gyro_y = gyro_f[1];
    *gyro_z = gyro_f[2];

    return ret;
}







