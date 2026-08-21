#include "icm20948.hpp"

/*寄存器定义*/

/*BANK_0*/
#define WHO_AM_I                        0x00    /* 芯片 ID 寄存器 0xEA */
#define USER_CTRL                       0x03    /* DMP FIFO I2C主机从机 DMP复位 SRAM复位 I2C主机复位 */
#define LP_CONFIG                       0x05    /* I2C主机占空比 加速度计占空比 陀螺仪占空比 主要是低功耗设置 */
#define PWR_MGMT_1                      0x06    /* 软复位，默认上电睡眠模式，低功耗开关，温度传感器，时钟源选择 */
#define PWR_MGMT_2                      0x07    /* 加速度计使能 陀螺仪使能*/
#define INT_PIN_CFG                     0x0F    /* 中断引脚配置，保持方式，清除方式，FSYNC中断配置，I2C旁路模式 */
#define INT_ENABLE                      0x10    /* 唤醒中断使能 运动唤醒中断，PLL中断，DMP中断， I2C主机中断 */
#define INT_ENABLE_1                    0x11    /* 传感器数据中断 到INT1 */
#define INT_ENABLE_2                    0x12    /* FIFO溢出中断 */
#define INT_ENABLE_3                    0x13    /* 使能对应 FIFO 的 水位阈值中断*/
#define I2C_MST_STATUS                  0x17    /* FSYNC 中断状态*/
#define INT_STATUS                      0x19    /* 运动唤醒中断发生， PLL上电中断发生 DMP中断发生 I2C主机中断发生*/
#define INT_STATUS_1                    0x1A    /* 所有传感器原始数据已更新，可读取*/
#define INT_STATUS_2                    0x1B    /* 指示 FIFO 溢出事件 是否发生。*/
#define INT_STATUS_3                    0x1C    /* FIFO 达到水位阈值 事件是否发生*/
#define DELAY_TIMEH                     0x28    /* 提供 FSYNC 事件到第一个陀螺仪采样输出（ODR事件）之间的延迟 的高 8 位。*/
#define DELAY_TIMEL                     0x29    /* 提供 FSYNC 事件到第一个陀螺仪采样输出的 延迟低字节。*/
#define ACCEL_XOUT_H                    0x2D    /* 存储 加速度计 X 轴测量值的高 8 位。*/
#define ACCEL_XOUT_L                    0x2E
#define ACCEL_YOUT_H                    0x2F
#define ACCEL_YOUT_L                    0x30
#define ACCEL_ZOUT_H                    0x31
#define ACCEL_ZOUT_L                    0x32
#define GYRO_XOUT_H                     0x33
#define GYRO_XOUT_L                     0x34
#define GYRO_YOUT_H                     0x35
#define GYRO_YOUT_L                     0x36
#define GYRO_ZOUT_H                     0x37
#define GYRO_ZOUT_L                     0x38
#define TEMP_OUT_H                      0x39    /* 温度数据*/
#define TEMP_OUT_L                      0x3A
#define EXT_SLV_SENS_DATA_00            0x3B    /* 存储通过 I?C Master 接口从 外部 I?C 从设备 读取的数据*/
#define EXT_SLV_SENS_DATA_01            0x3C    /* 存储通过 I?C Master 接口从 外部 I?C 从设备 读取的数据的 第 2 个字节。*/
#define EXT_SLV_SENS_DATA_02            0x3D
#define EXT_SLV_SENS_DATA_03            0x3E
#define EXT_SLV_SENS_DATA_04            0x3F
#define EXT_SLV_SENS_DATA_05            0x40
#define EXT_SLV_SENS_DATA_06            0x41
#define EXT_SLV_SENS_DATA_07            0x42
#define EXT_SLV_SENS_DATA_08            0x43
#define EXT_SLV_SENS_DATA_09            0x44
#define EXT_SLV_SENS_DATA_10            0x45
#define EXT_SLV_SENS_DATA_11            0x46
#define EXT_SLV_SENS_DATA_12            0x47
#define EXT_SLV_SENS_DATA_13            0x48
#define EXT_SLV_SENS_DATA_14            0x49
#define EXT_SLV_SENS_DATA_15            0x4A
#define EXT_SLV_SENS_DATA_16            0x4B
#define EXT_SLV_SENS_DATA_17            0x4C
#define EXT_SLV_SENS_DATA_18            0x4D
#define EXT_SLV_SENS_DATA_19            0x4E
#define EXT_SLV_SENS_DATA_20            0x4F
#define EXT_SLV_SENS_DATA_21            0x50
#define EXT_SLV_SENS_DATA_22            0x51
#define EXT_SLV_SENS_DATA_23            0x52
#define FIFO_EN_1                       0x66    /* 控制将通过 I?C Master 读取的外部从设备数据写入内部 FIFO 的功能*/
#define FIFO_EN_2                       0x67    /* 控制 加速度计、陀螺仪和温度传感器的数据 是否按采样率写入 FIFO。*/
#define FIFO_RST                        0x68    /* 通过软件复位清空 FIFO 并将 FIFO 尺寸置为 0。*/
#define FIFO_MODE                       0x69    /* 选择 FIFO 的写入策略，当 FIFO 满时数据的处理方式。*/
#define FIFO_COUNTH                     0x70    /* 提供 FIFO 当前已写入字节数的高 5 位（Bit 4:0）*/
#define FIFO_COUNTL                     0x71    /* 提供 FIFO 当前已写入字节数的低 8 位。*/
#define FIFO_R_W                        0x72    /* 通过这个寄存器可以 直接读写 FIFO 数据。*/
#define DATA_RDY_STATUS                 0x74    /* 指示传感器数据是否已加载到 FIFO 或 SRAM，可用于判断 MCU 是否可以读取新数据*/
#define FIFO_CFG                        0x76    /* 配置是否将 每个传感器的数据准备状态 与中断联动 */
#define REG_BANK_SEL                    0x7F    /* 选择当前访问的 User Bank，因为 ICM-20948 的寄存器被划分为 4 个*/


/*BANK_1*/
#define SELF_TEST_X_GYRO                0x02    /* 记录 X 轴陀螺仪在出厂自检时的参考输出，用户可以通过比较此值和实际自检结果来判断传感器是否正常。*/ 
#define SELF_TEST_Y_GYRO                0x03
#define SELF_TEST_Z_GYRO                0x04
#define SELF_TEST_X_ACCEL               0x0E
#define SELF_TEST_Y_ACCEL               0x0F
#define SELF_TEST_Z_ACCEL               0x10    /* 存储 X 轴加速度计偏置校正值的高 8 位，用于修正加速度计零点偏移*/
#define XA_OFFS_H                       0x14
#define XA_OFFS_L                       0x15
#define YA_OFFS_H                       0x17
#define YA_OFFS_L                       0x18
#define ZA_OFFS_H                       0x1A
#define ZA_OFFS_L                       0x1B
#define TIMEBASE_CORRECTION_PLL         0x28    /* 修正 系统 PLL 时钟周期误差，保证时钟精度。用户可以通过写入 TBC_PLL 来微调 PLL 频率，使系统计时更准确 */


/*BANK_2*/
#define GYRO_SMPLRT_DIV                 0x00    /* 陀螺仪采样速度，计算公式 1.1 kHz/(1+GYRO_SMPLRT_DIV[7:0])。*/
#define GYRO_CONFIG_1                   0x01    /* 配置陀螺仪的 量程（Full Scale） 和 低通滤波器（DLPF），影响陀螺仪数据精度和噪声特性。*/
#define GYRO_CONFIG_2                   0x02    /* 启用陀螺仪各轴自检 配置低功耗模式下的平均滤波倍数*/
#define XG_OFFS_USRH                    0x03    /* 存储 X 轴陀螺仪偏置校正的高 8 位，用于修正零点漂移。*/
#define XG_OFFS_USRL                    0x04
#define YG_OFFS_USRH                    0x05
#define YG_OFFS_USRL                    0x06
#define ZG_OFFS_USRH                    0x07
#define ZG_OFFS_USRL                    0x08
#define ODR_ALIGN_EN                    0x09    /* 输出数据率（ODR）起始时间对齐控制*/
#define ACCEL_SMPLRT_DIV_1              0x10    /* 储 加速度计采样率分频器的高 4 位（MSB），与 ACCEL_SMPLRT_DIV_2（低 8 位）组合后生成完整的 12 位分频值，用于计算加速度计输出数据率（ODR */
#define ACCEL_SMPLRT_DIV_2              0x11    /* 存储 加速度计采样率分频器的低 8 位（LSB），与 ACCEL_SMPLRT_DIV_1（高 4 位）组合后形成完整的 12 位分频值，用于计算加速度计输出数据率（ODR）*/
#define ACCEL_INTEL_CTRL                0x12    /* 控制加速度计的 智能运动检测（WOM）算法 和逻辑启用*/
#define ACCEL_WOM_THR                   0x13    /* 设置 加速度计 WOM 中断触发阈值（X/Y/Z 轴），当加速度变化超过阈值时会触发 WOM 中断*/
#define ACCEL_CONFIG                    0x14    /* 设置加速度计的 量程 和 低通滤波器配置。*/
#define ACCEL_CONFIG_2                  0x15    /* 配置加速度计的 自检功能 和 去除高频噪声的采样平均（Decimator）*/
#define FSYNC_CONFIG                    0x52    /* 配置 FSYNC 信号行为和外部同步数据采样。*/
#define TEMP_CONFIG                     0x53    /* 设置温度传感器的数字低通滤波器，平滑温度输出，去除高频噪声。*/
#define MOD_CTRL_USR                    0x54    /* 控制 DMP 在低功耗加速度计模式下的使能状态*/


/*BANK_3*/
#define I2C_MST_ODR_CONFIG              0x00    /* 控制 I?C Master 接口读取外部传感器数据的速率*/
#define I2C_MST_CTRL                    0x01    /* 配置 I?C Master 接口的多主机模式、读写间隔以及时钟频率*/
#define I2C_MST_DELAY_CTRL              0x02    /* 控制访问外部 I?C 从机的采样延迟，以及外部传感器数据阴影寄存器的更新时机*/
#define I2C_SLV0_ADDR                   0x03
#define I2C_SLV0_REG                    0x04
#define I2C_SLV0_CTRL                   0x05
#define I2C_SLV0_DO                     0x06
#define I2C_SLV1_ADDR                   0x07
#define I2C_SLV1_REG                    0x08
#define I2C_SLV1_CTRL                   0x09
#define I2C_SLV1_DO                     0x0A
#define I2C_SLV2_ADDR                   0x0B
#define I2C_SLV2_REG                    0x0C
#define I2C_SLV2_CTRL                   0x0D
#define I2C_SLV2_DO                     0x0E
#define I2C_SLV3_ADDR                   0x0F
#define I2C_SLV3_REG                    0x10
#define I2C_SLV3_CTRL                   0x11
#define I2C_SLV3_DO                     0x12
#define I2C_SLV4_ADDR                   0x13
#define I2C_SLV4_REG                    0x14
#define I2C_SLV4_CTRL                   0x15
#define I2C_SLV4_DO                     0x16
#define I2C_SLV4_DI                     0x17

/*磁力计寄存器 AK09916*/
#define AK09916_DEVICE_ID               0x01    /* 主机在初始化时读取 WIA，可以确认是否正确连接了目标传感器*/
#define AK09916_STATUS_1                0x10    /* 用于表示数据是否准备好以及是否发生数据溢出 */
#define AK09916_DATA                    0x11    /* 地址从 0x11 到 0x16，共 6 个寄存器，用于存储 X、Y、Z 三个轴的磁场测量值，每轴 16 位*/
#define AK09916_STATUS_2                0x18    /* 作为数据读取结束标志 在连续模式下，每次读完测量寄存器后，必须读 ST2 才算完整读取流程。*/
#define AK09916_CONTROL_2               0x31    /* 控制磁力计的工作模式*/
#define AK09916_CONTROL_3               0x32    /* 软复位：所有寄存器初始化到默认值，复位后 SRST 自动清零*/
/* */



typedef enum{
    ACCEL_FS_2G     = 0x0 << 1,  // ±2g
    ACCEL_FS_4G     = 0x1 << 1,  // ±4g
    ACCEL_FS_8G     = 0x2 << 1,  // ±8g
    ACCEL_FS_16G    = 0x3 << 1   // ±16g
}ACCEL_FS_SEL_t;

/* 陀螺仪量程枚举 (位7:5)*/
typedef enum{
    GYRO_FS_250DPS      = (0x00 << 1),      // ±250 dps (默认)
    GYRO_FS_500DPS      = (0x01 << 1),      // ±500 dps
    GYRO_FS_1000DPS     = (0x02 << 1),      // ±1000 dps
    GYRO_FS_2000DPS     = (0x03 << 1),      // ±2000 dps
}GYRO_FS_SEL_t;


icm20948::icm20948(I2C_HandleTypeDef *p_hi2c, uint8_t addr)
{
    this->flag = false;

    state = 0;

    printf("ICM-20948 ID = 0x%02X\r\n", 1);


    if(NULL==p_hi2c)return;
    printf("ICM-20948 ID = 0x%02X\r\n", 2);

    this->p_hi2c    = p_hi2c;
    this->addr      = addr;


    this->flag = true;

    for (size_t i = 0; i < 3; i++){
        acc_raw_bias[i] = 0;
        gyro_raw_bias[i] = 0;
    }


    gyro_scale = 1000.0f / 32768.0f;
    acc_scale = 2.0f / 32768.0f;

    printf("ICM-20948 ID = 0x%02X\r\n", 3);
    config();

    printf("ICM-20948 ID = 0x%02X\r\n", 4);
}

icm20948::~icm20948()
{
}


/*icm20948配置*/
void icm20948::config(void)
{
    uint8_t id;

    /*bank1 配置*/
    write_reg(REG_BANK_SEL, 0x00);          /*切换到bank0*/
    write_reg(PWR_MGMT_1, 0x80);            /*复位*/
    HAL_Delay(10);
    write_reg(PWR_MGMT_1, 0x01);            /*退出睡眠，关闭低功耗 启用温度传感器 时钟源自动选择*/
    write_reg(INT_PIN_CFG, 0xD2);           /*INT1高电平触发 推挽输出 长期保持 读取任意寄存器清除中断，禁用FSYNC 打开I2C直通*/
    write_reg(INT_ENABLE_1, 0x01);          /*开启数据中断*/
    
    /*bank2配置*/
    write_reg(REG_BANK_SEL, 0x20);  
    write_reg(GYRO_SMPLRT_DIV, 0x0A);                       /* 陀螺仪每秒采样  1.1K/(1+0x0A) 110Hz*/ 
    write_reg(GYRO_CONFIG_1, 0x30|GYRO_FS_1000DPS|0x01);               /* 低通滤波配置6 量程±1000 dps 启用低通滤波*/ 
    write_reg(ACCEL_SMPLRT_DIV_2, 0x0A);                    /* 加速度计的采样率 1.125K/(1+0x0A) 112Hz*/ 
    write_reg(ACCEL_CONFIG, 0x30|ACCEL_FS_2G|0x01);         /* 低通滤波配置 6、±2g 量程、启用低通滤波*/ 
    write_reg(ODR_ALIGN_EN, 0x01);                          /* 让acc gryo采样率对齐*/
    
    write_reg(REG_BANK_SEL, 0x00);                          /*切换到bank0*/


    /*磁力计配置*/
    write_reg(0x0C, AK09916_CONTROL_3, 0x01, HAL_MAX_DELAY);    /* 磁力计软复位 */
    HAL_Delay(10);
    write_reg(0x0C, AK09916_CONTROL_2, 0x04, HAL_MAX_DELAY);    /* 中速采样 大概20HZ一次*/


    printf("ICM-20948 ID = 0x%02X\r\n", id);


    /* 读取 ICM-20948 本体 ID */
    read_reg(0x69, WHO_AM_I, id);
    printf("ICM-20948 ID = 0x%02X\r\n", id);

    /* 读取磁力计 AK09916 ID */
    read_reg(0x0C, AK09916_DEVICE_ID, id);                      /* AK09916 WIA 寄存器 0x01 */
    printf("AK09916 MAG ID = 0x%02X\r\n", id);

}

int icm20948::read_reg(uint8_t addr, uint8_t reg, uint8_t &data, uint32_t timeout)
{
    if(false==flag)return 1;

    int ret = 0;
    ret = HAL_I2C_Mem_Read(p_hi2c, (addr<<1)|0x01, reg, I2C_MEMADD_SIZE_8BIT, &data, 1, timeout);
    if(ret){
        printf("icm20948: read_reg %d: NO: %d\r\n", reg, ret);
    }
    return ret;
}

int icm20948::read_reg(uint8_t reg, uint8_t &data, uint32_t timeout)
{
    if(false==flag)return 1;

    int ret = 0;
    ret = HAL_I2C_Mem_Read(p_hi2c, (addr<<1)|0x01, reg, I2C_MEMADD_SIZE_8BIT, &data, 1, timeout);
    if(ret){
        printf("icm20948: read_reg %d: NO: %d\r\n", reg, ret);
    }
    return ret;
}

int icm20948::read_reg(uint8_t reg, uint8_t *data, uint32_t len, uint32_t timeout)
{
    if(false==flag)return 1;

    int ret = 0;
    ret = HAL_I2C_Mem_Read(p_hi2c, (addr<<1)|0x01, reg, I2C_MEMADD_SIZE_8BIT, data, len, timeout);
    if(ret){
        printf("icm20948: read_reg %d: NO: %d\r\n", reg, ret);
    }
    return ret;
}

int icm20948::read_reg(uint8_t addr, uint8_t reg, uint8_t *data, uint32_t len, uint32_t timeout)
{
    if(false==flag)return 1;

    int ret = 0;
    ret = HAL_I2C_Mem_Read(p_hi2c, (addr<<1)|0x01, reg, I2C_MEMADD_SIZE_8BIT, data, len, timeout);
    if(ret){
        printf("icm20948: read_reg %d: NO: %d\r\n", reg, ret);
    }
    return ret;
}

int icm20948::write_reg(uint8_t reg, uint8_t data, uint32_t timeout)
{
    if(false==flag)return 1;

    int ret = 0;
    ret = HAL_I2C_Mem_Write(p_hi2c, (addr<<1), reg, I2C_MEMADD_SIZE_8BIT, &data, 1, timeout);
    if(ret){
        printf("icm20948: write_reg %d: NO: %d\r\n", reg, ret);
    }

    return ret;
}

int icm20948::write_reg(uint8_t addr, uint8_t reg, uint8_t data, uint32_t timeout)
{
    if(false==flag)return 1;

    int ret = 0;
    ret = HAL_I2C_Mem_Write(p_hi2c, (addr<<1), reg, I2C_MEMADD_SIZE_8BIT, &data, 1, timeout);
    if(ret){
        printf("icm20948: write_reg %d: NO: %d\r\n", reg, ret);
    }

    return ret;
}


uint8_t icm20948::read_int_status(void)
{
    uint8_t data = 0;
    read_reg(INT_STATUS_1, data);
    // printf("state: %02x\r\n", data);
    return data;
}


int icm20948::read_gyro_accel(float gyro[3], float accel[3])
{
    int ret = 0;

    uint8_t rx[12]          = {};
    int16_t gyro_raw[3]     = {};
    int16_t accel_raw[3]    = {}; 


    uint8_t int_status = read_int_status();
    /*如果数据没有准备好*/
    if( !(int_status&0x01))return 1;
    

    /* 读取六轴寄存器 */
    ret = read_reg(ACCEL_XOUT_H, rx, 12);

    accel_raw[0] = (int16_t)(rx[0]<<8 | rx[1]<<0)   - acc_raw_bias[0];
    accel_raw[1] = (int16_t)(rx[2]<<8 | rx[3]<<0)   - acc_raw_bias[1];
    accel_raw[2] = (int16_t)(rx[4]<<8 | rx[5]<<0)   - acc_raw_bias[2];
    gyro_raw[0]  = (int16_t)(rx[6]<<8  | rx[7]<<0)  - gyro_raw_bias[0];
    gyro_raw[1]  = (int16_t)(rx[8]<<8  | rx[9]<<0)  - gyro_raw_bias[1];
    gyro_raw[2]  = (int16_t)(rx[10]<<8 | rx[11]<<0) - gyro_raw_bias[2];

    switch (state)
    {
        case 0: /*启动校准*/
            for (size_t i = 0; i < 3; i++){
                acc_raw_bias[i] = 0;
                gyro_raw_bias[i] = 0;
                gyro_raw_sum[i] = 0;
                acc_raw_sum[i] = 0;
            }
            sample_count = 0;
            state = 1;
            ret = -1;
            break;
        case 1: /*采集数据 进行校准*/
            for (size_t i = 0; i < 3; i++)
            {
                gyro_raw_sum[i] += gyro_raw[i];
                acc_raw_sum[i]  += accel_raw[i];
            }

            sample_count++;
            if(sample_count>=ICM20948_CALIB_COUNT){
                for (size_t i = 0; i < 3; i++)
                {
                    gyro_raw_bias[i] = gyro_raw_sum[i] / sample_count;
                    acc_raw_bias[i]  = acc_raw_sum[i] / sample_count;
                }
                // Z 轴加速度应为 9.8，去除它的影响，只保留 X、Y 偏置
                acc_raw_bias[2] -=  1.0f/acc_scale;
                state=3;
                // printf("陀螺仪: x: %d y: %d z: %d\r\n", gyro_raw_sum[0], gyro_raw_sum[1], gyro_raw_sum[2]);
                // printf("加速度: x: %d y: %d z: %d\r\n", acc_raw_sum[0], acc_raw_sum[1], acc_raw_sum[2]);
                printf("陀螺仪偏置: x: %d y: %d z: %d\r\n", gyro_raw_bias[0], gyro_raw_bias[1], gyro_raw_bias[2]);
                printf("加速度偏置: x: %d y: %d z: %d\r\n", acc_raw_bias[0], acc_raw_bias[1], acc_raw_bias[2]);
            }
            ret = -1;
            break;
        case 3: /*正常运行*/
            ret = 0;
            break;
        default:
            break;
    }


    gyro[0] = (float)gyro_raw[0] *gyro_scale;
    gyro[1] = (float)gyro_raw[1] *gyro_scale;
    gyro[2] = (float)gyro_raw[2] *gyro_scale;

    accel[0] = (float)accel_raw[0] *acc_scale;
    accel[1] = (float)accel_raw[1] *acc_scale;
    accel[2] = (float)accel_raw[2] *acc_scale;

    // gyro[0] = (float)gyro_raw[0] ;
    // gyro[1] = (float)gyro_raw[1] ;
    // gyro[2] = (float)gyro_raw[2] ;

    // accel[0] = (float)accel_raw[0] ;
    // accel[1] = (float)accel_raw[1] ;
    // accel[2] = (float)accel_raw[2] ;

    return ret;
}


/**
 * @brief 
 * 
 * @param mag 
 * @return int -3为磁力计异常
 */
int icm20948::read_mag(int16_t mag[3])
{
    int ret = 0;

    uint8_t rx[6]          = {};
    int16_t mag_raw[3]     = {};

    // uint8_t int_status = read_int_status();
    // /*如果数据没有准备好*/
    // if( !(int_status&0x01))return 1;

    read_reg(0x0C, AK09916_STATUS_1, rx[0]);

    if(!(rx[0] & 0x01)){
        // printf("数据没有准备好\r\n");
        return 1;
    }

    /* 读取六轴寄存器 */
    ret |= read_reg(0x0C, AK09916_DATA, rx, 6);

    mag_raw[0] = (int16_t)(rx[1]<<8 | rx[0]<<0);
    mag_raw[1] = (int16_t)(rx[3]<<8 | rx[2]<<0);
    mag_raw[2] = (int16_t)(rx[5]<<8 | rx[4]<<0);

    rx[0] = 0;
    ret |= read_reg(0x0C, AK09916_STATUS_2, rx[0]);
    if(rx[0]&0x08){
        /*磁力计溢出，磁力计异常*/
        printf("磁力计溢出，磁力计异常\r\n");
        ret = -3;
    }

    for (size_t i = 0; i < 3; i++){
        mag[i] = mag_raw[i];
    }
    
    return ret;
}
