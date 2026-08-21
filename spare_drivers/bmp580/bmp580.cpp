#include "bmp580.hpp"

/* BMP580寄存器 */

#define CHIP_ID                     0x01        /* 存储芯片的唯一标识 ID，用于确认芯片型号或版本*/
#define REV_ID                      0x02        /* 表示芯片的版本号，用于区分不同版本的 ASIC*/
#define CHIP_STATUS                 0x11        /* 反映 ASIC 的状态信息，包括 I3C 错误和 HIF 模式状态 默认是SPI I2C自动配置*/
#define DRIVE_CONFIG                0x13        /* 配置 ASIC 的接口相关设置，包括 I2C pull-up、SPI 3/4 线模式，以及 IO 引脚驱动能力*/
#define INT_CONFIG                  0x14        /* 配置 INT 引脚的中断模式、极性、输出类型、使能以及驱动强度*/
#define INT_SOURCE                  0x15        /* 选择哪些事件会触发中断 INT 引脚*/
#define FIFO_CONFIG                 0x16        /* 配置 FIFO 模式和阈值*/
#define FIFO_COUNT                  0x17        /* 读取 FIFO 中当前存储的数据帧数*/
#define FIFO_SEL                    0x18        /* 选择 FIFO 数据源及解码方式*/
#define RESERVED_REG_0              0x1C
#define TEMP_DATA_XLSB              0x1D        /* 温度测量数据*/
#define TEMP_DATA_LSB               0x1E
#define TEMP_DATA_MSB               0x1F  
#define PRESS_DATA_XLSB             0x20        /* 压力测量数据*/
#define PRESS_DATA_LSB              0x21        
#define PRESS_DATA_MSB              0x22
#define RESERVED_REG1               0x23
#define RESERVED_REG2               0x24
#define RESERVED_REG3               0x25
#define RESERVED_REG4               0x26
#define INT_STATUS                  0x27        /* 读取中断状态，读后会自动清除*/
#define STATUS                      0x28        /* 反映 NVM（非易失性存储器）状态和设备状态*/
#define FIFO_DATA                   0x29        /* 读取 FIFO 中的数据*/
#define NVM_ADDR                    0x2B        /* 选择 NVM 编程/读取的地址行，以及启用 NVM 编程*/
#define NVM_DATA_LSB                0x2C        /* NVM 数据*/
#define NVM_DATA_MSB                0x2D
#define DSP_CONFIG                  0x30        /* 控制压力/温度数据的 IIR 滤波输出选择，以及强制刷新 IIR 滤波器*/
#define DSP_IIR                     0x31        /* 设置压力和温度数据 IIR 滤波器的系数*/
#define OOR_THR_P_LSB               0x32        /* 设置压力传感器下溢/上溢判断阈值的低 8 位*/
#define OOR_THR_P_MSB               0x33        
#define OOR_RANGE                   0x34        /* 设置压力 OOR（超出范围）检测的范围或模式*/
#define OOR_CONFIG                  0x35        /* 设置压力 OOR 阈值的 第 16 位 设置 OOR 检测的计数限制，用于滤除短暂异常值*/
#define OSR_CONFIG                  0x36        /* 配置压力 (P) 和温度 (T) 的过采样率 控制是否启用压力测量*/
#define ODR_CONFIG                  0x37        /* 配置传感器的输出数据率 (ODR 配置功率模式 深度待机禁用*/
#define OSR_EFF                     0x38        /* 显示实际生效的压力/温度 OSR 检查 ODR 配置是否有效*/
#define CMD                         0x7E        /* 写入特定命令触发芯片操作，寄存器读取总是返回 0x00 */







bmp580::bmp580(I2C_HandleTypeDef *p_hi2c, uint8_t addr)
{
    this->flag = false;
    state = 0;

    if(NULL==p_hi2c)return;
    this->p_hi2c    = p_hi2c;
    this->addr      = addr;
    this->flag = true;

    air_raw_bias = 0;
    air_raw_sum = 0;
    sample_count = 0;

    air_scale = 0;

    config();

}

bmp580::~bmp580()
{
}

void bmp580::config(void)
{
    uint8_t id = 0;

    write_reg(CMD, 0xB6);       /*软复位*/
    HAL_Delay(50);

     
    

    read_reg(CHIP_STATUS, id);
    printf("BMP-580 CHIP_STATUS = 0x%02X\r\n", id);
    HAL_Delay(10);

    write_reg(DRIVE_CONFIG, 0x01<<4|0x01<<0);           /*驱动配置*/
    {
        uint8_t data = 0;
        read_reg(DRIVE_CONFIG, data);
        printf("DRIVE_CONFIG: %02X %02X\r\n", data, 0x01<<4|0x01<<0);
    }

    /*中断配置*/
    write_reg(INT_CONFIG, (0x01<<4)|(0x01<<3)|(0x01<<2)|(0x01<<1)|(0x00<<0)); /*增强驱动, 使能中断，开漏输出，高电平有效，脉冲式*/
    HAL_Delay(10);
    {
        uint8_t data = 0;
        read_reg(INT_CONFIG, data);
        printf("INT_CONFIG: %02X %02X\r\n", data, (0x01<<4)|(0x01<<3)|(0x01<<2)|(0x01<<1)|(0x00<<0));
    }
    write_reg(INT_SOURCE, (0x01<<0));   /*打开数据就位中断源*/
    {
        uint8_t data = 0;
        read_reg(INT_SOURCE, data);
        printf("INT_SOURCE: %02X %02X\r\n", data, (0x01<<0));
    }

    /*滤波设置*/
    write_reg(DSP_CONFIG, (1<<7)|(1<<6)|(1<<5)|(1<<4)|(1<<3)|(0x00<<2));    /* 开启所有滤波后值选择*/
    {
        uint8_t data = 0;
        read_reg(DSP_CONFIG, data);
        printf("DSP_CONFIG: %02X %02X\r\n", data, (1<<7)|(1<<6)|(1<<5)|(1<<4)|(1<<3)|(0x00<<2));
    }

    write_reg(DSP_IIR, (0x07<<3)|(0x07<<0));                                /* 配置滤波系数*/
    {
        uint8_t data = 0;
        read_reg(DSP_IIR, data);
        printf("DSP_IIR: %02X %02X\r\n", data, (0x07<<3)|(0x07<<0));
    }

    /*速率控制*/
    write_reg(OSR_CONFIG, (0x01<<6)|(0x07<<3)|(0x07<<0));                   /* 开启压力测量 OSR设置最大*/
    {
        uint8_t data = 0;
        read_reg(OSR_CONFIG, data);
        printf("OSR_CONFIG: %02X %02X\r\n", data, (0x01<<6)|(0x07<<3)|(0x07<<0));
    }
    write_reg(ODR_CONFIG, (0x01<<7)|(0x0A<<2)|(0x03<<0));                   /* 禁止深度待机, 100Hz, 连续采样*/
    {
        uint8_t data = 0;
        read_reg(ODR_CONFIG, data);
        printf("ODR_CONFIG: %02X %02X\r\n", data, (0x01<<7)|(0x0A<<2)|(0x01<<0));
    }

    /* 读取 ICM-20948 本体 ID */
    read_reg(CHIP_ID, id);
    printf("BMP-580 CHIP_ID = 0x%02X\r\n", id);
    HAL_Delay(10);

    read_reg(REV_ID, id);
    printf("BMP-580 REV_ID = 0x%02X\r\n", id);
    
}

int bmp580::read_reg(uint8_t addr, uint8_t reg, uint8_t &data, uint32_t timeout)
{
    if(false==flag)return 1;

    int ret = 0;
    ret = HAL_I2C_Mem_Read(p_hi2c, (addr<<1)|0x01, reg, I2C_MEMADD_SIZE_8BIT, &data, 1, timeout);
    if(ret){
        printf("bmp580: read_reg %d: NO: %d\r\n", reg, ret);
    }
    return ret;
}

int bmp580::read_reg(uint8_t reg, uint8_t &data, uint32_t timeout)
{
    if(false==flag)return 1;

    int ret = 0;
    ret = HAL_I2C_Mem_Read(p_hi2c, (addr<<1)|0x01, reg, I2C_MEMADD_SIZE_8BIT, &data, 1, timeout);
    if(ret){
        printf("bmp580: read_reg %d: NO: %d\r\n", reg, ret);
    }
    return ret;
}

int bmp580::read_reg(uint8_t reg, uint8_t *data, uint32_t len, uint32_t timeout)
{
    if(false==flag)return 1;

    int ret = 0;
    ret = HAL_I2C_Mem_Read(p_hi2c, (addr<<1)|0x01, reg, I2C_MEMADD_SIZE_8BIT, data, len, timeout);
    if(ret){
        printf("bmp580: read_reg %d: NO: %d\r\n", reg, ret);
    }
    return ret;
}

int bmp580::read_reg(uint8_t addr, uint8_t reg, uint8_t *data, uint32_t len, uint32_t timeout)
{
    if(false==flag)return 1;

    int ret = 0;
    ret = HAL_I2C_Mem_Read(p_hi2c, (addr<<1)|0x01, reg, I2C_MEMADD_SIZE_8BIT, data, len, timeout);
    if(ret){
        printf("bmp580: read_reg %d: NO: %d\r\n", reg, ret);
    }
    return ret;
}

int bmp580::write_reg(uint8_t reg, uint8_t data, uint32_t timeout)
{
    if(false==flag)return 1;

    int ret = 0;
    ret = HAL_I2C_Mem_Write(p_hi2c, (addr<<1), reg, I2C_MEMADD_SIZE_8BIT, &data, 1, timeout);
    if(ret){
        printf("bmp580: write_reg %d: NO: %d\r\n", reg, ret);
    }

    return ret;
}

int bmp580::write_reg(uint8_t addr, uint8_t reg, uint8_t data, uint32_t timeout)
{
    if(false==flag)return 1;

    int ret = 0;
    ret = HAL_I2C_Mem_Write(p_hi2c, (addr<<1), reg, I2C_MEMADD_SIZE_8BIT, &data, 1, timeout);
    if(ret){
        printf("bmp580: write_reg %d: NO: %d\r\n", reg, ret);
    }

    return ret;
}














