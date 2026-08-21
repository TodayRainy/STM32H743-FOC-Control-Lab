#ifndef _ICM42688P_HPP
#define _ICM42688P_HPP




extern "C" {

}



#ifdef __cplusplus
extern "C" {  // 告诉 C++ 编译器：这部分用 C 的方式编译
#endif


#include <stdint.h>
#include <gpio.h>
#include <sys.h>
#include <spi.h>


#ifdef __cplusplus
}
#endif


#ifdef __cplusplus

/*寄存器列表*/
#define WHO_AM_I_REG  0x75
#define TIMEOUT       100
#define ICM42688P_READ_FLAG     0x80


/*************************************寄存器Bank0************************************************/
#define ICM_DEVICE_CONFIG               0x11        /*设备配置*/
#define ICM_DRIVE_CONFIG                0x13        /*驱动配置*/
#define ICM_INT_CONFIG                  0x14        /*中断配置*/
#define ICM_FIFO_CONFIG                 0x16        /*fifo配置*/
#define ICM_TEMP_DATA1                  0x1D        /*温度寄存器高字节*/
#define ICM_TEMP_DATA0                  0x1E        /*温度寄存器低字节*/

#define ICM_ACCEL_DATA_X1               0x1F        /*加速度 X轴 高字节*/
#define ICM_ACCEL_DATA_X0               0x20        /*加速度 X轴 低字节*/
#define ICM_ACCEL_DATA_Y1               0x21        /*加速度 Y轴 高字节*/    
#define ICM_ACCEL_DATA_Y0               0x22        /*加速度 Y轴 低字节*/
#define ICM_ACCEL_DATA_Z1               0x23        /*加速度 Z轴 高字节*/
#define ICM_ACCEL_DATA_Z0               0x24        /*加速度 Z轴 低字节*/
#define ICM_GYRO_DATA_X1                0x25        /*陀螺仪 X轴 高字节*/
#define ICM_GYRO_DATA_X0                0x26        /*陀螺仪 X轴 低字节*/
#define ICM_GYRO_DATA_Y1                0x27        /*陀螺仪 Y轴 高字节*/
#define ICM_GYRO_DATA_Y0                0x28        /*陀螺仪 Y轴 低字节*/
#define ICM_GYRO_DATA_Z1                0x29        /*陀螺仪 Z轴 高字节*/
#define ICM_GYRO_DATA_Z0                0x2A        /*陀螺仪 Z轴 低字节*/

#define ICM_TMST_FSYNCH                 0x2B
#define ICM_TMST_FSYNCL                 0x2C

#define ICM_INT_STATUS                  0x2D        /*中断状态*/
#define ICM_FIFO_COUNTH                 0x2E        /*FIFO长度*/
#define ICM_FIFO_COUNTL                 0x2F        /*FIFO长度*/
#define ICM_FIFO_DATA                   0x30        /*FIFO数据*/

#define ICM_APEX_DATA0                  0x31        /*计步器步数 低字节*/
#define ICM_APEX_DATA1                  0x32        /*计步器步数 高字节*/
#define ICM_APEX_DATA2                  0x33        /*步频*/
#define ICM_APEX_DATA3                  0x34        
#define ICM_APEX_DATA4                  0x35        
#define ICM_APEX_DATA5                  0x36

#define ICM_INT_STATUS2                 0x37        /*中断状态2*/
#define ICM_INT_STATUS3                 0x38        /*中断状态3*/
#define ICM_SIGNAL_PATH_RESET           0x4B        

#define ICM_INTF_CONFIG0                0x4C        /*中断配置0*/
#define ICM_INTF_CONFIG1                0x4D        /*中断配置1*/

#define ICM_PWR_MGMT0                   0x4E        /*电源配置*/
#define ICM_GYRO_CONFIG0                0x4F        /*陀螺仪配置*/
#define ICM_ACCEL_CONFIG0               0x50        /*加速度计配置*/
#define ICM_GYRO_CONFIG1                0x51        
#define ICM_GYRO_ACCEL_CONFIG0          0x52        /*陀螺仪 加速度计配置*/ 
#define ICM_ACCEL_CONFIG1               0x53
#define ICM_TMST_CONFIG                 0x54

#define ICM_APEX_CONFIG0                0x56        /*APEX配置*/
#define ICM_SMD_CONFIG                  0x57

#define ICM_FIFO_CONFIG1                0x5F        /*FIFO 配置*/
#define ICM_FIFO_CONFIG2                0x60
#define ICM_FIFO_CONFIG3                0x61
#define ICM_FSYNC_CONFIG                0x62

#define ICM_INT_CONFIG0                 0x63
#define ICM_INT_CONFIG1                 0x64
#define ICM_INT_SOURCE0                 0x65
#define ICM_INT_SOURCE1                 0x66
#define ICM_INT_SOURCE3                 0x68
#define ICM_INT_SOURCE4                 0x69

#define ICM_FIFO_LOST_PKT0              0x6C
#define ICM_FIFO_LOST_PKT1              0x6D

#define ICM_SELF_TEST_CONFIG            0x70

#define ICM_WHO_AM_I                    0x75
#define ICM_REG_BANK_SEL                0x76            /*BANK选择*/



/* 陀螺仪量程枚举 (位7:5)*/
typedef enum{
    GYRO_FS_2000DPS   = (0 << 5),  // ±2000 dps (默认)
    GYRO_FS_1000DPS   = (1 << 5),  // ±1000 dps
    GYRO_FS_500DPS    = (2 << 5),  // ±500 dps
    GYRO_FS_250DPS    = (3 << 5),  // ±250 dps
    GYRO_FS_125DPS    = (4 << 5),  // ±125 dps
    GYRO_FS_62_5DPS   = (5 << 5),  // ±62.5 dps
    GYRO_FS_31_25DPS  = (6 << 5),  // ±31.25 dps
    GYRO_FS_15_625DPS = (7 << 5)   // ±15.625 dps
}GYRO_FS_SEL_t;

/*陀螺仪ODR枚举 (位3:0)*/
typedef enum {
    GYRO_ODR_RESERVED_0   = 0x0,
    GYRO_ODR_32000HZ      = 0x1,
    GYRO_ODR_16000HZ      = 0x2,
    GYRO_ODR_8000HZ       = 0x3,
    GYRO_ODR_4000HZ       = 0x4,
    GYRO_ODR_2000HZ       = 0x5,
    GYRO_ODR_1000HZ       = 0x6,  // 默认
    GYRO_ODR_200HZ        = 0x7,
    GYRO_ODR_100HZ        = 0x8,
    GYRO_ODR_50HZ         = 0x9,
    GYRO_ODR_25HZ         = 0xA,
    GYRO_ODR_12_5HZ       = 0xB,
    GYRO_ODR_RESERVED_C   = 0xC,
    GYRO_ODR_RESERVED_D   = 0xD,
    GYRO_ODR_RESERVED_E   = 0xE,
    GYRO_ODR_500HZ        = 0xF
} GyroODR_t;

/* TEMP_FILT_BW 位7:5，温度信号DLPF带宽和延迟 */
typedef enum {
    TEMP_FILT_BW_4000HZ = 0x0 << 5,     // 4000 Hz, 延迟0.125ms (默认)
    TEMP_FILT_BW_170HZ  = 0x1 << 5,     // 170 Hz, 延迟1ms
    TEMP_FILT_BW_82HZ   = 0x2 << 5,     // 82 Hz, 延迟2ms
    TEMP_FILT_BW_40HZ   = 0x3 << 5,     // 40 Hz, 延迟4ms
    TEMP_FILT_BW_20HZ   = 0x4 << 5,     // 20 Hz, 延迟8ms
    TEMP_FILT_BW_10HZ   = 0x5 << 5,     // 10 Hz, 延迟16ms
    TEMP_FILT_BW_5HZ_1  = 0x6 << 5,     // 5 Hz, 延迟32ms
    TEMP_FILT_BW_5HZ_2  = 0x7 << 5      // 5 Hz, 延迟32ms
} TempFiltBW_t;

/* GYRO_UI_FILT_ORD 位3:2，陀螺仪主滤波器阶数 */
typedef enum {
    GYRO_UI_FILT_ORDER_1 = 0x0 << 2,  // 1阶滤波器
    GYRO_UI_FILT_ORDER_2 = 0x1 << 2,  // 2阶滤波器 (默认)
    GYRO_UI_FILT_ORDER_3 = 0x2 << 2   // 3阶滤波器
    // 0x3 保留
} GyroUIFiltOrder_t;

/* GYRO_DEC2_M2_ORD 位1:0，陀螺仪第二级滤波器阶数 */
typedef enum {
    GYRO_DEC2_M2_RESERVED_0 = 0x0,  // 保留
    GYRO_DEC2_M2_RESERVED_1 = 0x1,  // 保留
    GYRO_DEC2_M2_ORDER_3    = 0x2   // 3阶滤波器 (推荐)
    // 0x3 保留
} GyroDec2M2Order_t;

/* 加速度计滤波器带宽（ACCEL_UI_FILT_BW，位7:4） - LN模式下 */
typedef enum {
    ACCEL_UI_FILT_BW_ODR_DIV_2              = 0x0 << 4,   // BW = ODR/2
    ACCEL_UI_FILT_BW_MAX400HZ_ODR_DIV_4     = 0x1 << 4,   // BW = max(400Hz, ODR)/4 (默认)
    ACCEL_UI_FILT_BW_MAX400HZ_ODR_DIV_5     = 0x2 << 4,   // BW = max(400Hz, ODR)/5
    ACCEL_UI_FILT_BW_MAX400HZ_ODR_DIV_8     = 0x3 << 4,   // BW = max(400Hz, ODR)/8
    ACCEL_UI_FILT_BW_MAX400HZ_ODR_DIV_10    = 0x4 << 4,   // BW = max(400Hz, ODR)/10
    ACCEL_UI_FILT_BW_MAX400HZ_ODR_DIV_16    = 0x5 << 4,   // BW = max(400Hz, ODR)/16
    ACCEL_UI_FILT_BW_MAX400HZ_ODR_DIV_20    = 0x6 << 4,   // BW = max(400Hz, ODR)/20
    ACCEL_UI_FILT_BW_MAX400HZ_ODR_DIV_40    = 0x7 << 4    // BW = max(400Hz, ODR)/40
    // 8~13 Reserved
    // 14~15 低延迟选项未包含在此枚举，需单独处理
} AccelFiltBW_t;

/* 陀螺仪滤波器带宽（GYRO_UI_FILT_BW，位3:0） - LN模式下 */
typedef enum {
    GYRO_UI_FILT_BW_ODR_DIV_2           = 0x0,  // BW = ODR/2
    GYRO_UI_FILT_BW_MAX400HZ_ODR_DIV_4  = 0x1,  // BW = max(400Hz, ODR)/4 (默认)
    GYRO_UI_FILT_BW_MAX400HZ_ODR_DIV_5  = 0x2,  // BW = max(400Hz, ODR)/5
    GYRO_UI_FILT_BW_MAX400HZ_ODR_DIV_8  = 0x3,  // BW = max(400Hz, ODR)/8
    GYRO_UI_FILT_BW_MAX400HZ_ODR_DIV_10 = 0x4,  // BW = max(400Hz, ODR)/10
    GYRO_UI_FILT_BW_MAX400HZ_ODR_DIV_16 = 0x5,  // BW = max(400Hz, ODR)/16
    GYRO_UI_FILT_BW_MAX400HZ_ODR_DIV_20 = 0x6,  // BW = max(400Hz, ODR)/20
    GYRO_UI_FILT_BW_MAX400HZ_ODR_DIV_40 = 0x7   // BW = max(400Hz, ODR)/40
    // 8~13 Reserved
    // 14~15 低延迟选项需单独处理
} GyroFiltBW_t;


/* 加速度计满量程选择 (位7:5) */
typedef enum {
    ACCEL_FS_16G = 0x0 << 5,  // ±16g (默认)
    ACCEL_FS_8G  = 0x1 << 5,  // ±8g
    ACCEL_FS_4G  = 0x2 << 5,  // ±4g
    ACCEL_FS_2G  = 0x3 << 5   // ±2g
    // 其他值保留
} AccelFullScale_t;

/* 加速度计输出数据率ODR选择 (位3:0) */
typedef enum {
    ACCEL_ODR_32KHZ     = 0x1,  // 32kHz (LN模式)
    ACCEL_ODR_16KHZ     = 0x2,  // 16kHz (LN模式)
    ACCEL_ODR_8KHZ      = 0x3,  // 8kHz (LN模式)
    ACCEL_ODR_4KHZ      = 0x4,  // 4kHz (LN模式)
    ACCEL_ODR_2KHZ      = 0x5,  // 2kHz (LN模式)
    ACCEL_ODR_1KHZ      = 0x6,  // 1kHz (LN模式) (默认)
    ACCEL_ODR_200HZ     = 0x7,  // 200Hz (LP或LN模式)
    ACCEL_ODR_100HZ     = 0x8,  // 100Hz (LP或LN模式)
    ACCEL_ODR_50HZ      = 0x9,  // 50Hz (LP或LN模式)
    ACCEL_ODR_25HZ      = 0xA,  // 25Hz (LP或LN模式)
    ACCEL_ODR_12_5HZ    = 0xB,  // 12.5Hz (LP或LN模式)
    ACCEL_ODR_6_25HZ    = 0xC,  // 6.25Hz (LP模式)
    ACCEL_ODR_3_125HZ   = 0xD,  // 3.125Hz (LP模式)
    ACCEL_ODR_1_5625HZ  = 0xE,  // 1.5625Hz (LP模式)
    ACCEL_ODR_500HZ     = 0xF   // 500Hz (LP或LN模式)
} AccelODR_t;

/* ACCEL_UI_FILT_ORD 位4:3，加速度计UI滤波器阶数选择 */
typedef enum {
    ACCEL_UI_FILT_ORDER_1 = 0x0 << 3,  // 1阶滤波器
    ACCEL_UI_FILT_ORDER_2 = 0x1 << 3,  // 2阶滤波器
    ACCEL_UI_FILT_ORDER_3 = 0x2 << 3   // 3阶滤波器
    // 0x3保留
} AccelUIFiltOrder_t;

/* ACCEL_DEC2_M2_ORD 位2:1，加速度计DEC2_M2滤波器阶数选择 */
typedef enum {
    ACCEL_DEC2_M2_RESERVED_0 = 0x0 << 1,  // 保留
    ACCEL_DEC2_M2_RESERVED_1 = 0x1 << 1,  // 保留
    ACCEL_DEC2_M2_ORDER_3    = 0x2 << 1   // 3阶滤波器
    // 0x3保留
} AccelDec2M2Order_t;




class icm42688p
{
private:
    /* data */
    bool flag;

    float acc_bias[3];
    float gyro_bias[3];

    int16_t acc_raw_bias[3];
    int16_t gyro_raw_bias[3];
    
    /*硬件资源*/
    SPI_HandleTypeDef*      p_hspi;
    GPIO_TypeDef*           csPort;
    uint16_t                csPin;

    /*配置信息*/
    GYRO_FS_SEL_t gyro_fs_sel;
    float gyro_scale;

    AccelFullScale_t acc_full;
    float acc_scale;

    /*硬件初始化*/
    void HW_Init(void){};

    void config(void);

    void scale_calculation(void);

    void cs(uint8_t sta)
    {
        if(sta){
            HAL_GPIO_WritePin(csPort, csPin, GPIO_PIN_SET);
        }else{
            HAL_GPIO_WritePin(csPort, csPin, GPIO_PIN_RESET);
        }
    }

    void calibrate_bias(int sample_count = 100);

public:
    icm42688p(SPI_HandleTypeDef*p_hspi, GPIO_TypeDef *csPort, uint16_t csPin);
    ~icm42688p();

    int read_reg(uint8_t reg, uint8_t &data, uint32_t timeout = HAL_MAX_DELAY);

    int read_reg(uint8_t reg, uint8_t *data, uint32_t len, uint32_t timeout = HAL_MAX_DELAY);

    int read_id(uint8_t &data, uint32_t timeout = HAL_MAX_DELAY);

    int write_reg(uint8_t reg, uint8_t data, uint32_t timeout = HAL_MAX_DELAY);
   
    float get_temp(void);

    int get_acc_gyro(float *gyro_x, float *gyro_y, float *gyro_z, float *accel_x, float *accel_y, float *accel_z);

    int get_acc(float *accel_x, float *accel_y, float *accel_z);

    int get_gyro(float *gyro_x, float *gyro_y, float *gyro_z);


    
};




#endif

#endif





