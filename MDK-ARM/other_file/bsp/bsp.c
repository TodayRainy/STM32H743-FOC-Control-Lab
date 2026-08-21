// // #include "bsp.h"
// // #include <string.h>
// // #include <stdbool.h>
// // #include <i2c.h>
// #include "stm32h7xx_hal_pcd.h"
// // // 

// // void BSP_Init(void)
// // {
// //     delay_init(400);
    
// //     uart_init(115200);

// //     // MPU_Memory_Protection();                        /*内存保护*/

// //     // Cache_Enable();                                 //打开L1-Cache

// //     // ICM4688_Init();

// //     ICM42688P_Init();

// //     delay_ms(100);


// //     printf("BSP_Init: OK\r\n");

// //     // LAN8720_Init();
// //     // lwip_init();
// //     // Netif_Config();
// //     // lwip_comm_init();
// //     HAL_Delay(1000);
// // }




// // void BSP_Flash_Init(void)
// // {

// // }

// // //挂载文件系统
// // void BSP_Flash_mount(void)
// // {

// // }



// // //显示初始化
// // void BSP_Display_Init(void)
// // {

    

// // }

// // //触屏初始化
// // void BSP_Touch_Init(void)
// // {
// //    // GT9147_Init();
// // }



// #include "icm42688p.hpp"



// //icm42688p icm42688p0(&hspi1, SPI1_CS_GPIO_Port, SPI4_CS_Pin);

// /* 寄存器配置 */
// void icm42688p::config(void)
// {
//     int ret = 0;

// #if 1
//     ret |= write_reg(ICM_DEVICE_CONFIG, 0x00);     /* SPI模式0 软复位*/
//     HAL_Delay(50);

//     ret |= write_reg(ICM_REG_BANK_SEL, 0x00);      /* 切换回 Bank 0 */
//     HAL_Delay(50);

//     ret |= write_reg(ICM_DRIVE_CONFIG, 0x05);      /* SPI_SLEW_RATE:101  I2C_SLEW_RATE:000 */
//     HAL_Delay(50);

//     ret |= write_reg(ICM_INT_CONFIG, 0x02);        /* 中断1配置 推挽 低电平激活 脉冲模式 */
//     HAL_Delay(50);

//     ret |= write_reg(ICM_PWR_MGMT0, 0x0F);         /* 功耗配置 加速度计:低噪声模式。陀螺仪:低噪声模式 温度传感器启用 */
//     HAL_Delay(50);

//     ret |= write_reg(ICM_INT_CONFIG1, 0x00);       /* 中断1配置 */
//     HAL_Delay(50);

//     ret |= write_reg(ICM_INT_SOURCE0, 0x08);       /* 中断源配置 数据就绪中断*/
//     HAL_Delay(50);

//     /*陀螺仪初始化*/
//     ret |= write_reg(ICM_GYRO_CONFIG0, 0x06);       /* 陀螺仪初始化 1khz ±2000dps*/
//     HAL_Delay(50);
//     ret |= write_reg(ICM_GYRO_CONFIG1, 0x16);       /* 陀螺仪初始化 调整带宽和滤波次数 */
//     HAL_Delay(50);
//     ret |= write_reg(ICM_GYRO_ACCEL_CONFIG0, 0x16); /* 陀螺仪初始化 调整陀螺仪和加速度计的低通滤波器带宽 */
//     HAL_Delay(50);

//     /*加速度计初始化*/
//     ret |= write_reg(ICM_ACCEL_CONFIG0, 0x06);       /* 加速度计初始化 1khz ±16g*/
//     HAL_Delay(50);
//     ret |= write_reg(ICM_ACCEL_CONFIG1, 0x0D);       /* 加速度计初始化 调整带宽和滤波次数*/
//     HAL_Delay(50);

// #else
//     /*系统自检*/
//     // ret |= write_reg(ICM_SELF_TEST_CONFIG, 0xFF);    
//     // HAL_Delay(50);

//     ret |= write_reg(ICM_DEVICE_CONFIG, 0x01);      /* SPI模式0 软复位*/
//     HAL_Delay(50);

//     ret |= write_reg(ICM_PWR_MGMT0, 0x0F);          /* 启用加速度计和陀螺仪（低噪声模式）*/
//     HAL_Delay(50);

//     ret |= write_reg(ICM_GYRO_CONFIG0, 0x06);       /* 陀螺仪量程±2000dps，ODR=1kHz：FS_SEL=0b11（±2000dps）, ODR=0x06 */
//     HAL_Delay(50);

//     ret |= write_reg(ICM_ACCEL_CONFIG0, 0x06);      /* 加速度计量程±16g，ODR=1kHz： FS_SEL=0b11（±16g）, ODR=0x06 */
//     HAL_Delay(50);

//     ret |= write_reg(ICM_INT_CONFIG, 0x00);         /* ?禁用所有中断?（因用户要求） */
//     HAL_Delay(50);

//     ret |= write_reg(ICM_FIFO_CONFIG, 0x00);        /* ?禁用所有FIFO?（因用户要求） */
//     HAL_Delay(50);
// #endif

//     #if 0
//     uint8_t data = 0;
//     read_reg(ICM_DRIVE_CONFIG, data);       printf("ICM_DRIVE_CONFIG:       0x%02X\r\n", data);
//     read_reg(ICM_INT_CONFIG, data);         printf("ICM_INT_CONFIG:         0x%02X\r\n", data);
//     read_reg(ICM_PWR_MGMT0, data);          printf("ICM_PWR_MGMT0:          0x%02X\r\n", data);
//     read_reg(ICM_INT_CONFIG1, data);        printf("ICM_INT_CONFIG1:        0x%02X\r\n", data);
//     read_reg(ICM_INT_SOURCE0, data);        printf("ICM_INT_SOURCE0:        0x%02X\r\n", data);
//     read_reg(ICM_GYRO_CONFIG0, data);       printf("ICM_GYRO_CONFIG0:       0x%02X\r\n", data);
//     read_reg(ICM_GYRO_CONFIG1, data);       printf("ICM_GYRO_CONFIG1:       0x%02X\r\n", data);
//     read_reg(ICM_GYRO_ACCEL_CONFIG0, data); printf("ICM_GYRO_ACCEL_CONFIG0: 0x%02X\r\n", data);
//     read_reg(ICM_ACCEL_CONFIG0, data);      printf("ICM_ACCEL_CONFIG0:      0x%02X\r\n", data);
//     read_reg(ICM_ACCEL_CONFIG1, data);      printf("ICM_ACCEL_CONFIG1:      0x%02X\r\n", data);
//     #endif

//     if(ret){
//         printf("ICM42688P 配置: NO\r\n");
//         while(1);
//     }else{
//         printf("ICM42688P 配置: OK\r\n");
//     }

// }

// icm42688p::icm42688p(SPI_HandleTypeDef*p_hspi, GPIO_TypeDef *csPort, uint16_t csPin)
// {
//     this->flag = false;

//     if(NULL==p_hspi || NULL==csPort)return;

//     this->p_hspi    = p_hspi;
//     this->csPort    = csPort;
//     this->csPin     = csPin;
//     this->flag = true;


//     uint8_t id = 0;
//     read_id(id);
//     printf("icm42688p id: 0x%2X\r\n", id);

//     config();

// }

// icm42688p::~icm42688p()
// {
// }



// int icm42688p::read_reg(uint8_t reg, uint8_t &data, uint32_t timeout)
// {
//     if(false==flag)return -10;

//     int ret = 0;

//     uint8_t tx = reg | ICM42688P_READ_FLAG;

//     cs(0);

//     /*发送地址*/
//     ret |= HAL_SPI_Transmit(p_hspi, &tx, 1, timeout);

//     /*读取数据*/
//     ret |= HAL_SPI_Receive(p_hspi, &data, 1, timeout);

//     if(ret){
//         printf("read_reg: NO\r\n");
//     }
//     cs(1);

//     return ret;
// }

// int icm42688p::read_reg(uint8_t reg, uint8_t *data, uint32_t len, uint32_t timeout)
// {
//     if(false==flag)return -10;

//     int ret = 0;

//     uint8_t tx = reg | ICM42688P_READ_FLAG;

//     cs(0);

//     /*发送地址*/
//     ret |= HAL_SPI_Transmit(p_hspi, &tx, 1, timeout);

//     /*读取数据*/
//     ret |= HAL_SPI_Receive(p_hspi, data, len, timeout);

//     if(ret){
//         printf("read_reg: NO\r\n");
//     }
//     cs(1);

//     return ret;
// }


// int icm42688p::read_id(uint8_t &data, uint32_t timeout)
// {
//     int ret = read_reg(WHO_AM_I_REG, data, timeout);

//     return ret;
// }


// int icm42688p::write_reg(uint8_t reg, uint8_t data, uint32_t timeout)
// {
//     int ret = 0;
//     if(false==flag)return -11;
//     uint8_t tx[2] = {};

//     tx[0] = reg;
//     tx[1] = data;

//     cs(0);
//     ret = HAL_SPI_Transmit(p_hspi, tx, 2, timeout);
//     cs(1);

//     return ret;
// }


// /*获取温度*/
// float icm42688p::get_temp(void)
// {   
//     uint8_t rx[2] = {};
//     uint16_t temp = 0;

//     // read_reg(ICM_TEMP_DATA1, data);
//     // temp |= (data<<8);

//     // read_reg(ICM_TEMP_DATA0, data);
//     // temp |= (data<<0);


//     read_reg(ICM_TEMP_DATA1, rx, 2);

//     temp = rx[0]<<8 | rx[1];

//     float temp_f = ((float)temp/132.48f)+25.0f;

//     return temp_f;
// }

// /*获取加速度计和陀螺仪*/
// int icm42688p::get_acc_gyro(void)
// {
//     uint8_t rx[12] = {};

//     int16_t gyro[3] = {};
//     int16_t accel[3] = {};

//     float gyro_f[3] = {};
//     float accel_f[3] = {};

//     uint8_t status = 0;
//     read_reg(ICM_INT_STATUS, &status, 1); 
//     if (!(status & 0x08)){
//         return 0;
//     }

//     read_reg(ICM_ACCEL_DATA_X1, rx, 12);

//     accel[0] = (int16_t)(rx[0]<<8 | rx[1]<<0);
//     accel[1] = (int16_t)(rx[2]<<8 | rx[3]<<0);
//     accel[2] = (int16_t)(rx[4]<<8 | rx[5]<<0);

//     gyro[0] = (int16_t)(rx[6]<<8 | rx[7]<<0);
//     gyro[1] = (int16_t)(rx[8]<<8 | rx[9]<<0);
//     gyro[2] = (int16_t)(rx[10]<<8 | rx[11]<<0);

//     gyro_f[0] = (float)gyro[0]/32768.0f *2000.0f;
//     gyro_f[1] = (float)gyro[1]/32768.0f *2000.0f;
//     gyro_f[2] = (float)gyro[2]/32768.0f *2000.0f;

//     accel_f[0] = (float)accel[0]/32768.0f *16.0f*9.8f;
//     accel_f[1] = (float)accel[1]/32768.0f *16.0f*9.8f;
//     accel_f[2] = (float)accel[2]/32768.0f *16.0f*9.8f;


//     // printf("accel: x: %d  y: %d  z:%d\r\n", accel[0], accel[1], accel[2]);
//     // printf("gyro: x: %d  y: %d  z:%d\r\n\r\n", gyro[0], gyro[1], gyro[2]);

//     printf("加速度: x: %.4f  y: %.4f  z:%.4f  总体值: %.4f\r\n", accel_f[0], accel_f[1], accel_f[2], 
//             sqrtf(accel_f[0]*accel_f[0]+ accel_f[1]*accel_f[1]+accel_f[2]*accel_f[2]));
//     printf("角速度:  x: %.4f  y: %.4f  z:%.4f\r\n\r\n", gyro_f[0], gyro_f[1], gyro_f[2]);

//     return 0;
// }












