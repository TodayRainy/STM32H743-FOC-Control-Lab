#ifndef _ICM42688P_H
#define _ICM42688P_H


#include <stdint.h>

#include <gpio.h>

#include <sys.h>

/******************************************************************************************/
/* ICM42688P 片选 引脚 定义 */

#define ICM42688P_CS_GPIO_PORT           GPIOA
#define ICM42688P_CS_GPIO_PIN            GPIO_PIN_4
#define ICM42688P_CS_GPIO_CLK_ENABLE()   do{ __HAL_RCC_GPIOA_CLK_ENABLE(); }while(0)   /* PI口时钟使能 */
/* ICM42688P 片选信号 */
#define ICM42688P_CS(x)                  (HAL_GPIO_WritePin(ICM42688P_CS_GPIO_PORT, ICM42688P_CS_GPIO_PIN, x>0 ? GPIO_PIN_SET:GPIO_PIN_RESET))

#define ICM42688P2_CS(x)                 (HAL_GPIO_WritePin(SPI4_CS_GPIO_Port, SPI4_CS_Pin, x>0 ? GPIO_PIN_SET:GPIO_PIN_RESET))

/******************************************************************************************/

#define ICM42688P_WHO_AM_I_REG     0x75
#define ICM42688P_WHO_AM_I_ID      0x47

#define ICM42688P_SPI_TIMEOUT      1000

// 修改为你的 SPI 和 CS 引脚
#define ICM42688P_CS_PORT          GPIOA
#define ICM42688P_CS_PIN           GPIO_PIN_4
#define ICM42688P_CS_LOW()         HAL_GPIO_WritePin(ICM42688P_CS_PORT, ICM42688P_CS_PIN, GPIO_PIN_RESET)
#define ICM42688P_CS_HIGH()        HAL_GPIO_WritePin(ICM42688P_CS_PORT, ICM42688P_CS_PIN, GPIO_PIN_SET)

#define ICM42688P2_CS_LOW()         HAL_GPIO_WritePin(SPI4_CS_GPIO_Port, SPI4_CS_Pin, GPIO_PIN_RESET)
#define ICM42688P2_CS_HIGH()        HAL_GPIO_WritePin(SPI4_CS_GPIO_Port, SPI4_CS_Pin, GPIO_PIN_SET)


uint8_t ICM4688_Init(void);
uint8_t ICM42688P_Read_Reg(uint8_t reg);
void ICM42688P_Write_Reg(uint8_t reg, uint8_t data);
uint8_t ICM42688P_Init(void);










#include "spi.h"

typedef struct{
    /*硬件资源*/
    SPI_HandleTypeDef *hspi;
    GPIO_TypeDef *cs_port;
    uint16_t cs_pin;

    int (*In)(SPI_HandleTypeDef *hspi, GPIO_TypeDef *cs_port, uint16_t cs_pin);
    int (*DeInit)(void);

    void (*cs)(uint8_t sta);

    int (*read_reg)(uint8_t reg, uint8_t *data, uint32_t timeout);

    
}ICM42688Dev_t;





#endif





