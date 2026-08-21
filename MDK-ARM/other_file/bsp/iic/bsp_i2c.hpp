#ifndef _BSP_I2C_HPP
#define _BSP_I2C_HPP



#ifdef __cplusplus
extern "C" {  // 告诉 C++ 编译器：这部分用 C 的方式编译
#endif

#include <stdint.h>
#include <gpio.h>
#include <delay.h>


#ifdef __cplusplus
}
#endif





#ifdef __cplusplus

class bsp_i2c
{
private:
    /* data */
    bool flag;

    GPIO_TypeDef *GPIOx_SCL;
    GPIO_TypeDef *GPIOx_SDA;
    uint16_t GPIO_Pin_SCL;
    uint16_t GPIO_Pin_SDA;


    void SDA_IN(void){
        GPIOx_SDA->MODER&=~(3<<(5*2)); GPIOx_SDA->MODER|=0<<5*2;
    }

    void SDA_OUT(void){
        GPIOx_SDA->MODER&=~(3<<(5*2)); GPIOx_SDA->MODER|=1<<5*2;
    }

    void IIC_SCL(uint8_t sta){
        if(sta){
            HAL_GPIO_WritePin(GPIOx_SCL, GPIO_Pin_SCL, GPIO_PIN_SET);
        }else{
            HAL_GPIO_WritePin(GPIOx_SCL, GPIO_Pin_SCL, GPIO_PIN_RESET);
        }
    }

    void IIC_SDA(uint8_t sta){
        if(sta){
            HAL_GPIO_WritePin(GPIOx_SDA, GPIO_Pin_SDA, GPIO_PIN_SET);
        }else{
            HAL_GPIO_WritePin(GPIOx_SDA, GPIO_Pin_SDA, GPIO_PIN_RESET);
        }
    }

    GPIO_PinState READ_SDA(void){
        return HAL_GPIO_ReadPin(GPIOx_SDA, GPIO_Pin_SDA);
    }




public:
    bsp_i2c(GPIO_TypeDef *GPIOx_SCL, uint16_t GPIO_Pin_SCL, GPIO_TypeDef *GPIOx_SDA, uint16_t GPIO_Pin_SDA);
    ~bsp_i2c();

    void IIC_Send_Byte(uint8_t txd);
    uint8_t IIC_Read_Byte(uint8_t ack);

    void IIC_Start(void);				//发送IIC开始信号
    void IIC_Stop(void);
    void IIC_Ack(void);
    void IIC_NAck(void);
    uint8_t IIC_Wait_Ack(void);



};




#endif





#endif
