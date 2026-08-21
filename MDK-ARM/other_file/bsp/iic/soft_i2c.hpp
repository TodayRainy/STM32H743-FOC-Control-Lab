#ifndef _SOFT_I2C_HPP
#define _SOFT_I2C_HPP



#ifdef __cplusplus
extern "C" {  // 告诉 C++ 编译器：这部分用 C 的方式编译
#endif

#include <stdint.h>
#include <gpio.h>
#include <sys.h>
#include <i2c.h>
#include <delay.h>

#ifdef __cplusplus
}
#endif


#ifdef __cplusplus

class soft_i2c
{
private:
    /* data */
    GPIO_TypeDef* GPIOx_SCL;
    uint16_t GPIO_Pin_SCL;
    GPIO_TypeDef* GPIOx_SDA;
    uint16_t GPIO_Pin_SDA;

    uint16_t scl_pin;
    uint16_t sda_pin;

    void SCL_IN(void){
        GPIOx_SCL->MODER&=~(3<<(scl_pin*2));
        GPIOx_SCL->MODER|=0<<scl_pin*2;
    }
    
    void SCL_OUT(void){
        GPIOx_SCL->MODER&=~(3<<(scl_pin*2));
        GPIOx_SCL->MODER|=1<<scl_pin*2;
    }

    void SDA_IN(void){
        GPIOx_SDA->MODER&=~(3<<(sda_pin*2));
        GPIOx_SDA->MODER|=0<<sda_pin*2;
    }

    void SDA_OUT(void){
        GPIOx_SDA->MODER&=~(3<<(sda_pin*2));
        GPIOx_SDA->MODER|=1<<sda_pin*2;
    }

    void IIC_SCL(uint8_t n){
        if(n){
            HAL_GPIO_WritePin(GPIOx_SCL, GPIO_Pin_SCL, GPIO_PIN_SET);
        }else{
            HAL_GPIO_WritePin(GPIOx_SCL, GPIO_Pin_SCL, GPIO_PIN_RESET);
        }
    }

    void IIC_SDA(uint8_t n){
        if(n){
            HAL_GPIO_WritePin(GPIOx_SDA, GPIO_Pin_SDA, GPIO_PIN_SET);
        }else{
            HAL_GPIO_WritePin(GPIOx_SDA, GPIO_Pin_SDA, GPIO_PIN_RESET);
        }
    }

    GPIO_PinState READ_SDA(void){
        return HAL_GPIO_ReadPin(GPIOx_SDA, GPIO_Pin_SDA);
    }

    GPIO_PinState READ_SCL(void){
        return HAL_GPIO_ReadPin(GPIOx_SCL, GPIO_Pin_SCL);
    }


    void TwiStart(void);
    void TwiReStart(void);
    void TwiStop(void);

    bool TwiChkClkRelease(void);
    bool TwiSendData(uint8_t Data, uint8_t ClkFlg);
    uint8_t TwiGetData(uint8_t AckFlg);


    #if 1


    #endif





public:
    soft_i2c(GPIO_TypeDef* GPIOx_SCL, uint16_t GPIO_Pin_SCL, GPIO_TypeDef* GPIOx_SDA, uint16_t GPIO_Pin_SDA);
    ~soft_i2c();

    
    bool TwiWrite(uint8_t SlaveID,uint16_t WrAddr, uint8_t Length, uint8_t   *WrBuf);
    uint8_t TwiRead(uint8_t SlaveID, uint8_t RdAddr, uint8_t Length, uint8_t   *RdBuf);

    //IIC发送一个字节
    //返回从机有无应答
    //1，有应答
    //0，无应答			  
    void IIC_Send_Byte(u8 txd)
    {                        
        u8 t;   
        SDA_OUT(); 	    
        IIC_SCL(0);                 //拉低时钟开始数据传输
        for(t=0;t<8;t++)
        {              
            IIC_SDA((txd&0x80)>>7);
            txd<<=1; 	  
            delay_us(50);            //对TEA5767这三个延时都是必须的
            IIC_SCL(1);
            delay_us(50); 
            IIC_SCL(0);	
            delay_us(50);
        }	 
    } 	

    //读1个字节，ack=1时，发送ACK，ack=0，发送nACK   
    u8 IIC_Read_Byte(unsigned char ack)
    {
        unsigned char i,receive=0;
        SDA_IN();                   //SDA设置为输入
        for(i=0;i<8;i++ )
        {
            IIC_SCL(0); 
            delay_us(50);
            IIC_SCL(1);
            receive<<=1;
            if(READ_SDA())receive++;   
            delay_us(1); 
        }					 
        if (!ack)
            IIC_NAck();             //发送nACK
        else
            IIC_Ack();              //发送ACK   
        return receive;
    }

    void IIC_Start(void)
    {
        SDA_OUT();     //sda线输出

        IIC_SDA(1);	  	  
        IIC_SCL(1);
        delay_us(4);

        IIC_SDA(0);//START:when CLK is high,DATA change form high to low 
        delay_us(4);

        IIC_SCL(0);//钳住I2C总线，准备发送或接收数据 
    }

    //产生IIC停止信号
    void IIC_Stop(void)
    {
        SDA_OUT();//sda线输出

        IIC_SCL(0);
        IIC_SDA(0);//STOP:when CLK is high DATA change form low to high

        delay_us(4);
        IIC_SCL(1); 
        IIC_SDA(1);//发送I2C总线结束信号
        delay_us(4);							   	
    }

    //等待应答信号到来
    //返回值：1，接收应答失败
    //        0，接收应答成功
    u8 IIC_Wait_Ack(void)
    {
        uint16_t ucErrTime=0;
        SDA_IN();      //SDA设置为输入  
        IIC_SDA(1);delay_us(1);	   
        IIC_SCL(1);delay_us(1);	 
        while(READ_SDA())
        {
            ucErrTime++;
            // if(ucErrTime>250)
            if(ucErrTime>1000)
            {
                IIC_Stop();
                return 1;
            }
        }
        IIC_SCL(0);//时钟输出0 	   
        return 0;  
    }

    //产生ACK应答
    void IIC_Ack(void)
    {
        IIC_SCL(0);
        SDA_OUT();
        IIC_SDA(0);
        delay_us(50);
        IIC_SCL(1);
        delay_us(50);
        IIC_SCL(0);
    }

    //不产生ACK应答		    
    void IIC_NAck(void)
    {
        IIC_SCL(0);
        SDA_OUT();
        IIC_SDA(1);
        delay_us(50);
        IIC_SCL(1);
        delay_us(50);
        IIC_SCL(0);
    }




};




#endif


#endif
