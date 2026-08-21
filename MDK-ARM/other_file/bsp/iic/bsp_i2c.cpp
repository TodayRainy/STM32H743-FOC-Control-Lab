#include "bsp_i2c.hpp"






bsp_i2c::bsp_i2c(GPIO_TypeDef *GPIOx_SCL, uint16_t GPIO_Pin_SCL, GPIO_TypeDef *GPIOx_SDA, uint16_t GPIO_Pin_SDA)
{
    this->flag = false;

    this->GPIOx_SCL = GPIOx_SCL;
    this->GPIO_Pin_SCL = GPIO_Pin_SCL;
    this->GPIOx_SDA = GPIOx_SDA;
    this->GPIO_Pin_SDA = GPIO_Pin_SDA;

    if(GPIOx_SCL&&GPIOx_SDA){
        this->flag = true;
    }
}

bsp_i2c::~bsp_i2c()
{
    
}

//发送IIC开始信号s
void bsp_i2c::IIC_Start(void)
{
    SDA_OUT();      // sda线输出
	IIC_SDA(1);	  	  
	IIC_SCL(1);
	delay_us(4);
 	IIC_SDA(0);     // START:when CLK is high,DATA change form high to low 
	delay_us(4);
	IIC_SCL(0);     // 钳住I2C总线，准备发送或接收数据
}		

void bsp_i2c::IIC_Stop(void)
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
uint8_t bsp_i2c::IIC_Wait_Ack(void)
{
    uint8_t ucErrTime=0;
    SDA_IN();      //SDA设置为输入  
    IIC_SDA(1);delay_us(1);	   
    IIC_SCL(1);delay_us(1);	 
    while(READ_SDA())
    {
        ucErrTime++;
        if(ucErrTime>250)
        {
            IIC_Stop();
            return 1;
        }
    }
    IIC_SCL(0);//时钟输出0 	   
    return 0;  
} 

//产生ACK应答
void bsp_i2c::IIC_Ack(void)
{
    IIC_SCL(0);
    SDA_OUT();
    IIC_SDA(0);
    delay_us(2);
    IIC_SCL(1);
    delay_us(2);
    IIC_SCL(0);
}

//不产生ACK应答		    
void bsp_i2c::IIC_NAck(void)
{
    IIC_SCL(0);
    SDA_OUT();
    IIC_SDA(1);
    delay_us(2);
    IIC_SCL(1);
    delay_us(2);
    IIC_SCL(0);
}

//IIC发送一个字节
//返回从机有无应答
//1，有应答
//0，无应答			  
void bsp_i2c::IIC_Send_Byte(uint8_t txd)
{                        
    uint8_t t;   
    SDA_OUT(); 	    
    IIC_SCL(0);         //拉低时钟开始数据传输
    for(t=0;t<8;t++)
    {           
        IIC_SDA((txd&0x80)>>7);
        txd<<=1; 	  
        delay_us(2);    //对TEA5767这三个延时都是必须的
        IIC_SCL(1);
        delay_us(2); 
        IIC_SCL(0);	
        delay_us(2);
    }	 
}

//读1个字节，ack=1时，发送ACK，ack=0，发送nACK   
uint8_t bsp_i2c::IIC_Read_Byte(uint8_t ack)
{
    uint8_t i, receive=0;

    SDA_IN();                       //SDA设置为输入

    for(i=0;i<8;i++ )
    {
        IIC_SCL(0); 
        delay_us(2);
        IIC_SCL(1);
        receive<<=1;
        if(READ_SDA())receive++;   
        delay_us(1); 
    }					 
    if (!ack)
        IIC_NAck();                 //发送nACK
    else
        IIC_Ack();                  //发送ACK   
    return receive;
}














