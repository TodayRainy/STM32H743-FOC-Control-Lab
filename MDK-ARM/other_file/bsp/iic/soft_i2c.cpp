#include "soft_i2c.hpp"
#include "delay.h"
#include "crc/crc.hpp"

#define AFE_ID		0x34
#define E2PROM_ID	0xA0

//Define EEPROM Size
#define E2PROM_SIZE				0x10000  
#define E2PROM_BOOT_ADDR		0x10000-0x20
#define E2PROM_RTC_ADDR			0x10000-0x10


//Define For RTC Module
#define RTC_ID					0xDE
#define RTC_REG_SECOND			0x00
#define RTC_REG_MINUTE			0x01
#define RTC_REG_HOUR			0x02
#define RTC_REG_DATE			0x03
#define RTC_REG_MONTH			0x04
#define RTC_REG_YEAR			0x05
#define RTC_REG_WEEK			0x06
#define RTC_REG_STATUS			0x07




soft_i2c::soft_i2c(GPIO_TypeDef* GPIOx_SCL, uint16_t GPIO_Pin_SCL, GPIO_TypeDef* GPIOx_SDA, uint16_t GPIO_Pin_SDA)
{
    this->GPIOx_SCL     = GPIOx_SCL;
    this->GPIO_Pin_SCL  = GPIO_Pin_SCL;
    this->GPIOx_SDA     = GPIOx_SDA;
    this->GPIO_Pin_SDA  = GPIO_Pin_SDA;

    for (size_t i = 0; i < 16; i++)
    {
        if (GPIO_Pin_SDA & (1U << i)){
            sda_pin = i;
        }

        if (GPIO_Pin_SCL & (1U << i)){
            scl_pin = i;
        }        
    }

    printf("sda_pin: %d  scl_pin: %d\r\n", scl_pin, sda_pin);


    IIC_SDA(1);
    IIC_SCL(1);
}

soft_i2c::~soft_i2c()
{
}

void soft_i2c::TwiStart(void)
{
 
    IIC_SDA(1);
    IIC_SCL(1);

    SCL_OUT();
    SDA_OUT();

    IIC_SDA(0);     /* START:when CLK is high,DATA change form high to low */
    delay_us(100);
    IIC_SCL(0);     /* 钳住I2C总线，准备发送或接收数据 */

}

void soft_i2c::TwiReStart(void)
{
    SDA_OUT();
    IIC_SDA(1);
    IIC_SCL(1);
    delay_us(100);
    IIC_SDA(0);
    delay_us(100);
    IIC_SCL(0);
}

void soft_i2c::TwiStop(void)
{
    SDA_OUT();
    IIC_SDA(0);
    delay_us(100);
    IIC_SCL(1);
    delay_us(100);
    IIC_SDA(1);
    delay_us(100);
    SDA_IN();
    SCL_IN();
}


bool soft_i2c::TwiChkClkRelease(void)
{
    uint16_t TimeoutCnt=1000;
    bool result=0;

    SCL_IN();

    while (TimeoutCnt--)
    {
        /* code */
        /*RSTSTAT = RSTSTAT;*/
        delay_us(100);
        if(READ_SCL())
		{
			result = 1;
			break;
		}

        IIC_SCL(1);
        SCL_OUT();
    }
    return result;
}

bool soft_i2c::TwiSendData(uint8_t Data, uint8_t ClkFlg)
{
    uint8_t i;
    bool result=0;
    SDA_OUT();
    SCL_OUT();

    if(Data&0x80){
        IIC_SDA(1);
	}else{
        IIC_SDA(0);
    }

    if(ClkFlg==1){
        delay_us(100);
        if(TwiChkClkRelease()){
            IIC_SCL(0);
        }else{
            return result;
        }
    }else{
        delay_us(100);
        IIC_SCL(1);
        delay_us(100);
        IIC_SCL(0);
    }

    /*Send the remaining seven bit*/
    Data = Data<<1;
    for(i=0; i<7; i++)
	{
		if(Data&0x80)
		{
		  IIC_SDA(1);
		}
		else
		{
			IIC_SDA(0);
		}
		Data = Data<<1;
		delay_us(100);
		IIC_SCL(1);
		delay_us(100);
        IIC_SCL(0);
	}
    SDA_IN();
    delay_us(100);
    for(i=0; i<20; i++){
		if(READ_SDA() == 0)
		{
			result = 1;
			break;
		}
		delay_us(1);
	}

    IIC_SCL(1);

	delay_us(100);
	IIC_SDA(0);
	SDA_OUT();
    IIC_SCL(0);
	delay_us(100);
	
	return result;
}

uint8_t soft_i2c::TwiGetData(uint8_t AckFlg)
{
    uint8_t i, RdData=0;

    SDA_IN() ;
	delay_us(100);

    for(i=0; i<8; i++)
	{
        IIC_SCL(1);
		delay_us(100);
		if(READ_SDA())
		{
			RdData |= (1<<(7-i));
		}
        IIC_SCL(0);
		delay_us(100);
	}

    SDA_OUT();

    if(AckFlg != 0){
		IIC_SDA(0);
	}else{
		IIC_SDA(1);
	}

    IIC_SCL(1);
	delay_us(100);
    IIC_SCL(0);
	delay_us(100);
	
	return RdData;


}

bool soft_i2c::TwiWrite(uint8_t SlaveID,uint16_t WrAddr, uint8_t Length, uint8_t   *WrBuf)
{
    uint8_t i;
    uint8_t TempBuf[4];
    bool result = 0;
    TempBuf[0] = SlaveID;

    TempBuf[1] = (uint8_t)WrAddr;
    TempBuf[2] = *WrBuf;
    TempBuf[3] = crc_8(TempBuf, 3);

    if(Length > 0)
	{
		TwiStart();
		
		if(!TwiSendData(SlaveID, 1))			//Send Slave ID
		{
			goto WrErr;
		}
		
		if(SlaveID == E2PROM_ID)
		{
			if(!TwiSendData(WrAddr>>8, 0))		//Send Write Address(High 8bit) For EEPROM
			{
				goto WrErr;
			}
        }
        
		if(TwiSendData(WrAddr, 0))					//Send Write Address(Low 8bit)
		{
			result = 1;
			for(i=0; i<Length; i++)
			{
				if(TwiSendData(*WrBuf, 0))			//Send Write Data
				{
					WrBuf++;
				}
				else
				{
					result = 0;
					break;
				}
			}

            if(SlaveID == AFE_ID)
            {
                if(!TwiSendData(TempBuf[3], 0))            //write CRC
                {
                    result = 0;
                }
            }
		}
WrErr:
		TwiStop();
	}
	
	return result;
}

uint8_t soft_i2c::TwiRead(uint8_t SlaveID, uint8_t RdAddr, uint8_t Length, uint8_t   *RdBuf)
{
    uint8_t i;
	bool result=0;
    uint8_t   TempBuf[46];
    uint8_t RdCrc=0;

    TempBuf[0] = SlaveID;
    TempBuf[1] = (uint8_t)RdAddr;
    TempBuf[2] = Length;
    TempBuf[3] = SlaveID | 0x01;
	
	if(Length > 0)
	{
		TwiStart();
		
		if(!TwiSendData(SlaveID, 1))			//Send Slave ID
		{
			goto RdErr;
		}
		
		if(SlaveID == E2PROM_ID)
		{
			if(!TwiSendData(RdAddr>>8, 0))		//Send Read Address(High 8bit) For EEPROM
			{
				goto RdErr;
			}
		}
		if(!TwiSendData(RdAddr, 0))				//Send Read Address(Low 8bit)
		{
			goto RdErr;
		}

        if(SlaveID==AFE_ID)          //CRC
        {
            if(!TwiSendData(Length, 0))
            {
                goto RdErr;
            }
        }
		
		TwiReStart();
		
		if(TwiSendData(SlaveID|0x1, 0))			//Send Slave ID
		{
			result = 1;
            if(SlaveID==AFE_ID)
            {
    			for(i=0; i<Length+1; i++)
    			{
            		if(i == Length)
            		{
                        RdCrc = TwiGetData(0);      //Get Data
            		}
            		else
            		{
                        TempBuf[4+i] = TwiGetData(1);     //Get Data
            		}
    			}

                if(RdCrc != crc_8(TempBuf, 4+Length))
                {
                    result = 0;    
                }
                else
                {
                    for(i=0; i<Length; i++)
                    {
                        *RdBuf = TempBuf[4+i];
                        RdBuf++; 
                    }
                }
            }
            else
            {
                for(i=0; i<Length; i++)
                {
        			*RdBuf = TwiGetData(Length-i-1);//Get Data
                    RdBuf++;
                }
            }
		}
	
RdErr:
		TwiStop();
	}
	
	return result;
}



















