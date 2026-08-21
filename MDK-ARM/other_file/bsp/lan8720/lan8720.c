#include "lan8720.h"

#include <stdio.h>

#include <sys.h>
#include "bsp.h"
#include <eth.h>

extern ETH_TxPacketConfig TxConfig;

__attribute__((at(0x30040200))) uint8_t Rx_Buff[ETH_RX_DESC_CNT][ETH_MAX_PACKET_SIZE];  //以太网接收缓冲区

int32_t LAN8720_Init(void)
{
 //   u32 sn0;
		//sn0=*(vu32*)(0x1FF1E800);//获取STM32的唯一ID的前24位作为MAC地址后三字节
    //u8 macaddress[6];
    u32 timeout=0;
    u32 regval=0;
    u32 phylink=0;
    int32_t status=LAN8720_STATUS_OK;
    
    //硬件复位
    INTX_DISABLE();                         //关闭所有中断，复位过程不能被打断！
    PCF8574_WriteBit(ETH_RESET_IO,1);       //硬件复位
    delay_ms(100);
    PCF8574_WriteBit(ETH_RESET_IO,0);       //复位结束
    delay_ms(100);
    INTX_ENABLE();                          //开启所有中断 
    


    //HAL_ETH_ReadData();

    if(LAN8720_WritePHY(LAN8720_BCR,LAN8720_BCR_SOFT_RESET)>=0) //LAN8720软件复位
    {
        //等待软件复位完成
        if(LAN8720_ReadPHY(LAN8720_BCR,&regval)>=0)
        {
            while(regval&LAN8720_BCR_SOFT_RESET)
            {
                if(LAN8720_ReadPHY(LAN8720_BCR,&regval)<0)
                {
                    status=LAN8720_STATUS_READ_ERROR;
                    break;
                }
                delay_ms(10);
                timeout++;
                if(timeout>=LAN8720_TIMEOUT) break; //超时跳出,5S
            }
        }
        else
        {
            status=LAN8720_STATUS_READ_ERROR;
        }
    }
    else
    {
        status=LAN8720_STATUS_WRITE_ERROR;
    }

    LAN8720_StartAutoNego();                //开启自动协商功能
    
    if(status==LAN8720_STATUS_OK)           //如果前面运行正常就延时1s
        delay_ms(1000);                     //等待1s
       
    //等待网络连接成功
    timeout=0;
    while(LAN8720_GetLinkState()<=LAN8720_STATUS_LINK_DOWN)  
    {
        delay_ms(10);
        timeout++;
        if(timeout>=LAN8720_TIMEOUT) 
        {
            status=LAN8720_STATUS_LINK_DOWN;
            break; //超时跳出,5S
        }
    }
    phylink=LAN8720_GetLinkState();
    if(phylink==LAN8720_STATUS_100MBITS_FULLDUPLEX)
        printf("LAN8720: 100Mb/s FullDuplex\r\n");
    else if(phylink==LAN8720_STATUS_100MBITS_HALFDUPLEX)
        printf("LAN8720: 100Mb/s HalfDuplex\r\n");
    else if(phylink==LAN8720_STATUS_10MBITS_FULLDUPLEX)
        printf("LAN8720: 10Mb/s FullDuplex\r\n");
    else if(phylink==LAN8720_STATUS_10MBITS_HALFDUPLEX)
        printf("LAN8720: 10Mb/s HalfDuplex\r\n");
    else{
        printf("LAN8720: 0Mb/s HalfDuplex\r\n");
    }

    ETH_BufferTypeDef ETH_Buffer = {
        .buffer = "nihao",
        .len = 6,
    };
    TxConfig.TxBuffer = &ETH_Buffer;;

    // HAL_ETH_Start_IT(&heth);

    // HAL_ETH_Transmit(&heth, &TxConfig, 0);

    // printf("HAL_ETH_Transmit(&heth, &TxConfig, 0):  %d\r\n", HAL_ETH_Transmit(&heth, &TxConfig, 0));


    return status; 
}


//向LAN8720指定寄存器写入值
//reg:要写入的寄存器
//value:要写入的值
//返回值:0 写入正常，-1 写入失败
int32_t LAN8720_WritePHY(uint16_t reg, uint16_t value)
{
    u32 temp=value;
    if(HAL_ETH_WritePHYRegister(&heth, LAN8720_ADDR, reg, temp)!=HAL_OK)
        return -1;
    return 0;
}

//读取PHY寄存器值
//reg要读取的寄存器地址
//返回值:0 读取成功，-1 读取失败
int32_t LAN8720_ReadPHY(uint16_t reg,uint32_t *regval)
{
    if(HAL_ETH_ReadPHYRegister(&heth, LAN8720_ADDR, reg, regval)!=HAL_OK)
        return -1;
    return 0;
}

//开启LAN8720的自协商功能
void LAN8720_StartAutoNego(void)
{
    u32 readval=0;
    LAN8720_ReadPHY(LAN8720_BCR, &readval);
    readval|=LAN8720_BCR_AUTONEGO_EN;
    LAN8720_WritePHY(LAN8720_BCR, readval);
}

void LAN8720_SetLinkState(u32 linkstate)
{
    u32 bcrvalue=0;
    LAN8720_ReadPHY(LAN8720_BCR,&bcrvalue); //关闭连接配置，比如自动协商，速度和双工
    bcrvalue&=~(LAN8720_BCR_AUTONEGO_EN|LAN8720_BCR_SPEED_SELECT|LAN8720_BCR_DUPLEX_MODE);

    if(linkstate==LAN8720_STATUS_100MBITS_FULLDUPLEX){
        bcrvalue|=(LAN8720_BCR_SPEED_SELECT|LAN8720_BCR_DUPLEX_MODE);
    }else if(linkstate==LAN8720_STATUS_100MBITS_HALFDUPLEX){ //100M 半双工
        bcrvalue|=LAN8720_BCR_SPEED_SELECT;
    }else if(linkstate==LAN8720_STATUS_10MBITS_FULLDUPLEX){ //10M 全双工
        bcrvalue|=LAN8720_BCR_DUPLEX_MODE;
    }else{
        return;
    }
    LAN8720_WritePHY(LAN8720_BCR, bcrvalue);
}

//获取LAN8720的连接状态
//返回值：LAN8720_STATUS_LINK_DOWN              连接断开
//        LAN8720_STATUS_AUTONEGO_NOTDONE       自动协商完成
//        LAN8720_STATUS_100MBITS_FULLDUPLEX    100M全双工
//        LAN8720_STATUS_100MBITS_HALFDUPLEX    100M半双工
//        LAN8720_STATUS_10MBITS_FULLDUPLEX     10M全双工
//        LAN8720_STATUS_10MBITS_HALFDUPLEX     10M半双工
u32 LAN8720_GetLinkState(void)
{
    u32 readval=0;
    
    //读取两遍，确保读取正确！！！
    LAN8720_ReadPHY(LAN8720_BSR,&readval);
    LAN8720_ReadPHY(LAN8720_BSR,&readval);
    
    //获取连接状态(硬件，网线的连接，不是TCP、UDP等软件连接！)
    if((readval&LAN8720_BSR_LINK_STATUS)==0)
        return LAN8720_STATUS_LINK_DOWN;
    
    //获取自动协商状态
    LAN8720_ReadPHY(LAN8720_BCR,&readval);
    if((readval&LAN8720_BCR_AUTONEGO_EN)!=LAN8720_BCR_AUTONEGO_EN)  //未使能自动协商
    {
        if(((readval&LAN8720_BCR_SPEED_SELECT)==LAN8720_BCR_SPEED_SELECT)&&
                ((readval&LAN8720_BCR_DUPLEX_MODE)==LAN8720_BCR_DUPLEX_MODE)) 
            return LAN8720_STATUS_100MBITS_FULLDUPLEX;
        else if((readval&LAN8720_BCR_SPEED_SELECT)==LAN8720_BCR_SPEED_SELECT)
            return LAN8720_STATUS_100MBITS_HALFDUPLEX;
        else if((readval&LAN8720_BCR_DUPLEX_MODE)==LAN8720_BCR_DUPLEX_MODE)
            return LAN8720_STATUS_10MBITS_FULLDUPLEX;
        else
            return LAN8720_STATUS_10MBITS_HALFDUPLEX;
    }
    else                                                            //使能了自动协商    
    {
        LAN8720_ReadPHY(LAN8720_PHYSCSR,&readval);
        if((readval&LAN8720_PHYSCSR_AUTONEGO_DONE)==0)
            return LAN8720_STATUS_AUTONEGO_NOTDONE;
        if((readval&LAN8720_PHYSCSR_HCDSPEEDMASK)==LAN8720_PHYSCSR_100BTX_FD)
            return LAN8720_STATUS_100MBITS_FULLDUPLEX;
        else if ((readval&LAN8720_PHYSCSR_HCDSPEEDMASK)==LAN8720_PHYSCSR_100BTX_HD)
            return LAN8720_STATUS_100MBITS_HALFDUPLEX;
        else if ((readval&LAN8720_PHYSCSR_HCDSPEEDMASK)==LAN8720_PHYSCSR_10BT_FD)
            return LAN8720_STATUS_10MBITS_FULLDUPLEX;
        else
            return LAN8720_STATUS_10MBITS_HALFDUPLEX;         
    }
}

