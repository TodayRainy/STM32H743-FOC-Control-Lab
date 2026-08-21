#include "usbd_conf.h"
#include <stdbool.h>

#include <usb_otg.h>
#include <usbd_def.h>
#include "usbd_def.h"
#include "usbd_core.h"

//表示USB连接状态
//0,没有连接;
//1,已经连接;
volatile bool bDeviceState=0;		//默认没有连接

#if 0
//初始化PCD MSP 
//hpcd:PCD结构体指针
//返回值:无
void HAL_PCD_MspInit(PCD_HandleTypeDef * hpcd)
{
   
    GPIO_InitTypeDef GPIO_InitStruct;
    RCC_PeriphCLKInitTypeDef USBClkInit;
      
    //USB时钟设置，使用HSI48MHz
    USBClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
    USBClkInit.UsbClockSelection = RCC_USBCLKSOURCE_HSI48;
    HAL_RCCEx_PeriphCLKConfig(&USBClkInit);

    if(hpcd->Instance == USB2_OTG_FS)
    {
        __HAL_RCC_GPIOA_CLK_ENABLE();       //使能GPIOA时钟
        
        //配置PA11和PA12
        GPIO_InitStruct.Pin=GPIO_PIN_11|GPIO_PIN_12;
        GPIO_InitStruct.Mode=GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull=GPIO_NOPULL;
        GPIO_InitStruct.Speed=GPIO_SPEED_FREQ_VERY_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF10_OTG1_FS;
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
        
		GPIO_InitStruct.Pin=GPIO_PIN_13;    			//PG13
		GPIO_InitStruct.Mode=GPIO_MODE_OUTPUT_PP;  		//推挽输出
		GPIO_InitStruct.Pull=GPIO_PULLUP;          		//上拉
		GPIO_InitStruct.Speed=GPIO_SPEED_FREQ_VERY_HIGH; //高速
		HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);         //初始化
		HAL_GPIO_WritePin(GPIOG,GPIO_PIN_13,GPIO_PIN_SET);	//开启USB HOST电源供电  
		
        __HAL_RCC_USB2_OTG_FS_CLK_ENABLE();     //使能OTG FS时钟
        HAL_NVIC_SetPriority(OTG_FS_IRQn,1,0);  //设置中断优先级
        HAL_NVIC_EnableIRQ(OTG_FS_IRQn);        //使能OTG FS中断
    }
    else if (hpcd->Instance == USB1_OTG_HS)
    {
        //USB1 OTG本例程没用到,故不做处理
    }
}
#endif


///////////////////////USBD LL PCD驱动回调函数(PCD->USB Device Library)///////////////////////
 
//USBD配置阶段回调函数
//hpcd:PCD结构体指针
//返回值:无
void HAL_PCD_SetupStageCallback(PCD_HandleTypeDef * hpcd)
{
    USBD_DEBUG("**********USBD配置阶段回调函数: \r\n");
	USBD_LL_SetupStage(hpcd->pData, (uint8_t *) hpcd->Setup);


    USBD_DEBUG("\r\n");
    
}
 
//USBD OUT阶段回调函数
//hpcd:PCD结构体指针
//返回值:无
void HAL_PCD_DataOutStageCallback(PCD_HandleTypeDef * hpcd, uint8_t epnum)
{
 
    // USBD_DEBUG("**********OUT阶段回调函数 端点：%d xfer_len:%d\r\n", epnum, hpcd->OUT_ep[epnum].xfer_len);
    
    USBD_LL_DataOutStage(hpcd->pData, epnum, hpcd->OUT_ep[epnum].xfer_buff);

    // USBD_DEBUG("\r\n");
}

//USBD IN阶段回调函数
//hpcd:PCD结构体指针
//返回值:无
void HAL_PCD_DataInStageCallback(PCD_HandleTypeDef * hpcd, uint8_t epnum)
{
    // USBD_DEBUG("**********IN阶段回调函数 端点: %d\r\n", epnum);
	USBD_LL_DataInStage(hpcd->pData, epnum, hpcd->IN_ep[epnum].xfer_buff);
    // USBD_DEBUG("\r\n");
}

//USBD SOF回调函数
//hpcd:PCD结构体指针
//返回值:无
void HAL_PCD_SOFCallback(PCD_HandleTypeDef * hpcd)
{
    USBD_DEBUG("**********SOF回调函数\r\n");
	USBD_LL_SOF(hpcd->pData);
    USBD_DEBUG("\r\n");
}

//USBD 复位回调函数
//hpcd:PCD结构体指针
//返回值:无
void HAL_PCD_ResetCallback(PCD_HandleTypeDef * hpcd)
{
    USBD_DEBUG("**********复位回调函数\r\n");
	USBD_SpeedTypeDef speed = USBD_SPEED_FULL;
	/* Set USB Current Speed */
	switch (hpcd->Init.speed)
	{
		case PCD_SPEED_HIGH:
			USBD_DEBUG("USB Device Library  [HS]\r\n");
			speed = USBD_SPEED_HIGH;
			break;

		case PCD_SPEED_FULL:
			USBD_DEBUG("USB Device Library  [FS]\r\n");
			speed = USBD_SPEED_FULL;
			break; 
		default:
			USBD_DEBUG("USB Device Library  [FS?]\r\n");
			speed = USBD_SPEED_FULL;
			break;
	}  
	USBD_LL_Reset(hpcd->pData); 			//复位设备
	USBD_LL_SetSpeed(hpcd->pData, speed);
    USBD_DEBUG("\r\n");
} 

//USBD 挂起回调函数
//hpcd:PCD结构体指针
//返回值:无
void HAL_PCD_SuspendCallback(PCD_HandleTypeDef * hpcd)
{
	bDeviceState=0;
	USBD_DEBUG("**********挂起回调函数\r\n");
	USBD_LL_Suspend(hpcd->pData);

    USBD_DEBUG("\r\n");
}

//USBD 恢复回调函数
//hpcd:PCD结构体指针
//返回值:无
void HAL_PCD_ResumeCallback(PCD_HandleTypeDef * hpcd)
{
    USBD_DEBUG("**********恢复回调函数\r\n");

	USBD_LL_Resume(hpcd->pData);
    
    USBD_DEBUG("\r\n");
}
 
//USBD ISO OUT事务完成回调函数
//hpcd:PCD结构体指针
//epnum:端点号
//返回值:无
void HAL_PCD_ISOOUTIncompleteCallback(PCD_HandleTypeDef * hpcd, uint8_t epnum)
{
    USBD_DEBUG("**********OUT事务完成回调函数\r\n");
	USBD_LL_IsoOUTIncomplete(hpcd->pData, epnum);
    
    USBD_DEBUG("\r\n");
}

//USBD ISO IN事务完成回调函数
//hpcd:PCD结构体指针
//epnum:端点号
//返回值:无
void HAL_PCD_ISOINIncompleteCallback(PCD_HandleTypeDef * hpcd, uint8_t epnum)
{ 
    USBD_DEBUG("**********IN事务完成回调函数\r\n");
	USBD_LL_IsoINIncomplete(hpcd->pData, epnum);
    
    USBD_DEBUG("\r\n");
}

//USBD 连接成功回调函数
//hpcd:PCD结构体指针
//返回值:无
void HAL_PCD_ConnectCallback(PCD_HandleTypeDef * hpcd)
{
    USBD_DEBUG("**********连接成功回调函数\r\n");
	bDeviceState=1; 
	USBD_LL_DevConnected(hpcd->pData);
    USBD_DEBUG("\r\n");
}

//USBD 断开连接回调函数
//hpcd:PCD结构体指针
//返回值:无
void HAL_PCD_DisconnectCallback(PCD_HandleTypeDef * hpcd)
{
	bDeviceState=0;
	USBD_DEBUG("**********断开连接回调函数\r\n");

	USBD_LL_DevDisconnected(hpcd->pData);
    
    USBD_DEBUG("\r\n");
}

/////////////////////USBD LL 驱动接口函数(USB Device Library --> PCD)/////////////////////

// USBD底层初始化函数
// pdev:USBD句柄指针
// 返回值:0,正常
//      1,忙
//      2,失败
USBD_StatusTypeDef USBD_LL_Init(USBD_HandleTypeDef * pdev)
{

	//设置LL驱动相关参数
	hpcd_USB_OTG_FS.Instance = USB2_OTG_FS;			//使用USB2 OTG
	hpcd_USB_OTG_FS.Init.dev_endpoints = 8;			//端点数为8
	hpcd_USB_OTG_FS.Init.use_dedicated_ep1 = 0;		//禁止EP1 dedicated中断
	hpcd_USB_OTG_FS.Init.ep0_mps = 0x40;				//设置端点0的最大包大小为0X40(64字节)
	hpcd_USB_OTG_FS.Init.low_power_enable = 0;			//不使能低功耗模式
	hpcd_USB_OTG_FS.Init.phy_itface = PCD_PHY_EMBEDDED;//使用内部PHY
	hpcd_USB_OTG_FS.Init.Sof_enable = 0;				//使能SOF中断
	hpcd_USB_OTG_FS.Init.speed = PCD_SPEED_FULL;		//USB全速(12Mbps)	
	hpcd_USB_OTG_FS.Init.vbus_sensing_enable = 0;		//不使能VBUS检测
	hpcd_USB_OTG_FS.Init.lpm_enable = 0;				//使能连接电源管理
 
	hpcd_USB_OTG_FS.pData = pdev;						    //hpcd的pData指向pdev
	pdev->pData = &hpcd_USB_OTG_FS;					        //pdev的pData指向hpcd
 
	HAL_PCD_Init(&hpcd_USB_OTG_FS);					        //初始化LL驱动

	HAL_PCDEx_SetRxFiFo(&hpcd_USB_OTG_FS, 0x80);		    //设置接收FIFO大小为0X80(128字节)
	HAL_PCDEx_SetTxFiFo(&hpcd_USB_OTG_FS, 0, 0x40);	        //设置发送FIFO 0的大小为0X40(64字节)
	HAL_PCDEx_SetTxFiFo(&hpcd_USB_OTG_FS, 1, 0x80);	        //设置发送FIFO 1的大小为0X80(128字节)
    HAL_PCDEx_SetTxFiFo(&hpcd_USB_OTG_FS, 2, 0x80);	        //设置发送FIFO 2的大小为0X80(128字节)
    HAL_PCDEx_SetTxFiFo(&hpcd_USB_OTG_FS, 3, 0x80);	        //设置发送FIFO 2的大小为0X80(128字节)

    

  return USBD_OK;
} 

//USBD底层取消初始化(回复默认复位状态)函数 
//pdev:USBD句柄指针
//返回值:0,正常
//      1,忙
//      2,失败
USBD_StatusTypeDef USBD_LL_DeInit(USBD_HandleTypeDef * pdev)
{
	HAL_PCD_DeInit(pdev->pData);
	return USBD_OK;
}
 
//USBD底层驱动开始工作
//pdev:USBD句柄指针
//返回值:0,正常
//      1,忙
//      2,失败
USBD_StatusTypeDef USBD_LL_Start(USBD_HandleTypeDef * pdev)
{
	HAL_PCD_Start(pdev->pData);
	return USBD_OK;
}

//USBD底层驱动停止工作
//pdev:USBD句柄指针
//返回值:0,正常
//      1,忙
//      2,失败
USBD_StatusTypeDef USBD_LL_Stop(USBD_HandleTypeDef * pdev)
{
	HAL_PCD_Stop(pdev->pData);
	return USBD_OK;
}

//USBD初始化(打开)某个端点
//pdev:USBD句柄指针
//ep_addr:端点号
//ep_mps:端点最大包容量(字节)
//返回值:0,正常
//      1,忙
//      2,失败
USBD_StatusTypeDef USBD_LL_OpenEP(USBD_HandleTypeDef * pdev,uint8_t ep_addr,uint8_t ep_type, uint16_t ep_mps)
{
    USBD_DEBUG("打开端点:0x%x, 类型: %d, 端点数据包大小: %d\r\n", ep_addr, ep_type, ep_mps);

	HAL_PCD_EP_Open(pdev->pData, ep_addr, ep_mps, ep_type);
	return USBD_OK;
}

//USBD取消初始化(关闭)某个端点
//pdev:USBD句柄指针
//ep_addr:端点号 
//返回值:0,正常
//      1,忙
//      2,失败
USBD_StatusTypeDef USBD_LL_CloseEP(USBD_HandleTypeDef * pdev, uint8_t ep_addr)
{
    USBD_DEBUG("关闭端点:0x%x\r\n", ep_addr);

	HAL_PCD_EP_Close(pdev->pData, ep_addr);
	return USBD_OK;
} 

//USBD清空某个端点的数据
//pdev:USBD句柄指针
//ep_addr:端点号 
//返回值:0,正常
//      1,忙
//      2,失败
USBD_StatusTypeDef USBD_LL_FlushEP(USBD_HandleTypeDef * pdev, uint8_t ep_addr)
{
	HAL_PCD_EP_Flush(pdev->pData, ep_addr);
	return USBD_OK;
}

//USBD给某个端点设置一个暂停状态
//pdev:USBD句柄指针
//ep_addr:端点号 
//返回值:0,正常
//      1,忙
//      2,失败
USBD_StatusTypeDef USBD_LL_StallEP(USBD_HandleTypeDef * pdev, uint8_t ep_addr)
{
	HAL_PCD_EP_SetStall(pdev->pData, ep_addr);
	return USBD_OK;
}

//USBD取消某个端点的暂停状态
//pdev:USBD句柄指针
//ep_addr:端点号 
//返回值:0,正常
//      1,忙
//      2,失败
USBD_StatusTypeDef USBD_LL_ClearStallEP(USBD_HandleTypeDef * pdev,
                                        uint8_t ep_addr)
{
	HAL_PCD_EP_ClrStall(pdev->pData, ep_addr);
	return USBD_OK;
}

//USBD返回是否处于暂停状态
//pdev:USBD句柄指针
//ep_addr:端点号 
//返回值:0,非暂停; 1,暂停.
uint8_t USBD_LL_IsStallEP(USBD_HandleTypeDef * pdev, uint8_t ep_addr)
{
	PCD_HandleTypeDef *hpcd = pdev->pData;
	if ((ep_addr & 0x80) == 0x80)
	{
		return hpcd->IN_ep[ep_addr & 0x7F].is_stall;
	}
	else
	{
		return hpcd->OUT_ep[ep_addr & 0x7F].is_stall;
	}
}

//USBD为设备指定新的USB地址
//pdev:USBD句柄指针
//dev_addr:新的设备地址,USB1_OTG_HS/USB2_OTG_HS
//返回值:0,正常
//      1,忙
//      2,失败
USBD_StatusTypeDef USBD_LL_SetUSBAddress(USBD_HandleTypeDef * pdev,uint8_t dev_addr)
{
	bDeviceState=1;	//能执行到该函数,说明USB连接成功了 
    USBD_DEBUG("设置新的USB地址: 0x%x\r\n", dev_addr);
	HAL_PCD_SetAddress(pdev->pData, dev_addr);
	return USBD_OK;
}
 
//USBD通过端点发送数据
//pdev:USBD句柄指针
//ep_addr:端点号 
//pbuf:数据缓冲区首地址
//size:要发送的数据大小
//返回值:0,正常
//      1,忙
//      2,失败
USBD_StatusTypeDef USBD_LL_Transmit(USBD_HandleTypeDef * pdev,uint8_t ep_addr,uint8_t * pbuf, uint16_t size)
{
	  HAL_PCD_EP_Transmit(pdev->pData, ep_addr, pbuf, size);
	  return USBD_OK;
}
 
//USBD准备一个端点接收数据
//pdev:USBD句柄指针
//ep_addr:端点号 
//pbuf:数据缓冲区首地址
//size:要接收的数据大小
//返回值:0,正常
//      1,忙
//      2,失败
USBD_StatusTypeDef USBD_LL_PrepareReceive(USBD_HandleTypeDef * pdev,uint8_t ep_addr,uint8_t * pbuf, uint16_t size)
{
	  HAL_PCD_EP_Receive(pdev->pData, ep_addr, pbuf, size);
	  return USBD_OK;
} 

//USBD获取最后一个传输包的大小
//pdev:USBD句柄指针
//ep_addr:端点号  
//返回值:包大小
uint32_t USBD_LL_GetRxDataSize(USBD_HandleTypeDef * pdev, uint8_t ep_addr)
{
	return HAL_PCD_EP_GetRxCount(pdev->pData, ep_addr);
}

//USBD 延时函数(以ms为单位)
//Delay:延时的ms数.
void USBD_LL_Delay(uint32_t Delay)
{
	delay_ms(Delay);
}



























