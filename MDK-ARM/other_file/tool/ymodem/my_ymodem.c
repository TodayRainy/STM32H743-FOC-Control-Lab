#include "my_ymodem.h"
#include "delay.h"
 
void Ymodem_Init(u32 botelv)
{
    u8  key = 0;
    u32 num = 0;
	
	YMODEM_USARTx_init(botelv);
	USART_ITConfig(YMODEM_USARTx, USART_IT_RXNE, DISABLE);//关闭串口接受中断     内部使用查询获取串口数据
	USART_ITConfig(YMODEM_USARTx, USART_IT_IDLE, DISABLE);//关闭串口空闲IDEL中断
	
	printf("进入 bootloader\r\n");
    while (1)  
    {
		printf("\r\n\r\n");
		printf("*********************************************************\r\n");
		printf("请选择:\r\n");
		printf("1.更新应用区程序。\r\n");  
		printf("2.运行APP程序。\r\n");
		printf("*********************************************************\r\n");
 
		Receive_Byte(&key,100000);  //等待单位是时钟   
		if (key == '1')
		{
			SerialDownload();       //使用YModem协议接收app bin文件  
			break;					
		}
		if (key == '2')
		{
			break;					
		}
		delay_ms(1000);
		num++;if(num>=5)break;      //5s内如果没有选择,跳出GetKey函数
    } 
	printf( "\r\n\r\n进入APP...\r\n");
    printf("*********************************************************\r\n");		
	delay_ms(100);	//等待上面提示信息发送完毕
 
	if (((*(__IO u32*)ApplicationAddress) & 0x2FFE0000 ) == 0x20000000)//通过判断栈顶地址值是否正确(是否在0x2000 0000 - 0x2000 2000之间)来判断是否应用程序已经下载了
	{
		//跳转至用户代码
		Jump_To_Application = (pFunction) (*(__IO u32*) (ApplicationAddress + 4));

		//初始化用户程序的堆栈指针
		__set_MSP(*(__IO u32*) ApplicationAddress);
		Jump_To_Application();
	}
	else
	{
		printf("没有APP...\r\n\n");
	}
}
/*******************************************************************************
  * @函数名称：FLASH_PagesMask
  * @函数说明：计算Falsh页
  * @输入参数：Size:文件长度
  * @输出参数：无
  * @返回参数：页的数量
*******************************************************************************/

