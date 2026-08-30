#include "sys_m7.h"
#include <stdio.h>



#include <main.h>
#include "stm32h7xx_hal_pcd.h"


void soft_reset(void)
{
    NVIC_SystemReset();
}

/***************************串口映射到printf****************************/

#pragma import(__use_no_semihosting)
struct __FILE
{
    int handle;
};

int _ttywrch(int ch)
{
    ch = ch;
    return ch;
}

void _sys_exit(int x)
{
    x = x;
}

char *_sys_command_string(char *cmd, int len)
{
    return NULL;
}

FILE __stdout;

int fputc(int ch, FILE *f)
{
    while ((USART1->ISR & 0X40) == 0);
    USART1->TDR = (uint8_t)ch;
    return ch;
}
/*******************************************************/


/***************************delay****************************/


/**
 * @brief     初始化延迟函数
 * @param     sysclk: 系统时钟频率, 即CPU频率(HCLK),等于系统时钟主频，单位MHz
 * @retval    无
 */

static uint32_t sys_us = 0;

// void delay_init(uint16_t sysclk)
// {
//     uint32_t reload = sysclk*1000;                              /* 1ms触发一次中断 */
//     sys_us          = sysclk;

//     SysTick->CTRL   |= SysTick_CTRL_TICKINT_Msk;                /* 开启systick中断 */
// 	SysTick->LOAD   = reload; 					                /* 装载值 */
// 	SysTick->CTRL   |= SysTick_CTRL_ENABLE_Msk;                 /* 启动systick定时器 */

// }

void delay_init(uint16_t sysclk)
{
    uint32_t reload;
    HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);//SysTick频率为HCLK
    sys_us = sysclk;
    reload = sysclk*1000;

	SysTick->CTRL|=SysTick_CTRL_TICKINT_Msk;//开启SYSTICK中断
	SysTick->LOAD=reload; 					//每1/OS_TICKS_PER_SEC秒中断一次
	SysTick->CTRL|=SysTick_CTRL_ENABLE_Msk; //开启SYSTICK
}

void delay_us(uint32_t us)
{
    uint32_t ticks;
	uint32_t told,tnow,tcnt=0;
	uint32_t reload=SysTick->LOAD;
	ticks=us*sys_us;
	told=SysTick->VAL;
	while(1)
	{
		tnow=SysTick->VAL;
		if(tnow!=told)
		{
			if(tnow<told)tcnt+=told-tnow;
			else tcnt+=reload-tnow+told;
 			told=tnow;
			if(tcnt>=ticks)break;
		}
	};
}

void delay_ms(uint32_t ms)
{
    uint32_t i;
	for(i=0;i<ms;i++) delay_us(1000);
}


uint8_t usart6_rxbuf[RX_BUF_SZIE];

//bound:波特率
// void uart6_init(uint32_t bound)
// {
// 	//UART 初始化设置
// 	huart6.Instance=USART6;					    //USART1
// 	huart6.Init.BaudRate=bound;				    //波特率
// 	huart6.Init.WordLength=UART_WORDLENGTH_8B;   //字长为8位数据格式
// 	huart6.Init.StopBits=UART_STOPBITS_1;	    //一个停止位
// 	huart6.Init.Parity=UART_PARITY_NONE;		    //无奇偶校验位
// 	huart6.Init.HwFlowCtl=UART_HWCONTROL_NONE;   //无硬件流控
// 	huart6.Init.Mode=UART_MODE_TX_RX;		    //收发模式
// 	// HAL_UART_Init(&huart6);					    //HAL_UART_Init()会使能UART1

//     /*开启中断接收，必须满RX_BUF_SZIE大小才会触发中断*/
// 	HAL_UART_Receive_IT(&huart6,&usart6_rxbuf[0], 1);
//     // HAL_UARTEx_ReceiveToIdle_IT(&huart6, usart6_rxbuf, RX_BUF_SZIE);        /*开启空闲中断*/
// }

















//使能CPU的L1-Cache
void Cache_Enable(void)
{
    SCB_EnableICache();//使能I-Cache
    SCB_EnableDCache();//使能D-Cache
	SCB->CACR|=1<<2;   //强制D-Cache透写,如不开启,实际使用中可能遇到各种问题
}

//判断I_Cache是否打开
//返回值:0 关闭，1 打开
uint8_t Get_ICahceSta(void)
{
    uint8_t sta;
    sta=((SCB->CCR)>>17)&0X01;
    return sta;
}

//判断I_Dache是否打开
//返回值:0 关闭，1 打开
uint8_t Get_DCahceSta(void)
{
    uint8_t sta;
    sta=((SCB->CCR)>>16)&0X01;
    return sta;
}


