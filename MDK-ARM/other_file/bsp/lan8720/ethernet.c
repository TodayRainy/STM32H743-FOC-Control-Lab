/**
 ****************************************************************************************************
 * @file        ethernet.c
 * @author      正点原子团队(ALIENTEK)
 * @version     V1.0
 * @date        2021-10-14
 * @brief       ETHERNET 驱动代码
 * @license     Copyright (c) 2020-2032, 广州市星翼电子科技有限公司
 ****************************************************************************************************
 * @attention
 *
 * 实验平台:正点原子 F407电机开发板
 * 在线视频:www.yuanzige.com
 * 技术论坛:www.openedv.com
 * 公司网址:www.alientek.com
 * 购买地址:openedv.taobao.com
 *
 * 修改说明
 * V1.0 20211014
 * 第一次发布
 *
 ****************************************************************************************************
 */

#include "ethernet.h"
// #include "lwip_comm.h"
#include "delay.h"
#include "lan8720.h"
// #include "./MALLOC/malloc.h"


// ETH_HandleTypeDef g_eth_handler;            /* 以太网句柄 */
ETH_DMADescTypeDef *g_eth_dma_rx_dscr_tab;  /* 以太网DMA接收描述符数据结构体指针 */
ETH_DMADescTypeDef *g_eth_dma_tx_dscr_tab;  /* 以太网DMA发送描述符数据结构体指针 */
uint8_t *g_eth_rx_buf;                      /* 以太网底层驱动接收buffers指针 */
uint8_t *g_eth_tx_buf;                      /* 以太网底层驱动发送buffers指针 */


/**
 * @brief       以太网芯片初始化
 * @param       无
 * @retval      0,成功
 *              1,失败
 */
uint8_t ethernet_init(void)
{
    LAN8720_Init();
		
	return 0;
}

/**
 * @brief       ETH底层驱动，时钟使能，引脚配置
 *    @note     此函数会被HAL_ETH_Init()调用
 * @param       heth:以太网句柄
 * @retval      无
 */

/**
 * @breif       读取以太网芯片寄存器值
 * @param       reg：读取的寄存器地址
 * @retval      无
 */
uint32_t ethernet_read_phy(uint16_t reg)
{
    uint32_t regval;

    LAN8720_ReadPHY(reg, &regval);
    // HAL_ETH_ReadPHYRegister(&g_eth_handler, LAN8720_ADDR, reg, &regval);
    return regval;
}

/**
 * @breif       向以太网芯片指定地址写入寄存器值
 * @param       reg   : 要写入的寄存器
 * @param       value : 要写入的寄存器
 * @retval      无
 */
void ethernet_write_phy(uint16_t reg, uint16_t value)
{
    uint32_t temp = value;
    
    LAN8720_WritePHY(reg, temp);
    // HAL_ETH_ReadPHYRegister(&g_eth_handler, reg, &temp);
}

/**
 * @breif       获得网络芯片的速度模式
 * @param       无
 * @retval      1:获取100M成功
                0:失败
 */
uint8_t ethernet_chip_get_speed(void)
{
    uint8_t speed;
     
    speed = LAN8720_GetLinkState();
    return speed;
}

extern void lwip_pkt_handle(void);                  /* 在lwip_comm.c里面定义 */

/**
 * @breif       中断服务函数
 * @param       无
 * @retval      无
 */
// #include <stdio.h>
// void ETH_IRQHandler(void)
// {
//     // while (ethernet_get_eth_rx_size(g_eth_handler.RxDesc))
//     // {
//     //     lwip_pkt_handle();      /* 处理以太网数据，即将数据提交给LWIP */
//     // }

//     // printf("ETH_IRQHandler\r\n");

//     // __HAL_ETH_DMA_CLEAR_IT(&g_eth_handler, ETH_DMA_IT_NIS);   /* 清除DMA中断标志位 */
//     // __HAL_ETH_DMA_CLEAR_IT(&g_eth_handler, ETH_DMA_IT_R);     /* 清除DMA接收中断标志位 */
// }

/**
 * @breif       获取接收到的帧长度
 * @param       dma_rx_desc : 接收DMA描述符
 * @retval      frameLength : 接收到的帧长度
 */
uint32_t  ethernet_get_eth_rx_size(ETH_DMADescTypeDef *dma_rx_desc)
{
    uint32_t frameLength = 0;

    // if (((dma_rx_desc->Status & ETH_DMARXDESC_OWN) == (uint32_t)RESET) &&
    //     ((dma_rx_desc->Status & ETH_DMARXDESC_ES)  == (uint32_t)RESET) &&
    //     ((dma_rx_desc->Status & ETH_DMARXDESC_LS)  != (uint32_t)RESET))
    // {
    //     frameLength = ((dma_rx_desc->Status & ETH_DMARXDESC_FL) >> ETH_DMARXDESC_FRAME_LENGTHSHIFT);
    // }

    return frameLength;
}

/**
 * @breif       为ETH底层驱动申请内存
 * @param       无
 * @retval      0,正常
 *              1,失败
 */
uint8_t ethernet_mem_malloc(void)
{


    return 0;       /* 申请成功 */
}

/**
 * @breif       释放ETH 底层驱动申请的内存
 * @param       无
 * @retval      无
 */
void ethernet_mem_free(void)
{

}


