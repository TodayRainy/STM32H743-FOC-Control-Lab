/**
 ****************************************************************************************************
 * @file        ethernet.h
 * @author      正点原子团队(ALIENTEK)
 * @version     V1.0
 * @date        2021-12-02
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
 * V1.0 20211202
 * 第一次发布
 *
 ****************************************************************************************************
 */
 
#ifndef __ETHERNET_H
#define __ETHERNET_H
#include "sys.h"


uint32_t sys_now(void);



/******************************************************************************************/

/* ETH端口定义 */
#define ETHERNET_RST(x)  do{ x ? \
                            HAL_GPIO_WritePin(GPIOI, GPIO_PIN_8, GPIO_PIN_SET) : \
                            HAL_GPIO_WritePin(GPIOI, GPIO_PIN_8, GPIO_PIN_RESET); \
                        }while(0)



extern ETH_HandleTypeDef g_eth_handler;                                 /* 以太网句柄 */
extern ETH_DMADescTypeDef *g_eth_dma_rx_dscr_tab;                       /* 以太网DMA接收描述符数据结构体指针 */
extern ETH_DMADescTypeDef *g_eth_dma_tx_dscr_tab;                       /* 以太网DMA发送描述符数据结构体指针 */
extern uint8_t *g_eth_rx_buf;                                           /* 以太网底层驱动接收buffers指针 */
extern uint8_t *g_eth_tx_buf;                                           /* 以太网底层驱动发送buffers指针 */

uint8_t ethernet_init(void);                                            /* 以太网芯片初始化 */
uint32_t ethernet_read_phy(uint16_t reg);                               /* 读取以太网芯片寄存器值 */
void ethernet_write_phy(uint16_t reg, uint16_t value);                  /* 向以太网芯片指定地址写入寄存器值 */
uint8_t ethernet_chip_get_speed(void);                                  /* 获得以太网芯片的速度模式 */
uint32_t  ethernet_get_eth_rx_size(ETH_DMADescTypeDef *dma_rx_desc);    /* 获取接收到的帧长度 */

uint8_t ethernet_mem_malloc(void);                                      /* 为ETH底层驱动申请内存 */
void ethernet_mem_free(void);                                           /* 释放ETH 底层驱动申请的内存 */
#endif

