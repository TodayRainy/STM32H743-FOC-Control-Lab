#ifndef _SYS_M7_H
#define _SYS_M7_H

#include <main.h>

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif



void soft_reset(void);              /* 软复位 */

void delay_init(uint16_t sysclk);
void delay_us(uint32_t us);
void delay_ms(uint32_t ms);

#define RX_BUF_SZIE     1024
extern uint8_t usart6_rxbuf[RX_BUF_SZIE];
void uart6_init(uint32_t bound);

void Cache_Enable(void);
uint8_t Get_ICahceSta(void);
uint8_t Get_DCahceSta(void);


#ifdef __cplusplus
}
#endif






#endif

