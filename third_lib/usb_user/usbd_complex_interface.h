#ifndef __USBD_COMPLEX_IF_H
#define __USBD_COMPLEX_IF_H


#include "usbd_complex.h"



#define COMPLEX_USB_USART_REC_LEN	 	200     //USB串口接收缓冲区最大字节数
//轮询周期，最大65ms，最小1ms
#define COMPLEX_POLLING_INTERVAL    1       //轮询周期，最大65ms，最小1ms

extern u8  COMPLEX_USB_USART_RX_BUF[COMPLEX_USB_USART_REC_LEN]; //接收缓冲,最大COMPLEX_USB_USART_REC_LEN个字节.末字节为换行符 
extern u16 COMPLEX_USB_USART_RX_STA;   					//接收状态标记	

extern USBD_COMPLEX_ItfTypeDef  USBD_COMPLEX_fops;

void COMPLEX_VCP_DataTx(uint8_t *data,uint32_t Len);
uint16_t COMPLEX_VCP_DataRx(uint8_t* Buf, uint32_t Len);
void COMPLEX_usb_printf(char* fmt,...); 

#endif 

