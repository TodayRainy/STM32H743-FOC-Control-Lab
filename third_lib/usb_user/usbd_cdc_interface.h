#ifndef __USBD_CDC_IF_H
#define __USBD_CDC_IF_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "usbd_cdc.h"

#define USB_USART_REC_LEN       200U
#define CDC_POLLING_INTERVAL    1U
#define VCP_STATS_MAGIC         0x56504331UL

typedef struct
{
    uint32_t magic;
    uint32_t configured;
    uint32_t queue_depth;
    uint32_t queue_high_water;
    uint32_t tx_active;
    uint32_t enqueued_frames;
    uint32_t sent_frames;
    uint32_t sent_bytes;
    uint32_t dropped_frames;
    uint32_t oversized_frames;
    uint32_t not_configured;
    uint32_t busy_retries;
    uint32_t ll_errors;
    uint32_t rx_bytes;
} VCP_RuntimeStats_t;

extern uint8_t USB_USART_RX_BUF[USB_USART_REC_LEN];
extern uint16_t USB_USART_RX_STA;
extern USBD_CDC_ItfTypeDef USBD_CDC_fops;
extern volatile VCP_RuntimeStats_t VCP_RuntimeStats;

uint8_t VCP_DataTx(const uint8_t *data, uint32_t Len);
uint8_t VCP_TxReady(void);
uint16_t VCP_DataRx(uint8_t *Buf, uint32_t Len);
void VCP_Service(void);
void VCP_TxCpltCallback(void);
void usb_printf(const char *fmt, ...);

#ifdef __cplusplus
}
#endif

#endif
