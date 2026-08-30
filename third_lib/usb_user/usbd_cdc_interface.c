#include "usbd_cdc_interface.h"
#include "main.h"
#include "usb_device.h"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#define VCP_TX_QUEUE_DEPTH  8U
#define VCP_TX_MAX_PACKET   USB_USART_REC_LEN

typedef struct
{
    uint16_t length;
    uint8_t data[VCP_TX_MAX_PACKET];
} VCP_TxSlot_t;

USBD_CDC_LineCodingTypeDef LineCoding =
{
    115200,
    0x00,
    0x00,
    0x08
};

uint8_t USART_PRINTF_Buffer[USB_USART_REC_LEN];
uint8_t USB_USART_RX_BUF[USB_USART_REC_LEN];
uint8_t USBRxBuffer[USB_USART_REC_LEN];
uint16_t USB_USART_RX_STA = 0;

volatile VCP_RuntimeStats_t VCP_RuntimeStats = { VCP_STATS_MAGIC };

static VCP_TxSlot_t vcp_tx_queue[VCP_TX_QUEUE_DEPTH];
static volatile uint8_t vcp_tx_head = 0;
static volatile uint8_t vcp_tx_tail = 0;
static volatile uint8_t vcp_tx_count = 0;
static volatile uint8_t vcp_tx_active = 0;

static int8_t CDC_Itf_Init(void);
static int8_t CDC_Itf_DeInit(void);
static int8_t CDC_Itf_Control(uint8_t cmd, uint8_t *pbuf, uint16_t length);
static int8_t CDC_Itf_Receive(uint8_t *pbuf, uint32_t *Len);

USBD_CDC_ItfTypeDef USBD_CDC_fops =
{
    CDC_Itf_Init,
    CDC_Itf_DeInit,
    CDC_Itf_Control,
    CDC_Itf_Receive
};

static uint32_t VCP_EnterCritical(void)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    return primask;
}

static void VCP_ExitCritical(uint32_t primask)
{
    __set_PRIMASK(primask);
}

static void VCP_ResetTxQueue(void)
{
    uint32_t primask = VCP_EnterCritical();

    vcp_tx_head = 0;
    vcp_tx_tail = 0;
    vcp_tx_count = 0;
    vcp_tx_active = 0;
    VCP_RuntimeStats.queue_depth = 0;
    VCP_RuntimeStats.tx_active = 0;

    VCP_ExitCritical(primask);
}

static int8_t CDC_Itf_Init(void)
{
    VCP_ResetTxQueue();
    return (int8_t)USBD_CDC_SetRxBuffer(&USBD_Device, USBRxBuffer);
}

static int8_t CDC_Itf_DeInit(void)
{
    VCP_RuntimeStats.configured = 0;
    VCP_ResetTxQueue();
    return (USBD_OK);
}

static int8_t CDC_Itf_Control(uint8_t cmd, uint8_t *pbuf, uint16_t length)
{
    (void)length;

    switch (cmd)
    {
        case CDC_SET_LINE_CODING:
            LineCoding.bitrate = (uint32_t)(pbuf[0] | (pbuf[1] << 8) |
                                             (pbuf[2] << 16) | (pbuf[3] << 24));
            LineCoding.format = pbuf[4];
            LineCoding.paritytype = pbuf[5];
            LineCoding.datatype = pbuf[6];
            break;

        case CDC_GET_LINE_CODING:
            pbuf[0] = (uint8_t)(LineCoding.bitrate);
            pbuf[1] = (uint8_t)(LineCoding.bitrate >> 8);
            pbuf[2] = (uint8_t)(LineCoding.bitrate >> 16);
            pbuf[3] = (uint8_t)(LineCoding.bitrate >> 24);
            pbuf[4] = LineCoding.format;
            pbuf[5] = LineCoding.paritytype;
            pbuf[6] = LineCoding.datatype;
            break;

        default:
            break;
    }

    return (USBD_OK);
}

uint16_t VCP_DataRx(uint8_t *Buf, uint32_t Len)
{
    uint32_t i;

    for (i = 0; i < Len; ++i)
    {
        uint8_t value = Buf[i];

        if ((USB_USART_RX_STA & 0x8000U) == 0U)
        {
            if ((USB_USART_RX_STA & 0x4000U) != 0U)
            {
                if (value != 0x0AU)
                {
                    USB_USART_RX_STA = 0;
                }
                else
                {
                    USB_USART_RX_STA |= 0x8000U;
                }
            }
            else if (value == 0x0DU)
            {
                USB_USART_RX_STA |= 0x4000U;
            }
            else
            {
                USB_USART_RX_BUF[USB_USART_RX_STA & 0x3FFFU] = value;
                ++USB_USART_RX_STA;
                if (USB_USART_RX_STA > (USB_USART_REC_LEN - 1U))
                {
                    USB_USART_RX_STA = 0;
                }
            }
        }
    }

    return USBD_OK;
}

uint8_t VCP_DataTx(const uint8_t *data, uint32_t Len)
{
    uint32_t primask;
    uint8_t slot;

    if ((data == NULL) || (Len == 0U))
    {
        return USBD_FAIL;
    }

    if (Len > VCP_TX_MAX_PACKET)
    {
        ++VCP_RuntimeStats.oversized_frames;
        return USBD_FAIL;
    }

    if (USBD_Device.dev_state != USBD_STATE_CONFIGURED)
    {
        ++VCP_RuntimeStats.not_configured;
        return USBD_FAIL;
    }

    primask = VCP_EnterCritical();
    if (vcp_tx_count >= VCP_TX_QUEUE_DEPTH)
    {
        ++VCP_RuntimeStats.dropped_frames;
        VCP_ExitCritical(primask);
        return USBD_BUSY;
    }

    slot = vcp_tx_head;
    memcpy(vcp_tx_queue[slot].data, data, Len);
    vcp_tx_queue[slot].length = (uint16_t)Len;
    vcp_tx_head = (uint8_t)((vcp_tx_head + 1U) % VCP_TX_QUEUE_DEPTH);
    ++vcp_tx_count;
    ++VCP_RuntimeStats.enqueued_frames;
    VCP_RuntimeStats.queue_depth = vcp_tx_count;
    if (vcp_tx_count > VCP_RuntimeStats.queue_high_water)
    {
        VCP_RuntimeStats.queue_high_water = vcp_tx_count;
    }
    VCP_ExitCritical(primask);

    VCP_Service();
    return USBD_OK;
}

uint8_t VCP_TxReady(void)
{
    uint32_t primask;
    uint8_t ready;

    if (USBD_Device.dev_state != USBD_STATE_CONFIGURED)
    {
        return 0U;
    }

    primask = VCP_EnterCritical();
    ready = (vcp_tx_count < VCP_TX_QUEUE_DEPTH) ? 1U : 0U;
    VCP_ExitCritical(primask);
    return ready;
}

void VCP_Service(void)
{
    uint32_t primask;
    uint8_t slot;
    uint8_t status;

    if (USBD_Device.dev_state != USBD_STATE_CONFIGURED)
    {
        VCP_RuntimeStats.configured = 0;
        return;
    }

    VCP_RuntimeStats.configured = 1;
    primask = VCP_EnterCritical();
    if ((vcp_tx_active != 0U) || (vcp_tx_count == 0U))
    {
        VCP_ExitCritical(primask);
        return;
    }

    slot = vcp_tx_tail;
    vcp_tx_active = 1U;
    VCP_RuntimeStats.tx_active = 1U;
    VCP_ExitCritical(primask);

    status = USBD_CDC_SetTxBuffer(&USBD_Device,
                                  vcp_tx_queue[slot].data,
                                  vcp_tx_queue[slot].length);
    if (status == USBD_OK)
    {
        status = USBD_CDC_TransmitPacket(&USBD_Device);
    }

    if (status != USBD_OK)
    {
        primask = VCP_EnterCritical();
        vcp_tx_active = 0U;
        VCP_RuntimeStats.tx_active = 0U;
        VCP_ExitCritical(primask);

        if (status == USBD_BUSY)
        {
            ++VCP_RuntimeStats.busy_retries;
        }
        else
        {
            ++VCP_RuntimeStats.ll_errors;
        }
    }
}

void VCP_TxCpltCallback(void)
{
    if ((vcp_tx_active != 0U) && (vcp_tx_count != 0U))
    {
        VCP_RuntimeStats.sent_bytes += vcp_tx_queue[vcp_tx_tail].length;
        vcp_tx_tail = (uint8_t)((vcp_tx_tail + 1U) % VCP_TX_QUEUE_DEPTH);
        --vcp_tx_count;
        ++VCP_RuntimeStats.sent_frames;
        VCP_RuntimeStats.queue_depth = vcp_tx_count;
    }

    vcp_tx_active = 0U;
    VCP_RuntimeStats.tx_active = 0U;
}

void usb_printf(const char *fmt, ...)
{
    int length;
    va_list args;

    va_start(args, fmt);
    length = vsnprintf((char *)USART_PRINTF_Buffer,
                       sizeof(USART_PRINTF_Buffer),
                       fmt,
                       args);
    va_end(args);

    if (length <= 0)
    {
        return;
    }
    if ((uint32_t)length >= sizeof(USART_PRINTF_Buffer))
    {
        length = (int)sizeof(USART_PRINTF_Buffer) - 1;
    }

    (void)VCP_DataTx(USART_PRINTF_Buffer, (uint32_t)length);
}

static int8_t CDC_Itf_Receive(uint8_t *Buf, uint32_t *Len)
{
    (void)VCP_DataRx(Buf, *Len);
    VCP_RuntimeStats.rx_bytes += *Len;
    return (int8_t)USBD_CDC_ReceivePacket(&USBD_Device);
}
