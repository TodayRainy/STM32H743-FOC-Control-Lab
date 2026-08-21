/*VOFA上位机*/

#include "vofa_debug.h"
#include <bsp.h>
#include <stdlib.h>
#include <string.h>

/*发送vofa数据帧率*/



void Vofa_Init(void)
{
    /*开启DMA发送, 循环模式, 所以只需要开启一次*/
    // HAL_UART_Transmit_DMA(&huart2, (uint8_t*)&VOFA_Handle, sizeof(VOFA_Handle));
}

/*数据解析*/
/*前2个字节是通道, 中间一个空格 后20个字节是数据*/
void Vofa_Data_deal(uint8_t *buf)
{   
    uint8_t ch[4] = {0};
    uint8_t data[20] = {0};

    uint8_t ch_n = 0;

    memcpy(ch, buf, 2);
    memcpy(data, &buf[2], 20);

    ch_n = atoi((char *)ch);

    if(ch_n<VOFA_RECV_CH_COUNT){
        VOFA_Recv_Handle.fdata[ch_n] = atof((char *)data);
    }
}


/*vofa数据帧*/
VOFA_Send_Handle_t VOFA_Handle = {
    .tail = {0x00, 0x00, 0x80, 0x7f},
};

VOFA_Recv_Handle_t VOFA_Recv_Handle;

