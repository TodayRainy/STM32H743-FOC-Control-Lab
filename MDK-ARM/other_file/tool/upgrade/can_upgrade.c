#include "can_upgrade.h"

#include <stdio.h>
#include <string.h>
#include "pqueue/pqueue.h"



#define SLAVE_ACK               0XF1                            /*从机应答*/
#define SLAVE_NACK              0x0E                            /*从机非应答*/
#define SLAVE_EOT               0xFF                            /*从机结束帧*/

#define MAX_ERRORS              10000


P_QUEUE_t recv_queue;
#define QUEUE_LENGTH 100
#define ITEM_SIZE sizeof(CAN_MSG_t)
// uint8_t queue_buf[QUEUE_LENGTH*ITEM_SIZE]__attribute__((section("SRAM2"), zero_init));
uint8_t queue_buf[QUEUE_LENGTH*ITEM_SIZE];

/*CAN升级状态步*/
typedef enum{
    HAND_SHAKE = 0,             /* 握手状态 */
    SEND_DATA,                  /* 发送数据 */
    WAIT_DATA_ACK,              /* 等待数据帧ACK */
    WAIT_EOT,                   /* 等待结束帧 */
}CanDFUHostStep_t;

/*
typedef struct{
    uint8_t buf[8];
    uint32_t len;
    uint32_t ext_id;
}CAN_MSG_t;

*/
/*延时接口*/
static void CanDFU_Delay(uint32_t ms){
    delay_ms(ms);
}
/*CAN发送接口*/
static void CanDFU_HostSendPort(CAN_MSG_t msg)
{
    CAN1_Send_Msg(msg.ext_id, msg);
}
/*CAN接收接口*/
/*CAN接收入队*/
void CanDFU_HostRecvPortEn(CAN_MSG_t msg)
{
    // if(msg.ext_id==CAN_SLAVE_ID){
        P_Queue_en(&recv_queue, (uint8_t *)&msg);
    // }
}


/*CAN接收出队 
返回 0 成功 1失败
*/
int CanDFU_HostRecvPortDEn(CAN_MSG_t *msg)
{
    memset(msg, 0x00, sizeof(CAN_MSG_t));
    if(P_Queue_den(&recv_queue, (uint8_t *)msg)==0){
        return 0;
    }else{
        return 1;
    }
}

/*发送握手数据*/
static void CanDFU_HostSendHandShake(uint32_t filesize)
{
    CAN_MSG_t msg = {0};

    msg.buf[0] = (filesize>>0)&0xFF;
    msg.buf[1] = (filesize>>8)&0xFF;
    msg.buf[2] = (filesize>>16)&0xFF;
    msg.buf[3] = (filesize>>24)&0xFF;
    msg.len = 4;
    msg.ext_id = CAN_HAND_SHAKE_ID;

    CanDFU_HostSendPort(msg);                   /*发送握手帧*/
}

/*发送结束*/
static void CanDFU_HostSendEnd(void)
{
    CAN_MSG_t msg = {0};
    msg.buf[0] = SLAVE_EOT;
    msg.ext_id = CAN_HOST_ID;
    msg.len = 8;
    CanDFU_HostSendPort(msg);                   /*发送数据包*/
}

/*发送文件数据*/
static void CanDFU_HostSendData(uint32_t dataIndex)
{
    CAN_MSG_t msg = {0};

    norflash_read(&msg.buf[0], dataIndex, 8);
    msg.ext_id = CAN_HOST_ID;
    msg.len = 8;
    CanDFU_HostSendPort(msg);                   /*发送数据包*/
}

/*主机轮询发送  进行从机CAN升级
返回0 升级成功，其他失败
*/
int CanDFU_HostRUN()
{   
    int ret = 0;
    uint32_t errors = 0;
    uint32_t dataIndex = SPI_FLASH_DEVICECODE_ADDR;     /*flash读取起始地址*/
    CAN_MSG_t msg = {0};
    CanDFUHostStep_t step = HAND_SHAKE;

    /*初始化接收队列*/
    P_Queue_Init(&recv_queue, queue_buf, QUEUE_LENGTH, ITEM_SIZE);

    /*检测是否需要升级从CPU*/
    ImageHeaderCheck_t check = {0};
    uint32_t filesize = SlaveImageHeaderRead(&check);
    if(filesize==0)return 1;
    uint32_t size = filesize;

    printf("\r\n***********************************启动从机升级******************************\r\n");
    
    while(1){

        CanDFU_HostRecvPortDEn(&msg);                   /*接收从机CAN数据*/
        
        switch (step){
            case HAND_SHAKE:                            /*主机发送升级握手*/
                CanDFU_HostSendHandShake(filesize);
                if(msg.buf[0]==SLAVE_ACK){          
                    printf("握手成功，准备发送数据\r\n");
                    errors = 0;
                    step = SEND_DATA;
                }else{
                    errors++;
                }
                break;
            case SEND_DATA:                              /*主机发送数据*/
                CanDFU_HostSendData(dataIndex);
                step = WAIT_DATA_ACK;
                break;

            case WAIT_DATA_ACK:
                if(msg.buf[0]==SLAVE_ACK){              /*主机接收到数据应答*/
                    step = SEND_DATA;
                    dataIndex += 8;
                    errors=0;

                    if(filesize>8)filesize -= 8;
                    else filesize = 0;
                    if(filesize==0)step = WAIT_EOT;

                    printf("发送进度:  %.2f %%\r\n", (float)(dataIndex-SPI_FLASH_DEVICECODE_ADDR)/(float)size *100.0f);
                }else if(msg.buf[0]==SLAVE_NACK){
                    errors=0;
                    step = SEND_DATA;                   /*主机重传*/
                }else{
                    errors++;
                }
                break;
            case WAIT_EOT:
                if(msg.buf[0]==SLAVE_EOT){              /*主机接收到结束应答*/
                    CanDFU_HostSendEnd();
                    printf("***********************************从机升级完成******************************\r\n");
                    ret = 0;
                    goto RET;
                }
                break;
            default:
                break;
        }
        if(errors>MAX_ERRORS){
            ret = -1;
            printf("***********************************从机升级失败******************************\r\n");
            goto RET;
        }
        CanDFU_Delay(1);
    }
RET:
    return ret;
}


































































































