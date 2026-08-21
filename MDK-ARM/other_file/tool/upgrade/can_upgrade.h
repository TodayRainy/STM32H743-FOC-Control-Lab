#ifndef _UPGRADE_H
#define _UPGRADE_H

#include <stdbool.h>
#include <stdint.h>

#include <bsp.h>


/**********************基于CAN总线进行升级******************** */
#define CAN_HAND_SHAKE_ID       0x4005                    /*握手ID*/
#define CAN_HOST_ID             0x4006                    /*主机内容发送ID*/
#define CAN_SLAVE_ID            0x4007                    /*从机内从发送ID*/



void CanDFU_HostRecvPortEn(CAN_MSG_t msg);
int CanDFU_HostRUN(void);



#endif

