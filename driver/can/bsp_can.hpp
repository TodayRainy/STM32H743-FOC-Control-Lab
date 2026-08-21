#ifndef _BSP_CAN_HPP
#define _BSP_CAN_HPP

#include "fdcan.h"

typedef enum{
    CAN_125K = 0,
    CAN_250K,
    CAN_500K,
    CAN_1000K,
}CAN_BAUDRATE_t;

typedef enum{
    CAN_STD = 0,
    CAN_EXT,
}CAN_ID_TYPE_t;

class bsp_can
{
private:
    /* data */
    FDCAN_HandleTypeDef *hfdcan;


public:
    bsp_can(FDCAN_HandleTypeDef *hfdcan);
    ~bsp_can();

    void config(CAN_BAUDRATE_t baudrate);
    void send_msg(uint32_t id, CAN_ID_TYPE_t type, uint8_t msg[8], uint32_t len);
};


























#endif
