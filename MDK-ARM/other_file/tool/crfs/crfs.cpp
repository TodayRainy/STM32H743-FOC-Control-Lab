#include "crfs.hpp"

#include "main.h"
#include "tim.h"

#include <string.h>
#include <math.h>


/*线性缩放*/
static int scaleRange(int x, int srcMin, int srcMax, int destMin, int destMax) {
    long int a = ((long int) destMax - (long int) destMin) * ((long int) x - (long int) srcMin);
    long int b = (long int) srcMax - (long int) srcMin;
    return ((a / b) + destMin);
}

static int constrain(int amt, int low, int high)
{
    if (amt < low)
        return low;
    else if (amt > high)
        return high;
    else
        return amt;
}


crfs::crfs(/* args */)
{    
    frame_end = false;
    rx_state = 0;
    frame_index = 0;
    memset(&crfs_frame,0x00, sizeof(crfs_frame));
    memset(&link_state,0x00,sizeof(link_state));
    memset(&raw_data[0], 0x00, sizeof(crfs_frame));
    memset(&chn_data[1], 0x00, sizeof(crfs_frame));


}

crfs::~crfs()
{
}

uint8_t crfs::crc8_dvb_s2(uint8_t crc, uint8_t a)
{
    crc ^= a;
    for (size_t i = 0; i < 8; i++)
    {
        if (crc & 0x80) {
            crc = (crc << 1) ^ 0xD5;
        } else {
            crc = crc << 1;
        }
    }
    return crc;
}


uint32_t crfs::get_time_interval_us(void)
{
    uint32_t us = __HAL_TIM_GET_COUNTER(&htim7);
    __HAL_TIM_SET_COUNTER(&htim7, 0);

    return us;
}

void crfs::rx(uint8_t c)
{
    /*获取帧率长度*/
    int frame_len = frame_index<3?5:crfs_frame.frame.len + CRSF_FRAME_LENGTH_ADDRESS + CRSF_FRAME_LENGTH_FRAMELENGTH;

    /*如果延时周期大于 1000us则重新接收*/
    if(get_time_interval_us()>CRSF_TIME_NEEDED_PER_FRAME_US){
        rx_state = 0;
    }

    switch (rx_state){
        case 0:                     /*重新接收*/
            /* code */
            frame_index = 0;
            rx_state = 1;
            // break;
        case 1:                     /*进入接收*/
            if (frame_index < frame_len) {
                crfs_frame.bytes[frame_index++] = c;
                frame_end = frame_index >= frame_len ? true : false;
                if(frame_end)rx_state=2;
            }
            break;
        case 2:                     /*等待数据处理*/
            if(false==frame_end)rx_state=0;
            break;
        default:
            break;
    }

}


uint8_t crfs::crc_calc(void)
{
    uint8_t crc = crc8_dvb_s2(0, crfs_frame.frame.type);
    for (int ii = 0; ii < crfs_frame.frame.len - CRSF_FRAME_LENGTH_TYPE_CRC; ++ii) {
        crc = crc8_dvb_s2(crc, crfs_frame.frame.payload[ii]);
    }
    return crc;
}

uint8_t crfs::run(void)
{
    #if 1
    if (frame_end) {
        frame_end = false;
        if (crfs_frame.frame.type == CRSF_FRAMETYPE_RC_CHANNELS_PACKED) {
            /**/
            const uint8_t crc = crc_calc();
            if (crc != crfs_frame.frame.payload[CRSF_FRAME_RC_CHANNELS_PAYLOAD_SIZE]) {
                return RX_FRAME_PENDING;
            }
            crfs_frame.frame.len = CRSF_FRAME_RC_CHANNELS_PAYLOAD_SIZE + CRSF_FRAME_LENGTH_TYPE_CRC;

            const crfs_chn_rawdata_t* rcChannels = (crfs_chn_rawdata_t*)&crfs_frame.frame.payload;
            raw_data[0] = rcChannels->chn0;
            raw_data[1] = rcChannels->chn1;
            raw_data[2] = rcChannels->chn2;
            raw_data[3] = rcChannels->chn3;
            raw_data[4] = rcChannels->chn4;
            raw_data[5] = rcChannels->chn5;
            raw_data[6] = rcChannels->chn6;
            raw_data[7] = rcChannels->chn7;
            raw_data[8] = rcChannels->chn8;
            raw_data[9] = rcChannels->chn9;
            raw_data[10] = rcChannels->chn10;
            raw_data[11] = rcChannels->chn11;
            raw_data[12] = rcChannels->chn12;
            raw_data[13] = rcChannels->chn13;
            raw_data[14] = rcChannels->chn14;
            raw_data[15] = rcChannels->chn15;

            rx_state = 0;                           /*解析完毕重新接收*/
            return RX_FRAME_COMPLETE;
        }
        else if (crfs_frame.frame.type == CRSF_FRAMETYPE_LINK_STATISTICS) {
            // CRC includes type and payload of each frame
            const uint8_t crc = crc_calc();
            if (crc != crfs_frame.frame.payload[CRSF_FRAME_LINK_STATISTICS_PAYLOAD_SIZE]) {
                return RX_FRAME_PENDING;
            }
            crfs_frame.frame.len = CRSF_FRAME_LINK_STATISTICS_PAYLOAD_SIZE + CRSF_FRAME_LENGTH_TYPE_CRC;

            // Inject link quality into channel 17
            link_state = *(crfs_link_state_t*)&crfs_frame.frame.payload;
            raw_data[16] = scaleRange(constrain(link_state.uplink_lq, 0, 100), 0, 100, 191, 1791);    // will map to [1000;2000] range

            // This is not RC channels frame, update channel value but don't indicate frame completion
            return RX_FRAME_PENDING;
        }
    }

    for (size_t i = 0; i < CRSF_MAX_CHANNEL; i++)
    {
        chn_data[i] = read_raw_rc(i);
    }
    #endif

    return RX_FRAME_PENDING;
}

uint16_t crfs::read_raw_rc(uint8_t chn)
{
    return (raw_data[chn] * 1024 / 1639) + 881;
}

