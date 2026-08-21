#ifndef _CRFS_HPP
#define _CRFS_HPP

#include <stdint.h>


#define STATIC_UNIT_TESTED 
#define STATIC_INLINE_UNIT_TESTED  inline
#define INLINE_UNIT_TESTED inline
#define UNIT_TESTED


typedef enum portOptions_t {
    SERIAL_NOT_INVERTED  = 0 << 0,
    SERIAL_INVERTED      = 1 << 0,
    SERIAL_STOPBITS_1    = 0 << 1,
    SERIAL_STOPBITS_2    = 1 << 1,
    SERIAL_PARITY_NO     = 0 << 2,
    SERIAL_PARITY_EVEN   = 1 << 2,
    SERIAL_UNIDIR        = 0 << 3,
    SERIAL_BIDIR         = 1 << 3,

    /*
     * Note on SERIAL_BIDIR_PP
     * With SERIAL_BIDIR_PP, the very first start bit of back-to-back bytes
     * is lost and the first data byte will be lost by a framing error.
     * To ensure the first start bit to be sent, prepend a zero byte (0x00)
     * to actual data bytes.
     */
    SERIAL_BIDIR_OD      = 0 << 4,
    SERIAL_BIDIR_PP      = 1 << 4
} portOptions_t;


#pragma once

#define CRSF_BAUDRATE           420000
#define CRSF_PORT_OPTIONS       (SERIAL_STOPBITS_1 | SERIAL_PARITY_NO)
#define CRSF_PORT_MODE          MODE_RXTX

#define CRSF_MAX_CHANNEL        17

typedef enum {
    CRSF_FRAMETYPE_GPS = 0x02,
    CRSF_FRAMETYPE_BATTERY_SENSOR = 0x08,
    CRSF_FRAMETYPE_LINK_STATISTICS = 0x14,
    CRSF_FRAMETYPE_RC_CHANNELS_PACKED = 0x16,
    CRSF_FRAMETYPE_ATTITUDE = 0x1E,
    CRSF_FRAMETYPE_FLIGHT_MODE = 0x21
} crsfFrameTypes_e;

enum {
    CRSF_FRAME_GPS_PAYLOAD_SIZE = 15,
    CRSF_FRAME_BATTERY_SENSOR_PAYLOAD_SIZE = 8,
    CRSF_FRAME_LINK_STATISTICS_PAYLOAD_SIZE = 10,
    CRSF_FRAME_RC_CHANNELS_PAYLOAD_SIZE = 22, // 11 bits per channel * 16 channels = 22 bytes.
    CRSF_FRAME_ATTITUDE_PAYLOAD_SIZE = 6,
    CRSF_FRAME_LENGTH_ADDRESS = 1, // length of ADDRESS field
    CRSF_FRAME_LENGTH_FRAMELENGTH = 1, // length of FRAMELENGTH field
    CRSF_FRAME_LENGTH_TYPE = 1, // length of TYPE field
    CRSF_FRAME_LENGTH_CRC = 1, // length of CRC field
    CRSF_FRAME_LENGTH_TYPE_CRC = 2 // length of TYPE and CRC fields combined
};

enum {
    CRSF_ADDRESS_BROADCAST = 0x00,
    CRSF_ADDRESS_TBS_CORE_PNP_PRO = 0x8,
    CRSF_ADDRESS_RESERVED1 = 0x8A,
    CRSF_ADDRESS_CURRENT_SENSOR = 0xC0,
    CRSF_ADDRESS_TBS_BLACKBOX = 0xC4,
    CRSF_ADDRESS_COLIBRI_RACE_FC = 0xC8,
    CRSF_ADDRESS_RESERVED2 = 0xCA,
    CRSF_ADDRESS_RACE_TAG = 0xCC,
    CRSF_ADDRESS_RADIO_TRANSMITTER = 0xEA,
    CRSF_ADDRESS_CRSF_RECEIVER = 0xEC,
    CRSF_ADDRESS_CRSF_TRANSMITTER = 0xEE
};

#define CRSF_PAYLOAD_SIZE_MAX   32 // !!TODO needs checking
#define CRSF_FRAME_SIZE_MAX     (CRSF_PAYLOAD_SIZE_MAX + 4)

typedef struct crsfFrameDef_s {
    uint8_t deviceAddress;
    uint8_t frameLength;
    uint8_t type;
    uint8_t payload[CRSF_PAYLOAD_SIZE_MAX + 1]; // +1 for CRC at end of payload
} crsfFrameDef_t;

typedef union crsfFrame_u {
    uint8_t bytes[CRSF_FRAME_SIZE_MAX];
    crsfFrameDef_t frame;
} crsfFrame_t;


void crsfRxWriteTelemetryData(const void *data, int len);
void crsfRxSendTelemetryData(void);

struct rxConfig_s;
struct rxRuntimeConfig_s;
bool crsfRxInit(const struct rxConfig_s *initialRxConfig, struct rxRuntimeConfig_s *rxRuntimeConfig);
bool crsfRxIsActive(void);

 void crsfDataReceive(uint16_t c, void *rxCallbackData);
 uint8_t crsfFrameStatus(void *rxRuntimeConfig);



typedef enum {
    RX_FRAME_PENDING = 0,               // No new data available from receiver
    RX_FRAME_COMPLETE = (1 << 0),       // There is new data available
    RX_FRAME_FAILSAFE = (1 << 1),       // Receiver detected loss of RC link. Only valid when RX_FRAME_COMPLETE is set as well
    RX_FRAME_PROCESSING_REQUIRED = (1 << 2),
} rxFrameState_e;



#define CRSF_TIME_NEEDED_PER_FRAME_US   1000    /*接收或发送“一整帧 CRSF 数据”所需的最大时间 1000us*/
#define CRSF_TIME_BETWEEN_FRAMES_US     4000    /*CRSF 发射机（TX）固定每 4ms 发送一帧*/

#define CRSF_DIGITAL_CHANNEL_MIN 172            /*CRSF 协议中，RC 通道的“原始 11bit 数值边界”*/
#define CRSF_DIGITAL_CHANNEL_MAX 1811           /*最小值：172 最大值：1811  中位值：992*/






/*通道原始数据*/
typedef struct __attribute__ ((__packed__))
{
    unsigned int chn0:11;
    unsigned int chn1:11;
    unsigned int chn2:11;
    unsigned int chn3:11;
    unsigned int chn4:11;
    unsigned int chn5:11;
    unsigned int chn6:11;
    unsigned int chn7:11;
    unsigned int chn8:11;
    unsigned int chn9:11;
    unsigned int chn10:11;
    unsigned int chn11:11;
    unsigned int chn12:11;
    unsigned int chn13:11;
    unsigned int chn14:11;
    unsigned int chn15:11;
    unsigned int chn16:11;
}crfs_chn_rawdata_t;

/*crsf连接状态*/
typedef struct __attribute__ ((__packed__)) {
    uint8_t     uplink_rssi_ant1;           /*上行RSSI -天线1*/
    uint8_t     uplink_rssi_ant2;           /*上行RSSI -天线2*/
    uint8_t     uplink_lq;                  /*上行链路质量LQ*/
    int8_t      uplink_snr;                 /*上行信噪比SNR*/
    uint8_t     active_ant;                 /*当前使用的天线*/
    uint8_t     rf_mode;                    /*射频工作模式*/
    uint8_t     uplink_tx_power;            /*上行发射功率*/
    uint8_t     downlink_rssi;              /*下行RSSI*/
    uint8_t     downlink_lq;                /*下行链路质量LQ*/
    int8_t      downlink_snr;               /*下行信噪比*/
}crfs_link_state_t;

typedef struct{
    uint8_t dev_addr;                                   /*设备地址*/
    uint8_t len;                                        /*帧长度*/
    uint8_t type;                                       /*帧类型*/
    uint8_t payload[CRSF_PAYLOAD_SIZE_MAX + 1];         /*帧数据+一个crc校验*/
}crfs_frame_def_t;

typedef union {
    uint8_t bytes[CRSF_FRAME_SIZE_MAX];
    crfs_frame_def_t frame;
}crfs_frame_t;



class crfs
{
private:
    /* data */

    bool frame_end;                             /*一帧接收结束*/

    uint8_t rx_state;                           /*接受状态*/
    uint32_t frame_index;
        
    crfs_frame_t crfs_frame;                    /*帧率数据*/
    crfs_link_state_t link_state;               /*帧状态*/

    uint32_t raw_data[CRSF_MAX_CHANNEL];        /*通道原始数据*/   
    uint16_t chn_data[CRSF_MAX_CHANNEL];        /*通道数据*/

    uint8_t crc_calc(void);

    uint32_t get_time_interval_us(void);        /*计算时间间隔：us*/
    uint8_t crc8_dvb_s2(uint8_t crc, uint8_t a);

public:
    crfs(/* args */);
    ~crfs();

    /*接受数据*/
    void rx(uint8_t c);
    uint16_t read_raw_rc(uint8_t chn);

    uint8_t run(void);
};





















#endif
