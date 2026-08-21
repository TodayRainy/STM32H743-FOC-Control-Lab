#include "crfs.hpp"

#include "main.h"
#include "tim.h"

#include <string.h>
#include <math.h>

typedef uint32_t timeUs_t;
typedef int32_t timeDelta_t;
// millisecond time
typedef uint32_t timeMs_t ;

extern uint32_t tim7_cnt;
extern "C" uint32_t micros(void)
{
    // register uint32_t ms, cycle_cnt;
    uint32_t us = __HAL_TIM_GET_COUNTER(&htim7)+tim7_cnt*1000;

    __HAL_TIM_SET_COUNTER(&htim7, 0);
    tim7_cnt = 0;

    return us;
}

static inline timeDelta_t cmpTimeUs(timeUs_t a, timeUs_t b) { return (timeDelta_t)(a - b); }

int scaleRange(int x, int srcMin, int srcMax, int destMin, int destMax) {
    long int a = ((long int) destMax - (long int) destMin) * ((long int) x - (long int) srcMin);
    long int b = (long int) srcMax - (long int) srcMin;
    return ((a / b) + destMin);
}

int constrain(int amt, int low, int high)
{
    if (amt < low)
        return low;
    else if (amt > high)
        return high;
    else
        return amt;
}


#define CRSF_TIME_NEEDED_PER_FRAME_US   1000
#define CRSF_TIME_BETWEEN_FRAMES_US     4000 // a frame is sent by the transmitter every 4 milliseconds

#define CRSF_DIGITAL_CHANNEL_MIN 172
#define CRSF_DIGITAL_CHANNEL_MAX 1811

STATIC_UNIT_TESTED bool crsfFrameDone = false;
STATIC_UNIT_TESTED crsfFrame_t crsfFrame;

uint8_t crsfFramePosition = 0;

 uint32_t crsfChannelData[CRSF_MAX_CHANNEL];

// static serialPort_t *serialPort;
 timeUs_t crsfFrameStartAt = 0;
 uint8_t telemetryBuf[CRSF_FRAME_SIZE_MAX];
 uint8_t telemetryBufLen = 0;

/*高速串口传输 RC 遥控通道数据*/
struct crsfPayloadRcChannelsPacked_s {
    // 176 bits of data (11 bits per channel * 16 channels) = 22 bytes.
    unsigned int chan0 : 11;
    unsigned int chan1 : 11;
    unsigned int chan2 : 11;
    unsigned int chan3 : 11;
    unsigned int chan4 : 11;
    unsigned int chan5 : 11;
    unsigned int chan6 : 11;
    unsigned int chan7 : 11;
    unsigned int chan8 : 11;
    unsigned int chan9 : 11;
    unsigned int chan10 : 11;
    unsigned int chan11 : 11;
    unsigned int chan12 : 11;
    unsigned int chan13 : 11;
    unsigned int chan14 : 11;
    unsigned int chan15 : 11;
} __attribute__ ((__packed__));
typedef struct crsfPayloadRcChannelsPacked_s crsfPayloadRcChannelsPacked_t;

/*CRSF 协议中的 Link Statistics（链路状态）Payload*/
struct crsfPayloadLinkStatistics_s {
    uint8_t     uplinkRSSIAnt1;
    uint8_t     uplinkRSSIAnt2;
    uint8_t     uplinkLQ;
    int8_t      uplinkSNR;
    uint8_t     activeAntenna;
    uint8_t     rfMode;
    uint8_t     uplinkTXPower;
    uint8_t     downlinkRSSI;
    uint8_t     downlinkLQ;
    int8_t      downlinkSNR;
} __attribute__ ((__packed__));

typedef struct crsfPayloadLinkStatistics_s crsfPayloadLinkStatistics_t;


/*在串口 RX 中断中，按字节接收 CRSF 数据，通过“时间 + 长度”判断帧边界，把一整帧 CRSF 数据拼出来。*/
// Receive ISR callback, called back from serial port
uint32_t cnt = 0;
void crsfDataReceive(uint16_t c, void *rxCallbackData)
{
    UNUSED(rxCallbackData);

    const timeUs_t now = micros();
    

#ifdef DEBUG_CRSF_PACKETS
    debug[2] = now - crsfFrameStartAt;
#endif

    if (now > crsfFrameStartAt + CRSF_TIME_NEEDED_PER_FRAME_US) {
        // We've received a character after max time needed to complete a frame,
        // so this must be the start of a new frame.
        crsfFramePosition = 0;
    }

    if (crsfFramePosition == 0) {
        crsfFrameStartAt = now;
    }
    // assume frame is 5 bytes long until we have received the frame length
    // full frame length includes the length of the address and framelength fields
    const int fullFrameLength = crsfFramePosition < 3 ? 5 : crsfFrame.frame.frameLength + CRSF_FRAME_LENGTH_ADDRESS + CRSF_FRAME_LENGTH_FRAMELENGTH;

    if (crsfFramePosition < fullFrameLength) {
        crsfFrame.bytes[crsfFramePosition++] = (uint8_t)c;
        crsfFrameDone = crsfFramePosition < fullFrameLength ? false : true;

        if(true==crsfFrameDone){
            // __HAL_TIM_SET_COUNTER(&htim7, 0);
        }
    }
}


uint8_t crc8_dvb_s2(uint8_t crc, unsigned char a)
{
    crc ^= a;
    for (int ii = 0; ii < 8; ++ii) {
        if (crc & 0x80) {
            crc = (crc << 1) ^ 0xD5;
        } else {
            crc = crc << 1;
        }
    }
    return crc;
}

uint8_t crsfFrameCRC(void)
{
    // CRC includes type and payload
    uint8_t crc = crc8_dvb_s2(0, crsfFrame.frame.type);
    for (int ii = 0; ii < crsfFrame.frame.frameLength - CRSF_FRAME_LENGTH_TYPE_CRC; ++ii) {
        crc = crc8_dvb_s2(crc, crsfFrame.frame.payload[ii]);
    }
    return crc;
}

/*crsfFrameStatus() 是 CRSF RX 的“解析与分发层”：
ISR 只负责拼帧，这里才负责 CRC 校验 + 数据解包 + 通道更新。
*/
 uint8_t crsfFrameStatus(void *rxRuntimeConfig)
{
    UNUSED(rxRuntimeConfig);

    if (crsfFrameDone) {
        crsfFrameDone = false;
        if (crsfFrame.frame.type == CRSF_FRAMETYPE_RC_CHANNELS_PACKED) {

           

            // CRC includes type and payload of each frame
            const uint8_t crc = crsfFrameCRC();
            if (crc != crsfFrame.frame.payload[CRSF_FRAME_RC_CHANNELS_PAYLOAD_SIZE]) {
                return RX_FRAME_PENDING;
            }
            crsfFrame.frame.frameLength = CRSF_FRAME_RC_CHANNELS_PAYLOAD_SIZE + CRSF_FRAME_LENGTH_TYPE_CRC;

            // unpack the RC channels
            const crsfPayloadRcChannelsPacked_t* rcChannels = (crsfPayloadRcChannelsPacked_t*)&crsfFrame.frame.payload;
            crsfChannelData[0] = rcChannels->chan0;
            crsfChannelData[1] = rcChannels->chan1;
            crsfChannelData[2] = rcChannels->chan2;
            crsfChannelData[3] = rcChannels->chan3;
            crsfChannelData[4] = rcChannels->chan4;
            crsfChannelData[5] = rcChannels->chan5;
            crsfChannelData[6] = rcChannels->chan6;
            crsfChannelData[7] = rcChannels->chan7;
            crsfChannelData[8] = rcChannels->chan8;
            crsfChannelData[9] = rcChannels->chan9;
            crsfChannelData[10] = rcChannels->chan10;
            crsfChannelData[11] = rcChannels->chan11;
            crsfChannelData[12] = rcChannels->chan12;
            crsfChannelData[13] = rcChannels->chan13;
            crsfChannelData[14] = rcChannels->chan14;
            crsfChannelData[15] = rcChannels->chan15;
            return RX_FRAME_COMPLETE;
        }
        else if (crsfFrame.frame.type == CRSF_FRAMETYPE_LINK_STATISTICS) {
            // CRC includes type and payload of each frame
            const uint8_t crc = crsfFrameCRC();
            if (crc != crsfFrame.frame.payload[CRSF_FRAME_LINK_STATISTICS_PAYLOAD_SIZE]) {
                return RX_FRAME_PENDING;
            }
            crsfFrame.frame.frameLength = CRSF_FRAME_LINK_STATISTICS_PAYLOAD_SIZE + CRSF_FRAME_LENGTH_TYPE_CRC;

            // Inject link quality into channel 17
            const crsfPayloadLinkStatistics_t* linkStats = (crsfPayloadLinkStatistics_t*)&crsfFrame.frame.payload;
            crsfChannelData[16] = scaleRange(constrain(linkStats->uplinkLQ, 0, 100), 0, 100, 191, 1791);    // will map to [1000;2000] range

            // This is not RC channels frame, update channel value but don't indicate frame completion
            return RX_FRAME_PENDING;
        }
    }
    return RX_FRAME_PENDING;
}

/*把 CRSF 协议中的“原始 RC 通道值”转换成传统 RC 系统使用的 PWM 脉宽（微秒）表示。*/
STATIC_UNIT_TESTED uint16_t crsfReadRawRC(const void *rxRuntimeConfig, uint8_t chan)
{
    UNUSED(rxRuntimeConfig);
    /* conversion from RC value to PWM
     *       RC     PWM
     * min  172 ->  988us
     * mid  992 -> 1500us
     * max 1811 -> 2012us
     * scale factor = (2012-988) / (1811-172) = 0.62477120195241
     * offset = 988 - 172 * 0.62477120195241 = 880.53935326418548
     */
    return (crsfChannelData[chan] * 1024 / 1639) + 881;
}

/*把一帧“准备好的 CRSF 遥测数据”复制进内部发送缓冲区，并标记长度，等待后续在合适的时隙发出。*/
void crsfRxWriteTelemetryData(const void *data, int len)
{
    // len = MIN(len, (int)sizeof(telemetryBuf));
    len = len<=((int)sizeof(telemetryBuf))?len:(int)sizeof(telemetryBuf);
    memcpy(telemetryBuf, data, len);
    telemetryBufLen = len;
}

void crsfRxSendTelemetryData(void)
{
    // if there is telemetry data to write
    if (telemetryBufLen > 0) {
        // check that we are not in bi dir mode or that we are not currently receiving data (ie in the middle of an RX frame)
        // and that there is time to send the telemetry frame before the next RX frame arrives
        if (CRSF_PORT_OPTIONS & SERIAL_BIDIR) {
            /*单双工模式*/
            const timeDelta_t timeSinceStartOfFrame = cmpTimeUs(micros(), crsfFrameStartAt);
            if ((timeSinceStartOfFrame < CRSF_TIME_NEEDED_PER_FRAME_US) ||
                (timeSinceStartOfFrame > CRSF_TIME_BETWEEN_FRAMES_US - CRSF_TIME_NEEDED_PER_FRAME_US)) {
                return;
            }
        }
        // serialWriteBuf(serialPort, telemetryBuf, telemetryBufLen);

        /*串口发送*/

        telemetryBufLen = 0; // reset telemetry buffer
    }
}

bool crsfRxInit(const void *rxConfig, void *rxRuntimeConfig)
{
    for (int ii = 0; ii < CRSF_MAX_CHANNEL; ++ii) {
        // crsfChannelData[ii] = (16 * rxConfig->midrc) / 10 - 1408;
        crsfChannelData[ii] = (16 * 1500) / 10 - 1408;
    }

    // rxRuntimeConfig->channelCount = CRSF_MAX_CHANNEL;
    // rxRuntimeConfig->rxRefreshRate = 11000; //!!TODO this needs checking

    // rxRuntimeConfig->rcReadRawFn = crsfReadRawRC;
    // rxRuntimeConfig->rcFrameStatusFn = crsfFrameStatus;

    // const serialPortConfig_t *portConfig = findSerialPortConfig(FUNCTION_RX_SERIAL);
    // if (!portConfig) {
    //     return false;
    // }

    // serialPort = openSerialPort(portConfig->identifier,
    //     FUNCTION_RX_SERIAL,
    //     crsfDataReceive,
    //     NULL,
    //     CRSF_BAUDRATE,
    //     CRSF_PORT_MODE,
    //     CRSF_PORT_OPTIONS | (rxConfig->halfDuplex ? SERIAL_BIDIR : 0)
    //     );

    // return serialPort != NULL;
    return true;
}


bool crsfRxIsActive(void)
{
    return true;
}





crfs::crfs(/* args */)
{
    this->rx_state = 0;
    frame_index = 0;
    frame_end = false;
}

crfs::~crfs()
{
}

void crfs::rx(uint16_t c)
{
    #if 1
    // const timeUs_t now = micros();
    int fullFrameLength = 0;

    int frame_len = frame_index < 3 ? 5 : crsfFrame.frame.frameLength + CRSF_FRAME_LENGTH_ADDRESS + CRSF_FRAME_LENGTH_FRAMELENGTH;;

    /*如果延时周期大于 1000us则重新接收*/
    if(micros()>CRSF_TIME_NEEDED_PER_FRAME_US){
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
                crsfFrame.bytes[frame_index++] = (uint8_t)c;
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

    #else
    const timeUs_t now = micros();
    // __HAL_TIM_SET_COUNTER(&htim7, 0);
    

#ifdef DEBUG_CRSF_PACKETS
    debug[2] = now - crsfFrameStartAt;
#endif

    crsfFrameStartAt = 0;
    if (now > crsfFrameStartAt + CRSF_TIME_NEEDED_PER_FRAME_US) {
        // We've received a character after max time needed to complete a frame,
        // so this must be the start of a new frame.
        crsfFramePosition = 0;
    }

    // if (crsfFramePosition == 0) {
    //     crsfFrameStartAt = now;
    // }
    // assume frame is 5 bytes long until we have received the frame length
    // full frame length includes the length of the address and framelength fields
    const int fullFrameLength = crsfFramePosition < 3 ? 5 : crsfFrame.frame.frameLength + CRSF_FRAME_LENGTH_ADDRESS + CRSF_FRAME_LENGTH_FRAMELENGTH;

    if (crsfFramePosition < fullFrameLength) {
        crsfFrame.bytes[crsfFramePosition++] = (uint8_t)c;
        crsfFrameDone = crsfFramePosition < fullFrameLength ? false : true;

        if(true==crsfFrameDone){
            __HAL_TIM_SET_COUNTER(&htim7, 0);
        }
    }

    #endif

    

}


uint8_t crfs::crc_calc(void)
{
    uint8_t crc = crc8_dvb_s2(0, crsfFrame.frame.type);
    for (int ii = 0; ii < crsfFrame.frame.frameLength - CRSF_FRAME_LENGTH_TYPE_CRC; ++ii) {
        crc = crc8_dvb_s2(crc, crsfFrame.frame.payload[ii]);
    }
    return crc;
}

uint8_t crfs::run(void)
{
    if (frame_end) {
        frame_end = false;
        if (crsfFrame.frame.type == CRSF_FRAMETYPE_RC_CHANNELS_PACKED) {

           

            // CRC includes type and payload of each frame
            const uint8_t crc = crsfFrameCRC();
            if (crc != crsfFrame.frame.payload[CRSF_FRAME_RC_CHANNELS_PAYLOAD_SIZE]) {
                return RX_FRAME_PENDING;
            }
            crsfFrame.frame.frameLength = CRSF_FRAME_RC_CHANNELS_PAYLOAD_SIZE + CRSF_FRAME_LENGTH_TYPE_CRC;

            // unpack the RC channels
            const crsfPayloadRcChannelsPacked_t* rcChannels = (crsfPayloadRcChannelsPacked_t*)&crsfFrame.frame.payload;
            crsfChannelData[0] = rcChannels->chan0;
            crsfChannelData[1] = rcChannels->chan1;
            crsfChannelData[2] = rcChannels->chan2;
            crsfChannelData[3] = rcChannels->chan3;
            crsfChannelData[4] = rcChannels->chan4;
            crsfChannelData[5] = rcChannels->chan5;
            crsfChannelData[6] = rcChannels->chan6;
            crsfChannelData[7] = rcChannels->chan7;
            crsfChannelData[8] = rcChannels->chan8;
            crsfChannelData[9] = rcChannels->chan9;
            crsfChannelData[10] = rcChannels->chan10;
            crsfChannelData[11] = rcChannels->chan11;
            crsfChannelData[12] = rcChannels->chan12;
            crsfChannelData[13] = rcChannels->chan13;
            crsfChannelData[14] = rcChannels->chan14;
            crsfChannelData[15] = rcChannels->chan15;

            rx_state = 0;
            return RX_FRAME_COMPLETE;
        }
        else if (crsfFrame.frame.type == CRSF_FRAMETYPE_LINK_STATISTICS) {
            // CRC includes type and payload of each frame
            const uint8_t crc = crsfFrameCRC();
            if (crc != crsfFrame.frame.payload[CRSF_FRAME_LINK_STATISTICS_PAYLOAD_SIZE]) {
                return RX_FRAME_PENDING;
            }
            crsfFrame.frame.frameLength = CRSF_FRAME_LINK_STATISTICS_PAYLOAD_SIZE + CRSF_FRAME_LENGTH_TYPE_CRC;

            // Inject link quality into channel 17
            const crsfPayloadLinkStatistics_t* linkStats = (crsfPayloadLinkStatistics_t*)&crsfFrame.frame.payload;
            crsfChannelData[16] = scaleRange(constrain(linkStats->uplinkLQ, 0, 100), 0, 100, 191, 1791);    // will map to [1000;2000] range

            // This is not RC channels frame, update channel value but don't indicate frame completion
            return RX_FRAME_PENDING;
        }
    }

    for (size_t i = 0; i < CRSF_MAX_CHANNEL; i++)
    {
        chn_val[i] = read_raw_rc(i);
    }
    

    return RX_FRAME_PENDING;
}

uint16_t crfs::read_raw_rc(uint8_t chn)
{
    return (crsfChannelData[chn] * 1024 / 1639) + 881;
}

