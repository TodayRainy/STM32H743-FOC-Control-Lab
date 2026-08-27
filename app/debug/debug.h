#ifndef _DEBUG_TASK_H
#define _DEBUG_TASK_H

#include <stdint.h>

#include "foc_type.h"


/*vofa的通道数量*/
#define VOFA_CH_COUNT               36

#define VOFA_RECV_CH_COUNT               4

#define DEBUG_SCOPE_CHANNEL_COUNT   12U
#define DEBUG_SCOPE_SAMPLE_CAPACITY 4096U
#define DEBUG_SCOPE_MAGIC           0x31504353UL

typedef enum
{
    DEBUG_SCOPE_IDLE = 0,
    DEBUG_SCOPE_RECORDING,
    DEBUG_SCOPE_DONE,
} Debug_ScopeState_t;

typedef struct
{
    float ia;
    float ib;
    float iq;
    float id;
    float iq_ref;
    float id_ref;
    float uq;
    float ud;
    float elec_angle;
    float elec_speed;
    float speed_ref;
    float voltage_scale;
} Debug_ScopeSample_t;

typedef struct
{
    uint32_t magic;
    uint32_t state;
    uint32_t base_sample_hz;
    uint32_t divider;
    uint32_t sample_count;
    uint32_t sample_capacity;
    uint32_t channel_count;
    uint32_t sample_size;
    uint32_t max_capture_cycles;
} Debug_ScopeControl_t;

/*发送结构体*/
typedef struct{
    float fdata[VOFA_CH_COUNT];
    const unsigned char tail[4];          /*尾帧 0x00, 0x00, 0x80, 0x7f*/ 
}VOFA_Send_Handle_t;

/*接收结构体*/
typedef struct{
    float fdata[VOFA_RECV_CH_COUNT];
}VOFA_Recv_Handle_t;

#ifdef __cplusplus
extern "C" {
#endif


void Debug_Task(void);
void Debug_ScopeInit(uint32_t base_sample_hz);
void Debug_ScopeSample(const FOC_DEBUG_t *debug);

extern Debug_ScopeSample_t foc_scope_buffer[DEBUG_SCOPE_SAMPLE_CAPACITY];
extern volatile Debug_ScopeControl_t foc_scope_control;

#ifdef __cplusplus
}
#endif

#endif


