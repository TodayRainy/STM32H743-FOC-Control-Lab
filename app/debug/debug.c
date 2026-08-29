#include "debug.h"

#include "foc_type.h"
#include "main.h"
#include "usbd_cdc_interface.h"

#include <stdlib.h>
#include <string.h>

#define DEBUG_TELEMETRY_PERIOD_MS  2U
#define DEBUG_SNAPSHOT_RETRIES     4U
#define DEBUG_SCOPE_PROTOCOL_VER   1U
#define DEBUG_SCOPE_TX_CHUNK       200U

typedef struct
{
    uint32_t magic;
    uint16_t version;
    uint16_t channel_count;
    uint32_t sample_count;
    uint32_t divider;
    uint32_t base_sample_hz;
    uint32_t sample_size;
    uint32_t data_bytes;
    uint32_t max_capture_cycles;
    uint32_t cpu_hz;
    uint32_t reserved;
} Debug_ScopeHeader_t;

VOFA_Send_Handle_t VOFA_Handle =
{
    .tail = {0x00, 0x00, 0x80, 0x7f},
};

volatile uint32_t debug_snapshot_failures = 0;

#pragma arm section rwdata = "SCOPE_DATA", zidata = "SCOPE_DATA"

__align(32) Debug_ScopeSample_t foc_scope_buffer[DEBUG_SCOPE_SAMPLE_CAPACITY];
volatile Debug_ScopeControl_t foc_scope_control =
{
    DEBUG_SCOPE_MAGIC,
    DEBUG_SCOPE_IDLE,
    40000U,
    1U,
    0U,
    DEBUG_SCOPE_SAMPLE_CAPACITY,
    DEBUG_SCOPE_CHANNEL_COUNT,
    sizeof(Debug_ScopeSample_t),
    0U,
};

#pragma arm section

static Debug_ScopeHeader_t debug_scope_header;
static uint32_t debug_scope_divider_count = 0U;
static uint32_t debug_scope_dump_offset = 0U;
static uint8_t debug_scope_dump_stage = 0U;

extern FOC_DEBUG_t foc_debug;
extern volatile uint32_t foc_debug_seq;
extern uint8_t FOC_CommandStartSpeed(void);
extern uint8_t FOC_CommandStartHfi(void);
extern uint8_t FOC_CommandStartSensorless(float speed_rpm);
extern uint8_t FOC_CommandStartHfiShadow(float injection_voltage,
                                          float initial_offset_deg);
extern uint8_t FOC_CommandStartHfiCharacterization(float injection_voltage,
                                                    float initial_offset_deg);
extern uint8_t FOC_CommandStartHfiTensorCharacterization(
    float injection_voltage, float initial_offset_deg);
extern uint8_t FOC_CommandActivateHfiControl(void);
extern uint8_t FOC_CommandActivateHfiDAxisTracking(void);
extern uint8_t FOC_CommandStopHfiShadow(void);
extern uint8_t FOC_CommandStartCurrent(void);
extern uint8_t FOC_CommandStartPosition(void);
extern uint8_t FOC_CommandStartIdentificationRL(void);
extern uint8_t FOC_CommandStartIdentificationFlux(void);
extern uint8_t FOC_CommandPrintIdentification(void);
extern uint8_t FOC_CommandPrintIdentificationPoints(void);
extern uint8_t FOC_CommandSetSpeed(float speed_rpm);
extern uint8_t FOC_CommandSetSpeedSine(float amplitude_rpm,
                                       float frequency_hz);
extern uint8_t FOC_CommandStopSpeedSine(void);
extern uint8_t FOC_CommandSetSpeedOutputLimit(float limit_a);
extern uint8_t FOC_CommandSetSpeedEstimator(uint8_t mode);
extern uint8_t FOC_CommandSetCurrent(float iq, float id);
extern uint8_t FOC_CommandSetCurrentSine(uint8_t q_axis,
                                         float amplitude,
                                         float frequency_hz);
extern uint8_t FOC_CommandStopCurrentSine(void);
extern uint8_t FOC_CommandSetPosition(float position_deg);
extern uint8_t FOC_CommandSetPositionPI(float kp, float ki, float kd);
extern uint8_t FOC_CommandSetPositionSpeedLimit(float limit_rpm);
extern uint8_t FOC_CommandSetSpeedPI(float kp, float ki);
extern uint8_t FOC_CommandSetIqPI(float kp, float ki);
extern uint8_t FOC_CommandSetIdPI(float kp, float ki);
extern uint8_t FOC_CommandClearPI(void);
extern uint8_t FOC_CommandSetSpeedStep(void);
extern uint8_t FOC_CommandSetSpeedRamp(float accel_rpm_per_sec);
extern uint8_t FOC_CommandSetPositionStep(void);
extern uint8_t FOC_CommandSetPositionRamp(float velocity_deg_per_sec);
extern uint8_t FOC_CommandSetPositionTrapezoid(float velocity_deg_per_sec,
                                               float acceleration_deg_per_sec2);
extern uint8_t FOC_CommandSetPositionFeedforward(uint8_t enabled);
extern uint8_t FOC_CommandSetFeedforward(uint8_t enabled);
extern uint8_t FOC_CommandSetFeedforwardMode(uint8_t mode);
extern uint8_t FOC_CommandSetFeedforwardModel(uint8_t identified);
extern uint8_t FOC_CommandSetSpeedId(float id);
extern uint8_t FOC_CommandSetFieldWeakeningMode(uint8_t mode);
extern uint8_t FOC_CommandSetVoltageLimit(float ratio);
extern uint8_t FOC_CommandStop(void);
extern void FOC_ServiceFaultReport(void);

void Debug_ScopeInit(uint32_t base_sample_hz)
{
    foc_scope_control.base_sample_hz = base_sample_hz;
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->LAR = 0xC5ACCE55U;
    DWT->CYCCNT = 0U;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

static uint8_t Debug_ScopeArm(uint32_t divider)
{
    if(debug_scope_dump_stage != 0U){
        return 0U;
    }

    foc_scope_control.state = DEBUG_SCOPE_IDLE;
    __DMB();
    debug_scope_divider_count = 0U;
    foc_scope_control.divider = divider;
    foc_scope_control.sample_count = 0U;
    foc_scope_control.max_capture_cycles = 0U;
    __DMB();
    foc_scope_control.state = DEBUG_SCOPE_RECORDING;
    return 1U;
}

static uint8_t Debug_ScopeStartDump(void)
{
    if((foc_scope_control.state != DEBUG_SCOPE_DONE) ||
       (debug_scope_dump_stage != 0U)){
        return 0U;
    }

    __DMB();
    debug_scope_header.magic = DEBUG_SCOPE_MAGIC;
    debug_scope_header.version = DEBUG_SCOPE_PROTOCOL_VER;
    debug_scope_header.channel_count = DEBUG_SCOPE_CHANNEL_COUNT;
    debug_scope_header.sample_count = foc_scope_control.sample_count;
    debug_scope_header.divider = foc_scope_control.divider;
    debug_scope_header.base_sample_hz = foc_scope_control.base_sample_hz;
    debug_scope_header.sample_size = sizeof(Debug_ScopeSample_t);
    debug_scope_header.data_bytes = foc_scope_control.sample_count*sizeof(Debug_ScopeSample_t);
    debug_scope_header.max_capture_cycles = foc_scope_control.max_capture_cycles;
    debug_scope_header.cpu_hz = SystemCoreClock;
    debug_scope_header.reserved = 0U;

    debug_scope_dump_offset = 0U;
    debug_scope_dump_stage = 1U;
    return 1U;
}

static uint8_t Debug_ScopeDumpTask(void)
{
    uint32_t length;
    const uint8_t *data = (const uint8_t *)foc_scope_buffer;

    if(debug_scope_dump_stage == 0U){
        return 0U;
    }

    if(debug_scope_dump_stage == 1U){
        if(VCP_DataTx((const uint8_t *)&debug_scope_header,
                      sizeof(debug_scope_header)) == USBD_OK){
            debug_scope_dump_stage = 2U;
        }
        return 1U;
    }

    if(debug_scope_dump_offset >= debug_scope_header.data_bytes){
        debug_scope_dump_stage = 0U;
        return 1U;
    }

    length = debug_scope_header.data_bytes - debug_scope_dump_offset;
    if(length > DEBUG_SCOPE_TX_CHUNK){
        length = DEBUG_SCOPE_TX_CHUNK;
    }
    if(VCP_DataTx(&data[debug_scope_dump_offset], length) == USBD_OK){
        debug_scope_dump_offset += length;
    }
    return 1U;
}

#pragma arm section code = "RAMCODE"

void Debug_ScopeSample(const FOC_DEBUG_t *debug)
{
    Debug_ScopeSample_t *sample;
    uint32_t start_cycles;
    uint32_t elapsed_cycles;
    uint32_t index;

    if(foc_scope_control.state != DEBUG_SCOPE_RECORDING){
        return;
    }

    ++debug_scope_divider_count;
    if(debug_scope_divider_count < foc_scope_control.divider){
        return;
    }
    debug_scope_divider_count = 0U;

    start_cycles = DWT->CYCCNT;
    index = foc_scope_control.sample_count;
    sample = &foc_scope_buffer[index];
    sample->ia = debug->Iabc.a;
    sample->ib = debug->Iabc.b;
    sample->iq = debug->Iqd.q;
    sample->id = debug->Iqd.d;
    sample->iq_ref = debug->Iqd_ref.q;
    sample->id_ref = debug->Iqd_ref.d;
    sample->uq = debug->Uqd.q;
    sample->ud = debug->Uqd.d;
    sample->elec_angle = debug->abzenc_speed.elec_angle;
    sample->elec_speed = debug->elec_speed;
    sample->speed_ref = debug->speed_ref;
    sample->voltage_scale = debug->voltage_scale;
    sample->hfi_phase_error = debug->hfi_phase_error;
    sample->hfi_d_response = debug->hfi_d_response;
    sample->hfi_q_response = debug->hfi_q_response;

    ++index;
    foc_scope_control.sample_count = index;
    if(index >= DEBUG_SCOPE_SAMPLE_CAPACITY){
        __DMB();
        foc_scope_control.state = DEBUG_SCOPE_DONE;
    }

    elapsed_cycles = DWT->CYCCNT - start_cycles;
    if(elapsed_cycles > foc_scope_control.max_capture_cycles){
        foc_scope_control.max_capture_cycles = elapsed_cycles;
    }
}

#pragma arm section

static uint8_t Debug_ParseScopeDivider(const char *text, uint32_t *divider)
{
    if(strcmp(text, "1") == 0){
        *divider = 1U;
    }else if(strcmp(text, "10") == 0){
        *divider = 10U;
    }else if(strcmp(text, "100") == 0){
        *divider = 100U;
    }else{
        return 0U;
    }
    return 1U;
}

static uint8_t Debug_ParseFloat(const char *text, float *value)
{
    char *end;

    *value = strtof(text, &end);
    return ((end != text) && (*end == '\0')) ? 1U : 0U;
}

static uint8_t Debug_ParseTwoFloats(const char *text, float *first, float *second)
{
    char *end;

    *first = strtof(text, &end);
    if((end == text) || (*end != ' ')){
        return 0U;
    }
    text = end;
    while(*text == ' '){
        ++text;
    }

    *second = strtof(text, &end);
    return ((end != text) && (*end == '\0')) ? 1U : 0U;
}

static uint8_t Debug_ParseThreeFloats(const char *text, float *first,
                                      float *second, float *third)
{
    char *end;

    *first = strtof(text, &end);
    if((end == text) || (*end != ' ')){
        return 0U;
    }
    while(*end == ' '){
        ++end;
    }

    text = end;
    *second = strtof(text, &end);
    if((end == text) || (*end != ' ')){
        return 0U;
    }
    while(*end == ' '){
        ++end;
    }

    text = end;
    *third = strtof(text, &end);
    return ((end != text) && (*end == '\0')) ? 1U : 0U;
}

static void Debug_CommandTask(void)
{
    uint16_t status = USB_USART_RX_STA;
    uint16_t length;
    uint32_t divider;
    float value;
    float kp;
    float ki;
    float kd;

    if((status & 0x8000U) == 0U){
        return;
    }

    length = status & 0x3FFFU;
    USB_USART_RX_BUF[length] = '\0';

    if(strncmp((char *)USB_USART_RX_BUF, "SCOPE ARM ", 10U) == 0){
        if(Debug_ParseScopeDivider((char *)&USB_USART_RX_BUF[10], &divider) != 0U){
            (void)Debug_ScopeArm(divider);
        }
    }else if(strcmp((char *)USB_USART_RX_BUF, "SCOPE DUMP") == 0){
        (void)Debug_ScopeStartDump();
    }else if(strncmp((char *)USB_USART_RX_BUF, "START SENSORLESS ", 17U) == 0){
        if(Debug_ParseFloat((char *)&USB_USART_RX_BUF[17], &value) != 0U){
            (void)FOC_CommandStartSensorless(value);
        }
    }else if(strcmp((char *)USB_USART_RX_BUF, "START HFI") == 0){
        (void)FOC_CommandStartHfi();
    }else if(strncmp((char *)USB_USART_RX_BUF, "HFI SHADOW ", 11U) == 0){
        if(Debug_ParseTwoFloats((char *)&USB_USART_RX_BUF[11], &kp, &ki) != 0U){
            (void)FOC_CommandStartHfiShadow(kp, ki);
        }
    }else if(strncmp((char *)USB_USART_RX_BUF, "HFI FIXED ", 10U) == 0){
        if(Debug_ParseTwoFloats((char *)&USB_USART_RX_BUF[10], &kp, &ki) != 0U){
            (void)FOC_CommandStartHfiCharacterization(kp, ki);
        }
    }else if(strncmp((char *)USB_USART_RX_BUF, "HFI TENSOR ", 11U) == 0){
        if(Debug_ParseTwoFloats((char *)&USB_USART_RX_BUF[11], &kp, &ki) != 0U){
            (void)FOC_CommandStartHfiTensorCharacterization(kp, ki);
        }
    }else if(strcmp((char *)USB_USART_RX_BUF, "HFI CONTROL") == 0){
        (void)FOC_CommandActivateHfiControl();
    }else if(strcmp((char *)USB_USART_RX_BUF, "HFI DTRACK") == 0){
        (void)FOC_CommandActivateHfiDAxisTracking();
    }else if(strcmp((char *)USB_USART_RX_BUF, "HFI OFF") == 0){
        (void)FOC_CommandStopHfiShadow();
    }else if(strcmp((char *)USB_USART_RX_BUF, "START SPEED") == 0){
        (void)FOC_CommandStartSpeed();
    }else if(strcmp((char *)USB_USART_RX_BUF, "START CURRENT") == 0){
        (void)FOC_CommandStartCurrent();
    }else if(strcmp((char *)USB_USART_RX_BUF, "START POSITION") == 0){
        (void)FOC_CommandStartPosition();
    }else if(strcmp((char *)USB_USART_RX_BUF, "START IDENT RL") == 0){
        (void)FOC_CommandStartIdentificationRL();
    }else if(strcmp((char *)USB_USART_RX_BUF, "START IDENT FLUX") == 0){
        (void)FOC_CommandStartIdentificationFlux();
    }else if(strcmp((char *)USB_USART_RX_BUF, "IDENT RESULT") == 0){
        (void)FOC_CommandPrintIdentification();
    }else if(strcmp((char *)USB_USART_RX_BUF, "IDENT POINTS") == 0){
        (void)FOC_CommandPrintIdentificationPoints();
    }else if(strcmp((char *)USB_USART_RX_BUF, "STOP") == 0){
        (void)FOC_CommandStop();
    }else if(strcmp((char *)USB_USART_RX_BUF, "CURRENT SINE OFF") == 0){
        (void)FOC_CommandStopCurrentSine();
    }else if(strcmp((char *)USB_USART_RX_BUF, "SPEED SINE OFF") == 0){
        (void)FOC_CommandStopSpeedSine();
    }else if(strncmp((char *)USB_USART_RX_BUF, "SPEED SINE ", 11U) == 0){
        if(Debug_ParseTwoFloats((char *)&USB_USART_RX_BUF[11], &kp, &ki) != 0U){
            (void)FOC_CommandSetSpeedSine(kp, ki);
        }
    }else if(strncmp((char *)USB_USART_RX_BUF, "SPEED LIMIT ", 12U) == 0){
        if(Debug_ParseFloat((char *)&USB_USART_RX_BUF[12], &value) != 0U){
            (void)FOC_CommandSetSpeedOutputLimit(value);
        }
    }else if(strcmp((char *)USB_USART_RX_BUF, "SPEED EST BASE") == 0){
        (void)FOC_CommandSetSpeedEstimator(0U);
    }else if(strcmp((char *)USB_USART_RX_BUF, "SPEED EST WIN") == 0){
        (void)FOC_CommandSetSpeedEstimator(1U);
    }else if(strcmp((char *)USB_USART_RX_BUF, "SPEED EST PLL") == 0){
        (void)FOC_CommandSetSpeedEstimator(2U);
    }else if(strncmp((char *)USB_USART_RX_BUF, "CURRENT SINE D ", 15U) == 0){
        if(Debug_ParseTwoFloats((char *)&USB_USART_RX_BUF[15], &kp, &ki) != 0U){
            (void)FOC_CommandSetCurrentSine(0U, kp, ki);
        }
    }else if(strncmp((char *)USB_USART_RX_BUF, "CURRENT SINE Q ", 15U) == 0){
        if(Debug_ParseTwoFloats((char *)&USB_USART_RX_BUF[15], &kp, &ki) != 0U){
            (void)FOC_CommandSetCurrentSine(1U, kp, ki);
        }
    }else if(strncmp((char *)USB_USART_RX_BUF, "CURRENT SINE ", 13U) == 0){
        if(Debug_ParseTwoFloats((char *)&USB_USART_RX_BUF[13], &kp, &ki) != 0U){
            (void)FOC_CommandSetCurrentSine(0U, kp, ki);
        }
    }else if(strncmp((char *)USB_USART_RX_BUF, "CURRENT ", 8U) == 0){
        if(Debug_ParseTwoFloats((char *)&USB_USART_RX_BUF[8], &kp, &ki) != 0U){
            (void)FOC_CommandSetCurrent(kp, ki);
        }
    }else if(strncmp((char *)USB_USART_RX_BUF, "POSITION ", 9U) == 0){
        if(Debug_ParseFloat((char *)&USB_USART_RX_BUF[9], &value) != 0U){
            (void)FOC_CommandSetPosition(value);
        }
    }else if(strncmp((char *)USB_USART_RX_BUF, "SPEED ID ", 9U) == 0){
        if(Debug_ParseFloat((char *)&USB_USART_RX_BUF[9], &value) != 0U){
            (void)FOC_CommandSetSpeedId(value);
        }
    }else if(strncmp((char *)USB_USART_RX_BUF, "SPEED ", 6U) == 0){
        if(Debug_ParseFloat((char *)&USB_USART_RX_BUF[6], &value) != 0U){
            (void)FOC_CommandSetSpeed(value);
        }
    }else if(strncmp((char *)USB_USART_RX_BUF, "PI SPEED ", 9U) == 0){
        if(Debug_ParseTwoFloats((char *)&USB_USART_RX_BUF[9], &kp, &ki) != 0U){
            (void)FOC_CommandSetSpeedPI(kp, ki);
        }
    }else if(strncmp((char *)USB_USART_RX_BUF, "PI POSITION ", 12U) == 0){
        if(Debug_ParseThreeFloats((char *)&USB_USART_RX_BUF[12], &kp, &ki, &kd) != 0U){
            (void)FOC_CommandSetPositionPI(kp, ki, kd);
        }
    }else if(strncmp((char *)USB_USART_RX_BUF, "PI IQ ", 6U) == 0){
        if(Debug_ParseTwoFloats((char *)&USB_USART_RX_BUF[6], &kp, &ki) != 0U){
            (void)FOC_CommandSetIqPI(kp, ki);
        }
    }else if(strncmp((char *)USB_USART_RX_BUF, "PI ID ", 6U) == 0){
        if(Debug_ParseTwoFloats((char *)&USB_USART_RX_BUF[6], &kp, &ki) != 0U){
            (void)FOC_CommandSetIdPI(kp, ki);
        }
    }else if(strcmp((char *)USB_USART_RX_BUF, "PI CLEAR") == 0){
        (void)FOC_CommandClearPI();
    }else if(strcmp((char *)USB_USART_RX_BUF, "TRACK STEP") == 0){
        (void)FOC_CommandSetSpeedStep();
    }else if(strncmp((char *)USB_USART_RX_BUF, "TRACK RAMP ", 11U) == 0){
        if(Debug_ParseFloat((char *)&USB_USART_RX_BUF[11], &value) != 0U){
            (void)FOC_CommandSetSpeedRamp(value);
        }
    }else if(strncmp((char *)USB_USART_RX_BUF, "POS SPEED LIMIT ", 16U) == 0){
        if(Debug_ParseFloat((char *)&USB_USART_RX_BUF[16], &value) != 0U){
            (void)FOC_CommandSetPositionSpeedLimit(value);
        }
    }else if(strcmp((char *)USB_USART_RX_BUF, "POS TRACK STEP") == 0){
        (void)FOC_CommandSetPositionStep();
    }else if(strncmp((char *)USB_USART_RX_BUF, "POS TRACK RAMP ", 15U) == 0){
        if(Debug_ParseFloat((char *)&USB_USART_RX_BUF[15], &value) != 0U){
            (void)FOC_CommandSetPositionRamp(value);
        }
    }else if(strncmp((char *)USB_USART_RX_BUF, "POS TRACK TRAP ", 15U) == 0){
        if(Debug_ParseTwoFloats((char *)&USB_USART_RX_BUF[15], &kp, &ki) != 0U){
            (void)FOC_CommandSetPositionTrapezoid(kp, ki);
        }
    }else if(strcmp((char *)USB_USART_RX_BUF, "POS FF ON") == 0){
        (void)FOC_CommandSetPositionFeedforward(1U);
    }else if(strcmp((char *)USB_USART_RX_BUF, "POS FF OFF") == 0){
        (void)FOC_CommandSetPositionFeedforward(0U);
    }else if(strcmp((char *)USB_USART_RX_BUF, "FF MODE OFF") == 0){
        (void)FOC_CommandSetFeedforwardMode(0U);
    }else if(strcmp((char *)USB_USART_RX_BUF, "FF MODE BOTH") == 0){
        (void)FOC_CommandSetFeedforwardMode(1U);
    }else if(strcmp((char *)USB_USART_RX_BUF, "FF MODE CROSS") == 0){
        (void)FOC_CommandSetFeedforwardMode(2U);
    }else if(strcmp((char *)USB_USART_RX_BUF, "FF MODE BEMF") == 0){
        (void)FOC_CommandSetFeedforwardMode(3U);
    }else if(strcmp((char *)USB_USART_RX_BUF, "FF ON") == 0){
        (void)FOC_CommandSetFeedforward(1U);
    }else if(strcmp((char *)USB_USART_RX_BUF, "FF OFF") == 0){
        (void)FOC_CommandSetFeedforward(0U);
    }else if(strcmp((char *)USB_USART_RX_BUF, "FF MODEL LEGACY") == 0){
        (void)FOC_CommandSetFeedforwardModel(0U);
    }else if(strcmp((char *)USB_USART_RX_BUF, "FF MODEL IDENTIFIED") == 0){
        (void)FOC_CommandSetFeedforwardModel(1U);
    }else if(strcmp((char *)USB_USART_RX_BUF, "FW MODE OFF") == 0){
        (void)FOC_CommandSetFieldWeakeningMode(0U);
    }else if(strcmp((char *)USB_USART_RX_BUF, "FW MODE FEEDBACK") == 0){
        (void)FOC_CommandSetFieldWeakeningMode(1U);
    }else if(strcmp((char *)USB_USART_RX_BUF, "FW MODE HYBRID") == 0){
        (void)FOC_CommandSetFieldWeakeningMode(2U);
    }else if(strncmp((char *)USB_USART_RX_BUF, "VOLT LIMIT ", 11U) == 0){
        if(Debug_ParseFloat((char *)&USB_USART_RX_BUF[11], &value) != 0U){
            (void)FOC_CommandSetVoltageLimit(value);
        }
    }

    USB_USART_RX_STA = 0U;
}

static uint8_t Debug_CopySnapshot(FOC_DEBUG_t *snapshot)
{
    uint32_t attempt;

    for (attempt = 0; attempt < DEBUG_SNAPSHOT_RETRIES; ++attempt)
    {
        uint32_t seq_before = foc_debug_seq;
        uint32_t seq_after;

        if ((seq_before & 1U) != 0U)
        {
            continue;
        }

        __DMB();
        memcpy(snapshot, &foc_debug, sizeof(*snapshot));
        __DMB();
        seq_after = foc_debug_seq;

        if ((seq_before == seq_after) && ((seq_after & 1U) == 0U))
        {
            return 1U;
        }
    }

    ++debug_snapshot_failures;
    return 0U;
}

void Debug_Task(void)
{
    static uint32_t last_send_tick = 0;
    FOC_DEBUG_t snapshot;
    uint32_t now;

    VCP_Service();
    FOC_ServiceFaultReport();
    Debug_CommandTask();

    if(Debug_ScopeDumpTask() != 0U){
        return;
    }

    now = HAL_GetTick();
    if ((uint32_t)(now - last_send_tick) < DEBUG_TELEMETRY_PERIOD_MS)
    {
        return;
    }
    last_send_tick = now;

    if (Debug_CopySnapshot(&snapshot) == 0U)
    {
        return;
    }
    VOFA_Handle.fdata[0] = snapshot.Uabc.a;
    VOFA_Handle.fdata[1] = snapshot.Uabc.b;
    VOFA_Handle.fdata[2] = snapshot.Uabc.c;

    VOFA_Handle.fdata[3] = snapshot.Iabc.a + 12.0f;
    VOFA_Handle.fdata[4] = snapshot.Iabc.b + 12.0f;
    VOFA_Handle.fdata[5] = snapshot.Iabc.c + 12.0f;

    VOFA_Handle.fdata[6] = snapshot.Iqd.q;
    VOFA_Handle.fdata[7] = snapshot.Iqd.d;

    VOFA_Handle.fdata[8] = snapshot.vf_speed.elec_angle;
    VOFA_Handle.fdata[9] = snapshot.hfi_speed.elec_angle;
    VOFA_Handle.fdata[10] = snapshot.smo_speed.elec_angle;
    VOFA_Handle.fdata[11] = snapshot.flux_speed.elec_angle;
    VOFA_Handle.fdata[12] = snapshot.abzenc_speed.elec_angle;
    VOFA_Handle.fdata[13] = snapshot.absenc_speed.elec_angle;

    VOFA_Handle.fdata[14] = snapshot.vf_speed.elec_speed;
    VOFA_Handle.fdata[15] = snapshot.hfi_speed.elec_speed;
    VOFA_Handle.fdata[16] = snapshot.smo_speed.elec_speed;
    VOFA_Handle.fdata[17] = snapshot.flux_speed.elec_speed;
    VOFA_Handle.fdata[18] = snapshot.abzenc_speed.elec_speed;
    VOFA_Handle.fdata[19] = snapshot.absenc_speed.elec_speed;

    /* 控制环调试扩展，原有0~19通道顺序保持不变。 */
    VOFA_Handle.fdata[20] = snapshot.Uqd.q;
    VOFA_Handle.fdata[21] = snapshot.Uqd.d;
    VOFA_Handle.fdata[22] = snapshot.Iqd_ref.q;
    VOFA_Handle.fdata[23] = snapshot.Iqd_ref.d;
    VOFA_Handle.fdata[24] = snapshot.target_elec_speed;
    VOFA_Handle.fdata[25] = snapshot.elec_speed;
    VOFA_Handle.fdata[26] = snapshot.voltage_scale;
    VOFA_Handle.fdata[27] = (float)snapshot.motor_mode;
    VOFA_Handle.fdata[28] = snapshot.speed_ref;
    VOFA_Handle.fdata[29] = snapshot.vbus;
    VOFA_Handle.fdata[30] = snapshot.target_position;
    VOFA_Handle.fdata[31] = snapshot.position_ref;
    VOFA_Handle.fdata[32] = snapshot.position_feedback;
    VOFA_Handle.fdata[33] = snapshot.current_feedforward.q;
    VOFA_Handle.fdata[34] = snapshot.current_feedforward.d;
    VOFA_Handle.fdata[35] = (float)snapshot.current_feedforward_mode;

    (void)VCP_DataTx((const uint8_t *)&VOFA_Handle, sizeof(VOFA_Handle));
}
