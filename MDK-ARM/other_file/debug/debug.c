#include "debug.h"
#include <stdlib.h>

#include "usb_device.h"

/*vofa数据帧*/
VOFA_Send_Handle_t VOFA_Handle = {
    .tail = {0x00, 0x00, 0x80, 0x7f},
};

#define     _2PI                6.283185307179586f      /*2x圆周率*/

#include "foc_type.h"

extern FOC_DEBUG_t foc_debug;

void Debug_Task(void)
{
    VOFA_Handle.fdata[0] = foc_debug.Uabc.a;
    VOFA_Handle.fdata[1] = foc_debug.Uabc.b;
    VOFA_Handle.fdata[2] = foc_debug.Uabc.c;

    VOFA_Handle.fdata[3] = foc_debug.Iabc.a+12;
    VOFA_Handle.fdata[4] = foc_debug.Iabc.b+12;
    VOFA_Handle.fdata[5] = foc_debug.Iabc.c+12;

    VOFA_Handle.fdata[6] = foc_debug.Iqd.q;
    VOFA_Handle.fdata[7] = foc_debug.Iqd.d;

    VOFA_Handle.fdata[8] = foc_debug.vf_speed.elec_angle;
    VOFA_Handle.fdata[9] = foc_debug.hfi_speed.elec_angle;
    VOFA_Handle.fdata[10] = foc_debug.smo_speed.elec_angle;
    VOFA_Handle.fdata[11] = foc_debug.flux_speed.elec_angle;
    VOFA_Handle.fdata[12] = foc_debug.abzenc_speed.elec_angle;
    VOFA_Handle.fdata[13] = foc_debug.absenc_speed.elec_angle;

    // VOFA_Handle.fdata[14] = foc_debug.vf_speed.mech_speed/_2PI *60.0f;
    // VOFA_Handle.fdata[15] = foc_debug.hfi_speed.mech_speed/_2PI*60.0f;;
    // VOFA_Handle.fdata[16] = foc_debug.smo_speed.mech_speed/_2PI*60.0f;
    // VOFA_Handle.fdata[17] = foc_debug.flux_speed.mech_speed/_2PI*60.0f;
    // VOFA_Handle.fdata[18] = foc_debug.abzenc_speed.mech_speed/_2PI*60.0f;
    // VOFA_Handle.fdata[19] = foc_debug.absenc_speed.mech_speed/_2PI*60.0f;
    VOFA_Handle.fdata[14] = foc_debug.vf_speed.elec_speed;
    VOFA_Handle.fdata[15] = foc_debug.hfi_speed.elec_speed;;
    VOFA_Handle.fdata[16] = foc_debug.smo_speed.elec_speed;
    VOFA_Handle.fdata[17] = foc_debug.flux_speed.elec_speed;
    VOFA_Handle.fdata[18] = foc_debug.abzenc_speed.elec_speed;
    VOFA_Handle.fdata[19] = foc_debug.absenc_speed.elec_speed;
    // VOFA_Handle.fdata[19] = foc_debug.flux_speed.mech_speed/_2PI*60.0f;


    VCP_DataTx((uint8_t*)&VOFA_Handle, sizeof(VOFA_Handle));
}



















