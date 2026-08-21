
// #include "bsp.hpp"
#include "debug.h"
#include <stdlib.h>

#include "usb_device.h"

/*vofaÊý¾ÝÖ¡*/
VOFA_Send_Handle_t VOFA_Handle = {
    .tail = {0x00, 0x00, 0x80, 0x7f},
};





#include "foc_app.hpp"

void Debug_Task(void)
{
    VOFA_Handle.fdata[0] = 1;
    VOFA_Handle.fdata[1] = 2;
    VOFA_Handle.fdata[2] = 3;
    // VOFA_Handle.fdata[0] = FOC_MOTOR.pwm.Uabc.a;
    // VOFA_Handle.fdata[1] = FOC_MOTOR.pwm.Uabc.b;
    // VOFA_Handle.fdata[2] = FOC_MOTOR.pwm.Uabc.c;
    // VOFA_Handle.fdata[3] = FOC_MOTOR.pwm.Ualpha_beta.alpha+10;
    // VOFA_Handle.fdata[4] = FOC_MOTOR.pwm.Ualpha_beta.beta+10;
    // VOFA_Handle.fdata[5] = FOC_MOTOR.pwm.Uqd.q;
    // VOFA_Handle.fdata[6] = FOC_MOTOR.pwm.Uqd.d;
    // VOFA_Handle.fdata[7] = FOC_MOTOR.pwm.ElAngle;

    // VOFA_Handle.fdata[8] = FOC_MOTOR.abz_enc.speed.ElAngle;
    // VOFA_Handle.fdata[9] = FOC_MOTOR.abz_enc.speed.AvrMecSpeed;
    // VOFA_Handle.fdata[10] = enc_count;

    
    VCP_DataTx((uint8_t*)&VOFA_Handle, sizeof(VOFA_Handle));
}


