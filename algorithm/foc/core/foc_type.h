
#ifndef _FOC_TYPE_H
#define _FOC_TYPE_H

#include <stdint.h>
#include <stdbool.h>


/*角度类型*/
typedef enum{
	ELEC_ANGLE = 0,             /*电角度*/
    MECH_ANGLE,                 /*机械角度*/
    OUT_ANGLE,                  /*输出角度*/ 
}ANGLE_TYPE_t;

/* 速度类型 */
typedef enum{
	ELEC_SPD_RAD_S = 0,         /*电速度 rad/s*/
    MECH_SPD_RAD_S,             /*机械速度 rad/s*/
    OUT_SPD_RAD_S,              /*输出速度 rad/s*/ 

    ELEC_SPD_R_S,               /*电速度 r/s*/
    MECH_SPD_R_S,               /*机械速度 r/s*/
    OUT_SPD_R_S,                /*输出速度 r/s*/ 
    ELEC_SPD_R_M,               /*电速度 r/m*/
    MECH_SPD_R_M,               /*机械速度 r/m*/
    OUT_SPD_R_M,                /*输出速度 r/m*/ 
}SPD_TYPE_t;


typedef struct
{
  float a;
  float b;
  float c;                      
}abc_t;

/* alphabeta 坐标系*/
typedef struct
{
  float alpha;
  float beta;
}alphabeta_t;


/*qd坐标系*/
typedef struct
{
  float q;
  float d;
}qd_t;


typedef struct{
    /*角度统一用rad，速度统一用rad/s*/
    float offset;               /*电角度的偏移*/
    float elec_angle;           /*电角度*/
    float mech_angle;           /*机械角度*/
    float out_angle;            /*输出轴角度*/
                            
    float elec_speed;           /*电角速度 */
    float mech_speed;           /*机械速度 */
    float out_speed;            /*减速后的机械速度 */



    int32_t elec_round;         /*电角速度圈数*/
    int32_t mech_round;         /*机械速度圈数*/
    int32_t out_round;          /*输出轴圈数*/
    int32_t pole_pairs;         /*极对数*/
    int32_t gear_ratio;         /*减速比*/
}speed_t;




/*调试信息*/
typedef struct{
    qd_t Uqd;
    qd_t Iqd;
    qd_t Iqd_ref;

    float target_elec_speed;
    float speed_ref;
    float elec_speed;
    float voltage_scale;
    uint32_t motor_mode;

    float vbus;
    float target_position;
    float position_ref;
    float position_feedback;
    qd_t current_feedforward;
    uint32_t current_feedforward_mode;

    alphabeta_t Ualphabeta;
    abc_t Uabc;
    abc_t Tabc;
    abc_t Iabc;
    abc_t Iadc_ad;

    speed_t vf_speed;
    speed_t abzenc_speed;
    speed_t absenc_speed;
    speed_t smo_speed;
    speed_t flux_speed;
    speed_t hfi_speed;
}FOC_DEBUG_t;
 



#endif
