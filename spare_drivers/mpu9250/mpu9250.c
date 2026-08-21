#include "mpu9250.h"
#include <stdint.h>
#include <i2c.h>
#include <stdio.h>
#include <delay.h>
#include <math.h>
#include <stdlib.h>

int8_t g[6];

#define DEG_to_RAD		0.017453293f	/* 度转弧度 π/180 */
#define RAD_to_DEG		57.29578f		/* 弧度转度 180/π */


MPU9250_t mpu9250 = {0};

static uint8_t MPU_Write_Byte(uint8_t addr,uint8_t reg,uint8_t data)
{
    // HAL_I2C_Master_Transmit(&hi2c2, addr, &reg, 1, HAL_MAX_DELAY); //写入新的数据
    HAL_I2C_Mem_Write(&hi2c2, addr, reg, 1, &data, 1, HAL_MAX_DELAY);

    return 0;
}

static uint8_t MPU_Read_Byte(uint8_t addr,uint8_t reg)
{
    // HAL_I2C_Master_Receive(&hi2c2, addr, reg);
    uint8_t data = 0;

    HAL_I2C_Mem_Read(&hi2c2, addr|0x01, reg, 1, &data, 1, 1000);
    // HAL_Delay(1);
    // HAL_I2C_Mem_Write(&hi2c2, addr, reg, 1, &data, 1, HAL_MAX_DELAY);
    return data;
}

uint8_t MPU_Write_Len(uint8_t addr,uint8_t reg,uint8_t len,uint8_t *buf)
{
    HAL_I2C_Mem_Write(&hi2c2, addr, reg, 1, buf, len, HAL_MAX_DELAY);
    return 0;
} 


static uint8_t MPU_Read_Len(uint8_t addr,uint8_t reg,uint8_t len,uint8_t *buf)
{
    HAL_I2C_Mem_Read(&hi2c2, addr|0x01, reg, 1, buf, len, 1000);
    // HAL_Delay(1);
    return 0;       
}



//设置MPU9250陀螺仪传感器满量程范围
//fsr:0,±250dps;1,±500dps;2,±1000dps;3,±2000dps
//返回值:0,设置成功
//    其他,设置失败 
static uint8_t MPU_Set_Gyro_Fsr(uint8_t fsr)
{
	return MPU_Write_Byte(MPU9250_ADDR, MPU_GYRO_CFG_REG, (fsr<<3)|3);//设置陀螺仪满量程范围  
}

//设置MPU9250加速度传感器满量程范围
//fsr:0,±2g;1,±4g;2,±8g;3,±16g
//返回值:0,设置成功
//    其他,设置失败 
static uint8_t MPU_Set_Accel_Fsr(uint8_t fsr)
{
	return MPU_Write_Byte(MPU9250_ADDR,MPU_ACCEL_CFG_REG,fsr<<3);//设置加速度传感器满量程范围  
}

//设置MPU9250的数字低通滤波器
//lpf:数字低通滤波频率(Hz)
//返回值:0,设置成功
//    其他,设置失败 
static uint8_t MPU_Set_LPF(uint16_t lpf)
{
	uint8_t data=0;
	if(lpf>=188)data=1;
	else if(lpf>=98)data=2;
	else if(lpf>=42)data=3;
	else if(lpf>=20)data=4;
	else if(lpf>=10)data=5;
	else data=6; 
	return MPU_Write_Byte(MPU9250_ADDR,MPU_CFG_REG,data);//设置数字低通滤波器  
}

//设置MPU9250的采样率(假定Fs=1KHz)
//rate:4~1000(Hz)
//返回值:0,设置成功
//    其他,设置失败 
static uint8_t MPU_Set_Rate(uint16_t rate)
{
	uint8_t data;
	if(rate>1000)rate=1000;
	if(rate<4)rate=4;
	data=1000/rate-1;
	data=MPU_Write_Byte(MPU9250_ADDR,MPU_SAMPLE_RATE_REG,data);	//设置数字低通滤波器
 	return MPU_Set_LPF(rate/2);	//自动设置LPF为采样率的一半
}


/*获取陀螺仪角速度值*/
uint8_t MPU_Get_Gyroscope(int16_t *gx, int16_t *gy, int16_t *gz)
{
    uint8_t buf[6],res; 
	res=MPU_Read_Len(MPU9250_ADDR, MPU_GYRO_XOUTH_REG, 6, buf);
	if(res==0)
	{
		*gx=((u16)buf[0]<<8)|buf[1];  
		*gy=((u16)buf[2]<<8)|buf[3];  
		*gz=((u16)buf[4]<<8)|buf[5];
	} 	

    if(*gx>32768)*gx-=65536;
    if(*gy>32768)*gy-=65536;
    if(*gz>32768)*gz-=65536;

    return res;;
}

//得到加速度值(原始值)
uint8_t MPU_Get_Accelerometer(int16_t *ax, int16_t *ay, int16_t *az)
{
    uint8_t buf[6],res;  
	res=MPU_Read_Len(MPU9250_ADDR, MPU_ACCEL_XOUTH_REG, 6, buf);
	if(res==0)
	{
		*ax=((u16)buf[0]<<8)|buf[1];  
		*ay=((u16)buf[2]<<8)|buf[3];  
		*az=((u16)buf[4]<<8)|buf[5];
	} 	
    if(*ax>32768)*ax-=65536;
    if(*ay>32768)*ay-=65536;
    if(*az>32768)*az-=65536;

    return res;;
}

/*计算初始定位时的偏差*/
static void MPU9250_Bias_Calculate(void)
{
    uint16_t i = 0;
    int16_t temp[6] = {0};
    float Sum[6] = {0};

    for(i=0;i<1000;i++){
        MPU_Get_Gyroscope(&temp[0], &temp[1], &temp[2]);
        MPU_Get_Accelerometer(&temp[3], &temp[4], &temp[5]);
        Sum[0]+=temp[0];
        Sum[1]+=temp[1];
        Sum[2]+=temp[2];
        Sum[3]+=temp[3];
        Sum[4]+=temp[4];
        Sum[5]+=temp[5];
    }
    // mpu9250.Gyro_bias.x = Sum[0]/1000.0f;
    // mpu9250.Gyro_bias.y = Sum[1]/1000.0f;
    // mpu9250.Gyro_bias.z = Sum[2]/1000.0f;

    // mpu9250.Acc_bias.x = Sum[3]/1000.0f;
    // mpu9250.Acc_bias.y = Sum[4]/1000.0f;
    // mpu9250.Acc_bias.z = Sum[5]/1000.0f -16383.5f;
}

void MPU9250_Init(void)
{
    delay_ms(1000);
    uint8_t res;
    if(HAL_OK==HAL_I2C_IsDeviceReady(&hi2c2, MPU9250_ADDR, 10, HAL_MAX_DELAY)){
        printf("MPU9250 存在\r\n");
    }else{
        printf("MPU9250 不存在\r\n");
    }

    MPU_Write_Byte(MPU9250_ADDR,MPU_PWR_MGMT1_REG,0X80);//复位MPU9250
    delay_ms(100);  //延时100ms
    MPU_Write_Byte(MPU9250_ADDR,MPU_PWR_MGMT1_REG,0X00);//唤醒MPU9250
    MPU_Set_Gyro_Fsr(3);					        	//陀螺仪传感器,±2000dps
	MPU_Set_Accel_Fsr(0);					       	 	//加速度传感器,±2g
    MPU_Set_Rate(500);						       	 	//设置采样率50Hz
    MPU_Write_Byte(MPU9250_ADDR,MPU_INT_EN_REG,0X00);   //关闭所有中断
	MPU_Write_Byte(MPU9250_ADDR,MPU_USER_CTRL_REG,0X00);//I2C主模式关闭
	MPU_Write_Byte(MPU9250_ADDR,MPU_FIFO_EN_REG,0X00);	//关闭FIFO
	MPU_Write_Byte(MPU9250_ADDR,MPU_INTBP_CFG_REG,0X82);//INT引脚低电平有效，开启bypass模式，可以直接读取磁力计

    res=MPU_Read_Byte(MPU9250_ADDR,MPU_DEVICE_ID_REG);  //读取MPU6500的ID
    printf("MPU9250 ID: %x\r\n", res);

    MPU_Write_Byte(MPU9250_ADDR,MPU_PWR_MGMT1_REG,0X01);  	//设置CLKSEL,PLL X轴为参考
    MPU_Write_Byte(MPU9250_ADDR,MPU_PWR_MGMT2_REG,0X00);  	//加速度与陀螺仪都工作
    MPU_Set_Rate(500);						       			//设置采样率为50Hz   

    delay_ms(500);

    // MPU9250_Bias_Calculate();
    // HAL_I2C_Mem_Read_IT(&hi2c2, MPU9250_ADDR, MPU_GYRO_XOUTH_REG, 1, (uint8_t *)g, 6);
    // HAL_I2C_Mem_Read_DMA(&hi2c2, MPU9250_ADDR, MPU_GYRO_XOUTH_REG, 1, (uint8_t *)g, 6);
// HAL_I2C_Mem_Read(&hi2c2, addr, reg, 1, buf, len, 0xFF);
}


/*获取加速度值和角速度值*/
void MPU9250_Get_Acc_Gyro(void)
{
    int16_t temp[6] ={0};
    /*获取IMU信息*/
    MPU_Get_Gyroscope(&temp[0], &temp[1], &temp[2]);
    MPU_Get_Accelerometer(&temp[3], &temp[4], &temp[5]);

    // Gyro.x = (temp[0]-mpu9250.Gyro_bias.x)/32767.0f *2.0f;
    // Gyro.y = (temp[1]-mpu9250.Gyro_bias.y)/32767.0f *2.0f;
    // Gyro.z = (temp[2]-mpu9250.Gyro_bias.z)/32767.0f *2.0f;

    // Acc.x = (temp[3]-mpu9250.Acc_bias.x)/32767.0f *2.0f;
    // Acc.y = (temp[4]-mpu9250.Acc_bias.y)/32767.0f *2.0f;
    // Acc.z = (temp[5]-mpu9250.Acc_bias.z)/32767.0f *2.0f;
}


#include <bsp.h>
#if 0
/*欧拉角方式 互补滤波计算姿态*/
void IMU_Calculate_ComFilter_By_Euler(Axis3f_t acc, Axis3f_t gyro, float dt)
{
    float yaw;          /*航向角*/
    float roll;         /*横滚角*/
    float pitch;        /*俯仰角*/

    float K = 0.02f;
    // float K1 = (1- fabsf(gyro.y/2000.0f));
    // float K2 = (1- fabsf(gyro.x/2000.0f));
    Axis3f_t acc_angle = {0};

    arm_sqrt_f32();

    pitch = atan2f(acc.x, sqrt(acc.y * acc.y + acc.z * acc.z))*RAD_to_DEG;
    roll = atan2f(acc.y, sqrt(acc.x * acc.x + acc.z * acc.z))*RAD_to_DEG;

    mpu9250.pitch = K *pitch +  (mpu9250.pitch-gyro.y*dt)*(1.0f-K);
    mpu9250.roll  = K *roll +  (mpu9250.roll+gyro.x*dt)*(1.0f-K);
}

void Imu_Get(void)
{
    int16_t temp[3] ={0};
    /*获取IMU信息*/
    MPU_Get_Gyroscope(&temp[0], &temp[1], &temp[2]);

    Gyro.x = (temp[0]-mpu9250.Gyro_bias.x)/32768.0f * 2000.0f;
    Gyro.y = (temp[1]-mpu9250.Gyro_bias.y)/32768.0f * 2000.0f;
    Gyro.z = (temp[2]-mpu9250.Gyro_bias.z)/32768.0f * 2000.0f;

    MPU_Get_Accelerometer(&temp[0], &temp[1], &temp[2]);

    Acc.x = (temp[0]-mpu9250.Acc_bias.x)/32767.0f *2.0f;
    Acc.y = (temp[1]-mpu9250.Acc_bias.y)/32767.0f *2.0f;
    Acc.z = (temp[2]-mpu9250.Acc_bias.z)/32767.0f *2.0f;

    /*计算欧拉角*/
    // mpu9250.pitch = atan2f(Acc.x, Acc.z)*RAD_to_DEG;

    // mpu9250.roll = atan2f(Acc.y, Acc.z)*RAD_to_DEG;
    IMU_Calculate_ComFilter_By_Euler(Acc, Gyro, 0.001);
}

/*IMU姿态解算*/
void Imu_Update(void)
{

    // Imu_Get();

    VOFA_Handle.fdata[0] = Acc.x;
    VOFA_Handle.fdata[1] = Acc.y;
    VOFA_Handle.fdata[2] = Acc.z;

    VOFA_Handle.fdata[3] = mpu9250.pitch;
    VOFA_Handle.fdata[4] = mpu9250.roll;


    // VOFA_Handle.fdata[0] = g[0]<<8|g[1];
    // VOFA_Handle.fdata[1] = g[2]<<8|g[3];
    // VOFA_Handle.fdata[2] = g[4]<<8|g[5];
   
    VCP_DataTx((uint8_t*)&VOFA_Handle, sizeof(VOFA_Handle));

    /*角速度单位转为弧度*/
    // gyro.x = gyro.x * DEG_to_RAD;	/* 度转弧度 */
	// gyro.y = gyro.y * DEG_to_RAD;
	// gyro.z = gyro.z * DEG_to_RAD;

    // /* 加速度计输出有效时,利用加速度计补偿陀螺仪*/
	// if((acc.x != 0.0f) || (acc.y != 0.0f) || (acc.z != 0.0f)){
    // }
}

#endif

void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    // printf("123\r\n");
    // HAL_I2C_Mem_Read_DMA(&hi2c2, MPU9250_ADDR, MPU_ACCEL_XOUTH_REG, 1, (uint8_t *)g, 6);
    // delay_ms(1);

    // VCP_DataTx((uint8_t*)&VOFA_Handle, sizeof(VOFA_Handle));
    // HAL_I2C_Mem_Read_DMA(&hi2c2, MPU9250_ADDR, MPU_ACCEL_XOUTH_REG, 1, (uint8_t *)g, 6);
}

// static uint32_t tim6cout;

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{   

    /*定时器6用来计算速度*/
    if(htim->Instance == TIM6){     /*2000Hz的频率 0.5ms计算一次*/
        // Imu_Get();
    }
}

