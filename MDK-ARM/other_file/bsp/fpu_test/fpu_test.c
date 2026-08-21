#include "fpu_test.h"
#include <stdint.h>
#include <main.h>
#include <stdio.h>

#define 	ITERATION			128			//迭代次数
#define	 	REAL_CONSTANT		0.285f		//实部常量
#define	 	IMG_CONSTANT		0.01f		//虚部常量

//缩放因子列表
const uint16_t zoom_ratio[] =
{
	120, 110, 100, 150, 200, 275, 350, 450,
	600, 800, 1000, 1200, 1500, 2000, 1500,
	1200, 1000, 800, 600, 450, 350, 275, 200,
	150, 100, 110,
}; 



//产生Julia分形图形
//size_x,size_y:屏幕x,y方向的尺寸
//offset_x,offset_y:屏幕x,y方向的偏移
//zoom:缩放因子
void GenerateJulia_fpu(uint16_t size_x,uint16_t size_y,uint16_t offset_x,uint16_t offset_y,uint16_t zoom)
{
	uint8_t i;
	uint16_t x,y;  
	float tmp1,tmp2;
	float num_real,num_img;
	float radius; 
	for(y=0;y<size_y;y++)
	{
       	for(x=0;x<size_x;x++)
   		{
       		num_real=y-offset_y;
         	num_real=num_real/zoom;
           	num_img=x-offset_x;
          	num_img=num_img/zoom;
           	i=0;
          	radius=0;
           	while((i<ITERATION-1)&&(radius<4))
           	{
           		tmp1=num_real*num_real;
               	tmp2=num_img*num_img;
              	num_img=2*num_real*num_img+IMG_CONSTANT;
             	num_real=tmp1-tmp2+REAL_CONSTANT;
               	radius=tmp1+tmp2;
              	i++;
           	}

       	}
	}
}

void FPU_Test(void)
{
    uint32_t timeout = 0;
    while(1){
        timeout = HAL_GetTick();
        GenerateJulia_fpu(800,480,400,240,zoom_ratio[0]);
        timeout = HAL_GetTick()-timeout;
        printf("timeout=%d\r\n", timeout);
    }
}











