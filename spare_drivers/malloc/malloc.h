#ifndef __MALLOC_H
#define __MALLOC_H
#include "sys.h" 

#ifdef __cplusplus
extern "C" {
#endif

//#include "FreeRTOSConfig.h"



//定义六个内存池
#define AXI_SRAM	 	0		//AXI内存池,AXI共512KB 
#define SDRAM   	    1		//外部内存池(SDRAM),SDRAM共32MB
#define AHB_SRAM1		2		//
#define AHB_SRAM2		3		//
#define AHB_SRAM3 	    4		//
#define AHB_SRAM4 	    5		//

#define SRAMBANK 	    6		//定义支持的SRAM块数.	



//mem1内存参数设定.mem1是H7内部的AXI内存.
#define MEM1_BLOCK_SIZE			64  	  						//内存块大小为64字节
#define MEM1_MAX_SIZE			(100 *1024)  			        //最大管理内存 448K,H7的AXI内存总共512KB
#define MEM1_ALLOC_TABLE_SIZE	MEM1_MAX_SIZE/MEM1_BLOCK_SIZE 	//内存表大小

//mem2内存参数设定.mem2是外部的SDRAM内存
#define MEM2_BLOCK_SIZE			64  	  						//内存块大小为64字节
#define MEM2_MAX_SIZE			(28912 *1024)                   //最大管理内存28912K,外扩SDRAM总共32MB,LTDC占了2MB,还剩30MB.
#define MEM2_ALLOC_TABLE_SIZE	MEM2_MAX_SIZE/MEM2_BLOCK_SIZE 	//内存表大小
		 
//mem3内存参数设定.mem3是H7内部的SRAM1
#define MEM3_BLOCK_SIZE			64  	  						//内存块大小为64字节
#define MEM3_MAX_SIZE			100 *1024  						//100k,H7的SRAM1
#define MEM3_ALLOC_TABLE_SIZE	MEM3_MAX_SIZE/MEM3_BLOCK_SIZE 	//内存表大小
		 
//mem4内存参数设定.mem4是H7内部的SRAM2内存
#define MEM4_BLOCK_SIZE			64  	  						//内存块大小为64字节
#define MEM4_MAX_SIZE			100 *1024  						//最大管理内存100K,H7的SRAM2
#define MEM4_ALLOC_TABLE_SIZE	MEM4_MAX_SIZE/MEM4_BLOCK_SIZE 	//内存表大小
		 
//mem4内存参数设定.mem4是H7内部的SRAM3内存
#define MEM5_BLOCK_SIZE			64  	  						//内存块大小为64字节
#define MEM5_MAX_SIZE			20 *1024  						//最大管理内存20K,H7的SRAM3
#define MEM5_ALLOC_TABLE_SIZE	MEM5_MAX_SIZE/MEM5_BLOCK_SIZE 	//内存表大小
		 
//mem6内存参数设定.mem4是H7内部的SRAM4内存
#define MEM6_BLOCK_SIZE			64  	  						//内存块大小为64字节
#define MEM6_MAX_SIZE			(50*1024)                       //最大管理内存50K,H7的SRAM4
#define MEM6_ALLOC_TABLE_SIZE	MEM6_MAX_SIZE/MEM6_BLOCK_SIZE 	//内存表大小
		 


//内存管理控制器
struct _m_mallco_dev
{
	void (*init)(u8);					//初始化
	u16 (*perused)(u8);		  	    	//内存使用率
	u8 	*membase[SRAMBANK];				//内存池 管理SRAMBANK个区域的内存
	u32 *memmap[SRAMBANK]; 				//内存管理状态表
	u8  memrdy[SRAMBANK]; 				//内存管理是否就绪
};
extern struct _m_mallco_dev mallco_dev;	 //在mallco.c里面定义

void mymemset(void *s,u8 c,u32 count);	//设置内存
void mymemcpy(void *des,void *src,u32 n);//复制内存     
void my_mem_init(u8 memx);				//内存管理初始化函数(外/内部调用)
u32 my_mem_malloc(u8 memx,u32 size);	//内存分配(内部调用)
u8 my_mem_free(u8 memx,u32 offset);		//内存释放(内部调用)
u16 my_mem_perused(u8 memx) ;			//获得内存使用率(外/内部调用) 
////////////////////////////////////////////////////////////////////////////////
//用户调用函数
void myfree(u8 memx,void *ptr);  			//内存释放(外部调用)
void *mymalloc(u8 memx,u32 size);			//内存分配(外部调用)
void *myrealloc(u8 memx,void *ptr,u32 size);//重新分配内存(外部调用)












#ifdef __cplusplus
}
#endif

















#endif













