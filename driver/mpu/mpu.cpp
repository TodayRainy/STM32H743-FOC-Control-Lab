#include "mpu.hpp"




static void MPU_Config(uint8_t Number, uint32_t BaseAddress, uint8_t Size, uint8_t TypeExtField, uint8_t AccessPermission, \
                   uint8_t DisableExec, uint8_t IsShareable, uint8_t IsCacheable, uint8_t IsBufferable)
{
    MPU_Region_InitTypeDef MPU_Initure = {};

    HAL_MPU_Disable();

    MPU_Initure.Enable                  = MPU_REGION_ENABLE;
    MPU_Initure.Number                  = Number;
    MPU_Initure.BaseAddress             = BaseAddress;
    MPU_Initure.Size                    = Size;
    MPU_Initure.SubRegionDisable        = 0;
    MPU_Initure.TypeExtField            = TypeExtField;
    MPU_Initure.AccessPermission        = AccessPermission;
    MPU_Initure.DisableExec             = DisableExec;
    MPU_Initure.IsShareable             = IsShareable;
    MPU_Initure.IsCacheable             = IsCacheable;
    MPU_Initure.IsBufferable            = IsBufferable;

    HAL_MPU_ConfigRegion(&MPU_Initure);                    

    /*启用特权模式默认内存映射。
    在NMI和HardFault中禁用特权默认映射。
    在HardFault和NMI中使能MPU。
    */
    HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT|MPU_HFNMI_PRIVDEF_NONE|MPU_HARDFAULT_NMI|MPU_HFNMI_PRIVDEF);			        //开启MPU
}


void MPU_Memory_Protection(void)
{
    /*DTCM 128k*/
    MPU_Config(MPU_REGION_NUMBER0,
                        0x20000000,
                        MPU_REGION_SIZE_128KB,
                        MPU_TEX_LEVEL0,
                        MPU_REGION_FULL_ACCESS,
                        MPU_INSTRUCTION_ACCESS_DISABLE,
                        MPU_ACCESS_SHAREABLE,
                        MPU_ACCESS_CACHEABLE,
                        MPU_ACCESS_BUFFERABLE
                        );

    /*ITCM 64k*/
    MPU_Config(MPU_REGION_NUMBER1,
                        0x00000000,
                        MPU_REGION_SIZE_64KB,
                        MPU_TEX_LEVEL0,
                        MPU_REGION_FULL_ACCESS,
                        MPU_INSTRUCTION_ACCESS_ENABLE,
                        MPU_ACCESS_SHAREABLE,
                        MPU_ACCESS_CACHEABLE,
                        MPU_ACCESS_BUFFERABLE
                        );
    


    //保护整个D1 SRAM 512KB
    MPU_Config(MPU_REGION_NUMBER2,
                        0x24000000,
                        MPU_REGION_SIZE_512KB,
                        MPU_TEX_LEVEL0,
                        MPU_REGION_FULL_ACCESS,
                        MPU_INSTRUCTION_ACCESS_DISABLE,
                        MPU_ACCESS_SHAREABLE,
                        MPU_ACCESS_CACHEABLE,
                        MPU_ACCESS_BUFFERABLE
                        );
    
    //保护SDRAM区域,共32M字节  
    MPU_Config(MPU_REGION_NUMBER3,
                        0XC0000000,
                        MPU_REGION_SIZE_32MB,
                        MPU_TEX_LEVEL0,
                        MPU_REGION_FULL_ACCESS,
                        MPU_INSTRUCTION_ACCESS_ENABLE,
                        MPU_ACCESS_SHAREABLE,
                        MPU_ACCESS_CACHEABLE,
                        MPU_ACCESS_BUFFERABLE
                        );
    
    //以太网用的sram  
    MPU_Config(MPU_REGION_NUMBER4,
                        0x30040000,
                        MPU_REGION_SIZE_256B,
                        MPU_TEX_LEVEL0,
                        MPU_REGION_FULL_ACCESS,
                        MPU_INSTRUCTION_ACCESS_DISABLE,
                        MPU_ACCESS_SHAREABLE,
                        MPU_ACCESS_NOT_CACHEABLE,
                        MPU_ACCESS_BUFFERABLE
                        );

    /* D2 SRAM1 + SRAM2: normal non-cacheable memory, directly visible to J-Link. */
    MPU_Config(MPU_REGION_NUMBER5,
                        0x30000000,
                        MPU_REGION_SIZE_256KB,
                        MPU_TEX_LEVEL1,
                        MPU_REGION_FULL_ACCESS,
                        MPU_INSTRUCTION_ACCESS_DISABLE,
                        MPU_ACCESS_SHAREABLE,
                        MPU_ACCESS_NOT_CACHEABLE,
                        MPU_ACCESS_NOT_BUFFERABLE
                        );

}






