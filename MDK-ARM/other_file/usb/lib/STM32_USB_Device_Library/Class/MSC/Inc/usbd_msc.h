/**
  ******************************************************************************
  * @file    usbd_msc.h
  * @author  MCD Application Team
  * @version V2.4.2
  * @date    11-December-2015
  * @brief   Header for the usbd_msc.c file
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2015 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */ 

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __USBD_MSC_H
#define __USBD_MSC_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include  "usbd_msc_bot.h"
#include  "usbd_msc_scsi.h"
#include  "usbd_ioreq.h"

/** @addtogroup USBD_MSC_BOT
  * @{
  */
  
/** @defgroup USBD_MSC
  * @brief This file is the Header file for usbd_msc.c
  * @{
  */ 


/** @defgroup USBD_BOT_Exported_Defines
  * @{
  */ 
#define MSC_MAX_FS_PACKET            0x40
#define MSC_MAX_HS_PACKET            0x200

#define BOT_GET_MAX_LUN              0xFE
#define BOT_RESET                    0xFF
#define USB_MSC_CONFIG_DESC_SIZ      32
 

#define MSC_EPIN_ADDR                0x83
#define MSC_EPOUT_ADDR               0x03

/**
  * @}
  */ 

/** @defgroup USB_CORE_Exported_Types
  * @{
  */ 
typedef struct _USBD_STORAGE
{
  int8_t (* Init) (uint8_t lun);
  int8_t (* GetCapacity) (uint8_t lun, uint32_t *block_num, uint16_t *block_size);
  int8_t (* IsReady) (uint8_t lun);
  int8_t (* IsWriteProtected) (uint8_t lun);
  int8_t (* Read) (uint8_t lun, uint8_t *buf, uint32_t blk_addr, uint16_t blk_len);
  int8_t (* Write)(uint8_t lun, uint8_t *buf, uint32_t blk_addr, uint16_t blk_len);
  int8_t (* GetMaxLun)(void);       /*获取磁盘数量*/
  int8_t *pInquiry;
  
}USBD_StorageTypeDef;


typedef struct
{
  uint32_t                 max_lun;                         /*最大逻辑单元数量*/
  uint32_t                 interface;                       /*MSC的接口号*/
  uint8_t                  bot_state;                       /*表示BOT层的状�?*/
  uint8_t                  bot_status;                      /* BOT协�??状态码*/
  uint16_t                 bot_data_length;                 /* BOT数据长度，表示BOT数据缓冲区的长度*/
  uint8_t                  bot_data[MSC_MEDIA_PACKET];      /*BOT数据缓冲区，用于存储MSC BOT层的数据*/
  USBD_MSC_BOT_CBWTypeDef  cbw;                             /*结构体，用于存储CBW信息*/
  USBD_MSC_BOT_CSWTypeDef  csw;                             /*用于存储CSW信息*/
  
  USBD_SCSI_SenseTypeDef   scsi_sense [SENSE_LIST_DEEPTH];  
  uint8_t                  scsi_sense_head;
  uint8_t                  scsi_sense_tail;
  
  uint16_t                 scsi_blk_size;                   /*表示逻辑单元的块大小*/
  uint32_t                 scsi_blk_nbr[STORAGE_LUN_NBR];	//将scsi_blk_nbr改为数组形式,否则无法�?�?3�?磁盘!
  
  uint64_t 				   scsi_blk_addr;					//对大�?4GB的SD�?,要用64位表�?
  uint32_t                 scsi_blk_len;                    /*SCSI块长�?*/
}
USBD_MSC_BOT_HandleTypeDef; 

/* Structure for MSC process */
extern USBD_ClassTypeDef  USBD_MSC;
#define USBD_MSC_CLASS    &USBD_MSC

uint8_t  USBD_MSC_RegisterStorage  (USBD_HandleTypeDef   *pdev, 
                                    USBD_StorageTypeDef *fops);
/**
  * @}
  */ 

/**
  * @}
  */ 

#ifdef __cplusplus
}
#endif


uint8_t  USBD_MSC_Init (USBD_HandleTypeDef *pdev, 
                            uint8_t cfgidx);

uint8_t  USBD_MSC_DeInit (USBD_HandleTypeDef *pdev, 
                              uint8_t cfgidx);

uint8_t  USBD_MSC_Setup (USBD_HandleTypeDef *pdev, 
                             USBD_SetupReqTypedef *req);

uint8_t  USBD_MSC_DataIn (USBD_HandleTypeDef *pdev, 
                              uint8_t epnum);


uint8_t  USBD_MSC_DataOut (USBD_HandleTypeDef *pdev, 
                               uint8_t epnum);

uint8_t  *USBD_MSC_GetHSCfgDesc (uint16_t *length);

uint8_t  *USBD_MSC_GetFSCfgDesc (uint16_t *length);

uint8_t  *USBD_MSC_GetOtherSpeedCfgDesc (uint16_t *length);

uint8_t  *USBD_MSC_GetDeviceQualifierDescriptor (uint16_t *length);




#endif  /* __USBD_MSC_H */
/**
  * @}
  */ 
  
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
