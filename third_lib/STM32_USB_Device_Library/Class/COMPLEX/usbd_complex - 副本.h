/**
  ******************************************************************************
  * @file    usbd_COMPLEX.h
  * @author  MCD Application Team
  * @version V2.4.2
  * @date    11-December-2015
  * @brief   header file for the usbd_COMPLEX.c file.
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
#ifndef __USB_COMPLEX_H
#define __USB_COMPLEX_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include  "usbd_ioreq.h"

/** @addtogroup STM32_USB_DEVICE_LIBRARY
  * @{
  */
  
/** @defgroup usbd_COMPLEX
  * @brief This file is the Header file for usbd_COMPLEX.c
  * @{
  */ 


/** @defgroup usbd_COMPLEX_Exported_Defines
  * @{
  */ 

#define USBD_IAD_DESC_SIZE                                  0x08



/* COMPLEX Endpoints parameters: you can fine tune these values depending on the needed baudrates and performance. */
#define COMPLEX_DATA_HS_MAX_PACKET_SIZE                 512  /* Endpoint IN & OUT Packet size */
#define COMPLEX_DATA_FS_MAX_PACKET_SIZE                 64  /* Endpoint IN & OUT Packet size */
#define COMPLEX_CMD_PACKET_SIZE                         8  /* Control Endpoint Packet size */ 

#define USB_COMPLEX_CONFIG_DESC_SIZ                     0x09+USBD_IAD_DESC_SIZE+0x09+0x05+0x05+0x04+0x05+0x07+0x09+0x07+0x07 + USBD_IAD_DESC_SIZE +0x09+0x05+0x05+0x04+0x05+0x07+0x09+0x07+0x07
#define COMPLEX_DATA_HS_IN_PACKET_SIZE                  COMPLEX_DATA_HS_MAX_PACKET_SIZE
#define COMPLEX_DATA_HS_OUT_PACKET_SIZE                 COMPLEX_DATA_HS_MAX_PACKET_SIZE

#define COMPLEX_DATA_FS_IN_PACKET_SIZE                  COMPLEX_DATA_FS_MAX_PACKET_SIZE
#define COMPLEX_DATA_FS_OUT_PACKET_SIZE                 COMPLEX_DATA_FS_MAX_PACKET_SIZE

/*---------------------------------------------------------------------*/
/*  COMPLEX definitions                                                    */
/*---------------------------------------------------------------------*/
#define COMPLEX_SEND_ENCAPSULATED_COMMAND               0x00
#define COMPLEX_GET_ENCAPSULATED_RESPONSE               0x01
#define COMPLEX_SET_COMM_FEATURE                        0x02
#define COMPLEX_GET_COMM_FEATURE                        0x03
#define COMPLEX_CLEAR_COMM_FEATURE                      0x04
#define COMPLEX_SET_LINE_CODING                         0x20
#define COMPLEX_GET_LINE_CODING                         0x21
#define COMPLEX_SET_CONTROL_LINE_STATE                  0x22
#define COMPLEX_SEND_BREAK                              0x23


#define USBD_MSC_FIRST_INTERFACE     0x00
#define USBD_CDC_FIRST_INTERFACE     0x01

/*CDC接口配置*/
#define USBD_CDC_INTERFACE_NUM       0x02 /*CDC接口数量*/
#define USBD_IAD_CDC_DEVICE          0x02  /*这个IAD是一个CDC设备*/
/*MSC接口配置*/
#define USBD_MSC_INTERFACE_NUM       0x01
#define USBD_IAD_MSC_DEVICE          0x08

/**
  * @}
  */ 


/** @defgroup USBD_CORE_Exported_TypesDefinitions
  * @{
  */

/**
  * @}
  */ 
typedef struct
{
  uint32_t bitrate;
  uint8_t  format;
  uint8_t  paritytype;
  uint8_t  datatype;
}USBD_COMPLEX_LineCodingTypeDef;

typedef struct _USBD_COMPLEX_Itf
{
  int8_t (* Init)          (void);
  int8_t (* DeInit)        (void);
  int8_t (* Control)       (uint8_t, uint8_t * , uint16_t);   
  int8_t (* Receive)       (uint8_t *, uint32_t *);  

}USBD_COMPLEX_ItfTypeDef;


typedef struct
{
  uint32_t data[COMPLEX_DATA_HS_MAX_PACKET_SIZE/4];      /* Force 32bits alignment */
  uint8_t  CmdOpCode;
  uint8_t  CmdLength;    
  uint8_t  *RxBuffer;  
  uint8_t  *TxBuffer;   
  uint32_t RxLength;
  uint32_t TxLength;    
  
  __IO uint32_t TxState;     
  __IO uint32_t RxState;    
}
USBD_COMPLEX_HandleTypeDef; 



/** @defgroup USBD_CORE_Exported_Macros
  * @{
  */ 
  
/**
  * @}
  */ 

/** @defgroup USBD_CORE_Exported_Variables
  * @{
  */ 

extern USBD_ClassTypeDef  USBD_COMPLEX;
#define USBD_COMPLEX_CLASS    &USBD_COMPLEX
/**
  * @}
  */ 

/** @defgroup USB_CORE_Exported_Functions
  * @{
  */
uint8_t  USBD_COMPLEX_RegisterInterface  (USBD_HandleTypeDef   *pdev, 
                                      USBD_COMPLEX_ItfTypeDef *fops);

uint8_t  USBD_COMPLEX_SetTxBuffer        (USBD_HandleTypeDef   *pdev,
                                      uint8_t  *pbuff,
                                      uint16_t length);

uint8_t  USBD_COMPLEX_SetRxBuffer        (USBD_HandleTypeDef   *pdev,
                                      uint8_t  *pbuff);
  
uint8_t  USBD_COMPLEX_ReceivePacket      (USBD_HandleTypeDef *pdev);

uint8_t  USBD_COMPLEX_TransmitPacket     (USBD_HandleTypeDef *pdev);
/**
  * @}
  */ 

#ifdef __cplusplus
}
#endif

#endif  /* __USB_COMPLEX_H */
/**
  * @}
  */ 

/**
  * @}
  */ 
  
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
