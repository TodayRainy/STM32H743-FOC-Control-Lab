/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    USB_Device/CDC_Standalone/USB_Device/App/usb_device.c
  * @author  MCD Application Team
  * @brief   This file implements the USB Device
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "usb_device.h"


/* Private variables ---------------------------------------------------------*/

// __IO uint32_t remotewakeupon = 0;
// uint8_t HID_Buffer[4];
extern PCD_HandleTypeDef hpcd_USB_OTG_FS;

/* USER CODE END PV */

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/
void USBD_Clock_Config(void);
/* USER CODE END PFP */

/* USB Device Core handle declaration. */
USBD_HandleTypeDef USBD_Device;
volatile uint32_t usb_device_init_status = 0xFFFFFFFFU;

/**
  * Init USB device Library, add supported class and start the library
  * @retval None
  */
uint8_t usb_buf[128];
void MX_USB_Device_Init(void)
{
    uint8_t status;
    usb_device_init_status = 0xFFFFFFFEU;
    // USBD_Init(&USBD_Device, &MSC_Desc, DEVICE_FS);       /* 初始化USB */
    // USBD_RegisterClass(&USBD_Device,  &USBD_MSC);        /* 添加类 */
    // USBD_MSC_RegisterStorage(&USBD_Device,  &USBD_DISK_fops);    /* 为MSC类添加回调函数 */
    // USBD_Start(&USBD_Device);                           /* 开启USB */

    /*单独使用cdc*/
    status = USBD_Init(&USBD_Device, &VCP_Desc, DEVICE_FS);
    if (status != USBD_OK)
    {
        usb_device_init_status = 0x00000100U | status;
        return;
    }
    status = USBD_RegisterClass(&USBD_Device, USBD_CDC_CLASS);
    if (status != USBD_OK)
    {
        usb_device_init_status = 0x00000200U | status;
        return;
    }
    status = USBD_CDC_RegisterInterface(&USBD_Device, &USBD_CDC_fops);
    if (status != USBD_OK)
    {
        usb_device_init_status = 0x00000300U | status;
        return;
    }
    status = USBD_Start(&USBD_Device);
    if (status != USBD_OK)
    {
        usb_device_init_status = 0x00000400U | status;
        return;
    }
    usb_device_init_status = 0;

    /*使用复合设备*/
    // USBD_Init(&USBD_Device, &COMPLEX_Desc, 0);
    // USBD_RegisterClass(&USBD_Device, USBD_COMPLEX_CLASS);
    // USBD_Start(&USBD_Device);                                   //开启USB

    //cdc_vcp_data_tx(usb_buf, 1);
  //  USBD_CDC_SetTxBuffer(&USBD_Device, usb_buf, 128);
}

