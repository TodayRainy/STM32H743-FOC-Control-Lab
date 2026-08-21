
#include "usbd_complex.h"
#include <bsp.h>


#include "usbd_cdc.h"


static uint8_t  USBD_COMPLEX_Init (USBD_HandleTypeDef *pdev, 
                               uint8_t cfgidx);

static uint8_t  USBD_COMPLEX_DeInit (USBD_HandleTypeDef *pdev, 
                                 uint8_t cfgidx);

static uint8_t  USBD_COMPLEX_Setup (USBD_HandleTypeDef *pdev, 
                                USBD_SetupReqTypedef *req);

static uint8_t  USBD_COMPLEX_DataIn (USBD_HandleTypeDef *pdev, 
                                 uint8_t epnum);

static uint8_t  USBD_COMPLEX_DataOut (USBD_HandleTypeDef *pdev, 
                                 uint8_t epnum);

static uint8_t  USBD_COMPLEX_EP0_RxReady (USBD_HandleTypeDef *pdev);

static uint8_t  *USBD_COMPLEX_GetFSCfgDesc (uint16_t *length);

static uint8_t  *USBD_COMPLEX_GetHSCfgDesc (uint16_t *length);

static uint8_t  *USBD_COMPLEX_GetOtherSpeedCfgDesc (uint16_t *length);

static uint8_t  *USBD_COMPLEX_GetOtherSpeedCfgDesc (uint16_t *length);

uint8_t  *USBD_COMPLEX_GetDeviceQualifierDescriptor (uint16_t *length);

/* USB Standard Device Descriptor */
__ALIGN_BEGIN static uint8_t USBD_COMPLEX_DeviceQualifierDesc[USB_LEN_DEV_QUALIFIER_DESC] __ALIGN_END =
{
  USB_LEN_DEV_QUALIFIER_DESC,
  USB_DESC_TYPE_DEVICE_QUALIFIER,
  0x00,
  0x02,
  0x00,
  0x00,
  0x00,
  0x40,
  0x01,
  0x00,
};

/**
  * @}
  */ 

/** @defgroup USBD_COMPLEX_Private_Variables
  * @{
  */ 


/* CDC interface class callbacks structure */
USBD_ClassTypeDef  USBD_COMPLEX = 
{
  USBD_COMPLEX_Init,
  USBD_COMPLEX_DeInit,
  USBD_COMPLEX_Setup,
  NULL,                 /* EP0_TxSent, */
  USBD_COMPLEX_EP0_RxReady,
  USBD_COMPLEX_DataIn,
  USBD_COMPLEX_DataOut,
  NULL,
  NULL,
  NULL,     
  USBD_COMPLEX_GetHSCfgDesc,  
  USBD_COMPLEX_GetFSCfgDesc,    
  USBD_COMPLEX_GetOtherSpeedCfgDesc, 
  USBD_COMPLEX_GetDeviceQualifierDescriptor,
};

/* USB CDC device Configuration Descriptor */
/*配置描述符*/



/* USB CDC device Configuration Descriptor */
/*复合设备配置描述符 快速模式*/
__ALIGN_BEGIN uint8_t USBD_COMPLEX_CfgFSDesc[USB_COMPLEX_CONFIG_DESC_SIZ] __ALIGN_END =
{

    0x09,   /* 长度: 0x09 */
    USB_DESC_TYPE_CONFIGURATION,            /* 这是一个配置描述符 */
    USB_COMPLEX_CONFIG_DESC_SIZ, 0x00,      /*描述符总长度，CONFIG + IAD + CDC + IAD + MSC */
    0x01,   /* 接口数量  3个接口，CDC占2个，MSC占1个*/
    0x01,   /* 配置值 */
    0x00,   /* 配置描述符字符串索引: */
    0xC0,   /* 配置特性: */
    0x64,   /* MaxPower 100 mA */

  #if 1
/*******************************MSC****************************************/
    /* IAD描述符，用于指示MSC */
    USBD_IAD_DESC_SIZE,  		            /*IAD描述符大小: 0x08*/  
    USBD_IAD_DESCRIPTOR_TYPE, 			    /*描述符类型: IAD*/
    USBD_MSC_FIRST_INTERFACE,  		        /*接口的起始序号*/
    USBD_MSC_INTERFACE_NUM,  			    /*该IAD下的接口数量*/
    USBD_IAD_MSC_DEVICE,  				    /*MSC，表明该IAD是一个MSC类型的设备*/
    0x06,  									/*bFunctionSubClass：子类型*/
    0x50,  									/* bFunctionProtocol：控制协议*/
    0x05,  									/*iFunction*/
    /* MSC接口描述符 */
    0x09,                                   /* 长度0x09*/
    USB_DESC_TYPE_INTERFACE,                /* 描述符类型: 接口描述符*/
    USBD_MSC_FIRST_INTERFACE,               /* 该接口的序号:*/
    0x00,                                   /* 该接口的设置序号 */
    0x02,                                   /* 端点数量*/
    0x08,                                   /* bInterfaceClass: MSC Class */
    0x06,                                   /* bInterfaceSubClass : SCSI transparent*/
    0x50,                                   /* nInterfaceProtocol */
    0x05,                                   /* 字符串描述的索引号 */

    /* MSC端点描述符 */
    0x07,                                   /* 描述符长度 */
    USB_DESC_TYPE_ENDPOINT,                 /* 描述符类型: 端点描述符 */
    MSC_EPIN_ADDR,                          /* 端点地址 */
    USBD_EP_TYPE_BULK,                      /* 端点类型: 突发型 */
    LOBYTE(MSC_MAX_FS_PACKET),              /* 端点包大小 */
    HIBYTE(MSC_MAX_FS_PACKET),
    0x00,   /*Polling interval in milliseconds */

    /* MSC端点描述符 */
    0x07,                                   /* 描述符长度 */
    0x05,                                   /* 描述符类型: 端点描述符 */
    MSC_EPOUT_ADDR,                         /* 端点地址 */
    USBD_EP_TYPE_BULK,                      /* 端点类型: 突发型*/
    LOBYTE(MSC_MAX_FS_PACKET),              /* 端点包大小 */
    HIBYTE(MSC_MAX_FS_PACKET),
    0x00,     /*Polling interval in milliseconds*/
    #endif



#if 1
/***************************************CDC**********************************************/
/*CDC IDA描述符*/      
    USBD_IAD_DESC_SIZE,  								// bLength: Interface Descriptor size，固定值
    USBD_IAD_DESCRIPTOR_TYPE,  					        // bDescriptorType: IAD，固定值
    USBD_CDC_FIRST_INTERFACE,  					        // bFirstInterface，第一个接口的起始序号，从0开始
    USBD_CDC_INTERFACE_NUM,  						    // bInterfaceCount，本IAD下的接口数量
    USBD_IAD_CDC_DEVICE,  							    // bFunctionClass: CDC，表明该IAD是一个CDC类型的设备
    0X02,  											    // bFunctionSubClass：子类型
    0X01,  												// bFunctionProtocol：控制协议
    0X00,  												// iFunction

    /*接口描述符 */
    0x09,                                               /* 长度: 0x09 */
    USB_DESC_TYPE_INTERFACE,                            /* 描述符类型: 接口描述符 */
    USBD_CDC_FIRST_INTERFACE,                           /* 该接口的序号: */
    0x00,                                               /* 该接口的设置序号 */
    0x01,                                               /* 支持多少个非0的EP端点 */
    0x02,                                               /* CDC类 */
    0x02,                                               /* CDC子类 */
    0x01,                                               /* CDC支持的协议 */
    0x00,                                               /* 在众多字符串描述符中序号: */
  
    /*描述 CDC（通信设备类）接口的头部功能*/
    0x05,                                               /* 长度: 0x05 */
    0x24,                                               /* bDescriptorType: CS_INTERFACE */
    0x00,                                               /* bDescriptorSubtype: Header Func Desc */
    0x10,                                               /* bcdCDC: spec release number */
    0x01,
  
  /*Call Management Functional Descriptor*/
    0x05,   /* bFunctionLength */
    0x24,   /* bDescriptorType: CS_INTERFACE */
    0x01,   /* bDescriptorSubtype: Call Management Func Desc */
    0x00,   /* bmCapabilities: D0+D1 */
    0x01,   /* bDataInterface: 1 */
  
  /*ACM Functional Descriptor*/
    0x04,   /* bFunctionLength */
    0x24,   /* bDescriptorType: CS_INTERFACE */
    0x02,   /* bDescriptorSubtype: Abstract Control Management desc */
    0x02,   /* bmCapabilities */
  
  /*Union Functional Descriptor*/
    0x05,   /* bFunctionLength */
    0x24,   /* bDescriptorType: CS_INTERFACE */
    0x06,   /* bDescriptorSubtype: Union func desc */
    0x00,   /* bMasterInterface: Communication class interface */
    0x01,   /* bSlaveInterface0: Data Class Interface */
  
  /*Endpoint 2 Descriptor*/
    0x07,                           /* bLength: Endpoint Descriptor size */
    USB_DESC_TYPE_ENDPOINT,         /* bDescriptorType: Endpoint */
    CDC_CMD_EP,                     /* bEndpointAddress */
    USBD_EP_TYPE_INTR,                           /* bmAttributes: Interrupt */
    LOBYTE(CDC_CMD_PACKET_SIZE),     /* wMaxPacketSize: */
    HIBYTE(CDC_CMD_PACKET_SIZE),
    0x10,                           /* bInterval: */ 
  /*---------------------------------------------------------------------------*/
  
  /*Data class interface descriptor*/
    0x09,   /* bLength: Endpoint Descriptor size */
    USB_DESC_TYPE_INTERFACE,        /* bDescriptorType: */
    USBD_CDC_FIRST_INTERFACE+0x01,   /* bInterfaceNumber: Number of Interface */
    0x00,   /* bAlternateSetting: Alternate setting */
    0x02,   /* bNumEndpoints: Two endpoints used */
    0x0A,   /* bInterfaceClass: CDC */
    0x00,   /* bInterfaceSubClass: */
    0x00,   /* bInterfaceProtocol: */
    0x00,   /* iInterface: */
  
  /*Endpoint OUT Descriptor*/
    0x07,   /* bLength: Endpoint Descriptor size */
    USB_DESC_TYPE_ENDPOINT,      /* bDescriptorType: Endpoint */
    CDC_OUT_EP,                        /* bEndpointAddress */
    USBD_EP_TYPE_BULK,                              /* bmAttributes: Bulk */
    LOBYTE(CDC_DATA_HS_MAX_PACKET_SIZE),  /* wMaxPacketSize: */
    HIBYTE(CDC_DATA_HS_MAX_PACKET_SIZE),
    0x00,                              /* bInterval: ignore for Bulk transfer */
  
  /*Endpoint IN Descriptor*/
    0x07,   /* bLength: Endpoint Descriptor size */
    USB_DESC_TYPE_ENDPOINT,      /* bDescriptorType: Endpoint */
    CDC_IN_EP,                         /* bEndpointAddress */
    USBD_EP_TYPE_BULK,                              /* bmAttributes: Bulk */
    LOBYTE(CDC_DATA_HS_MAX_PACKET_SIZE),  /* wMaxPacketSize: */
    HIBYTE(CDC_DATA_HS_MAX_PACKET_SIZE),
    0x00,                               /* bInterval: ignore for Bulk transfer */
#endif
} ;


/**
  * @}
  */ 

/** @defgroup USBD_COMPLEX_Private_Functions
  * @{
  */ 


/**
  * @brief  USBD_COMPLEX_Init
  *         Initialize the CDC interface
  * @param  pdev: device instance
  * @param  cfgidx: Configuration index
  * @retval status
  */    /*复合设备初始化*/
static uint8_t  USBD_COMPLEX_Init (USBD_HandleTypeDef *pdev, 
                               uint8_t cfgidx)
{
    uint8_t ret = 0;
    printf("***************************************************************USBD_COMPLEX_Init:%d\r\n", cfgidx);
    // USBD_CDC.Init(pdev, cfgidx);
    USBD_MSC.Init(pdev, cfgidx);

    return ret;
}

/**
  * @brief  USBD_COMPLEX_Init
  *         DeInitialize the CDC layer
  * @param  pdev: device instance
  * @param  cfgidx: Configuration index
  * @retval status
  */
static uint8_t  USBD_COMPLEX_DeInit (USBD_HandleTypeDef *pdev, 
                                 uint8_t cfgidx)
{
    uint8_t ret = 0;
    
    // USBD_CDC.DeInit(pdev, cfgidx);
    USBD_MSC.DeInit(pdev, cfgidx);
    
    return ret;
}

/**
  * @brief  USBD_COMPLEX_Setup
  *         Handle the CDC specific requests
  * @param  pdev: instance
  * @param  req: usb requests
  * @retval status
  */
static uint8_t  USBD_COMPLEX_Setup (USBD_HandleTypeDef *pdev, 
                                USBD_SetupReqTypedef *req)
{
    // USBD_MSC.Setup(pdev, req);
    printf("******************************************************************req->wIndex:%d\r\n", req->wIndex);
    switch(req->wIndex)
	{
		case USBD_CDC_FIRST_INTERFACE:
            printf("******************************************************************USBD_CDC:配置\r\n");
			USBD_CDC.Setup(pdev, req);
			break;
		case USBD_MSC_FIRST_INTERFACE:
            printf("******************************************************************USBD_MSC:配置\r\n");
			USBD_MSC.Setup(pdev, req);
			break;
		default:
			break;
	}

    return USBD_OK;
}

/**
  * @brief  USBD_COMPLEX_DataIn
  *         Data sent on non-control IN endpoint
  * @param  pdev: device instance
  * @param  epnum: endpoint number
  * @retval status
  */
static uint8_t  USBD_COMPLEX_DataIn (USBD_HandleTypeDef *pdev, uint8_t epnum)
{
    switch(epnum | 0x80){
        case MSC_EPIN_ADDR:
            USBD_MSC.DataIn(pdev, epnum);
            break; 
        case CDC_IN_EP:
            USBD_CDC.DataIn(pdev, epnum);
            break;
        default:
            break;
    }
    return USBD_OK;
}

/**
  * @brief  USBD_COMPLEX_DataOut
  *         Data received on non-control Out endpoint
  * @param  pdev: device instance
  * @param  epnum: endpoint number
  * @retval status
  */
static uint8_t  USBD_COMPLEX_DataOut (USBD_HandleTypeDef *pdev, uint8_t epnum)
{      
    switch(epnum)
    {
        case MSC_EPOUT_ADDR:
            USBD_MSC.DataOut(pdev, epnum);
            break;
        case CDC_OUT_EP:
            USBD_CDC.DataOut(pdev, epnum);
            break;
        default:
            break;
    }
    return USBD_OK;
}



/**
  * @brief  USBD_COMPLEX_DataOut
  *         Data received on non-control Out endpoint
  * @param  pdev: device instance
  * @param  epnum: endpoint number
  * @retval status
  */
static uint8_t  USBD_COMPLEX_EP0_RxReady (USBD_HandleTypeDef *pdev)
{ 
	return USBD_CDC.EP0_RxReady(pdev);
}

/**
  * @brief  USBD_COMPLEX_GetFSCfgDesc 
  *         Return configuration descriptor
  * @param  speed : current device speed
  * @param  length : pointer data length
  * @retval pointer to descriptor buffer
  */
static uint8_t  *USBD_COMPLEX_GetFSCfgDesc (uint16_t *length)
{
  *length = sizeof (USBD_COMPLEX_CfgFSDesc);
  return USBD_COMPLEX_CfgFSDesc;
}

/**
  * @brief  USBD_COMPLEX_GetHSCfgDesc 
  *         Return configuration descriptor
  * @param  speed : current device speed
  * @param  length : pointer data length
  * @retval pointer to descriptor buffer
  */
static uint8_t  *USBD_COMPLEX_GetHSCfgDesc (uint16_t *length)
{
  *length = sizeof (USBD_COMPLEX_CfgFSDesc);
  return USBD_COMPLEX_CfgFSDesc;
}

/**
  * @brief  USBD_COMPLEX_GetCfgDesc 
  *         Return configuration descriptor
  * @param  speed : current device speed
  * @param  length : pointer data length
  * @retval pointer to descriptor buffer
  */
static uint8_t  *USBD_COMPLEX_GetOtherSpeedCfgDesc (uint16_t *length)
{
  *length = sizeof (USBD_COMPLEX_CfgFSDesc);
  return USBD_COMPLEX_CfgFSDesc;
}

/**
* @brief  DeviceQualifierDescriptor 
*         return Device Qualifier descriptor
* @param  length : pointer data length
* @retval pointer to descriptor buffer
*/
uint8_t  *USBD_COMPLEX_GetDeviceQualifierDescriptor (uint16_t *length)
{
  *length = sizeof (USBD_COMPLEX_DeviceQualifierDesc);
  return USBD_COMPLEX_DeviceQualifierDesc;
}



/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
