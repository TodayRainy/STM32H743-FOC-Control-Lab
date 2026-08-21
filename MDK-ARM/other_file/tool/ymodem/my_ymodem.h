#ifndef _my_ymodem_H_
#define _my_ymodem_H_

#include "sys.h"
#include "ymodem.h"

#include "usart1.h"
//SecureCR软件内部设置:选项--会话选项--XYZmodem--1024byte
#define YMODEM_USARTx         USART1
#define YMODEM_USARTx_init    USART1_Init
#define SerialPutString       printf 
#define ApplicationAddress    0x8005000  //本bootloder 起始地址0x8000000 偏移0x5000=20k
 
#if defined (STM32F10X_MD) || defined (STM32F10X_MD_VL)
 #define PAGE_SIZE                         (0x400)    /* 1 Kbyte */
 #define FLASH_SIZE                        (0x20000)  /* 128 KBytes */
#elif defined STM32F10X_CL
 #define PAGE_SIZE                         (0x800)    /* 2 Kbytes */
 #define FLASH_SIZE                        (0x40000)  /* 256 KBytes */
#elif defined STM32F10X_HD || defined (STM32F10X_HD_VL)
 #define PAGE_SIZE                         (0x800)    /* 2 Kbytes */
 #define FLASH_SIZE                        (0x40000)  /* 512 KBytes */
#elif defined STM32F10X_XL
 #define PAGE_SIZE                         (0x800)    /* 2 Kbytes */
 #define FLASH_SIZE                        (0x100000) /* 1 MByte */
#else 
 #error "Please select first the STM32 device to be used (in stm32f10x.h)"    
#endif
#define FLASH_IMAGE_SIZE     (u32) (FLASH_SIZE - (ApplicationAddress - 0x08000000))//计算上传文件大小,是否超过剩余FLASH

#define IS_AF(c)             ((c >= 'A') && (c <= 'F'))
#define IS_af(c)             ((c >= 'a') && (c <= 'f'))
#define IS_09(c)             ((c >= '0') && (c <= '9'))
#define ISVALIDHEX(c)        IS_AF(c) || IS_af(c) || IS_09(c)
#define ISVALIDDEC(c)        IS_09(c)
#define CONVERTDEC(c)        (c - '0')
#define CONVERTHEX_alpha(c)  (IS_AF(c) ? (c - 'A'+10) : (c - 'a'+10))
#define CONVERTHEX(c)        (IS_09(c) ? (c - '0') : CONVERTHEX_alpha(c))
 


#endif

