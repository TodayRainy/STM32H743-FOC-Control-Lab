#ifndef _W25QXX_HPP
#define _W25QXX_HPP


#include "stdint.h"
#include "sys_m7.h"

/******************************************************************************************/



/* FLASHÐ¾Æ¬ÁÐ±í */
#define W25Q80      0XEF13          /* W25Q80   Ð¾Æ¬ID */
#define W25Q16      0XEF14          /* W25Q16   Ð¾Æ¬ID */
#define W25Q32      0XEF15          /* W25Q32   Ð¾Æ¬ID */
#define W25Q64      0XEF16          /* W25Q64   Ð¾Æ¬ID */
#define W25Q128     0XEF17          /* W25Q128  Ð¾Æ¬ID */
#define W25Q256     0XEF18          /* W25Q256  Ð¾Æ¬ID */
#define BY25Q64     0X6816          /* BY25Q64  Ð¾Æ¬ID */
#define BY25Q128    0X6817          /* BY25Q128 Ð¾Æ¬ID */
#define NM25Q64     0X5216          /* NM25Q64  Ð¾Æ¬ID */
#define NM25Q128    0X5217          /* NM25Q128 Ð¾Æ¬ID */
 
/* Ö¸Áî±í */
#define FLASH_WriteEnable           0x06 
#define FLASH_WriteDisable          0x04 
#define FLASH_ReadStatusReg1        0x05 
#define FLASH_ReadStatusReg2        0x35 
#define FLASH_ReadStatusReg3        0x15 
#define FLASH_WriteStatusReg1       0x01 
#define FLASH_WriteStatusReg2       0x31 
#define FLASH_WriteStatusReg3       0x11 
#define FLASH_ReadData              0x03 
#define FLASH_FastReadData          0x0B 
#define FLASH_FastReadDual          0x3B 
#define FLASH_FastReadQuad          0xEB  
#define FLASH_PageProgram           0x02 
#define FLASH_PageProgramQuad       0x32 
#define FLASH_BlockErase            0xD8 
#define FLASH_SectorErase           0x20 
#define FLASH_ChipErase             0xC7 
#define FLASH_PowerDown             0xB9 
#define FLASH_ReleasePowerDown      0xAB 
#define FLASH_DeviceID              0xAB 
#define FLASH_ManufactDeviceID      0x90 
#define FLASH_JedecDeviceID         0x9F 
#define FLASH_Enable4ByteAddr       0xB7
#define FLASH_Exit4ByteAddr         0xE9
#define FLASH_SetReadParam          0xC0 
#define FLASH_EnterQPIMode          0x38
#define FLASH_ExitQPIMode           0xFF


#if 0
/* ¾²Ì¬º¯Êý */
static void norflash_wait_busy(void);                                               /* µÈ´ý¿ÕÏÐ */
static void norflash_send_address(uint32_t address);                                /* ·¢ËÍµØÖ· */
static void norflash_write_page(uint8_t *pbuf, uint32_t addr, uint16_t datalen);    /* Ð´Èëpage */
static void norflash_write_nocheck(uint8_t *pbuf, uint32_t addr, uint16_t datalen); /* Ð´flash,²»´ø²Á³ý */

/* ÆÕÍ¨º¯Êý */
void norflash_init(void);                                                           /* ³õÊ¼»¯25QXX */
uint16_t norflash_read_id(void);                                                    /* ¶ÁÈ¡FLASH ID */
void norflash_write_enable(void);                                                   /* Ð´Ê¹ÄÜ */
uint8_t norflash_read_sr(uint8_t regno);                                            /* ¶ÁÈ¡×´Ì¬¼Ä´æÆ÷ */
void norflash_write_sr(uint8_t regno,uint8_t sr);                                   /* Ð´×´Ì¬¼Ä´æÆ÷ */

void norflash_erase_chip(void);                                                     /* ÕûÆ¬²Á³ý */
void norflash_erase_sector(uint32_t saddr);                                         /* ÉÈÇø²Á³ý */
void norflash_read(uint8_t *pbuf, uint32_t addr, int32_t datalen);                 /* ¶ÁÈ¡flash */
void norflash_write(uint8_t *pbuf, uint32_t addr, uint16_t datalen);                /* Ð´Èëflash */

#endif


/*********************************Ó¦ÓÃÏà¹Ø**************************************** */




class w25qxx
{
private:
    /* data */

    /*Ó²¼þ×ÊÔ´*/
    SPI_HandleTypeDef*      p_hspi;
    GPIO_TypeDef*           csPort;
    uint16_t                csPin;

    uint16_t norflash_type;

    uint8_t buf[4096];                                          /* ÉÈÇø»º´æ */


    void cs(uint8_t sta){
        if(sta){
            HAL_GPIO_WritePin(csPort, csPin, GPIO_PIN_SET);
        }else{
            HAL_GPIO_WritePin(csPort, csPin, GPIO_PIN_RESET);
        }
    }

private:
    int read_write_byte(uint8_t tx, uint8_t &rx, uint32_t timeout = HAL_MAX_DELAY);
    
    int read_write_buf(uint8_t *tx, uint8_t *rx, uint32_t len, uint32_t timeout = HAL_MAX_DELAY);

    int read_buf(uint8_t *rx, uint32_t len, uint32_t timeout = HAL_MAX_DELAY);
    int write_buf(uint8_t *tx, uint32_t len, uint32_t timeout = HAL_MAX_DELAY);

    void write_enable(uint32_t timeout = HAL_MAX_DELAY);
    void send_address(uint32_t addr, uint32_t timeout = HAL_MAX_DELAY);

    int read_sr(uint8_t regno, uint8_t &data, uint32_t timeout = HAL_MAX_DELAY);

    int write_sr(uint8_t regno, uint8_t sr, uint32_t timeout = HAL_MAX_DELAY);
    void wait_busy(void);

    void write_nocheck(uint8_t *pbuf, uint32_t addr, uint16_t datalen, uint32_t timeout = HAL_MAX_DELAY);


public:
    w25qxx(SPI_HandleTypeDef*p_hspi, GPIO_TypeDef *csPort, uint16_t csPin);
    ~w25qxx();

    uint16_t read_id(uint32_t timeout = HAL_MAX_DELAY);

    void read(uint8_t *pbuf, uint32_t addr, int32_t len, uint32_t timeout = HAL_MAX_DELAY);
    void write_page(uint8_t *pbuf, uint32_t addr, uint16_t datalen, uint32_t timeout = HAL_MAX_DELAY);
    
    void write(uint8_t *pbuf, uint32_t addr, uint16_t datalen, uint32_t timeout = HAL_MAX_DELAY);


    void erase_sector(uint32_t saddr);
    void erase_chip(void);

};


















#endif
