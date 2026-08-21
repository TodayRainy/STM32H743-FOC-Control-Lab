#ifndef _MT6835GT_HPP
#define _MT6835GT_HPP

#ifdef __cplusplus
extern "C" {
#endif


#include "spi.h"

#include "math.h"
#include "stdio.h"

#define MT6835_CMD_RD           0x3000          /*读寄存器*/
#define MT6835_CMD_WR           0x6000          /*写寄存器*/
#define MT6835_CMD_EEPROM       0xC000          /*烧录EEPROM*/
#define MT6835_CMD_ZERO         0x5000          /*自动设置零点寄存器*/
#define MT6835_CMD_BURST        0xA000          /*连续读角度寄存器*/

#define MT6835_REG_ID           0x001           /*用户ID*/
#define MT6835_REG_ANGLE3       0x003           /*角度寄存器3*/
#define MT6835_REG_ANGLE2       0x004           /*角度寄存器2*/
#define MT6835_REG_ANGLE1       0x005           /*角度寄存器1*/
#define MT6835_REG_CRC          0x006           /*CRC校验*/
#define MT6835_REG_ABZ_RES2     0x007           /*ABZ输出分辨率*/
#define MT6835_REG_ABZ_RES1     0x008           
#define MT6835_REG_ZERO2        0x009           /*零点寄存器*/
#define MT6835_REG_ZERO1        0x00A           
#define MT6835_REG_UVW          0x00B           /*UVW寄存器*/
#define MT6835_REG_DIR          0x00D           /*方向寄存器*/

class mt6835gt
{
private:
    /* data */
    int8_t                  dir_count;          /*方向计数*/
    float                   angle;
    float                   last_angle;
    
    /*硬件资源*/
    SPI_HandleTypeDef*      p_hspi;
    GPIO_TypeDef*           csPort;
    uint16_t                csPin;

    uint8_t tx_burst[6];                     /*连续读取角度的发送缓冲区*/
    uint8_t rx_burst[6];                     /*连续读取角度的接受缓冲区*/

public:
    bool OverSpeed;             /*转速过快*/
    bool WeakField;             /*磁场虚弱*/
    bool UnderVoltage;          /*电源欠压*/

private:
    /*spi使能*/
    void cs(uint8_t sta)
    {
        if(sta){
            HAL_GPIO_WritePin(csPort, csPin, GPIO_PIN_SET);
        }else{
            HAL_GPIO_WritePin(csPort, csPin, GPIO_PIN_RESET);
        }
    }

    void Angle_Deal(uint8_t rx[6]);

public:
    mt6835gt(SPI_HandleTypeDef*p_hspi, GPIO_TypeDef *csPort, uint16_t csPin);
    ~mt6835gt();

    int read_reg(uint32_t reg, uint8_t &data, uint32_t timeout = HAL_MAX_DELAY);
    int write_reg(uint32_t reg, uint8_t data, uint32_t timeout = HAL_MAX_DELAY);

    int write_zero(void);
    int write_eeprom(void);
    int read_angle(float &angle);

  

    int read_angle_dma_tx(void);
    void read_angle_dma_rx(void);

    bool read_dir(void);
};




























#ifdef __cplusplus
}
#endif
#endif
