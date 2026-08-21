#include "sh367309.hpp"

#include "crc/crc.hpp"

#include "bsp.hpp"


/*寄存器配置*/
/*EEPROM寄存器*/
#define     SCONF1              0x00
#define     SCONF2              0x01 
#define     OVT_LDRT_OVH        0x02
#define     OVL                 0x03
#define     UVT_OVRH            0x04
#define     OVRL                0x05
#define     UV                  0x06
#define     UVR                 0x07
#define     BALV                0x08
#define     PREV                0x09
#define     L0V                 0x0A
#define     PFV                 0x0B
#define     OCD1V_OCD1T         0x0C
#define     OCD2V_OCD2T         0x0D
#define     SCV_SCT             0x0E
#define     OCCV_OCCT           0x0F
#define     MOST_OCRT_PFT       0x10
#define     OTC                 0x11
#define     OTCR                0x12
#define     UTC                 0x13
#define     UTCR                0x14
#define     OTD                 0x15
#define     OTDR                0x16
#define     UTD                 0x17
#define     UTDR                0x18
#define     TR                  0x19

/*RAM寄存器*/
#define     CONF                0x40
#define     BALANCEH            0x41
#define     BALANCEL            0x42
#define     BSTATUS1            0x43
#define     BSTATUS2            0x44
#define     BSTATUS3            0x45
#define     TEMP1H              0x46
#define     TEMP1L              0x47
#define     TEMP2H              0x48
#define     TEMP2L              0x49
#define     TEMP3H              0x4A
#define     TEMP3L              0x4B
#define     CURH                0x4C
#define     CURL                0x4D
#define     CELL1H              0x4E
#define     CELL1L              0x4F
#define     CELL2H              0x50
#define     CELL2L              0x51
#define     CELL3H              0x52
#define     CELL3L              0x53
#define     CELL4H              0x54
#define     CELL4L              0x55
#define     CELL5H              0x56
#define     CELL5L              0x57
#define     CELL6H              0x58
#define     CELL6L              0x59
#define     CELL7H              0x5A
#define     CELL7L              0x5B
#define     CELL8H              0x5C
#define     CELL8L              0x5D
#define     CELL9H              0x5E
#define     CELL9L              0x5F
#define     CELL10H             0x60
#define     CELL10L             0x61
#define     CELL11H             0x62
#define     CELL11L             0x63
#define     CELL12H             0x64
#define     CELL12L             0x65
#define     CELL13H             0x66
#define     CELL13L             0x67
#define     CELL14H             0x68
#define     CELL14L             0x69
#define     CELL15H             0x6A
#define     CELL15L             0x6B
#define     CELL16H             0x6C
#define     CELL16L             0x6D
#define     CADCDH              0x6E
#define     CADCDL              0x6F
#define     BFLAG1              0x70
#define     BFLAG2              0x71
#define     RSTSTAT             0x72










sh367309::sh367309(I2C_HandleTypeDef *p_hi2c, uint8_t addr)
{
    this->flag = false;

    if(NULL==p_hi2c)return;
    this->p_hi2c    = p_hi2c;
    this->addr      = addr;
    this->flag = true;

    uint8_t data[2] = {};

   // soft_i2c_0 = new soft_i2c(I2C3_SCL_GPIO_Port, I2C3_SCL_Pin, I2C3_SDA_GPIO_Port, I2C3_SDA_Pin);

    // write_reg_crc8(0x70, 0x00);
    config();

    // HAL_Delay(1000);

    // if(0==read_reg(0x70, data, 1)){
    //     printf("sh367309: 0x%02x crc:0x%02x  crc8:0x%02x\r\n", data[0], data[1], crc_8(data, 1));
    // }
    

    // HAL_Delay(1000);

    // read_reg(0x44, data, 2);
    // printf("sh367309: 0x%02x crc:0x%02x  crc8:0x%02x\r\n", data[0], data[1], crc_8(data, 1));

    HAL_Delay(10);
    // soft_read_reg(0x00, data[0]);

    read_reg(0x43, data[0]);
    printf("sh367309: 0x%02x crc:0x%02x  crc8:0x%02x\r\n", data[0], data[1], crc_8(data, 1));
}

sh367309::~sh367309()
{
	
}

int sh367309::soft_write_reg_crc8(uint8_t reg, uint8_t data, uint32_t timeout)
{
    soft_i2c_0->IIC_Start();

    soft_i2c_0->IIC_Send_Byte((addr<<1)|0x01);
    if(soft_i2c_0->IIC_Wait_Ack()){
        printf("应答失败1\r\n");
    }


    return 0;
}


int sh367309::soft_read_reg(uint8_t reg, uint8_t &data, uint32_t timeout)
{
    uint8_t crc8 = 0;

    soft_i2c_0->IIC_Start();

    soft_i2c_0->IIC_Send_Byte((addr<<1));

    if(soft_i2c_0->IIC_Wait_Ack()){
        printf("应答失败1\r\n");
    }
    
    soft_i2c_0->IIC_Send_Byte(reg);
    if(soft_i2c_0->IIC_Wait_Ack()){
        printf("应答失败2\r\n");
    }
    soft_i2c_0->IIC_Send_Byte(1);
    if(soft_i2c_0->IIC_Wait_Ack()){
        printf("应答失败3\r\n");
    }

    soft_i2c_0->IIC_Start();
    soft_i2c_0->IIC_Send_Byte((addr<<1)|0x01);
    if(soft_i2c_0->IIC_Wait_Ack()){
        printf("应答失败4\r\n");
    }
    data = soft_i2c_0->IIC_Read_Byte(1);
    crc8 = soft_i2c_0->IIC_Read_Byte(0);

    soft_i2c_0->IIC_Stop();

    printf("crc8: 0x%02X\r\n", crc8);

    return 0;
}


int sh367309::write_reg_crc8(uint8_t reg, uint8_t data, uint32_t timeout)
{
    int ret = 0;
#if 1
    uint8_t buf[4] = {};
    buf[0] = (addr << 1) | 0; // 写地址 (不带R/W位时也有人直接用 devAddr)
    buf[1] = reg;
    buf[2] = data;
    buf[3] = crc_8(buf, 3);

    ret |= HAL_I2C_Master_Transmit(p_hi2c, addr << 1, &buf[1], 3, timeout);
    if(ret){
        // printf("write_reg_crc8: NO\r\n");
    }else{
        // printf("write_reg_crc8: OK\r\n");
    }
#else   

    uint8_t buf[4] = {};
    buf[0] = (addr << 1) | 0; // 写地址 (不带R/W位时也有人直接用 devAddr)
    buf[1] = reg;
    buf[2] = data;
    buf[3] = crc_8(buf, 3);
    
    ret |= HAL_I2C_Mem_Write(p_hi2c, (addr<<1), reg, I2C_MEMADD_SIZE_8BIT, &buf[2], 2, timeout);
    if(ret){
        printf("write_reg_crc8: NO\r\n");
    }else{
        printf("write_reg_crc8: OK\r\n");
    }

#endif

    return ret;
}

int sh367309::read_reg(uint8_t reg, uint8_t &data, uint32_t timeout)
{
    int ret = 0;
    uint8_t addrBuf[6];


    

    ret |= HAL_I2C_Master_Transmit(p_hi2c, addr << 1, &reg, 1, timeout);

    uint8_t rxBuf[2];

    ret |= HAL_I2C_Master_Receive(p_hi2c, (addr << 1)|0x01, rxBuf, 2, timeout);

    addrBuf[0] = (addr << 1) | 0;
    addrBuf[1] = reg;
    addrBuf[2] = 2;
    addrBuf[3] = (addr << 1) | 0x01;
    addrBuf[4] = rxBuf[0];
    uint8_t crc = crc_8(addrBuf, 6);  



    if(ret){
        // printf("read_reg: NO\r\n");
    }else{
        // printf("rxBuf crc: 0x%02x  \r\n", rxBuf[1]);
        // printf("crc8: 0x%02x  \r\n", crc);
        data = rxBuf[0];
    }


    return ret;
}



void sh367309::config(void)
{
    uint8_t data = 0;

    /*关闭预充电 放电 充电mos, 关闭看门狗，sleep、idle*/
    while(write_reg_crc8(CONF, (0x00<<7 | 0x01<<6 | 0x01<<5 | 0x01<<4 | 0x01<<3 | 0x00<<2 | 0x01<<1 | 0x01<<0 ))); 

    while(read_reg(CONF, data));
    printf("reg:0x%02X data: 0x%02X\r\n", CONF, data);

    printf("sh367309 config: OK\r\n");
}


#if 0

int sh367309::read_reg(uint8_t addr, uint8_t reg, uint8_t &data, uint32_t timeout)
{
    if(false==flag)return 1;

    int ret = 0;
    ret = HAL_I2C_Mem_Read(p_hi2c, (addr<<1)|0x01, reg, I2C_MEMADD_SIZE_8BIT, &data, 1, timeout);
    if(ret){
        printf("sh367309: read_reg %d: NO: %d\r\n", reg, ret);
    }

    return ret;
}

int sh367309::read_reg(uint8_t reg, uint8_t &data, uint32_t timeout)
{
    if(false==flag)return 1;

    int ret = 0;
    ret = HAL_I2C_Mem_Read(p_hi2c, (addr<<1)|0x01, reg, I2C_MEMADD_SIZE_8BIT, &data, 1, timeout);
    if(ret){
        printf("sh367309: read_reg %d: NO: %d\r\n", reg, ret);
    }

    return ret;
}

int sh367309::read_reg(uint8_t reg, uint8_t *data, uint32_t len, uint32_t timeout)
{
    if(false==flag)return 1;

    int ret = 0;
    ret = HAL_I2C_Mem_Read(p_hi2c, (addr<<1)|0x01, reg, I2C_MEMADD_SIZE_8BIT, data, len, timeout);
    if(ret){
        printf("sh367309: read_reg 0x%x: NO: %d\r\n", reg, ret);
    }

    return ret;
}

int sh367309::read_reg(uint8_t addr, uint8_t reg, uint8_t *data, uint32_t len, uint32_t timeout)
{
    if(false==flag)return 1;

    int ret = 0;
    ret = HAL_I2C_Mem_Read(p_hi2c, (addr<<1)|0x01, reg, I2C_MEMADD_SIZE_8BIT, data, len, timeout);
    if(ret){
        printf("sh367309: read_reg %d: NO: %d\r\n", reg, ret);
    }
    return ret;
}

unsigned char CRC8(unsigned char *ptr, unsigned char len,unsigned char key)
{
	unsigned char i;
	unsigned char crc=0;
	while(len--!=0)
	{
		for(i=0x80; i!=0; i/=2)
		{
			if((crc & 0x80) != 0)
			{
				crc *= 2;
				crc ^= key;
			}
			else
				crc *= 2;

			if((*ptr & i)!=0)
				crc ^= key;
		}
		ptr++;
	}
	return(crc);
}

int sh367309::write_reg_crc8(uint8_t reg, uint8_t data, uint32_t timeout)
{
    if(false==flag)return 1;

    int ret = 0;

#if 1
    uint8_t tx[2] = {};

    uint8_t crc8_buf[] = {(addr<<1), reg, data};
    tx[0] = data;
    // tx[1] = crc_8(crc8_buf, sizeof(crc8_buf));

    tx[1] = CRC8(crc8_buf, 3, 7);

    // printf("crc_8: %d  CRC8: %d\r\n", crc_8(crc8_buf, sizeof(crc8_buf)),  CRC8(crc8_buf, 3, 7));

    ret = HAL_I2C_Mem_Write(p_hi2c, (addr<<1), reg, I2C_MEMADD_SIZE_8BIT, tx, 2, timeout);
    if(ret){
        printf("sh367309: write_reg 0x%02x: NO: %d\r\n", reg, ret);
    }
#else

    uint8_t buf[4] = {};
    buf[0] = addr<<1;
    buf[1] = reg;                       // 寄存器地址
    buf[2] = data;                      // 寄存器地址
    buf[3] = CRC8(buf, 3, 7);     // 地址+数据 计算 CRC8

    uint8_t tx[3] = { reg, data, buf[3]};

    ret = HAL_I2C_Master_Transmit(p_hi2c, (addr<<1), tx, 3, 1000);
    if(ret){
        printf("sh367309: write_reg %d: NO: %d\r\n", reg, ret);
    }

    uint8_t crc8_buf[] = {(addr<<1), reg, data};
    printf("crc_8: %d  CRC8: %d\r\n", crc_8(crc8_buf, sizeof(crc8_buf)),  CRC8(crc8_buf, 3, 7));

#endif

    return ret;
}

int sh367309::write_reg_crc8(uint8_t addr, uint8_t reg, uint8_t data, uint32_t timeout)
{
    if(false==flag)return 1;

    int ret = 0;
    uint8_t tx[2] = {};
    tx[0] = data;
    tx[1] = crc_8(&data, 1);

    ret = HAL_I2C_Mem_Write(p_hi2c, (addr<<1), reg, I2C_MEMADD_SIZE_8BIT, tx, 2, timeout);
    if(ret){
        printf("sh367309: write_reg %d: NO: %d\r\n", reg, ret);
    }

    return ret;
}


#endif













