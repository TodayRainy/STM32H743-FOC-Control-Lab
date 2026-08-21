#ifndef _ECBM_MODBUS_H_
#define _ECBM_MODBUS_H_
/*-------------------------------------------------------------------------------------
The MIT License (MIT)

Copyright (c) 2023 奈特

Permission is hereby granted, free of charge, to any person obtaining a copy of
this software and associated documentation files (the "Software"), to deal in
the Software without restriction, including without limitation the rights to
use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
the Software, and to permit persons to whom the Software is furnished to do so,
subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

免责说明：
    本软件库以MIT开源协议免费向大众提供。作者只保证原始版本是由作者在维护修BUG，
其他通过网络传播的版本也许被二次修改过，由此出现的BUG与作者无关。而当您使用原始
版本出现BUG时，请联系作者解决。
                            **************************
                            * 联系方式：进群778916610 *
                            * 若1群满人：进群927297508*
                            **************************
------------------------------------------------------------------------------------*/
//-----------------以下是图形设置界面，可在Configuration Wizard界面设置-----------------
//<<< Use Configuration Wizard in Context Menu >>>
//<q>使能IO控制读写功能？
//<i>使能后需要自己定义实现接口函数，函数内部放控制IO的语句，IO电平由形参传入
//<i>当modbus需要控制IO的时候会调用这个接口函数
//<i>void ecbm_modbus_rtu_set_io(emu8 dat)
//<i>{
//<i>    gpio_out_fast(GPIO_P5, GPIO_PIN_5, dat);
//<i>    //gpio_out(RS485_ENA,dat);
//<i>}
#define ECBM_MODBUS_IO_EN 0
//<h>MODBUS_RTU
#define ECBM_MODBUS_VERSION "V1.0.4"
//<i>这是modbus协议的RTU模式。
//<o>本机地址/ID
//<1-247>
//<i>8位的地址，可使用的范围是1~247，0作为广播地址。
#define ECBM_MODBUS_RTU_ID_ADRESS 1
//<o>超时时间
//<i>单位为定时器中断次数。比如500就是定时器中断500次，若每次中断的间隔是1ms，那么就是500mS超时。
#define ECBM_MODBUS_RTU_TIME_MAX  5
//<h>线圈读写功能设置
//<e>线圈缓存
//<i>勾选后会定义一个数组作为缓存，线圈指令操作都会指向这个数组，适合新项目的开始。可以使用读写函数获取数组内部的值。
//<i>不勾选的话，可以通过移植两个读写函数将读写线圈功能映射到任意的地方。适合为老项目增加modbus功能。
#define ECBM_MODBUS_RTU_BIT_BUF_EN 1
//<o>线圈起始地址
//<i>有时候为了适配别人的协议，地址不一定是从0开始的，在这里可以设置一个偏移。
#define ECBM_MODBUS_RTU_BIT_START_ADDR 0
//<o>线圈缓存总数
//<i>单位是字节（8位），请根据需求填写。比如设备要有10个线圈，因为至少需要两个字节才能存完10个，那么就应该填2。
#define ECBM_MODBUS_RTU_BIT_BUF_SIZE  3
//</e>
//<h>线圈指令使能
//<i>如果和线圈有关的指令都没有被使能，那么线圈读写相关的函数和缓存都将被优化掉。
//<q>[01]读线圈
#define ECBM_MODBUS_RTU_CMD01_EN 1
//<q>[05]写单个线圈
#define ECBM_MODBUS_RTU_CMD05_EN 1
//</h>
//</h>
//<h>寄存器读写功能设置
//<e>寄存器缓存
#define ECBM_MODBUS_RTU_REG_BUF_EN 1
//<o>寄存器起始地址
//<i>有时候为了适配别人的协议，地址不一定是从0开始的，在这里可以设置一个偏移。
#define ECBM_MODBUS_RTU_REG_START_ADDR 0
//<o>寄存器缓存总数
//<i>单位是字（16位），请根据需求填写。
#define ECBM_MODBUS_RTU_REG_BUF_SIZE  1300
//</e>
//<h>寄存器指令使能
//<i>如果和寄存器有关的指令都没有被使能，那么线圈读写相关的函数和缓存都将被优化掉。
//<q>[03]读寄存器
#define ECBM_MODBUS_RTU_CMD03_EN 1
//<q>[06]写单个寄存器
#define ECBM_MODBUS_RTU_CMD06_EN 1
//<e>[10]写多个寄存器
#define ECBM_MODBUS_RTU_CMD10_EN 1
//<o>寄存器写入缓存总数
//<i>单位是字节（8位），由于CRC的存在，在多字节写入的时候，不能立刻把数据存入，需要用一个缓存存起来，待CRC验证完毕之后才能一起写入。
#define ECBM_MODBUS_RTU_BUF_SIZE  1300
//</e>
//</h>
//</h>
//<h>IO系统指令使能
//<i>如果和IO系统有关的指令都没有被使能，那么IO相关的函数和缓存都将被优化掉。

//<q>[02]读离散量输入
#define ECBM_MODBUS_RTU_CMD02_EN 1
#define ECBM_MODBUS_RTU_INPUT_BIT_BUF_EN         1                                   /*输入离散量寄存器使能*/                                 
#define ECBM_MODBUS_RTU_INPUT_BIT_START_ADDR     0                               /*输入离散量寄存器起始地址*/
#define ECBM_MODBUS_RTU_INPUT_BIT_BUF_SIZE       160                                 /*输入离散量寄存器缓存大小*/
//<q>[04]读输入寄存器
#define ECBM_MODBUS_RTU_CMD04_EN 1
#define ECBM_MODBUS_RTU_INPUT_REG_BUF_EN         1                                      /*输入寄存器使能*/                                 
#define ECBM_MODBUS_RTU_INPUT_REG_START_ADDR     0                                      /*输入寄存器起始地址*/
#define ECBM_MODBUS_RTU_INPUT_REG_BUF_SIZE       600                                    /*输入寄存器缓存大小*/


//</h>
#define ECBM_MODBUS_RTU_CMD_BIT_EN  (ECBM_MODBUS_RTU_CMD01_EN+ECBM_MODBUS_RTU_CMD05_EN)
#define ECBM_MODBUS_RTU_CMD_REG_EN  (ECBM_MODBUS_RTU_CMD03_EN+ECBM_MODBUS_RTU_CMD06_EN+ECBM_MODBUS_RTU_CMD10_EN)
#define ECBM_MODBUS_RTU_CMD_IO_EN   (ECBM_MODBUS_RTU_CMD02_EN+ECBM_MODBUS_RTU_CMD04_EN)
#define ECBM_MODBUS_RTU_CMD_ALL_EN  (ECBM_MODBUS_RTU_CMD_BIT_EN+ECBM_MODBUS_RTU_CMD_REG_EN+ECBM_MODBUS_RTU_CMD_IO_EN)
//<<< end of configuration section >>>
//-----------------以上是图形设置界面，可在Configuration Wizard界面设置-----------------
/*-----------------------------------数据类型宏定义--------------------------------*/
//#define fast_type      idata        //快速内存区。
//#define large_type     xdata        //大容量内存区。
typedef unsigned char  emu8;     //8位无符号型变量。
typedef unsigned short emu16;    //16位无符号型变量。
typedef unsigned long  emu32;    //32位无符号型变量。
/*------------------------------------状态机宏定义---------------------------------*/
#define ECBM_MODBUS_RTU_READY           0   //就绪态，此时可以接受一帧modbus数据。
#define ECBM_MODBUS_RTU_WAIT            1   //等待态，当发生错误之后，在此等待当前数据帧结束。
#define ECBM_MODBUS_RTU_ID_ERR          2   //ID错误，当接收到的ID不是本机也不是广播地址时会到这里。
#define ECBM_MODBUS_RTU_FUN_NUM         3   //读取功能码，在此读取功能码。
#define ECBM_MODBUS_RTU_FUN_NUM_ERR     4   //功能码错误，功能码原则上小于128，超过就会到这里。
#define ECBM_MODBUS_RTU_FUN_NUM_VOID    5   //功能码不存在，主要原因是本机不支持该功能码。
#define ECBM_MODBUS_RTU_ADDRH           6   //读取地址高8位。
#define ECBM_MODBUS_RTU_ADDRL           7   //读取地址低8位。
#define ECBM_MODBUS_RTU_DATA_COUNTH     8   //读取数据或者个数的高8位。
#define ECBM_MODBUS_RTU_DATA_COUNTL     9   //读取数据或者个数的低8位。
#define ECBM_MODBUS_RTU_COUNT_ERR       10  //读取个数错误。
#define ECBM_MODBUS_RTU_CRCH            11  //读取CRC的高8位。
#define ECBM_MODBUS_RTU_CRCL            12  //读取CRC的低8位。
#define ECBM_MODBUS_RTU_CRC_ERR         13  //CRC错误，当接收到的数据的CRC和传输的CRC不一致时会报错。
#define ECBM_MODBUS_RTU_DO              14  //处理态，在该状态下处理数据。
#define ECBM_MODBUS_RTU_DATAH           15  //读取地址高8位。
#define ECBM_MODBUS_RTU_DATAL           16  //读取地址低8位。
#define ECBM_MODBUS_RTU_BYTE_COUNT      17  //读取字节数量。
#define ECBM_MODBUS_RTU_BYTE_ERR        18  //读取字节数量对应不上。
/*-------------------------------------返回值定义----------------------------------*/
#define ECBM_MODBUS_RTU_OK              0   //正常。
#define ECBM_MODBUS_RTU_BIT_ADDR_ERR    1   //比特位地址错误。
#define ECBM_MODBUS_RTU_REG_ADDR_ERR    2   //寄存器地址错误。
#define ECBM_MODBUS_RTU_IO_ADDR_ERR     3   //IO地址错误。
/*-------------------------------220709增加使能脚控制功能----------------------------*/
#define ECBM_MODBUS_READ    0   //读使能
#define ECBM_MODBUS_WRITE   1   //写使能
/*--------------------------------------变量定义-----------------------------------*/
extern emu16  ecbm_modbus_rtu_crc;          //初始化CRC变量各位为1。
extern emu8   ecbm_modbus_rtu_status;       //状态机使用的变量。
extern emu8   ecbm_modbus_rtu_id;           //本机的设备ID。
extern emu8   ecbm_modbus_rtu_fun_code;     //功能码。
extern emu8   ecbm_modbus_rtu_fun_err_num;  //异常码。
extern emu16  ecbm_modbus_rtu_address;      //数据地址。
extern emu16  ecbm_modbus_rtu_data_count;   //数据/个数。
extern emu16  ecbm_modbus_rtu_crc_buf;      //CRC缓存。
extern emu16  ecbm_modbus_rtu_timeout;      //超时计算。
extern emu16  ecbm_modbus_rtu_uart_crc;     //CRC计算缓存。
extern emu8   ecbm_modbus_rtu_cmd_count;    //指令缓存计数。
extern emu8   ecbm_modbus_rtu_broadcast_en; //广播模式。

#if ECBM_MODBUS_RTU_INPUT_BIT_BUF_EN
extern emu8   ecbm_modbus_rtu_input_bit_buf[ECBM_MODBUS_RTU_INPUT_BIT_BUF_SIZE];        //输入bit寄存器存放变量。
#endif
#if ECBM_MODBUS_RTU_INPUT_REG_BUF_EN
extern emu16   ecbm_modbus_rtu_input_reg_buf[ECBM_MODBUS_RTU_INPUT_REG_BUF_SIZE];        //只读寄存器存放变量。
#endif

#if ECBM_MODBUS_RTU_BIT_BUF_EN
extern emu8   ecbm_modbus_rtu_bit_buf[ECBM_MODBUS_RTU_BIT_BUF_SIZE];//比特线圈存放变量。
#endif
#if ECBM_MODBUS_RTU_REG_BUF_EN
extern emu16  ecbm_modbus_rtu_reg_buf[ECBM_MODBUS_RTU_REG_BUF_SIZE];//寄存器存放变量。
#endif
#if ECBM_MODBUS_RTU_CMD10_EN
extern emu16  ecbm_modbus_rtu_cmd_buf[ECBM_MODBUS_RTU_BUF_SIZE];//指令缓存。
#endif
/*--------------------------------------函数定义-----------------------------------*/
/**
 * @brief 设置485芯片的读写IO脚,220709
 * 
 * @param dat 电平状态
 */
extern void ecbm_modbus_rtu_set_io(emu8 dat);
/*-------------------------------------------------------
函数名：ecbm_modbus_rtu_set_data
描  述：发送数据函数。需要用户移植。
输  入：
    dat     要发送的数据
输  出：无
返回值：无
创建者：奈特
调用例程：无
创建日期：2021-03-05
修改记录：
-------------------------------------------------------*/
//extern void ecbm_modbus_rtu_set_data(emu8 dat);
#include <stdbool.h>
extern void ecbm_modbus_rtu_set_data(emu8 dat, bool isend);
/*-------------------------------------------------------
函数名：ecbm_modbus_rtu_get_data
描  述：数据接收函数。需要用户移植。
输  入：无
输  出：无
返回值：接收到的数据。
创建者：奈特
调用例程：无
创建日期：2021-03-05
修改记录：
-------------------------------------------------------*/
extern emu8 ecbm_modbus_rtu_get_data(void);
/*-------------------------------------------------------
函数名：ECBM_MODBUS_RTU_TIMEOUT_RUN
描  述：走时宏定义，为了加快运行速度，所以做成了宏定义。需要用户移植。
输  入：无
输  出：无
返回值：无
创建者：奈特
调用例程：无
创建日期：2021-03-10
修改记录：
-------------------------------------------------------*/
#define ECBM_MODBUS_RTU_TIMEOUT_RUN()   \
do{\
    if(ecbm_modbus_rtu_timeout>0)/*不为0的时候才增加。*/\
        ecbm_modbus_rtu_timeout++;/*时间往上加。*/\
    if(ecbm_modbus_rtu_timeout>ECBM_MODBUS_RTU_TIME_MAX){/*超时的时候，*/\
        ecbm_modbus_rtu_status=ECBM_MODBUS_RTU_READY;/*回到就绪态。*/\
        ecbm_modbus_rtu_timeout=0;/*同时关闭时间累积。*/\
    }\
}while(0)
/*-------------------------------------------------------
函数名：ecbm_modbus_rtu_crc16
描  述：CRC16计算函数。
输  入：
    buf     需要计算CRC的数据
输  出：
    ecbm_modbus_rtu_crc     计算的结果
返回值：无
创建者：奈特
调用例程：无
创建日期：2021-03-10 
修改记录：
-------------------------------------------------------*/
extern void ecbm_modbus_rtu_crc16(emu8 buf);
/*-------------------------------------------------------
函数名：ecbm_modbus_rtu_check_fun_num
描  述：检测功能码函数。
输  入：
    fun_num     功能码
输  出：无
返回值：功能码存在返回本身，不存在返回0x80+功能码。
创建者：奈特
调用例程：无
创建日期：2021-03-05
修改记录：
-------------------------------------------------------*/
extern emu8 ecbm_modbus_rtu_check_fun_num(emu8 fun_num);
/*-------------------------------------------------------
函数名：ecbm_modbus_cmd_read_io_bit
描  述：读输入离散量函数。
输  入：
    addr    离散量地址
输  出：
    dat     比特值
返回值：读取结果
创建者：奈特
调用例程：无
创建日期：2021-03-15
修改记录：
-------------------------------------------------------*/
extern emu8 ecbm_modbus_cmd_read_io_bit(emu16 addr,emu8 * dat);
/*-------------------------------------------------------
函数名：ecbm_modbus_cmd_read_io_reg
描  述：读输入寄存器函数。
输  入：
    addr    寄存器地址
输  出：
    dat     寄存器值
返回值：读取结果
创建者：奈特
调用例程：无
创建日期：2021-03-15
修改记录：
-------------------------------------------------------*/
extern emu8 ecbm_modbus_cmd_read_io_reg(emu16 addr,emu16 * dat);
/*-------------------------------------------------------
函数名：ecbm_modbus_cmd_write_bit
描  述：写比特数据函数。
输  入：
    addr    比特位地址
    dat     比特值
输  出：无
返回值：无
创建者：奈特
调用例程：无
创建日期：2021-03-05
修改记录：
2021-03-16：取消返回值，优化运行速度。
-------------------------------------------------------*/
extern void ecbm_modbus_cmd_write_bit(emu16 addr,emu8 dat);
/*-------------------------------------------------------
函数名：ecbm_modbus_cmd_read_bit
描  述：读比特数据函数。
输  入：
    addr    比特位地址
输  出：
    dat     比特值
返回值：无
创建者：奈特
调用例程：无
创建日期：2021-03-05
修改记录：
2021-03-16：取消返回值，优化运行速度。
-------------------------------------------------------*/
extern void ecbm_modbus_cmd_read_bit(emu16 addr,emu8 * dat);
/*-------------------------------------------------------
函数名：ecbm_modbus_cmd_write_reg
描  述：写寄存器数据函数。
输  入：
    addr    寄存器地址
    dat     寄存器值
输  出：无
返回值：读取结果
创建者：奈特
调用例程：无
创建日期：2021-03-05
修改记录：
2021-03-16：取消返回值，优化运行速度。
-------------------------------------------------------*/
extern void ecbm_modbus_cmd_write_reg(emu16 addr,emu16 dat);
/*-------------------------------------------------------
函数名：ecbm_modbus_cmd_input_reg
描  述：读寄存器函数。
输  入：
    addr    寄存器地址
输  出：
    dat     寄存器值
返回值：读取结果
创建者：奈特
调用例程：无
创建日期：2021-03-05
修改记录：
2021-03-16：取消返回值，优化运行速度。
-------------------------------------------------------*/
extern void ecbm_modbus_cmd_input_reg(emu16 addr,emu16 * dat);
/*-------------------------------------------------------
函数名：ecbm_modbus_rtu_cmd_0x01
描  述：1号功能码处理函数。
输  入：无
输  出：无
返回值：无
创建者：奈特
调用例程：无
创建日期：2021-03-05
修改记录：
2021-03-09：新增CRC校验。
2021-03-15：修改了异常码02的触发条件。
2021-03-16：新增异常码04的触发条件。
-------------------------------------------------------*/
extern void ecbm_modbus_rtu_cmd_0x01(void);
/*-------------------------------------------------------
函数名：ecbm_modbus_rtu_cmd_0x02
描  述：2号功能码处理函数。
输  入：无
输  出：无
返回值：无
创建者：奈特
调用例程：无
创建日期：2021-03-15
修改记录：
-------------------------------------------------------*/
extern void ecbm_modbus_rtu_cmd_0x02(void);
/*-------------------------------------------------------
函数名：ecbm_modbus_rtu_cmd_0x03
描  述：3号功能码处理函数。
输  入：无
输  出：无
返回值：无
创建者：奈特
调用例程：无
创建日期：2021-03-05
修改记录：
2021-03-09：新增CRC校验。
-------------------------------------------------------*/
extern void ecbm_modbus_rtu_cmd_0x03(void);
/*-------------------------------------------------------
函数名：ecbm_modbus_rtu_cmd_0x05
描  述：5号功能码处理函数。
输  入：无
输  出：无
返回值：无
创建者：奈特
调用例程：无
创建日期：2021-03-05
修改记录：
2021-03-09：新增CRC校验。
2021-03-16：新增异常码04的触发条件，优化了部分执行代码。
-------------------------------------------------------*/
extern void ecbm_modbus_rtu_cmd_0x05(void);
/*-------------------------------------------------------
函数名：ecbm_modbus_rtu_cmd_0x06
描  述：6号功能码处理函数。
输  入：无
输  出：无
返回值：无
创建者：奈特
调用例程：无
创建日期：2021-03-05
修改记录：
2021-03-09：新增CRC校验。
-------------------------------------------------------*/
extern void ecbm_modbus_rtu_cmd_0x06(void);
/*-------------------------------------------------------
函数名：ecbm_modbus_rtu_cmd_0x10
描  述：16号功能码处理函数。
输  入：无
输  出：无
返回值：无
创建者：奈特
调用例程：无
创建日期：2021-03-12
修改记录：
-------------------------------------------------------*/
void ecbm_modbus_rtu_cmd_0x10(void);
/*-------------------------------------------------------
函数名：ecbm_modbus_rtu_receive
描  述：modbus接收处理函数。
输  入：无
输  出：无
返回值：无
创建者：奈特
调用例程：无
创建日期：2021-03-05
修改记录：
2021-03-09：新增CRC校验。
2021-03-12：新增0x10指令相关处理,新增0xff广播地址。
-------------------------------------------------------*/
extern void ecbm_modbus_rtu_receive(emu8 data);
/*-------------------------------------------------------
函数名：ecbm_modbus_rtu_run
描  述：modbus主循环处理函数。
输  入：无
输  出：无
返回值：无
创建者：奈特
调用例程：无
创建日期：2021-03-05
修改记录：
2021-03-09：新增CRC校验相关代码。
2021-03-12：新增0x10指令相关处理。
-------------------------------------------------------*/
extern void ecbm_modbus_rtu_run(void);
#endif 







