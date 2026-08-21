#include "modbus.h"
#include "stdio.h"
#include "stdbool.h"
#include "stdint.h"

extern void Modbus_Comand_Deal(uint32_t Comand_ID, uint32_t addr);

/*--------------------------------------变量定义-----------------------------------*/
emu16  ecbm_modbus_rtu_crc           =0xFFFF;                    //初始化CRC变量各位为1。
emu8   ecbm_modbus_rtu_status        =ECBM_MODBUS_RTU_READY;     //状态机使用的变量。
emu8   ecbm_modbus_rtu_id            =ECBM_MODBUS_RTU_ID_ADRESS; //本机的设备ID。
emu8   ecbm_modbus_rtu_fun_code      =0;                         //功能码。
emu8   ecbm_modbus_rtu_fun_err_num   =0;                         //异常码。
emu16  ecbm_modbus_rtu_address       =0;                         //数据地址。
emu16  ecbm_modbus_rtu_data_count    =0;                         //数据/个数。
emu16  ecbm_modbus_rtu_crc_buf       =0;                         //CRC缓存。
emu16  ecbm_modbus_rtu_timeout       =0;                         //超时计算。
emu16  ecbm_modbus_rtu_uart_crc      =0xFFFF;                    //CRC计算缓存。
emu8   ecbm_modbus_rtu_cmd_count     =0;                         //指令缓存计数。
emu8   ecbm_modbus_rtu_broadcast_en  =0;                         //广播模式。


#if ECBM_MODBUS_RTU_INPUT_BIT_BUF_EN
emu8   ecbm_modbus_rtu_input_bit_buf[ECBM_MODBUS_RTU_INPUT_BIT_BUF_SIZE] = {0,0,};        //输入bit寄存器存放变量。
#endif
#if ECBM_MODBUS_RTU_INPUT_REG_BUF_EN
emu16   ecbm_modbus_rtu_input_reg_buf[ECBM_MODBUS_RTU_INPUT_REG_BUF_SIZE] = {0,0,};        //只读寄存器存放变量。
#endif
#if ECBM_MODBUS_RTU_BIT_BUF_EN
emu8   ecbm_modbus_rtu_bit_buf[ECBM_MODBUS_RTU_BIT_BUF_SIZE];        //比特线圈存放变量。
#endif
#if ECBM_MODBUS_RTU_REG_BUF_EN
emu16  ecbm_modbus_rtu_reg_buf[ECBM_MODBUS_RTU_REG_BUF_SIZE] = {0};      //寄存器存放变量。  
#endif
#if ECBM_MODBUS_RTU_CMD10_EN
emu16  ecbm_modbus_rtu_cmd_buf[ECBM_MODBUS_RTU_BUF_SIZE];        //指令缓存。
#endif
/*-------------------------------------------------------
CRC16计算函数。
-------------------------------------------------------*/
void ecbm_modbus_rtu_crc16(emu8 buf){
    emu8 j;
    ecbm_modbus_rtu_crc^=buf;                                   //当前数据异或CRC低字节，在C51里可以直接处理。
    for(j=0;j<8;j++){                                           //计算每个字节的每一位。
        if(ecbm_modbus_rtu_crc&0x01){                           //判断右移前最低位是否为1。
            ecbm_modbus_rtu_crc=(ecbm_modbus_rtu_crc>>1)^0xA001;//如果为1则右移并异或表达式。
        }else{
            ecbm_modbus_rtu_crc>>=1;                            //否则直接右移一位。
        }           
    }
}
/*-------------------------------------------------------
检测功能码函数。
-------------------------------------------------------*/
emu8 ecbm_modbus_rtu_check_fun_num(emu8 fun_num){
    switch(fun_num){
/*-------------------------------------------------
0x02|读离散量输入  |物理离散量输入      |            |
----+-------------+------------------|             |
0x01|读线圈        |                  |             |
----+--------------|                 |   比特访问   |
0x05|写单个线圈     |内部比特或物理线圈 |             |
----+--------------|                 |             |
0x0F|写多个线圈     |                 |             |
----+--------------+-----------------+-------------|
0x04|读输入寄存器   |输入寄存器        |             |
----+--------------+-----------------|             |
0x03|读多个寄存器   |                 |             |
----+--------------|                 |             |
0x06|写单个寄存器   |                 |             |
----+--------------|                 |             |
0x10|写多个寄存器   |内部储存器或物理  |  16比特访问  |
----+--------------|输出寄存器        |             |
0x17|读/写多个寄存器|                 |             |
----+--------------|                 |             |
0x16|屏蔽写寄存器   |                 |             |
----+--------------|                 |             |
0x18|读FIFO队列     |                |             |
-------------------------------------------------*/
#if ECBM_MODBUS_RTU_CMD01_EN
        case 0x01://读线圈
#endif
#if ECBM_MODBUS_RTU_CMD02_EN
        case 0x02://读离散量输入
#endif
#if ECBM_MODBUS_RTU_CMD05_EN
        case 0x05://写单个线圈
#endif
#if ECBM_MODBUS_RTU_CMD04_EN
        case 0x04://读输入寄存器
#endif
#if ECBM_MODBUS_RTU_CMD03_EN
        case 0x03://读多个寄存器
#endif
#if ECBM_MODBUS_RTU_CMD06_EN
        case 0x06://写单个寄存器
#endif
#if ECBM_MODBUS_RTU_CMD10_EN
        case 0x10://写多个寄存器
#endif
#if ECBM_MODBUS_RTU_CMD_ALL_EN
        {return fun_num;};//支持的功能码就返回本身。
#endif
        default:{return 0x80+fun_num;};//不支持的功能码就返回0x80+功能码。
    }
}
/*-------------------------------------------------------
写比特数据函数。
-------------------------------------------------------*/
#if ECBM_MODBUS_RTU_BIT_BUF_EN
void ecbm_modbus_cmd_write_bit(emu16 addr,emu8 dat){
    emu8 temp;
    temp=addr%8;
    if(dat){
        ecbm_modbus_rtu_bit_buf[addr/8]|= (0x01<<temp);
    }else{
        ecbm_modbus_rtu_bit_buf[addr/8]&=~(0x01<<temp);
    }
}
#endif
/*-------------------------------------------------------
读比特数据函数。
-------------------------------------------------------*/
#if ECBM_MODBUS_RTU_BIT_BUF_EN
void ecbm_modbus_cmd_read_bit(emu16 addr,emu8 * dat){
    emu8 temp;
    temp=addr%8;
    if((ecbm_modbus_rtu_input_bit_buf[addr/8])&(0x01<<temp)){
        *dat=0x01;
    }else{
        *dat=0x00;
    }
}
#endif
/*-------------------------------------------------------
写寄存器数据函数。
-------------------------------------------------------*/
#if ECBM_MODBUS_RTU_REG_BUF_EN
void ecbm_modbus_cmd_write_reg(emu16 addr,emu16 dat){
    ecbm_modbus_rtu_reg_buf[addr]=dat;
}
#endif
/*-------------------------------------------------------
读寄存器函数。
-------------------------------------------------------*/
#if ECBM_MODBUS_RTU_REG_BUF_EN
void ecbm_modbus_cmd_input_reg(emu16 addr,emu16 * dat){
    *dat=ecbm_modbus_rtu_reg_buf[addr];
}
#endif
/*读 输入寄存器函数*/
#if ECBM_MODBUS_RTU_INPUT_REG_BUF_EN
emu8 ecbm_modbus_cmd_read_io_reg(emu16 addr,emu16 * dat)
{
    * dat = ecbm_modbus_rtu_input_reg_buf[addr];
    return * dat;
}
#endif

/*读 输入寄存器bit*/
#if ECBM_MODBUS_RTU_INPUT_BIT_BUF_EN
emu8 ecbm_modbus_cmd_read_io_bit(emu16 addr,emu8 * dat)
{
    emu8 temp;
    temp=addr%8;
    if((ecbm_modbus_rtu_input_bit_buf[addr/8])&(0x01<<temp)){
        *dat=0x01;
    }else{
        *dat=0x00;
    }
    return *dat;
}

#endif
/*-------------------------------------------------------
1号功能码处理函数。
-------------------------------------------------------*/
#if ECBM_MODBUS_RTU_CMD01_EN
void ecbm_modbus_rtu_cmd_0x01(void){
    emu8 temp8,t1,c1,i,j,qw;
    emu16 temp16;
    ecbm_modbus_rtu_crc= 0xFFFF;
    if((ecbm_modbus_rtu_data_count>=1)&&(ecbm_modbus_rtu_data_count<=0x07D0)){      //读取的个数要在1个以上，且不能大于2000。
        if(((emu32)ecbm_modbus_rtu_data_count+(emu32)ecbm_modbus_rtu_address)<=65536UL){   //判断地址+个数有没有超限。
            #if ECBM_MODBUS_RTU_BIT_BUF_EN
            if(                                                                        //遇到以下问题，说明写入肯定会失败。
                (ecbm_modbus_rtu_address<ECBM_MODBUS_RTU_BIT_START_ADDR)||             //输入地址比起始地址还小的，
                (ecbm_modbus_rtu_address+ecbm_modbus_rtu_data_count)>(ECBM_MODBUS_RTU_BIT_BUF_SIZE*8+ECBM_MODBUS_RTU_BIT_START_ADDR)//输入的地址+输入的数量超过本机定义的数量。
            ){
                ecbm_modbus_rtu_fun_err_num=0x04;                       //记录异常码04。
                if(ecbm_modbus_rtu_broadcast_en){
                    ecbm_modbus_rtu_crc16(ecbm_modbus_rtu_id);              //计算CRC，下同。
                    ecbm_modbus_rtu_crc16(0x81);
                    ecbm_modbus_rtu_crc16(0x04);
                    ecbm_modbus_rtu_set_data(ecbm_modbus_rtu_id, false);           //发送地址。
                    ecbm_modbus_rtu_set_data(0x81, false);                         //发送0x80+功能码回复。
                    ecbm_modbus_rtu_set_data(0x04, false);                         //异常码04。
                    ecbm_modbus_rtu_set_data((emu8)(ecbm_modbus_rtu_crc), false);    //发送CRC。
                    ecbm_modbus_rtu_set_data((emu8)(ecbm_modbus_rtu_crc>>8), true);
                }
            #else
            if(0){  //如果是不用内置数组缓存的话，只要地址在范围内，无论是多少都是被允许的。
            #endif
            }else{
                if(ecbm_modbus_rtu_broadcast_en){
                    t1=(emu8)(ecbm_modbus_rtu_data_count/8);      //统计要读取的数据能凑够多少个字节。
                    if(ecbm_modbus_rtu_data_count%8)t1++;       //剩下的不足8位的数据也要一个字节来传输。
                    #if ECBM_MODBUS_RTU_BIT_BUF_EN
                    temp16=ecbm_modbus_rtu_address-ECBM_MODBUS_RTU_BIT_START_ADDR;             //获取数据起始地址。
                    #else
                    temp16=ecbm_modbus_rtu_address;             //获取数据起始地址。
                    #endif
                    ecbm_modbus_rtu_crc16(ecbm_modbus_rtu_id);  //计算CRC，下同。
                    ecbm_modbus_rtu_crc16(0x01);
                    ecbm_modbus_rtu_crc16(t1);
                    ecbm_modbus_rtu_set_data(ecbm_modbus_rtu_id, false);//发送地址。
                    ecbm_modbus_rtu_set_data(0x01, false);             //发送功能码回复。
                    ecbm_modbus_rtu_set_data(t1, false);               //发送数据长度。
                    for(i=0;i<t1;i++){                          //从起始位置开始组装数据。
                        temp8=0;                                //清零变量。
                        if(ecbm_modbus_rtu_data_count>=8){      //如果剩余要读的数据多于8个比特。
                            c1=8;                               //那么本轮循环先读8比特数据。
                        }else{                                  //否则，
                            c1=(emu8)ecbm_modbus_rtu_data_count;  //就有多少读多少。
                        }
                        for(j=0;j<c1;j++){      
                            ecbm_modbus_cmd_read_bit(temp16++,&qw);//将地址送到读取函数中，获取对应的比特值。
                            temp8|=(qw<<j);                     //组装数据。
                        }
                        ecbm_modbus_rtu_crc16(temp8);           //计算CRC。
                        ecbm_modbus_rtu_set_data(temp8, false);        //发送组装好的数据。
                        ecbm_modbus_rtu_data_count-=(emu16)c1;    //计算剩下要读的数据个数。
                    }
                    ecbm_modbus_rtu_set_data((emu8)(ecbm_modbus_rtu_crc), false);
                    ecbm_modbus_rtu_set_data((emu8)(ecbm_modbus_rtu_crc>>8), true);
                }
            }
        }else{                                              //如果地址+个数超过2000。
            ecbm_modbus_rtu_fun_err_num=0x02;               //记录异常码02。
            if(ecbm_modbus_rtu_broadcast_en){
                ecbm_modbus_rtu_crc16(ecbm_modbus_rtu_id);      //计算CRC，下同。
                ecbm_modbus_rtu_crc16(0x81);
                ecbm_modbus_rtu_crc16(0x02);
                ecbm_modbus_rtu_set_data(ecbm_modbus_rtu_id, false);   //发送地址。
                ecbm_modbus_rtu_set_data(0x81, false);                 //发送0x80+功能码回复。
                ecbm_modbus_rtu_set_data(0x02, false);                 //异常码02。
                ecbm_modbus_rtu_set_data((emu8)(ecbm_modbus_rtu_crc), false);//发送CRC。
                ecbm_modbus_rtu_set_data((emu8)(ecbm_modbus_rtu_crc>>8), true);
            }
        }
    }else{                                                  //如果个数本身就超过2000了。
        ecbm_modbus_rtu_fun_err_num=0x03;                   //记录异常码03。
        if(ecbm_modbus_rtu_broadcast_en){	
            ecbm_modbus_rtu_crc16(ecbm_modbus_rtu_id);          //计算CRC，下同。
            ecbm_modbus_rtu_crc16(0x81);
            ecbm_modbus_rtu_crc16(0x03);
            ecbm_modbus_rtu_set_data(ecbm_modbus_rtu_id, false);       //发送地址。
            ecbm_modbus_rtu_set_data(0x81, false);                     //发送0x80+功能码回复。
            ecbm_modbus_rtu_set_data(0x03, false);                     //异常码03
            ecbm_modbus_rtu_set_data((emu8)(ecbm_modbus_rtu_crc), false);//发送CRC。
            ecbm_modbus_rtu_set_data((emu8)(ecbm_modbus_rtu_crc>>8), true);
        }
    }
}
#endif
/*-------------------------------------------------------
2号功能码处理函数。
-------------------------------------------------------*/
#if ECBM_MODBUS_RTU_CMD02_EN
void ecbm_modbus_rtu_cmd_0x02(void){
    emu8 temp8,t1,c1,i,j,qw;
    emu16 temp16;
    ecbm_modbus_rtu_crc= 0xFFFF;
    if((ecbm_modbus_rtu_data_count>=1)&&(ecbm_modbus_rtu_data_count<=0x07D0)){      //读取的个数要在1个以上，且地址不能大于2000。
        if(((emu32)ecbm_modbus_rtu_data_count+(emu32)ecbm_modbus_rtu_address)<=65536UL){   //判断地址+个数有没有超限。
           if((ecbm_modbus_rtu_address<ECBM_MODBUS_RTU_INPUT_BIT_START_ADDR)||             //输入地址比起始地址还小的，
                (ecbm_modbus_rtu_address+ecbm_modbus_rtu_data_count)>(ECBM_MODBUS_RTU_INPUT_BIT_BUF_SIZE*8+ECBM_MODBUS_RTU_INPUT_BIT_START_ADDR)){//输入的地址+输入的数量超过本机定义的数量。
               ecbm_modbus_rtu_fun_err_num=0x04;                       //记录异常码04。
               if(ecbm_modbus_rtu_broadcast_en){
                   ecbm_modbus_rtu_crc16(ecbm_modbus_rtu_id);              //计算CRC，下同。
                   ecbm_modbus_rtu_crc16(0x82);
                   ecbm_modbus_rtu_crc16(0x04);
                   ecbm_modbus_rtu_set_data(ecbm_modbus_rtu_id, false);           //发送地址。
                   ecbm_modbus_rtu_set_data(0x82, false);                         //发送0x80+功能码回复。
                   ecbm_modbus_rtu_set_data(0x04, false);                         //异常码04。
                   ecbm_modbus_rtu_set_data((emu8)(ecbm_modbus_rtu_crc), false);    //发送CRC。
                   ecbm_modbus_rtu_set_data((emu8)(ecbm_modbus_rtu_crc>>8), true);
               }
           }else{
                if(ecbm_modbus_rtu_broadcast_en){
                    t1=(emu8)(ecbm_modbus_rtu_data_count/8);      //统计要读取的数据能凑够多少个字节。
                    if(ecbm_modbus_rtu_data_count%8)t1++;       //剩下的不足8位的数据也要一个字节来传输。
                    temp16=ecbm_modbus_rtu_address-ECBM_MODBUS_RTU_INPUT_BIT_START_ADDR;
                    ecbm_modbus_rtu_crc16(ecbm_modbus_rtu_id);  //计算CRC，下同。
                    ecbm_modbus_rtu_crc16(0x02);
                    ecbm_modbus_rtu_crc16(t1);
                    ecbm_modbus_rtu_set_data(ecbm_modbus_rtu_id, false);//发送地址。
                    ecbm_modbus_rtu_set_data(0x02, false);             //发送功能码回复。
                    ecbm_modbus_rtu_set_data(t1, false);               //发送数据长度。
                    for(i=0;i<t1;i++){                          //从起始位置开始组装数据。
                        temp8=0;                                //清零变量。
                        if(ecbm_modbus_rtu_data_count>=8){      //如果剩余要读的数据多于8个比特。
                            c1=8;                               //那么本轮循环先读8比特数据。
                        }else{                                  //否则，
                            c1=(emu8)ecbm_modbus_rtu_data_count;  //就有多少读多少。
                        }
                        for(j=0;j<c1;j++){      
                            ecbm_modbus_cmd_read_io_bit(temp16++,&qw);//将地址送到读取函数中，获取对应的比特值。
                            temp8|=(qw<<j);                     //组装数据。
                        }
                        ecbm_modbus_rtu_crc16(temp8);           //计算CRC。
                        ecbm_modbus_rtu_set_data(temp8, false);        //发送组装好的数据。
                        ecbm_modbus_rtu_data_count-=(emu16)c1;    //计算剩下要读的数据个数。
                    }
                    ecbm_modbus_rtu_set_data((emu8)(ecbm_modbus_rtu_crc), false);
                    ecbm_modbus_rtu_set_data((emu8)(ecbm_modbus_rtu_crc>>8), true);
                }
          }
        }else{                                              //如果地址+个数超过2000。
            ecbm_modbus_rtu_fun_err_num=0x02;               //记录异常码02。
            if(ecbm_modbus_rtu_broadcast_en){
                ecbm_modbus_rtu_crc16(ecbm_modbus_rtu_id);      //计算CRC，下同。
                ecbm_modbus_rtu_crc16(0x82);
                ecbm_modbus_rtu_crc16(0x02);
                ecbm_modbus_rtu_set_data(ecbm_modbus_rtu_id, false);   //发送地址。
                ecbm_modbus_rtu_set_data(0x82, false);                 //发送0x80+功能码回复。
                ecbm_modbus_rtu_set_data(0x02, false);                 //异常码02。
                ecbm_modbus_rtu_set_data((emu8)(ecbm_modbus_rtu_crc), false);//发送CRC。
                ecbm_modbus_rtu_set_data((emu8)(ecbm_modbus_rtu_crc>>8), true);
            }
        }
    }else{                                                  //如果个数本身就超过2000了。
        ecbm_modbus_rtu_fun_err_num=0x03;                   //记录异常码03。
        if(ecbm_modbus_rtu_broadcast_en){	
            ecbm_modbus_rtu_crc16(ecbm_modbus_rtu_id);          //计算CRC，下同。
            ecbm_modbus_rtu_crc16(0x82);
            ecbm_modbus_rtu_crc16(0x03);
            ecbm_modbus_rtu_set_data(ecbm_modbus_rtu_id, false);       //发送地址。
            ecbm_modbus_rtu_set_data(0x82, false);                     //发送0x80+功能码回复。
            ecbm_modbus_rtu_set_data(0x03, false);                     //异常码03
            ecbm_modbus_rtu_set_data((emu8)(ecbm_modbus_rtu_crc), false);//发送CRC。
            ecbm_modbus_rtu_set_data((emu8)(ecbm_modbus_rtu_crc>>8), true);
        }
    }
}
#endif
/*-------------------------------------------------------
3号功能码处理函数。
-------------------------------------------------------*/
#if ECBM_MODBUS_RTU_CMD03_EN
void ecbm_modbus_rtu_cmd_0x03(void){
    emu16 temp16,i;
    ecbm_modbus_rtu_crc=0xffff;
    if((ecbm_modbus_rtu_data_count>=1)&&(ecbm_modbus_rtu_data_count<=0x007D)){          //读取的个数要在1个以上，且地址不能大于125。
        if(((emu32)ecbm_modbus_rtu_data_count+(emu32)ecbm_modbus_rtu_address)<65536UL){     //判断地址+个数有没有超限。
            #if ECBM_MODBUS_RTU_REG_BUF_EN
            //printf("ECBM_MODBUS_RTU_REG_START_ADDR_REAL: %d\r\n", ECBM_MODBUS_RTU_REG_START_ADDR_REAL);
            if(                                                                        //遇到以下问题，说明写入肯定会失败。
                ((ecbm_modbus_rtu_address<ECBM_MODBUS_RTU_REG_START_ADDR))||             //输入地址比起始地址还小的，
                (ecbm_modbus_rtu_address+ecbm_modbus_rtu_data_count)>(ECBM_MODBUS_RTU_REG_BUF_SIZE+ECBM_MODBUS_RTU_REG_START_ADDR)//输入的地址+输入的数量超过本机定义的数量。
            ){
                ecbm_modbus_rtu_fun_err_num=0x04;           //记录异常码04。
                if(ecbm_modbus_rtu_broadcast_en){
                    ecbm_modbus_rtu_crc16(ecbm_modbus_rtu_id);  //计算CRC，下同。
                    ecbm_modbus_rtu_crc16(0x83);
                    ecbm_modbus_rtu_crc16(0x04);
                    ecbm_modbus_rtu_set_data(ecbm_modbus_rtu_id, false);//发送地址。
                    ecbm_modbus_rtu_set_data(0x83, false);             //发送0x80+功能码回复。
                    ecbm_modbus_rtu_set_data(0x04, false);             //异常码04。
                    ecbm_modbus_rtu_set_data((emu8)(ecbm_modbus_rtu_crc), false);//发送CRC。
                    ecbm_modbus_rtu_set_data((emu8)(ecbm_modbus_rtu_crc>>8), true);
                }
            #else
            //if(0){
            #endif
            }else{
                if(ecbm_modbus_rtu_broadcast_en){
                    ecbm_modbus_rtu_crc16(ecbm_modbus_rtu_id);  //计算CRC，下同。
                    ecbm_modbus_rtu_crc16(0x03);
                    ecbm_modbus_rtu_crc16(ecbm_modbus_rtu_data_count*2);
                    ecbm_modbus_rtu_set_data(ecbm_modbus_rtu_id, false);//发送地址。
                    ecbm_modbus_rtu_set_data(0x03, false);             //发送功能码回复。
                    ecbm_modbus_rtu_set_data(ecbm_modbus_rtu_data_count*2, false);//发送回复字节数。
                    #if ECBM_MODBUS_RTU_REG_BUF_EN
                        ecbm_modbus_rtu_address-=ECBM_MODBUS_RTU_REG_START_ADDR;
                    #endif
                    for(i=0;i<ecbm_modbus_rtu_data_count;i++){
                        ecbm_modbus_cmd_input_reg(ecbm_modbus_rtu_address+i,&temp16);//从寄存器变量里把数据取出来。
                        ecbm_modbus_rtu_crc16((emu8)(temp16>>8)); //计算CRC，下同。
                        ecbm_modbus_rtu_crc16((emu8)(temp16));
                        ecbm_modbus_rtu_set_data((emu8)(temp16>>8), false);//发送数据。
                        ecbm_modbus_rtu_set_data((emu8)(temp16), false);
                    }
                    ecbm_modbus_rtu_set_data((emu8)(ecbm_modbus_rtu_crc), false);//发送CRC。
                    ecbm_modbus_rtu_set_data((emu8)(ecbm_modbus_rtu_crc>>8), true);
                }else{
                    /*如果是收到广播*/
                    ecbm_modbus_rtu_crc16(ecbm_modbus_rtu_id);  //计算CRC，下同。
                    ecbm_modbus_rtu_crc16(0x03);
                    ecbm_modbus_rtu_crc16(ecbm_modbus_rtu_data_count*2);
                    ecbm_modbus_rtu_set_data(ecbm_modbus_rtu_id, false);//发送地址。
                    ecbm_modbus_rtu_set_data(0x03, false);             //发送功能码回复。
                    ecbm_modbus_rtu_set_data(ecbm_modbus_rtu_data_count*2, false);//发送回复字节数。
                    #if ECBM_MODBUS_RTU_REG_BUF_EN
                        ecbm_modbus_rtu_address-=ECBM_MODBUS_RTU_REG_START_ADDR;
                    #endif
                    for(i=0;i<ecbm_modbus_rtu_data_count;i++){
                        ecbm_modbus_cmd_input_reg(ecbm_modbus_rtu_address+i,&temp16);//从寄存器变量里把数据取出来。
                        ecbm_modbus_rtu_crc16((emu8)(temp16>>8)); //计算CRC，下同。
                        ecbm_modbus_rtu_crc16((emu8)(temp16));
                        ecbm_modbus_rtu_set_data((emu8)(temp16>>8), false);//发送数据。
                        ecbm_modbus_rtu_set_data((emu8)(temp16), false);
                    }
                    ecbm_modbus_rtu_set_data((emu8)(ecbm_modbus_rtu_crc), false);//发送CRC。
                    ecbm_modbus_rtu_set_data((emu8)(ecbm_modbus_rtu_crc>>8), true);
                }
            }
        }else{
            ecbm_modbus_rtu_fun_err_num=0x02;           //记录异常码02。
            if(ecbm_modbus_rtu_broadcast_en){
                ecbm_modbus_rtu_crc16(ecbm_modbus_rtu_id);  //计算CRC，下同。
                ecbm_modbus_rtu_crc16(0x83);
                ecbm_modbus_rtu_crc16(0x02);
                ecbm_modbus_rtu_set_data(ecbm_modbus_rtu_id, false);//发送地址。
                ecbm_modbus_rtu_set_data(0x83, false);              //发送0x80+功能码回复。
                ecbm_modbus_rtu_set_data(0x02, false);              //异常码02。
                ecbm_modbus_rtu_set_data((emu8)(ecbm_modbus_rtu_crc), false);//发送CRC。
                ecbm_modbus_rtu_set_data((emu8)(ecbm_modbus_rtu_crc>>8), true);
            }
        }
    }else{                                              //如果个数本身就超过125了。
        ecbm_modbus_rtu_fun_err_num=0x03;               //记录异常码03。	
        if(ecbm_modbus_rtu_broadcast_en){
            ecbm_modbus_rtu_crc16(ecbm_modbus_rtu_id);      //计算CRC，下同。
            ecbm_modbus_rtu_crc16(0x83);
            ecbm_modbus_rtu_crc16(0x03);
            ecbm_modbus_rtu_set_data(ecbm_modbus_rtu_id, false);   //发送地址。
            ecbm_modbus_rtu_set_data(0x83, false);                 //发送0x80+功能码回复。
            ecbm_modbus_rtu_set_data(0x03, false);                 //异常码03。
            ecbm_modbus_rtu_set_data((emu8)(ecbm_modbus_rtu_crc), false);//发送CRC。
            ecbm_modbus_rtu_set_data((emu8)(ecbm_modbus_rtu_crc>>8), true);
        }
    }
}
#endif
/*-------------------------------------------------------
4号功能码处理函数。
-------------------------------------------------------*/
#if ECBM_MODBUS_RTU_CMD04_EN
void ecbm_modbus_rtu_cmd_0x04(void){
    emu16 temp16,i;
    ecbm_modbus_rtu_crc=0xffff;
    if((ecbm_modbus_rtu_data_count>=1)&&(ecbm_modbus_rtu_data_count<=0x007D)){          //读取的个数要在1个以上，且地址不能大于125。
        if(((emu32)ecbm_modbus_rtu_data_count+(emu32)ecbm_modbus_rtu_address)<65536UL){     //判断地址+个数有没有超限。
            if((ecbm_modbus_rtu_address<ECBM_MODBUS_RTU_INPUT_REG_START_ADDR)||             //输入地址比起始地址还小的，
                (ecbm_modbus_rtu_address+ecbm_modbus_rtu_data_count)>(ECBM_MODBUS_RTU_INPUT_REG_BUF_SIZE+ECBM_MODBUS_RTU_INPUT_REG_START_ADDR)){//输入的地址+输入的数量超过本机定义的数量。
                ecbm_modbus_rtu_fun_err_num=0x04;           //记录异常码04。
                if(ecbm_modbus_rtu_broadcast_en){
                    ecbm_modbus_rtu_crc16(ecbm_modbus_rtu_id);  //计算CRC，下同。
                    ecbm_modbus_rtu_crc16(0x84);
                    ecbm_modbus_rtu_crc16(0x04);
                    ecbm_modbus_rtu_set_data(ecbm_modbus_rtu_id, false);//发送地址。
                    ecbm_modbus_rtu_set_data(0x84, false);             //发送0x80+功能码回复。
                    ecbm_modbus_rtu_set_data(0x04, false);             //异常码04。
                    ecbm_modbus_rtu_set_data((emu8)(ecbm_modbus_rtu_crc), false);//发送CRC。
                    ecbm_modbus_rtu_set_data((emu8)(ecbm_modbus_rtu_crc>>8), true);
                }
            }else{
                if(ecbm_modbus_rtu_broadcast_en){
                    ecbm_modbus_rtu_crc16(ecbm_modbus_rtu_id);  //计算CRC，下同。
                    ecbm_modbus_rtu_crc16(0x04);
                    ecbm_modbus_rtu_crc16(ecbm_modbus_rtu_data_count*2);
                    ecbm_modbus_rtu_set_data(ecbm_modbus_rtu_id, false);//发送地址。
                    ecbm_modbus_rtu_set_data(0x04, false);             //发送功能码回复。
                    ecbm_modbus_rtu_set_data(ecbm_modbus_rtu_data_count*2, false);//发送回复字节数。
                    ecbm_modbus_rtu_address-=ECBM_MODBUS_RTU_INPUT_REG_START_ADDR;
                    for(i=0;i<ecbm_modbus_rtu_data_count;i++){
                        ecbm_modbus_cmd_read_io_reg(ecbm_modbus_rtu_address+i,&temp16);//从寄存器变量里把数据取出来。
                        ecbm_modbus_rtu_crc16((emu8)(temp16>>8)); //计算CRC，下同。
                        ecbm_modbus_rtu_crc16((emu8)(temp16));
                        ecbm_modbus_rtu_set_data((emu8)(temp16>>8), false);//发送数据。
                        ecbm_modbus_rtu_set_data((emu8)(temp16), false);
                    }
                    ecbm_modbus_rtu_set_data((emu8)(ecbm_modbus_rtu_crc), false);//发送CRC。
                    ecbm_modbus_rtu_set_data((emu8)(ecbm_modbus_rtu_crc>>8), true);
                }
            }
        }else{
            ecbm_modbus_rtu_fun_err_num=0x02;           //记录异常码02。
            if(ecbm_modbus_rtu_broadcast_en){
                ecbm_modbus_rtu_crc16(ecbm_modbus_rtu_id);  //计算CRC，下同。
                ecbm_modbus_rtu_crc16(0x84);
                ecbm_modbus_rtu_crc16(0x02);
                ecbm_modbus_rtu_set_data(ecbm_modbus_rtu_id, false);//发送地址。
                ecbm_modbus_rtu_set_data(0x84, false);              //发送0x80+功能码回复。
                ecbm_modbus_rtu_set_data(0x02, false);              //异常码02。
                ecbm_modbus_rtu_set_data((emu8)(ecbm_modbus_rtu_crc), false);//发送CRC。
                ecbm_modbus_rtu_set_data((emu8)(ecbm_modbus_rtu_crc>>8), true);
            }
        }
    }else{                                              //如果个数本身就超过125了。
        ecbm_modbus_rtu_fun_err_num=0x03;               //记录异常码03。	
        if(ecbm_modbus_rtu_broadcast_en){
            ecbm_modbus_rtu_crc16(ecbm_modbus_rtu_id);      //计算CRC，下同。
            ecbm_modbus_rtu_crc16(0x84);
            ecbm_modbus_rtu_crc16(0x03);
            ecbm_modbus_rtu_set_data(ecbm_modbus_rtu_id, false);   //发送地址。
            ecbm_modbus_rtu_set_data(0x84, false);                 //发送0x80+功能码回复。
            ecbm_modbus_rtu_set_data(0x03, false);                 //异常码03。
            ecbm_modbus_rtu_set_data((emu8)(ecbm_modbus_rtu_crc), false);//发送CRC。
            ecbm_modbus_rtu_set_data((emu8)(ecbm_modbus_rtu_crc>>8), true);
        }
    }
}
#endif
/*-------------------------------------------------------
5号功能码处理函数。
-------------------------------------------------------*/
#if ECBM_MODBUS_RTU_CMD05_EN
void ecbm_modbus_rtu_cmd_0x05(void){
    ecbm_modbus_rtu_crc=0xffff;
    if((ecbm_modbus_rtu_data_count==0x0000)||(ecbm_modbus_rtu_data_count==0xFF00)){//判断输出值是否合法。
//      if((ecbm_modbus_rtu_address>=0)&&(ecbm_modbus_rtu_address<=65535)){//其实能传进来的地址都符合这个范围，所以在这里优化掉了。
            #if ECBM_MODBUS_RTU_BIT_BUF_EN
            if(                                                                        //遇到以下问题，说明写入肯定会失败。
                (ecbm_modbus_rtu_address<ECBM_MODBUS_RTU_BIT_START_ADDR)||             //输入地址比起始地址还小的，
                (ecbm_modbus_rtu_address>=(ECBM_MODBUS_RTU_BIT_BUF_SIZE*8+ECBM_MODBUS_RTU_BIT_START_ADDR))//输入地址比总的设计地址还大的，
            ){
                if(ecbm_modbus_rtu_broadcast_en){
                    ecbm_modbus_rtu_fun_err_num=0x04;                           //记录异常码04。
                    ecbm_modbus_rtu_crc16(ecbm_modbus_rtu_id);                  //计算CRC，下同。
                    ecbm_modbus_rtu_crc16(0x85);
                    ecbm_modbus_rtu_crc16(0x04);
                    ecbm_modbus_rtu_set_data(ecbm_modbus_rtu_id, false);               //发送地址。
                    ecbm_modbus_rtu_set_data(0x85, false);                             //发送0x80+功能码回复。
                    ecbm_modbus_rtu_set_data(0x04, false);                             //异常码04。
                    ecbm_modbus_rtu_set_data((emu8)(ecbm_modbus_rtu_crc), false);        //发送CRC。
                    ecbm_modbus_rtu_set_data((emu8)(ecbm_modbus_rtu_crc>>8), true);
                }
            #else
            if(0){  //如果是不用内置数组缓存的话，只要地址在范围内，无论是多少都是被允许的。
            #endif
            }else{
                #if ECBM_MODBUS_RTU_BIT_BUF_EN
                ecbm_modbus_cmd_write_bit(ecbm_modbus_rtu_address-ECBM_MODBUS_RTU_BIT_START_ADDR,(emu8)(ecbm_modbus_rtu_data_count>>8));
                #else
                ecbm_modbus_cmd_write_bit(ecbm_modbus_rtu_address,(emu8)(ecbm_modbus_rtu_data_count>>8));
                #endif
                if(ecbm_modbus_rtu_broadcast_en){
                    ecbm_modbus_rtu_crc16(ecbm_modbus_rtu_id);                  //计算CRC，下同。
                    ecbm_modbus_rtu_crc16(0x05);
                    ecbm_modbus_rtu_crc16((emu8)(ecbm_modbus_rtu_address>>8));
                    ecbm_modbus_rtu_crc16((emu8)(ecbm_modbus_rtu_address));
                    ecbm_modbus_rtu_crc16((emu8)(ecbm_modbus_rtu_data_count>>8));
                    ecbm_modbus_rtu_crc16((emu8)(ecbm_modbus_rtu_data_count));
                    ecbm_modbus_rtu_set_data(ecbm_modbus_rtu_id, false);               //发送地址。
                    ecbm_modbus_rtu_set_data(0x05, false);                             //发送功能码回复。
                    ecbm_modbus_rtu_set_data((emu8)(ecbm_modbus_rtu_address>>8), false); //发送地址回复。
                    ecbm_modbus_rtu_set_data((emu8)(ecbm_modbus_rtu_address), false);
                    ecbm_modbus_rtu_set_data((emu8)(ecbm_modbus_rtu_data_count>>8), false);//发送计数回复。
                    ecbm_modbus_rtu_set_data((emu8)(ecbm_modbus_rtu_data_count), false);
                    ecbm_modbus_rtu_set_data((emu8)(ecbm_modbus_rtu_crc), false);        //发送CRC。
                    ecbm_modbus_rtu_set_data((emu8)(ecbm_modbus_rtu_crc>>8), true);
                }
            }
//      }else{
//          ecbm_modbus_rtu_fun_err_num=0x02;                   //异常码02。	
//          if(ecbm_modbus_rtu_broadcast_en){
//              ecbm_modbus_rtu_crc16(ecbm_modbus_rtu_id);          //计算CRC，下同。
//              ecbm_modbus_rtu_crc16(0x85);
//              ecbm_modbus_rtu_crc16(0x02);
//              ecbm_modbus_rtu_set_data(ecbm_modbus_rtu_id);       //发送地址。
//              ecbm_modbus_rtu_set_data(0x85);                     //发送0x80+功能码回复。
//              ecbm_modbus_rtu_set_data(0x02);                     //异常码02。
//              ecbm_modbus_rtu_set_data((emu8)(ecbm_modbus_rtu_crc));//发送CRC。
//              ecbm_modbus_rtu_set_data((emu8)(ecbm_modbus_rtu_crc>>8));
//          }
//      }
    }else{                                                  //如果不合法。
        ecbm_modbus_rtu_fun_err_num=0x03;                   //记录异常码03。	
        if(ecbm_modbus_rtu_broadcast_en){
            ecbm_modbus_rtu_crc16(ecbm_modbus_rtu_id);          //计算CRC，下同。
            ecbm_modbus_rtu_crc16(0x85);
            ecbm_modbus_rtu_crc16(0x03);
            ecbm_modbus_rtu_set_data(ecbm_modbus_rtu_id, false);       //发送地址。
            ecbm_modbus_rtu_set_data(0x85, false);                     //发送0x80+功能码回复。
            ecbm_modbus_rtu_set_data(0x03, false);                     //异常码03
            ecbm_modbus_rtu_set_data((emu8)(ecbm_modbus_rtu_crc), false);//发送CRC。
            ecbm_modbus_rtu_set_data((emu8)(ecbm_modbus_rtu_crc>>8), true);
        }
    }
}
#endif
/*-------------------------------------------------------
6号功能码处理函数。
-------------------------------------------------------*/
#if ECBM_MODBUS_RTU_CMD06_EN
void ecbm_modbus_rtu_cmd_0x06(void){
    ecbm_modbus_rtu_crc=0xffff;
//  if((ecbm_modbus_rtu_data_count>=0x0000)&&(ecbm_modbus_rtu_data_count<=0xFFFF)){//其实能传进来的数据都符合这个范围，所以在这里优化掉了。
//      if((ecbm_modbus_rtu_address>=0)&&(ecbm_modbus_rtu_address<=65536UL)){//其实能传进来的地址都符合这个范围，所以在这里优化掉了。
            #if ECBM_MODBUS_RTU_REG_BUF_EN
            if(                                                                        //遇到以下问题，说明写入肯定会失败。
                (ecbm_modbus_rtu_address<ECBM_MODBUS_RTU_REG_START_ADDR)||             //输入地址比起始地址还小的，
                (ecbm_modbus_rtu_address)>=(ECBM_MODBUS_RTU_REG_BUF_SIZE+ECBM_MODBUS_RTU_REG_START_ADDR)//输入的地址+输入的数量超过本机定义的数量。
            ){
                if(ecbm_modbus_rtu_broadcast_en){
                    ecbm_modbus_rtu_fun_err_num=0x04;               //记录异常码04。
                    ecbm_modbus_rtu_crc16(ecbm_modbus_rtu_id);      //计算CRC，下同。
                    ecbm_modbus_rtu_crc16(0x86);
                    ecbm_modbus_rtu_crc16(0x04);
                    ecbm_modbus_rtu_set_data(ecbm_modbus_rtu_id, false);   //发送地址。
                    ecbm_modbus_rtu_set_data(0x86, false);                 //发送0x80+功能码回复。
                    ecbm_modbus_rtu_set_data(0x04, false);                 //异常码04。
                    ecbm_modbus_rtu_set_data((emu8)(ecbm_modbus_rtu_crc), false);//发送CRC。
                    ecbm_modbus_rtu_set_data((emu8)(ecbm_modbus_rtu_crc>>8), true);
                }
            #else
            if(0){    
            #endif
            }else{
                #if ECBM_MODBUS_RTU_REG_BUF_EN
                ecbm_modbus_cmd_write_reg(ecbm_modbus_rtu_address-ECBM_MODBUS_RTU_REG_START_ADDR,ecbm_modbus_rtu_data_count);//把数据写入寄存器。
                #else
                ecbm_modbus_cmd_write_reg(ecbm_modbus_rtu_address,ecbm_modbus_rtu_data_count);//把数据写入寄存器。
                #endif
                if(ecbm_modbus_rtu_broadcast_en){
                    ecbm_modbus_rtu_crc16(ecbm_modbus_rtu_id);          //计算CRC，下同。
                    ecbm_modbus_rtu_crc16(0x06);
                    ecbm_modbus_rtu_crc16((emu8)(ecbm_modbus_rtu_address>>8));
                    ecbm_modbus_rtu_crc16((emu8)(ecbm_modbus_rtu_address));
                    ecbm_modbus_rtu_crc16((emu8)(ecbm_modbus_rtu_data_count>>8));
                    ecbm_modbus_rtu_crc16((emu8)(ecbm_modbus_rtu_data_count));
                    ecbm_modbus_rtu_set_data(ecbm_modbus_rtu_id, false);       //发送地址。
                    ecbm_modbus_rtu_set_data(0x06, false);                     //发送功能码回复。
                    ecbm_modbus_rtu_set_data((emu8)(ecbm_modbus_rtu_address>>8), false);//发送地址。
                    ecbm_modbus_rtu_set_data((emu8)(ecbm_modbus_rtu_address), false);
                    ecbm_modbus_rtu_set_data((emu8)(ecbm_modbus_rtu_data_count>>8), false);//发送个数。
                    ecbm_modbus_rtu_set_data((emu8)(ecbm_modbus_rtu_data_count), false);
                    ecbm_modbus_rtu_set_data((emu8)(ecbm_modbus_rtu_crc), false);//发送CRC。
                    ecbm_modbus_rtu_set_data((emu8)(ecbm_modbus_rtu_crc>>8), true);
                }
            }
//      }else{
//          ecbm_modbus_rtu_fun_err_num=0x02;               //记录异常码02。
//          if(ecbm_modbus_rtu_broadcast_en){	
//              ecbm_modbus_rtu_crc16(ecbm_modbus_rtu_id);      //计算CRC，下同。
//              ecbm_modbus_rtu_crc16(0x86);
//              ecbm_modbus_rtu_crc16(0x02);
//              ecbm_modbus_rtu_set_data(ecbm_modbus_rtu_id);   //发送地址。
//              ecbm_modbus_rtu_set_data(0x86);                 //发送0x80+功能码回复。
//              ecbm_modbus_rtu_set_data(0x02);                 //异常码02。
//              ecbm_modbus_rtu_set_data((emu8)(ecbm_modbus_rtu_crc));//发送CRC。
//              ecbm_modbus_rtu_set_data((emu8)(ecbm_modbus_rtu_crc>>8));
//          }
//      }
//  }else{                                                  //如果不合法。
//      ecbm_modbus_rtu_fun_err_num=0x03;                   //记录异常码03。	
//      if(ecbm_modbus_rtu_broadcast_en){
//          ecbm_modbus_rtu_crc16(ecbm_modbus_rtu_id);          //计算CRC，下同。
//          ecbm_modbus_rtu_crc16(0x86);
//          ecbm_modbus_rtu_crc16(0x03);
//          ecbm_modbus_rtu_set_data(ecbm_modbus_rtu_id);       //发送地址。
//          ecbm_modbus_rtu_set_data(0x86);                     //发送0x80+功能码回复。
//          ecbm_modbus_rtu_set_data(0x03);                     //异常码03。
//          ecbm_modbus_rtu_set_data((emu8)(ecbm_modbus_rtu_crc));//发送CRC。
//          ecbm_modbus_rtu_set_data((emu8)(ecbm_modbus_rtu_crc>>8));
//      }
//  }
}
#endif
/*-------------------------------------------------------
16号功能码处理函数。
-------------------------------------------------------*/
#if ECBM_MODBUS_RTU_CMD10_EN
void ecbm_modbus_rtu_cmd_0x10(void){
    emu8 i;

    ecbm_modbus_rtu_crc=0xffff;
    if((ecbm_modbus_rtu_data_count>=0x0001)&&(ecbm_modbus_rtu_data_count<=0x007B)){             //输入的数据个数得符合要求。
        if(((emu32)ecbm_modbus_rtu_address+(emu32)ecbm_modbus_rtu_data_count)<65536UL){             //地址不能超过65536。
            #if ECBM_MODBUS_RTU_REG_BUF_EN
            if(                                                                        //遇到以下问题，说明写入肯定会失败。
                (ecbm_modbus_rtu_address<ECBM_MODBUS_RTU_REG_START_ADDR)||             //输入地址比起始地址还小的，
                (ecbm_modbus_rtu_address+ecbm_modbus_rtu_data_count)>(ECBM_MODBUS_RTU_REG_BUF_SIZE+ECBM_MODBUS_RTU_REG_START_ADDR)//输入的地址+输入的数量超过本机定义的数量。
            ){
                if(ecbm_modbus_rtu_broadcast_en){
                    ecbm_modbus_rtu_fun_err_num=0x04;               //记录异常码04。
                    ecbm_modbus_rtu_crc16   (ecbm_modbus_rtu_id);   //计算CRC，下同。
                    ecbm_modbus_rtu_crc16   (0x90);
                    ecbm_modbus_rtu_crc16   (0x04);
                    ecbm_modbus_rtu_set_data(ecbm_modbus_rtu_id, false);   //发送地址。
                    ecbm_modbus_rtu_set_data(0x90, false);                 //发送0x80+功能码回复。
                    ecbm_modbus_rtu_set_data(0x04, false);                 //异常码04。
                    ecbm_modbus_rtu_set_data((emu8)(ecbm_modbus_rtu_crc), false);//发送CRC。
                    ecbm_modbus_rtu_set_data((emu8)(ecbm_modbus_rtu_crc>>8), true);
                }
            #else 
            if(0){    
            #endif
            }else{//12511
                #if ECBM_MODBUS_RTU_REG_BUF_EN
                ecbm_modbus_rtu_address-=ECBM_MODBUS_RTU_REG_START_ADDR;
                #endif
                for(i=0;i<ecbm_modbus_rtu_cmd_count;i++){   
                    ecbm_modbus_cmd_write_reg(ecbm_modbus_rtu_address+i,ecbm_modbus_rtu_cmd_buf[i]);  //把数据写入寄存器。
                }
                if(ecbm_modbus_rtu_address==0x10){
                    
                }
                if(ecbm_modbus_rtu_broadcast_en){
                        ecbm_modbus_rtu_crc16   (ecbm_modbus_rtu_id);          //计算CRC，下同。
                        ecbm_modbus_rtu_crc16   (0x10);
                        ecbm_modbus_rtu_crc16   ((emu8)(ecbm_modbus_rtu_address>>8));
                        ecbm_modbus_rtu_crc16   ((emu8)(ecbm_modbus_rtu_address));
                        ecbm_modbus_rtu_crc16   ((emu8)(ecbm_modbus_rtu_data_count>>8));
                        ecbm_modbus_rtu_crc16   ((emu8)(ecbm_modbus_rtu_data_count));
                        ecbm_modbus_rtu_set_data(ecbm_modbus_rtu_id, false);       //发送地址。
                        ecbm_modbus_rtu_set_data(0x10, false);                     //发送功能码回复。
                        ecbm_modbus_rtu_set_data((emu8)(ecbm_modbus_rtu_address>>8), false);//发送地址。
                        ecbm_modbus_rtu_set_data((emu8)(ecbm_modbus_rtu_address), false);
                        ecbm_modbus_rtu_set_data((emu8)(ecbm_modbus_rtu_data_count>>8), false);//发送个数。
                        ecbm_modbus_rtu_set_data((emu8)(ecbm_modbus_rtu_data_count), false);
                        ecbm_modbus_rtu_set_data((emu8)(ecbm_modbus_rtu_crc), false);//发送CRC。
                        ecbm_modbus_rtu_set_data((emu8)(ecbm_modbus_rtu_crc>>8), true);
                }
            }
        }else{
            ecbm_modbus_rtu_fun_err_num=0x02;               //记录异常码02。
            if(ecbm_modbus_rtu_broadcast_en){	
                ecbm_modbus_rtu_crc16   (ecbm_modbus_rtu_id);   //计算CRC，下同。
                ecbm_modbus_rtu_crc16   (0x90);
                ecbm_modbus_rtu_crc16   (0x02);
                ecbm_modbus_rtu_set_data(ecbm_modbus_rtu_id, false);   //发送地址。
                ecbm_modbus_rtu_set_data(0x90, false);                 //发送0x80+功能码回复。
                ecbm_modbus_rtu_set_data(0x02, false);                 //异常码02。
                ecbm_modbus_rtu_set_data((emu8)(ecbm_modbus_rtu_crc), false);//发送CRC。
                ecbm_modbus_rtu_set_data((emu8)(ecbm_modbus_rtu_crc>>8), true);
            }
        }
    }else{                                                  //如果不合法。
        ecbm_modbus_rtu_fun_err_num=0x03;                   //记录异常码03。	
        if(ecbm_modbus_rtu_broadcast_en){
            ecbm_modbus_rtu_crc16(ecbm_modbus_rtu_id);          //计算CRC，下同。
            ecbm_modbus_rtu_crc16(0x90);
            ecbm_modbus_rtu_crc16(0x03);
            ecbm_modbus_rtu_set_data(ecbm_modbus_rtu_id, false);       //发送地址。
            ecbm_modbus_rtu_set_data(0x90, false);                     //发送0x80+功能码回复。
            ecbm_modbus_rtu_set_data(0x03, false);                     //异常码03。
            ecbm_modbus_rtu_set_data((emu8)(ecbm_modbus_rtu_crc), false);//发送CRC。
            ecbm_modbus_rtu_set_data((emu8)(ecbm_modbus_rtu_crc>>8), true);
        }
    }
}
#endif
/*-------------------------------------------------------
modbus接收处理函数。
22-07-09：串口收到数据后回调用这里
-------------------------------------------------------*/
void ecbm_modbus_rtu_receive(emu8 data){
    emu8 dat;//11162
    ecbm_modbus_rtu_timeout=1;
    dat = data;
    //dat=ecbm_modbus_rtu_get_data();//220709:从串口BUFF寄存器中取数据
    
    switch(ecbm_modbus_rtu_status){
        case ECBM_MODBUS_RTU_READY:{            //就绪，等待接收数据。
            if((dat==ecbm_modbus_rtu_id)||(dat==0)||(dat==0xff)){      //如果收到的ID和本机的匹配。
                ecbm_modbus_rtu_status=ECBM_MODBUS_RTU_FUN_NUM;//切换到功能码识别
                if((dat==0)||(dat==0xff)){
                    ecbm_modbus_rtu_broadcast_en=0;//写入通信模式，为了简化代码，采用直接赋值。
                }else{
                    ecbm_modbus_rtu_broadcast_en=1;
                }
                ecbm_modbus_rtu_fun_code=0;     //清零功能码缓存。
                ecbm_modbus_rtu_fun_err_num=0;  //清零异常码缓存。
                ecbm_modbus_rtu_uart_crc=0xffff;//重置CRC初值。
                ecbm_modbus_rtu_crc=ecbm_modbus_rtu_uart_crc;
                ecbm_modbus_rtu_crc16(dat);     //计算CRC。
                ecbm_modbus_rtu_uart_crc=ecbm_modbus_rtu_crc;
            }else{                             //如果ID不匹配，进入相应的错误界面。
                ecbm_modbus_rtu_status=ECBM_MODBUS_RTU_ID_ERR;
            }
        }break;
        case ECBM_MODBUS_RTU_FUN_NUM:{                                      //正在接收功能码。
            if(dat>127){                                                    //判断功能码是否合法，
                ecbm_modbus_rtu_status=ECBM_MODBUS_RTU_FUN_NUM_ERR;         //不合法就跳到功能码错误。
                ecbm_modbus_rtu_fun_err_num=0x01;                           //异常码01。	
            }else{                                                          //如果合法，
                ecbm_modbus_rtu_fun_code=ecbm_modbus_rtu_check_fun_num(dat);//判断是不是本机支持的功能码。
                if(ecbm_modbus_rtu_fun_code>127){
                    ecbm_modbus_rtu_fun_code&=~0x80;
                    ecbm_modbus_rtu_status=ECBM_MODBUS_RTU_FUN_NUM_VOID;    //不存在的功能码就跳到功能码空。
                    ecbm_modbus_rtu_fun_err_num=0x01;                       //异常码01。
                }else{
                    ecbm_modbus_rtu_status=ECBM_MODBUS_RTU_ADDRH;           //如果是存在的功能码，就考虑开始接收数据了。
                    ecbm_modbus_rtu_crc=ecbm_modbus_rtu_uart_crc;
                    ecbm_modbus_rtu_crc16(dat);
                    ecbm_modbus_rtu_uart_crc=ecbm_modbus_rtu_crc;
                }
            }
        }break;
        case ECBM_MODBUS_RTU_ADDRH:{                        //接收地址的高8位。
            ecbm_modbus_rtu_address=(emu16)dat<<8;            //组装地址。
            ecbm_modbus_rtu_status=ECBM_MODBUS_RTU_ADDRL;   //到下一步。
            ecbm_modbus_rtu_crc=ecbm_modbus_rtu_uart_crc;   //计算CRC。
            ecbm_modbus_rtu_crc16(dat);
            ecbm_modbus_rtu_uart_crc=ecbm_modbus_rtu_crc;
        }break;
        case ECBM_MODBUS_RTU_ADDRL:{                        //接收地址的低8位。
            ecbm_modbus_rtu_address+=(emu16)dat;              //组装地址。
            ecbm_modbus_rtu_status=ECBM_MODBUS_RTU_DATA_COUNTH;//到下一步。
            ecbm_modbus_rtu_crc=ecbm_modbus_rtu_uart_crc;   //计算CRC。
            ecbm_modbus_rtu_crc16(dat);
            ecbm_modbus_rtu_uart_crc=ecbm_modbus_rtu_crc;
        }break;
        case ECBM_MODBUS_RTU_DATA_COUNTH:{                  //接收数据/个数的高8位。
            ecbm_modbus_rtu_data_count=(emu16)dat<<8;         //组装数据/个数。
            ecbm_modbus_rtu_status=ECBM_MODBUS_RTU_DATA_COUNTL;//到下一步。
            ecbm_modbus_rtu_crc=ecbm_modbus_rtu_uart_crc;   //计算CRC。
            ecbm_modbus_rtu_crc16(dat);
            ecbm_modbus_rtu_uart_crc=ecbm_modbus_rtu_crc;
        }break;
        case ECBM_MODBUS_RTU_DATA_COUNTL:{                  //接收数据/个数的低8位。
            ecbm_modbus_rtu_data_count+=(emu16)dat;           //组装数据/个数。
            if(ecbm_modbus_rtu_fun_code==0x10){
                ecbm_modbus_rtu_cmd_count=0;
                ecbm_modbus_rtu_status=ECBM_MODBUS_RTU_BYTE_COUNT;//到字节读取。
            }else{
                ecbm_modbus_rtu_status=ECBM_MODBUS_RTU_CRCL;//到CRC。
            }
            ecbm_modbus_rtu_crc=ecbm_modbus_rtu_uart_crc;   //计算CRC。
            ecbm_modbus_rtu_crc16(dat);
            ecbm_modbus_rtu_uart_crc=ecbm_modbus_rtu_crc;
        }break;
        case ECBM_MODBUS_RTU_CRCL:{                         //接收CRC的低8位。
            ecbm_modbus_rtu_crc_buf=(emu16)dat;               //组合CRC。
            ecbm_modbus_rtu_status=ECBM_MODBUS_RTU_CRCH;    //到下一步。
        }break;
        case ECBM_MODBUS_RTU_CRCH:{                         //接收CRC的高8位。
            ecbm_modbus_rtu_crc_buf+=(emu16)dat<<8;           //组合CRC。
            if(ecbm_modbus_rtu_crc_buf==ecbm_modbus_rtu_uart_crc){//判断接收的CRC和计算的CRC是否相等。
                ecbm_modbus_rtu_status=ECBM_MODBUS_RTU_DO;  //相等的话就到执行步骤。
            }else{                                          //否则，
                ecbm_modbus_rtu_status=ECBM_MODBUS_RTU_CRC_ERR;//就跳到CRC错误界面。
            }
        }break;
        #if ECBM_MODBUS_RTU_CMD10_EN
        case ECBM_MODBUS_RTU_BYTE_COUNT:{                   //接收数据字节数。
            ecbm_modbus_rtu_cmd_count=dat;
            if((ecbm_modbus_rtu_cmd_count==0)||(ecbm_modbus_rtu_cmd_count!=(emu8)(ecbm_modbus_rtu_data_count*2))){//如果数量不合。
                ecbm_modbus_rtu_fun_err_num=0x03;           //字节数对不上，异常码03。
                ecbm_modbus_rtu_status=ECBM_MODBUS_RTU_BYTE_ERR;//调到对应的处理界面。
            }else{
                ecbm_modbus_rtu_cmd_count=0;                    //复用该变量。
                ecbm_modbus_rtu_status=ECBM_MODBUS_RTU_DATAH;//调到对应的处理界面。
                ecbm_modbus_rtu_crc=ecbm_modbus_rtu_uart_crc;   //计算CRC。
                ecbm_modbus_rtu_crc16(dat);
                ecbm_modbus_rtu_uart_crc=ecbm_modbus_rtu_crc;
            }
        }break;
        case ECBM_MODBUS_RTU_DATAH:{
            ecbm_modbus_rtu_cmd_buf[ecbm_modbus_rtu_cmd_count]=((emu16)dat)<<8;
            ecbm_modbus_rtu_status=ECBM_MODBUS_RTU_DATAL;   //到低8位。
            ecbm_modbus_rtu_crc=ecbm_modbus_rtu_uart_crc;   //计算CRC。
            ecbm_modbus_rtu_crc16(dat);
            ecbm_modbus_rtu_uart_crc=ecbm_modbus_rtu_crc;
        }break;
        case ECBM_MODBUS_RTU_DATAL:{
            ecbm_modbus_rtu_cmd_buf[ecbm_modbus_rtu_cmd_count]+=((emu16)dat);
            ecbm_modbus_rtu_cmd_count++;
            if(ecbm_modbus_rtu_cmd_count==(emu8)ecbm_modbus_rtu_data_count){
                ecbm_modbus_rtu_status=ECBM_MODBUS_RTU_CRCL;//到CRC。
            }else{
                ecbm_modbus_rtu_status=ECBM_MODBUS_RTU_DATAH;//到下一个数据的高8位。
            }
            ecbm_modbus_rtu_crc=ecbm_modbus_rtu_uart_crc;   //计算CRC。
            ecbm_modbus_rtu_crc16(dat);
            ecbm_modbus_rtu_uart_crc=ecbm_modbus_rtu_crc;
        }break;
        #endif
    }
}
/*-------------------------------------------------------
modbus主循环处理函数。
-------------------------------------------------------*/
void ecbm_modbus_rtu_run(void){
    switch(ecbm_modbus_rtu_status){
        case ECBM_MODBUS_RTU_FUN_NUM_VOID://	debug("不支持该功能码\r\n");
        case ECBM_MODBUS_RTU_FUN_NUM_ERR:{//	debug("功能码错误\r\n");
            if(ecbm_modbus_rtu_broadcast_en){
                ecbm_modbus_rtu_crc=0xFFFF;
                ecbm_modbus_rtu_crc16(ecbm_modbus_rtu_id);              //计算CRC，下同。
                ecbm_modbus_rtu_crc16(0x80+ecbm_modbus_rtu_fun_code); 
                ecbm_modbus_rtu_crc16(ecbm_modbus_rtu_fun_err_num);
                ecbm_modbus_rtu_set_data(ecbm_modbus_rtu_id, false);           //发送地址。
                ecbm_modbus_rtu_set_data(0x80+ecbm_modbus_rtu_fun_code, false);//发送0x80+功能码回复。
                ecbm_modbus_rtu_set_data(ecbm_modbus_rtu_fun_err_num, false);  //异常码。
                ecbm_modbus_rtu_set_data((emu8)(ecbm_modbus_rtu_crc), false);    //发送CRC。
                ecbm_modbus_rtu_set_data((emu8)(ecbm_modbus_rtu_crc>>8), true);
            }
            ecbm_modbus_rtu_status=ECBM_MODBUS_RTU_WAIT;
        }break;
        case ECBM_MODBUS_RTU_ID_ERR:{//ID错误。
            //这里可以做些处理，但默认留空。
            ecbm_modbus_rtu_status=ECBM_MODBUS_RTU_WAIT;
        }break;
        case ECBM_MODBUS_RTU_CRC_ERR:{//CRC错误。
            //这里可以做些处理，比如发送个重发信号，但默认留空。
            ecbm_modbus_rtu_status=ECBM_MODBUS_RTU_WAIT;
        }break;
        case ECBM_MODBUS_RTU_DO:{//动作执行。
            #if ECBM_MODBUS_RTU_CMD_ALL_EN
                #if ECBM_MODBUS_IO_EN
                ecbm_modbus_rtu_set_io(ECBM_MODBUS_WRITE);//220709:控制使能脚写
                #endif
                switch(ecbm_modbus_rtu_fun_code){//根据功能码来选择对应的动作。
                #if ECBM_MODBUS_RTU_CMD01_EN
                    case 0x01:ecbm_modbus_rtu_cmd_0x01();break;
                #endif
                #if ECBM_MODBUS_RTU_CMD02_EN
                    case 0x02:ecbm_modbus_rtu_cmd_0x02();break;
                #endif
                #if ECBM_MODBUS_RTU_CMD03_EN
                    case 0x03:ecbm_modbus_rtu_cmd_0x03();break;
                #endif
                #if ECBM_MODBUS_RTU_CMD04_EN
                    case 0x04:ecbm_modbus_rtu_cmd_0x04();break;
                #endif
                #if ECBM_MODBUS_RTU_CMD05_EN
                    case 0x05:ecbm_modbus_rtu_cmd_0x05();break;
                #endif
                #if ECBM_MODBUS_RTU_CMD06_EN
                    case 0x06:ecbm_modbus_rtu_cmd_0x06();
                    Modbus_Comand_Deal(ecbm_modbus_rtu_fun_code, ecbm_modbus_rtu_address);
                    break;
                #endif
                #if ECBM_MODBUS_RTU_CMD10_EN
                    case 0x10:ecbm_modbus_rtu_cmd_0x10();
                    Modbus_Comand_Deal(ecbm_modbus_rtu_fun_code, ecbm_modbus_rtu_address);
                    break;
                #endif
                }
            #endif
            #if ECBM_MODBUS_IO_EN
            ecbm_modbus_rtu_set_io(ECBM_MODBUS_READ);//220709:控制使能脚
            #endif
            ecbm_modbus_rtu_status=ECBM_MODBUS_RTU_READY;
        }break;
        case ECBM_MODBUS_RTU_BYTE_ERR:{
            if(ecbm_modbus_rtu_broadcast_en){
                ecbm_modbus_rtu_crc=0xFFFF;
                ecbm_modbus_rtu_crc16(ecbm_modbus_rtu_id);              //计算CRC，下同。
                ecbm_modbus_rtu_crc16(0x80+ecbm_modbus_rtu_fun_code); 
                ecbm_modbus_rtu_crc16(ecbm_modbus_rtu_fun_err_num);
                ecbm_modbus_rtu_set_data(ecbm_modbus_rtu_id, false);           //发送地址。
                ecbm_modbus_rtu_set_data(0x80+ecbm_modbus_rtu_fun_code, false);//发送0x80+功能码回复。
                ecbm_modbus_rtu_set_data(ecbm_modbus_rtu_fun_err_num, false);  //异常码。
                ecbm_modbus_rtu_set_data((emu8)(ecbm_modbus_rtu_crc), false);    //发送CRC。
                ecbm_modbus_rtu_set_data((emu8)(ecbm_modbus_rtu_crc>>8), true);
            }
            ecbm_modbus_rtu_status=ECBM_MODBUS_RTU_WAIT;
        }break;
    }
} 


