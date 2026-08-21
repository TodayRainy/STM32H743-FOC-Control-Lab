#ifndef _CRC_HPP
#define _CRC_HPP





#ifdef __cplusplus
extern "C" {  // 告诉 C++ 编译器：这部分用 C 的方式编译
#endif

#include <stdint.h>
#include <stdio.h>

uint16_t crc_16(unsigned char *buf, int len);

uint8_t crc_8(const unsigned char *buf, int len);


void crc8_test(void);

#ifdef __cplusplus
}
#endif










#endif































