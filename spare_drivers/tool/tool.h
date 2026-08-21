#ifndef _TOOL_H
#define _TOOL_H

#include <stdint.h>
#include <stdbool.h>


#include "./crc/mycrc.h"
#include "./modbus/modbus.h"
#include "./ymodem/ymodem.h"


#define IS_AF(c)  ((c >= 'A') && (c <= 'F'))
#define IS_af(c)  ((c >= 'a') && (c <= 'f'))
#define IS_09(c)  ((c >= '0') && (c <= '9'))
#define ISVALIDHEX(c)  IS_AF(c) || IS_af(c) || IS_09(c)
#define ISVALIDDEC(c)  IS_09(c)
#define CONVERTDEC(c)  (c - '0')

#define CONVERTHEX_alpha(c)  (IS_AF(c) ? (c - 'A'+10) : (c - 'a'+10))
#define CONVERTHEX(c)   (IS_09(c) ? (c - '0') : CONVERTHEX_alpha(c))


extern void Tool_Init(void);

extern void Int2Str(uint8_t* str, int32_t intnum);
extern uint32_t Str2Int(uint8_t *inputstr, int32_t *intnum);

extern bool sd_mount;
  

void ymode_Enqueue(uint8_t buf);
uint8_t ymode_Dequeue(uint8_t *buf);


#endif
