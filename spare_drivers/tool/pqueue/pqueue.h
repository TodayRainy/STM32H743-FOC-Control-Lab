#ifndef _PQUEUE_H
#define _PQUEUE_H

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>


/*指针队列结构体*/
typedef struct{
    char *name;
    bool init_flag;
    uint32_t num;               /*队列长度*/
    uint32_t count;             /*队列当前长度*/
    uint32_t cell_size;         /*单位成员大小*/
    uint32_t front;             /*首*/         
    uint32_t rear;              /*尾*/
    uint8_t *data;              /*缓冲区*/
}P_QUEUE_t;


int P_Queue_Init(P_QUEUE_t *queue, uint8_t *buf, uint32_t cell_num, uint32_t cell_size);
int P_Queue_en(P_QUEUE_t *queue,  uint8_t *data);
int P_Queue_den(P_QUEUE_t *queue, uint8_t *data);
int P_Queue_count(P_QUEUE_t *queue);


// /*队列*/
// typedef struct{
//     P_QUEUE_t *queue;

//     int (*Init)(uint8_t *buf, uint8_t cell_num, uint8_t cell_size);
//     int (*isEmpty)(void);
//     int (*isFull)(void);
//     int (*enqueue)(uint8_t *data);                  /*入队*/
//     int (*denqueue)(uint8_t *data);                 /*出队*/
// }PQUEUE_t;


#endif


