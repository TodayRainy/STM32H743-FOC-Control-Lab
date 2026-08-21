#include "pqueue.h"


/*队列是否为空 返回1为空

返回 0为空， 1未空
*/
static int P_Queue_isEmpty(P_QUEUE_t *queue)
{   
    if(NULL==queue)return -1;
    if(false==queue->init_flag)return -1;

    /*这种方法导致缓冲区必须多分配一个*/
    if(queue->front == queue->rear){
        return 0;
    }else{
        return 1;
    }
}

/*队列是否已满
返回 0为满， 1未满
*/
static int P_Queue_isFull(P_QUEUE_t *queue)
{
    if(NULL==queue)return -1;
    if(false==queue->init_flag)return -1;

    if((queue->rear+1)%queue->num == queue->front){

        return 0;
    }else{
        return 1;
    }
}




/*初始化队列，设置队列长度 实际长度=cell_num-1
返回 0成功， 其他失败
*/
int P_Queue_Init(P_QUEUE_t *queue, uint8_t *buf, uint32_t cell_num, uint32_t cell_size)
{
    if(NULL==queue)return -1;

    // queue->data = (uint8_t*)calloc(cell_num+1, cell_size);
    memset(buf, 0x00, cell_num*cell_size);

    queue->data = buf;

    if(NULL==queue->data){

        printf("Queue_Init: NO\r\n");
        return -1;
    } 

    queue->num = cell_num;
    queue->cell_size = cell_size;
    queue->front = 0;
    queue->rear  = 0;
    queue->init_flag = true;
    queue->count = 0;

    return 0;
}


/*入队操作
返回 0成功， 其他失败
*/
int P_Queue_en(P_QUEUE_t *queue,  uint8_t *data)
{
    int ret = 0;
    uint8_t *ptr;
    if(NULL==queue||NULL==data||false==queue->init_flag)return -1;

    /*检查队列是否已满*/
    if(P_Queue_isFull(queue)==0){
        ret = -1;
        goto RET;
    }
    ptr = &queue->data[queue->rear * queue->cell_size];
    memcpy(ptr, data, queue->cell_size);
    // printf("入队:ptr: %p\r\n", ptr);
    
    queue->rear = (queue->rear+1)%queue->num;
    queue->count++;

RET:
    return ret;
}

/*出队操作
返回 0成功， 其他失败
*/
int P_Queue_den(P_QUEUE_t *queue, uint8_t *data)
{
    int ret=0;
    uint8_t *ptr;
    if(NULL==queue||NULL==data||false==queue->init_flag)return -1;

    /*检查队列是否已空*/
    if(P_Queue_isEmpty(queue)==0){
        // printf("条件变量唤醒\r\n");
        ret = -1;
        goto RET;
    }
    ptr = &queue->data[queue->front*queue->cell_size];
    // printf("出队ptr: %s\r\n", ptr);
    memcpy(data, ptr, queue->cell_size);
    queue->front = (queue->front+1)%queue->num;
    queue->count--;
RET:
    return ret;
}

int P_Queue_count(P_QUEUE_t *queue)
{
    if(NULL==queue||false==queue->init_flag)return -1;

    return queue->count;
}
