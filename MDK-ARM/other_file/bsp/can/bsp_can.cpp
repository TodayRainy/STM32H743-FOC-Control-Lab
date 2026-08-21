#include "bsp_can.hpp"


#pragma arm section code  = "RAMCODE"

bsp_can::bsp_can(FDCAN_HandleTypeDef *hfdcan)
{
    this->hfdcan = hfdcan;

    config(CAN_1000K);
}

bsp_can::~bsp_can()
{
}

void bsp_can::config(CAN_BAUDRATE_t baudrate)
{
    uint32_t NominalPrescaler = 20;
    uint32_t NominalSyncJumpWidth = 3;
    uint32_t NominalTimeSeg1 = 16;
    uint32_t NominalTimeSeg2 = 3;

    switch (baudrate){
        case CAN_125K:
            /* code */
            NominalPrescaler = 40;
            break;
        case CAN_250K:
            NominalPrescaler = 20;
            break;
        case CAN_500K:
            NominalPrescaler = 10;
            break;
        case CAN_1000K:
            NominalPrescaler = 5;
            break;
        default:
            break;
    }

    FDCAN_FilterTypeDef FDCAN1_RXFilter = {};
    HAL_FDCAN_DeInit(hfdcan); 
    
    hfdcan->Init.FrameFormat = FDCAN_FRAME_CLASSIC;                 /*使用经典 CAN 帧格式（非 CAN FD，最大 8 字节）*/
    hfdcan->Init.Mode = FDCAN_MODE_NORMAL;                          /*正常工作模式：参与仲裁、发送 ACK*/
    hfdcan->Init.AutoRetransmission = DISABLE;                      /*关闭自动重发（发送失败不会自动再发，适合实时控制）*/
    hfdcan->Init.TransmitPause = DISABLE;                           /*关闭发送暂停（允许连续发送，提高总线利用率）*/
    hfdcan->Init.ProtocolException = DISABLE;                       /*关闭协议异常处理（一般保持默认）*/
    hfdcan->Init.NominalPrescaler = NominalPrescaler;               /*仲裁相位分频系数：FDCAN 时钟 / 8*/
    hfdcan->Init.NominalSyncJumpWidth = NominalSyncJumpWidth;       /*仲裁相位 SJW = 1 TQ（允许的最大重同步补偿，偏小）*/
    hfdcan->Init.NominalTimeSeg1 = NominalTimeSeg1;                 /*仲裁相位 TimeSeg1（采样点前时间）*/
    hfdcan->Init.NominalTimeSeg2 = NominalTimeSeg2;                 /*仲裁相位 TimeSeg2（采样点后时间）*/

    hfdcan->Init.DataPrescaler = 1;                                 /*数据相位分频（经典 CAN 模式下无效）*/
    hfdcan->Init.DataSyncJumpWidth = 1;                             /*数据相位 SJW（经典 CAN 模式下无效）*/
    hfdcan->Init.DataTimeSeg1 = 1;                                  /*数据相位 TimeSeg1（经典 CAN 模式下无效）*/
    hfdcan->Init.DataTimeSeg2 = 1;                                  /*数据相位 TimeSeg2（经典 CAN 模式下无效）*/

    hfdcan->Init.MessageRAMOffset = 0;                              /*Message RAM 起始偏移地址（通常为 0）*/
    hfdcan->Init.StdFiltersNbr = 1;                                 /*标准 ID 滤波器数量（0 表示不使用标准滤波）*/
    hfdcan->Init.ExtFiltersNbr = 1;                                 /*扩展 ID 滤波器数量（0 表示不使用扩展滤波）*/
    hfdcan->Init.RxFifo0ElmtsNbr = 64;                              /*RX FIFO0 元素数量（0 = 不接收任何报文 ?）*/
    hfdcan->Init.RxFifo0ElmtSize = FDCAN_DATA_BYTES_8;              /*RX FIFO0 单个元素数据长度（8 字节）*/
    hfdcan->Init.RxFifo1ElmtsNbr = 0;                              /*RX FIFO1 元素数量（未使用）*/
    hfdcan->Init.RxFifo1ElmtSize = FDCAN_DATA_BYTES_8;              /*RX FIFO1 单个元素数据长度（8 字节）*/
    hfdcan->Init.RxBuffersNbr = 0;                                  /*RX Buffer 数量（未使用）*/
    hfdcan->Init.RxBufferSize = FDCAN_DATA_BYTES_8;                 /*RX Buffer 单个大小（8 字节）*/
    hfdcan->Init.TxEventsNbr = 0;                                   /*TX 事件 FIFO 数量（不记录发送事件）*/
    hfdcan->Init.TxBuffersNbr = 0;                                  /*TX Buffer 数量（不使用 TX Buffer）*/
    hfdcan->Init.TxFifoQueueElmtsNbr = 32;                          /*TX FIFO / Queue 元素数量（1 个发送邮箱）*/
    hfdcan->Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;         /*TX FIFO 工作模式（FIFO 顺序发送）*/
    hfdcan->Init.TxElmtSize = FDCAN_DATA_BYTES_8;                   /*TX 元素数据长度（8 字节）*/


    if(HAL_FDCAN_Init(hfdcan) != HAL_OK)
    {
        Error_Handler();
    }



    HAL_FDCAN_Start(hfdcan);                               //开启FDCAN
    HAL_FDCAN_ActivateNotification(hfdcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE,0);


    FDCAN1_RXFilter.IdType = FDCAN_STANDARD_ID;                     /*标准ID*/     
    FDCAN1_RXFilter.FilterIndex = 0;                                /*滤波器索引*/
    FDCAN1_RXFilter.FilterType=FDCAN_FILTER_DUAL;                   //滤波器类型
    FDCAN1_RXFilter.FilterConfig=FDCAN_FILTER_TO_RXFIFO0;           //过滤器0关联到FIFO0  
    FDCAN1_RXFilter.FilterID1=0x001;                                //32位ID
    FDCAN1_RXFilter.FilterID2=0x003;                                //如果FDCAN配置为传统模式的话，这里是32位掩码

    if(HAL_FDCAN_ConfigFilter(hfdcan, &FDCAN1_RXFilter)!=HAL_OK) return ;   //滤波器初始化
    HAL_FDCAN_ConfigGlobalFilter(hfdcan,
    FDCAN_REJECT,       // 拒绝所有远程帧
    FDCAN_REJECT,       // 拒绝不匹配的标准帧
    ENABLE,             // 拒绝不匹配的扩展帧
    ENABLE);
}

void bsp_can::send_msg(uint32_t id, CAN_ID_TYPE_t type, uint8_t msg[8], uint32_t len)
{
    FDCAN_TxHeaderTypeDef FDCAN_TxHeader = {};

    uint32_t TxFrameType = 0;

    switch (type){
        case CAN_STD:
            /* code */
            TxFrameType = FDCAN_STANDARD_ID;
            break;
        case CAN_EXT:
            /* code */
            TxFrameType = FDCAN_EXTENDED_ID;
            break;
        default:
            break;
    }

    FDCAN_TxHeader.Identifier = id;
    FDCAN_TxHeader.IdType=TxFrameType;                        //
    FDCAN_TxHeader.TxFrameType=FDCAN_DATA_FRAME;              //数据帧
    FDCAN_TxHeader.DataLength=len;                            //数据长度
    FDCAN_TxHeader.ErrorStateIndicator=FDCAN_ESI_ACTIVE;            
    FDCAN_TxHeader.BitRateSwitch=FDCAN_BRS_OFF;               //关闭速率切换
    FDCAN_TxHeader.FDFormat=FDCAN_CLASSIC_CAN;                //传统的CAN模式
    FDCAN_TxHeader.TxEventFifoControl=FDCAN_NO_TX_EVENTS;     //无发送事件
    FDCAN_TxHeader.MessageMarker=0;             

    HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &FDCAN_TxHeader, msg);

    // HAL_FDCAN_AddMessageToTxBuffer(hfdcan, &FDCAN_TxHeader, msg, FDCAN_TX_BUFFER0);
    // HAL_FDCAN_EnableTxBufferRequest(hfdcan, FDCAN_TX_BUFFER0);

}


#pragma arm section




