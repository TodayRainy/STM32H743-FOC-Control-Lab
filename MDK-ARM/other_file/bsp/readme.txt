
sdram的初始化需要加上
    SDRAM_Initialization_Sequence(&hsdram1, 917);       //480HZ         发送SDRAM初始化序列
    SDRAM_Initialization_Sequence(&hsdram1, 761);       //400MHZ        发送SDRAM初始化序列

使用molloc那部分时，因为sdram控制器未初始化，不能直接把sdram的数组赋值给molloc设备

USB枚举流程：
    复位 -> 打开端点0 -> 设置usb地址 -> 发送设备描述符 -> 设备配置描述符 -> 发送字符串描述符 -> 序列号信息描述符 -> 发送语言字符串描述符 
    -> 发送设备产品名称字符串 -> 发送设备限定描述符 ->


