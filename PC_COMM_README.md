# PC 通信模块使用说明（UART6）

## 📋 概述

本模块用于通过 **UART6** 接收上位机/视觉端发送的 **16 字节定长数据包**，支持 CRC8 校验。

**替代了原来的裁判系统接收功能**，使用新的 `periph_pc_comm.c/h` 模块。

---

## 🔧 硬件配置

### UART6 引脚
- **TX:** `PG14` (GPIO_AF8_USART6)
- **RX:** `PG9` (GPIO_AF8_USART6)

### 串口参数
- **波特率:** 115200bps
- **数据位:** 8 位
- **停止位:** 1 位
- **校验位:** 无
- **模式:** 全双工 (TX+RX)

### DMA 配置
- **DMA 通道:** DMA2_Stream1, Channel_5
- **模式:** 循环模式 (Circular)
- **缓冲区大小:** 64 字节

---

## 📦 数据帧格式

**固定 16 字节数据包结构：**

```
字节索引 |  内容        | 说明
--------|-------------|------------------
[0]     |  0xFF        | 帧头 (固定)
[1-4]   |  Data1       | 数据 1 (如 linear_x)
[5-8]   |  Data2       | 数据 2 (如 linear_y)
[9-12]  |  Data3       | 数据 3 (如 linear_z)
[13]    |  Custom      | 自定义数据
[14]    |  CRC8        | CRC8 校验和 (前 14 字节)
[15]    |  0x0D        | 帧尾 (固定)
```

**注意：** 数据段 [1-13] 的具体含义由用户自定义，上述仅为示例。

---

## 🚀 使用方法

### 1. 初始化（在 app_init.c 中已自动调用）

```c
#include "periph_pc_comm.h"

void Init_InitAll() {
    // ... 其他初始化 ...
    
    PC_Comm_Init();  // 初始化 PC 通信
    
    // ... 其他初始化 ...
}
```

### 2. 读取数据

```c
#include "periph_pc_comm.h"

// 方法 1: 直接访问全局变量
if (PC_Comm_Data.state == PC_COMM_STATE_CONNECTED) {
    uint8_t* data = PC_Comm_Data.raw_data_payload;
    // 解析 data[0] 到 data[12]
}

// 方法 2: 使用获取指针函数
PC_Comm_DataTypeDef* pData = PC_Comm_GetDataPtr();
if (pData->state == PC_COMM_STATE_CONNECTED) {
    // 使用 pData->raw_data_payload
}
```

### 3. 检查通信状态

```c
// 判断是否在线
if (PC_Comm_IsOnline()) {
    // 通信正常
} else {
    // 通信离线
}

// 获取详细状态
PC_Comm_StateEnum state = PC_Comm_GetState();
// state 可能值：
// - PC_COMM_STATE_NULL: 未初始化
// - PC_COMM_STATE_CONNECTED: 已连接
// - PC_COMM_STATE_LOST: 丢失/超时
// - PC_COMM_STATE_ERROR: 校验错误
```

---

## 📝 应用层示例

### 示例 1: 提取浮点数据

```c
#include "app_pc_comm_example.h"

TargetData target;
PC_Comm_ExtractTargetData(&target);

// 现在可以使用 target.linear_x, target.linear_y 等数据
```

### 示例 2: 自定义解析函数

```c
void MyDataParse(void) {
    if (PC_Comm_Data.state != PC_COMM_STATE_CONNECTED) return;
    
    uint8_t* data = PC_Comm_Data.raw_data_payload;
    
    // 假设数据格式：
    // [0-3]: 目标 X 坐标 (float)
    // [4-7]: 目标 Y 坐标 (float)
    // [8-11]: 目标 Z 坐标 (float)
    // [12]: 射击命令
    
    uint32_t temp;
    
    // 解析 X 坐标
    temp = (data[0] << 0) | (data[1] << 8) | 
           (data[2] << 16) | (data[3] << 24);
    float targetX = *(float*)&temp;
    
    // 解析 Y 坐标
    temp = (data[4] << 0) | (data[5] << 8) | 
           (data[6] << 16) | (data[7] << 24);
    float targetY = *(float*)&temp;
    
    // 解析射击命令
    uint8_t shootCmd = data[12];
    
    // 使用这些数据...
}
```

---

## ⚠️ 注意事项

### 1. CRC8 校验
- **多项式:** 0x31 (x^8 + x^5 + x^4 + 1)
- **初始值:** 0x00
- **校验范围:** 第 0-13 字节（帧头 + 数据段）
- **校验码位置:** 第 14 字节

### 2. 超时判断
- **超时时间:** 100ms（可根据需要修改）
- **判断方法:** 比较 `HAL_GetTick()` 与 `last_update_time`

### 3. 中断处理
- PC 通信回调已在 `Uart_RxIdleCallback()` 中注册
- 无需手动配置中断，系统会自动调用 `PC_Comm_RXCallback()`

---

## 🔄 替换原裁判系统

### 已完成的修改：

1. ✅ `util_uart.c` - 将 UART6 回调从 `Referee_RXCallback` 改为 `PC_Comm_RXCallback`
2. ✅ `app_init.c` - 将 `Referee_InitReferee()` 改为 `PC_Comm_Init()`
3. ✅ `periph_pc_comm.c/h` - 完善实现和接口

### 如需恢复裁判系统功能：

如果需要同时支持裁判系统和 PC 通信，建议：
- **裁判系统** 使用 USART1 或 USART3
- **PC 通信** 继续使用 USART6

---

## 🛠️ 调试技巧

### 1. 检查数据接收
```c
// 在中断中设置断点
void PC_Comm_RXCallback(UART_HandleTypeDef* huart) {
    // 在这里查看 PC_Comm_RxData 数组
    // 确认是否收到正确的数据
}
```

### 2. 验证 CRC 校验
```c
// 手动计算 CRC 并与接收到的对比
uint8_t calculated_crc = PC_Comm_CalculateCRC8(data, 14);
uint8_t received_crc = data[14];
if (calculated_crc == received_crc) {
    // CRC 正确
}
```

### 3. 监控通信状态
```c
// 定期打印状态
printf("PC Comm State: %d, Last Update: %lu\n", 
       PC_Comm_Data.state, 
       HAL_GetTick() - PC_Comm_Data.last_update_time);
```

---

## 📞 常见问题

**Q: 收不到数据？**
A: 检查以下几点：
1. UART6 引脚连接是否正确
2. 波特率是否与上位机匹配（115200）
3. 数据格式是否为 16 字节定长
4. 帧头是否为 0xFF，帧尾是否为 0x0D

**Q: CRC 校验失败？**
A: 确认：
1. CRC 多项式是否为 0x31
2. 校验范围是否为前 14 字节
3. 数据传输是否有误码

**Q: 如何修改数据解析方式？**
A: 修改 `app_pc_comm_example.c` 中的 `PC_Comm_ExtractTargetData()` 函数，根据实际协议解析。

---

## 📄 相关文件

- `Inc/Periphal/periph_pc_comm.h` - PC 通信头文件
- `Src/Periphal/periph_pc_comm.c` - PC 通信实现
- `Src/Application/app_pc_comm_example.c` - 应用层示例
- `Inc/Application/app_pc_comm_example.h` - 应用层示例头文件
- `Src/Application/app_init.c` - 初始化代码（已修改）
- `Src/Utility/util_uart.c` - UART 工具（已修改）

---

**更新日期:** 202X-XX-XX  
**版本:** v1.0
