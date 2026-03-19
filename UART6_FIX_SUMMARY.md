# UART6 问题修复总结

## ✅ 已完成的修复

### 1. 修复 PC_Comm_Init() 函数
**文件:** `Src/Periphal/periph_pc_comm.c`

**修改内容:**
```c
// ❌ 修改前
#include "periph_pc_comm.h"
#include "periph_remote.h"

void PC_Comm_Init(void) {
    // ... 
    HAL_UART_Receive_DMA(Const_PC_Comm_UART_HANDLER, PC_Comm_RxData, PC_COMM_RX_BUFF_LEN);
}

// ✅ 修改后
#include "periph_pc_comm.h"
#include "periph_remote.h"
#include "util_uart.h"  // 新增

void PC_Comm_Init(void) {
    // ...
    Uart_InitUartDMA(Const_PC_Comm_UART_HANDLER);           // 使用统一函数
    Uart_ReceiveDMA(Const_PC_Comm_UART_HANDLER, PC_Comm_RxData, PC_COMM_RX_BUFF_LEN);
}
```

**修复的问题:**
- ✅ 添加了 `util_uart.h` 头文件
- ✅ 使用项目统一的 `Uart_InitUartDMA()` 启用 IDLE 中断
- ✅ 使用 `Uart_ReceiveDMA()` 启动 DMA，保持与回调函数的一致性

---

## 🔍 系统完整性检查清单

### ✅ 硬件层 (HAL)
- [x] UART6 外设初始化 (`MX_USART6_UART_Init` in main.c:104)
- [x] GPIO 引脚配置 (PG9-RX, PG14-TX, AF8)
- [x] DMA2_Stream1 配置 (Channel_5, Circular 模式)
- [x] NVIC 中断使能 (USART6_IRQn, DMA2_Stream1_IRQn)
- [x] 中断优先级配置 (Priority: 5, SubPriority: 0)

### ✅ FreeRTOS 层
- [x] FreeRTOS 已初始化 (`MX_FREERTOS_Init`)
- [x] 调度器已启动 (`osKernelStart`)
- [x] 中断优先级配置正确 (`configMAX_SYSCALL_INTERRUPT_PRIORITY = 5`)
- [x] DMA 中断优先级 (5) 未被 FreeRTOS 屏蔽

### ✅ 应用层
- [x] PC_Comm 模块初始化 (`PC_Comm_Init` in app_init.c)
- [x] UART6 IDLE 中断处理 (`USART6_IRQHandler`)
- [x] DMA 中断处理 (`DMA2_Stream1_IRQHandler`)
- [x] 回调函数分发 (`Uart_RxIdleCallback` → `PC_Comm_RXCallback`)
- [x] 数据包解析 (`PC_Comm_DecodePacket`)

---

## 🧪 调试步骤

### 第 1 步：编译并下载程序
```bash
# 在 Keil MDK 中
1. Project -> Rebuild all target files
2. Flash -> Download
```

### 第 2 步：验证 DMA 是否运行

在 `app_init.c` 的 `Init_InitAll()` 函数末尾添加：

```c
void Init_InitAll() {
    // ... 原有代码 ...
    
    PC_Comm_Init();// 初始化 PC 通信（上位机/视觉端）
    
    // === 添加调试代码 ===
    if (__HAL_DMA_GET_COUNTER(&hdma_usart6_rx) == PC_COMM_RX_BUFF_LEN) {
        // DMA 计数器正常，正在运行
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_SET);  // 点亮 LED1
    } else {
        // DMA 未正常运行
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_RESET); // 熄灭 LED1
    }
}
```

### 第 3 步：验证中断是否触发

修改 `stm32f4xx_it.c`:

```c
// USART6 中断服务函数
void USART6_IRQHandler(void)
{
    static uint32_t irq_count = 0;
    irq_count++;
    
    // 翻转 LED2 显示中断触发
    HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_13);
    
    /* USER CODE BEGIN USART6_IRQn 0 */
    Uart_ReceiveHandler(&huart6);
    /* USER CODE END USART6_IRQn 0 */
    HAL_UART_IRQHandler(&huart6);
    /* USER CODE BEGIN USART6_IRQn 1 */
    /* USER CODE END USART6_IRQn 1 */
}

// DMA2_Stream1 中断服务函数
void DMA2_Stream1_IRQHandler(void)
{
    static uint32_t dma_irq_count = 0;
    dma_irq_count++;
    
    // 翻转 LED3 显示 DMA 中断
    HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_14);
    
    /* USER CODE BEGIN DMA2_Stream1_IRQn 0 */
    HAL_DMA_IRQHandler(&hdma_usart6_rx);
    /* USER CODE END DMA2_Stream1_IRQn 0 */
}
```

### 第 4 步：验证回调函数执行

修改 `periph_pc_comm.c`:

```c
void PC_Comm_RXCallback(UART_HandleTypeDef* huart) {
    // 翻转 LED4 显示进入回调
    HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_15);
    
    if (huart->Instance == Const_PC_Comm_UART_HANDLER->Instance) {
        // 暂停 DMA 避免数据覆写
        __HAL_DMA_DISABLE(huart->hdmarx);
        
        // 计算当前接收长度
        uint16_t rxdatalen = PC_COMM_RX_BUFF_LEN - 
                            Uart_DMACurrentDataCounter(huart->hdmarx->Instance);
        
        // 翻转 LED5 显示收到数据
        if (rxdatalen > 0) {
            HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);  // 假设有数据就翻转
        }
        
        // ... 原有代码 ...
    }
}
```

### 第 5 步：使用串口助手发送测试数据

**数据格式 (16 字节):**
```
帧头：0xFF
数据：0x00 0x00 0x80 0x3F  (linear_x = 1.0f)
      0x00 0x00 0x00 0x40  (linear_y = 2.0f)
      0x00 0x00 0x40 0x40  (linear_z = 3.0f)
自定义：0x55
CRC8:   0x??  (根据 CRC8_TABLE 计算)
帧尾：0x0D
```

**CRC8 计算工具:**
```c
// 可以编写一个简单的 Python 脚本计算 CRC8
def calculate_crc8(data):
    crc_table = [0x00, 0x31, 0x62, 0x53, ...]  // 使用前 256 个值
    crc = 0x00
    for byte in data:
        crc = crc_table[crc ^ byte]
    return crc
```

### 第 6 步：读取调试信息

如果有条件，可以使用：
1. **ITM/SWO 输出** - 通过 SWO 引脚打印调试信息
2. **UART 打印** - 使用另一个串口（如 USART3）打印状态
3. **逻辑分析仪** - 抓取 UART6、LED 引脚的波形

---

## 📊 预期结果

### ✅ 正常工作情况

| 现象 | LED 状态 | 说明 |
|------|---------|------|
| 系统启动 | LED1 常亮 | DMA 正常运行 |
| 无数据时 | LED2~LED5 不亮 | 没有中断和回调 |
| 收到数据 | LED2 闪烁 | USART6 中断触发 |
| DMA 传输完成 | LED3 闪烁 | DMA 中断触发 |
| 数据处理 | LED4 闪烁 | 回调函数执行 |
| 有效数据帧 | LED5 闪烁 | 检测到数据 |

### ❌ 异常情况及解决方案

#### 情况 1: LED1 不亮
**原因:** DMA 未启动或配置错误  
**解决:**
1. 检查 `MX_DMA_Init()` 是否调用
2. 检查 `MX_USART6_UART_Init()` 是否成功
3. 用调试器查看 `hdma_usart6_rx.Instance->CR` 寄存器

#### 情况 2: LED1 亮但 LED2 不闪
**原因:** 没有收到数据或中断未触发  
**解决:**
1. 检查上位机是否发送数据
2. 用示波器测量 PG9 引脚是否有波形
3. 检查波特率是否匹配 (115200)
4. 检查 TX/RX 是否交叉连接

#### 情况 3: LED2 闪但 LED4 不闪
**原因:** 进入了中断但回调函数未执行  
**解决:**
1. 检查 `Uart_ReceiveHandler()` 中的 IDLE 标志判断
2. 在 `USART6_IRQHandler` 中设置断点调试
3. 检查 `huart6.hdmarx` 是否为 NULL

#### 情况 4: LED4 闪但数据不正确
**原因:** 数据解析错误或 CRC 校验失败  
**解决:**
1. 在 `PC_Comm_DecodePacket` 中设置断点
2. 打印 `PC_Comm_RxData` 数组内容
3. 手动计算 CRC 并与接收到的对比

---

## 🔧 高级调试技巧

### 技巧 1: 使用 printf 调试

```c
// 重定向 printf 到 USART3
int fputc(int ch, FILE *f) {
    HAL_UART_Transmit(&huart3, (uint8_t *)&ch, 1, 0xFFFF);
    return ch;
}

// 在关键位置打印
void PC_Comm_RXCallback(UART_HandleTypeDef* huart) {
    uint16_t rxdatalen = PC_COMM_RX_BUFF_LEN - 
                        Uart_DMACurrentDataCounter(huart->hdmarx->Instance);
    printf("[PC_Comm] RX Callback! Len=%d\n", rxdatalen);
    
    for (int i = 0; i < rxdatalen && i < 20; i++) {
        printf("%02X ", PC_Comm_RxData[i]);
    }
    printf("\n");
    
    // ... 原有代码 ...
}
```

### 技巧 2: 使用调试变量

```c
// 定义全局调试变量
volatile struct {
    uint32_t dma_counter;
    uint32_t usart_irq_count;
    uint32_t dma_irq_count;
    uint32_t callback_count;
    uint32_t last_rxdatalen;
    uint8_t rx_buffer[64];
} g_debug;

// 在中断中更新
void USART6_IRQHandler(void) {
    g_debug.usart_irq_count++;
    // ...
}

void PC_Comm_RXCallback(UART_HandleTypeDef* huart) {
    g_debug.callback_count++;
    g_debug.last_rxdatalen = PC_COMM_RX_BUFF_LEN - 
                             Uart_DMACurrentDataCounter(huart->hdmarx->Instance);
    memcpy(g_debug.rx_buffer, PC_Comm_RxData, 64);
    // ...
}
```

### 技巧 3: 使用示波器/逻辑分析仪

连接以下信号：
- **UART6 RX (PG9)** - 观察接收波形
- **UART6 TX (PG14)** - 观察是否有异常发送
- **LED 引脚** - 观察各阶段执行情况
- **DMA 请求信号** - 观察 DMA 触发时机

---

## 📝 维护建议

1. **统一接口:** 所有 UART DMA 接收都使用 `Uart_InitUartDMA()` + `Uart_ReceiveDMA()`
2. **文档化:** 在代码中添加详细注释说明数据格式和协议
3. **模块化:** 将数据解析与接收分离，便于维护
4. **错误处理:** 添加超时检测和自动恢复机制
5. **性能监控:** 定期检查缓冲区是否溢出

---

## 🎯 下一步行动

1. ✅ 编译下载修复后的代码
2. ⬜ 执行调试步骤 1-6
3. ⬜ 记录 LED 状态和现象
4. ⬜ 根据现象对照故障排除表
5. ⬜ 如仍无法解决，提供调试信息寻求进一步帮助

---

**修复日期:** 2026-03-14  
**修复版本:** v1.0  
**相关文件:**
- `Src/Periphal/periph_pc_comm.c` ✅ 已修复
- `Src/Utility/util_uart.c` ✅ 无需修改
- `Src/Application/app_init.c` ✅ 无需修改
- `Src/stm32f4xx_it.c` ✅ 无需修改
