/*
 * Project      : Infantry_Neptune
 * file         : periph_pc_comm.c
 * Description  : 接收电脑（上位机/视觉端）发送的定长 16 字节通信数据，带 CRC8 校验
 * Date         : 202X-XX-XX
 */

#include "periph_pc_comm.h"
#include "periph_remote.h"
#include "util_uart.h"


UART_HandleTypeDef* Const_PC_Comm_UART_HANDLER = &huart6;

uint8_t PC_Comm_RxData[PC_COMM_RX_BUFF_LEN];
PC_Comm_DataTypeDef PC_Comm_Data;    // 原始接收数据
PC_ParsedData_t PC_Parsed;           // 解析后的结构化数据

// RMOSS CRC8 查表表 (多项式: 0x31)
static const uint8_t CRC8_TABLE[256] = {
    0x00, 0x31, 0x62, 0x53, 0xc4, 0xf5, 0xa6, 0x97, 0xb9, 0x88, 0xdb, 0xea, 0x7d, 0x4c, 0x1f, 0x2e,
    0x43, 0x72, 0x21, 0x10, 0x87, 0xb6, 0xe5, 0xd4, 0xfa, 0xcb, 0x98, 0xa9, 0x3e, 0x0f, 0x5c, 0x6d,
    0x86, 0xb7, 0xe4, 0xd5, 0x42, 0x73, 0x20, 0x11, 0x3f, 0x0e, 0x5d, 0x6c, 0xfb, 0xca, 0x99, 0xa8,
    0xc5, 0xf4, 0xa7, 0x96, 0x01, 0x30, 0x63, 0x52, 0x7c, 0x4d, 0x1e, 0x2f, 0xb8, 0x89, 0xda, 0xeb,
    0x3d, 0x0c, 0x5f, 0x6e, 0xf9, 0xc8, 0x9b, 0xaa, 0x84, 0xb5, 0xe6, 0xd7, 0x40, 0x71, 0x22, 0x13,
    0x7e, 0x4f, 0x1c, 0x2d, 0xba, 0x8b, 0xd8, 0xe9, 0xc7, 0xf6, 0xa5, 0x94, 0x03, 0x32, 0x61, 0x50,
    0xbb, 0x8a, 0xd9, 0xe8, 0x7f, 0x4e, 0x1d, 0x2c, 0x02, 0x33, 0x60, 0x51, 0xc6, 0xf7, 0xa4, 0x95,
    0xf8, 0xc9, 0x9a, 0xab, 0x3c, 0x0d, 0x5e, 0x6f, 0x41, 0x70, 0x23, 0x12, 0x85, 0xb4, 0xe7, 0xd6,
    0x7a, 0x4b, 0x18, 0x29, 0xbe, 0x8f, 0xdc, 0xed, 0xc3, 0xf2, 0xa1, 0x90, 0x07, 0x36, 0x65, 0x54,
    0x39, 0x08, 0x5b, 0x6a, 0xfd, 0xcc, 0x9f, 0xae, 0x80, 0xb1, 0xe2, 0xd3, 0x44, 0x75, 0x26, 0x17,
    0xfc, 0xcd, 0x9e, 0xaf, 0x38, 0x09, 0x5a, 0x6b, 0x45, 0x74, 0x27, 0x16, 0x81, 0xb0, 0xe3, 0xd2,
    0xbf, 0x8e, 0xdd, 0xec, 0x7b, 0x4a, 0x19, 0x28, 0x06, 0x37, 0x64, 0x55, 0xc2, 0xf3, 0xa0, 0x91,
    0x47, 0x76, 0x25, 0x14, 0x83, 0xb2, 0xe1, 0xd0, 0xfe, 0xcf, 0x9c, 0xad, 0x3a, 0x0b, 0x58, 0x69,
    0x04, 0x35, 0x66, 0x57, 0xc0, 0xf1, 0xa2, 0x93, 0xbd, 0x8c, 0xdf, 0xee, 0x79, 0x48, 0x1b, 0x2a,
    0xc1, 0xf0, 0xa3, 0x92, 0x05, 0x34, 0x67, 0x56, 0x78, 0x49, 0x1a, 0x2b, 0xbc, 0x8d, 0xde, 0xef,
    0x82, 0xb3, 0xe0, 0xd1, 0x46, 0x77, 0x24, 0x15, 0x3b, 0x0a, 0x59, 0x68, 0xff, 0xce, 0x9d, 0xac
};

void PC_Comm_ResetData(void) {
    PC_Comm_StateEnum state = PC_Comm_Data.state;
    uint32_t last_update_time = PC_Comm_Data.last_update_time;
    
    memset(&PC_Comm_Data, 0, sizeof(PC_Comm_DataTypeDef));
    memset(&PC_Parsed, 0, sizeof(PC_ParsedData_t));
    
    PC_Comm_Data.state = state;
    PC_Comm_Data.last_update_time = last_update_time;
}

void PC_Comm_Init(void) {
    PC_Comm_ResetData();
    PC_Comm_Data.last_update_time = HAL_GetTick();
    
    // 开启 DMA 循环接收
    Uart_InitUartDMA(Const_PC_Comm_UART_HANDLER);
    Uart_ReceiveDMA(Const_PC_Comm_UART_HANDLER, PC_Comm_RxData, PC_COMM_RX_BUFF_LEN);
}

/**
  * @brief      计算 RMOSS 规则下的 CRC8 校验值
  * @param      data: 数据区指针
  * @param      len: 数据长度
  * @retval     计算得出的 CRC8 值
  */
uint8_t PC_Comm_CalculateCRC8(const uint8_t *data, uint16_t len) {
    uint8_t crc = 0x00; // 初始化值为 0x00
    for (uint16_t i = 0; i < len; ++i) {
        crc = CRC8_TABLE[crc ^ data[i]];
    }
    return crc;
}

/**
  * @brief      验证整个16字节包的校验和
  * @param      buff: 指向包头(0xFF)的指针
  * @retval     1: 校验成功, 0: 校验失败
  */
uint8_t PC_Comm_VerifyChecksum(uint8_t *buff) {
    // 只对 payload（buff[1]~buff[13]）算校验和
    uint8_t expected_crc = PC_Comm_CalculateCRC8(&buff[1], 13);

    if (expected_crc == buff[14]) {
        return 1; // 校验通过
    }
    return 0; // 校验失败
}

/**
  * @brief      串口接收中断回调函数 (在 USART6 的 IDLE 中断中调用)
  * @param      huart: 指针指向串口句柄
  * @retval     无
  */
void PC_Comm_RXCallback(UART_HandleTypeDef* huart) {
    if (huart->Instance == Const_PC_Comm_UART_HANDLER->Instance) {
        // 暂停 DMA 避免数据覆写
        __HAL_DMA_DISABLE(huart->hdmarx);
        
        // 计算当前接收长度
        uint16_t rxdatalen = PC_COMM_RX_BUFF_LEN - Uart_DMACurrentDataCounter(huart->hdmarx->Instance);
        
        // 滑动窗口寻找有效帧
        for (uint16_t i = 0; i <= rxdatalen - PC_COMM_PACKET_LEN; i++) {
            if (PC_Comm_RxData[i] == PC_COMM_HEADER_SOF) {
                // 找到帧头，检查帧尾和校验
                if (PC_Comm_RxData[i + 15] == PC_COMM_TAIL_EOF) {
                    // 送去解包与校验
                    PC_Comm_DecodePacket(&PC_Comm_RxData[i], rxdatalen - i);
                    i += (PC_COMM_PACKET_LEN - 1); // 跳过当前帧长度
                }
            }
        }
        
        // 重置 DMA 接收计数并重启 DMA
        __HAL_DMA_SET_COUNTER(huart->hdmarx, PC_COMM_RX_BUFF_LEN);
        __HAL_DMA_ENABLE(huart->hdmarx);
    }
}



/**
  * @brief      数据包解包与业务分发函数（新版本 - 使用嵌套结构体）
  */
void PC_Comm_DecodePacket(uint8_t *buff, uint16_t rxdatalen) {
    // 1-4. 前面的长度、帧头、帧尾、CRC 校验逻辑保持不变...
    if (rxdatalen < PC_COMM_PACKET_LEN) return; 
    if (buff[0] != PC_COMM_HEADER_SOF) return;
    if (buff[15] != PC_COMM_TAIL_EOF) return;
    if (!PC_Comm_VerifyChecksum(buff)) return; 
    
    // --- 通过所有校验 ---
    PC_Comm_Data.state = PC_COMM_STATE_CONNECTED;
    PC_Comm_Data.last_update_time = HAL_GetTick();
    
    // 暂存这 13 个有效字节 (buff[1] 到 buff[13])
    memcpy(PC_Comm_Data.raw_data_payload, &buff[1], PC_COMM_DATA_LEN);
    
    // 提取命令码 (在 payload 的第 0 个位置)
    uint8_t cmd_id = PC_Comm_Data.raw_data_payload[0];
    
    // 提取纯数据段指针 (在 payload 的第 1 个位置，跳过命令码)
    uint8_t *data_ptr = &PC_Comm_Data.raw_data_payload[1];
    
    // 根据命令码，将数据指针强转为对应的结构体指针并读取
    switch (cmd_id) {
        case PC_CMD_GIMBAL: {
            PC_Recv_Gimbal_t *gimbal = (PC_Recv_Gimbal_t *)data_ptr;
            // 存储到控制数据结构体
            PC_Parsed.control.gimbal = *gimbal;
            // TODO: 调用云台控制接口，例如：
            // Gimbal_SetTargetAngle(gimbal->pitch, gimbal->yaw);
            break;
        }
        case PC_CMD_CHASSIS: {
            PC_Recv_Chassis_t *chassis = (PC_Recv_Chassis_t *)data_ptr;
            // 存储到底盘控制数据结构体
            PC_Parsed.control.chassis = *chassis;
            // TODO: 调用底盘控制接口，例如：
            // Chassis_SetSpeed(chassis->vx, chassis->vy, chassis->wz);
            break;
        }
        case PC_CMD_POWER_HEAT: {
            PC_Recv_PowerHeat_t *pwr = (PC_Recv_PowerHeat_t *)data_ptr;
            // 存储到裁判系统数据结构体
            PC_Parsed.referee.power_heat = *pwr;
            // TODO: 更新功率和热量变量
            break;
        }
        case PC_CMD_DAMAGE_HEALTH: {
            PC_Recv_Damage_t *dmg = (PC_Recv_Damage_t *)data_ptr;
            // 存储到裁判系统数据结构体
            PC_Parsed.referee.damage = *dmg;
            // TODO: 更新扣血状态
            break;
        }
        case PC_CMD_SHOOT_STATUS: {
            PC_Recv_Shoot_t *shoot = (PC_Recv_Shoot_t *)data_ptr;
            // 存储到裁判系统数据结构体
            PC_Parsed.referee.shoot = *shoot;
            // TODO: 更新射速和余弹量
            break;
        }
        case PC_CMD_FIELD_BUFF: {
            PC_Recv_FieldBuff_t *field_buff = (PC_Recv_FieldBuff_t *)data_ptr;
            // 存储到裁判系统数据结构体
            PC_Parsed.referee.field_buff = *field_buff;
            // TODO: 更新增益状态
            break;
        }
        default:
            // 未知指令
            break;
    }
}

/**
  * @brief      获取解析后的数据指针
  * @param      NULL
  * @retval     指向 PC_ParsedData_t 结构体的指针
  */
PC_ParsedData_t* PC_Comm_GetParsedDataPtr(void) {
    return &PC_Parsed;
}
