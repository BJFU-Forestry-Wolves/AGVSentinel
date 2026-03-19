/*
 * Project      : Infantry_Neptune
 * file         : periph_pc_comm.h
 * Description  : 接收电脑（上位机/视觉端）发送的定长 16 字节通信数据，带 CRC8 校验
 * Date         : 202X-XX-XX
 */

#ifndef PERIPH_PC_COMM_H
#define PERIPH_PC_COMM_H

#ifdef __cplusplus
extern "C" {
#endif

#include "usart.h"
#include "stdint.h"
#include "string.h"

#include "util_uart.h"
#include "stdlib.h"

/********** START OF PC COMM MACRO DEFINITION **********/

#define PC_COMM_PACKET_LEN      16      // 数据包固定长度 16 Bytes
#define PC_COMM_HEADER_SOF      0xFF    // 帧头
#define PC_COMM_TAIL_EOF        0x0D    // 帧尾
#define PC_COMM_DATA_LEN        13      // 纯数据段长度
#define PC_COMM_RX_BUFF_LEN     64      // 接收缓冲区大小 (建议设为包长的整数倍)

/********** END OF PC COMM MACRO DEFINITION **********/

extern UART_HandleTypeDef* Const_PC_Comm_UART_HANDLER;

// 电脑端通信状态枚举
typedef enum {
    PC_COMM_STATE_NULL      = 0,
    PC_COMM_STATE_CONNECTED = 1,
    PC_COMM_STATE_LOST      = 2,
    PC_COMM_STATE_ERROR     = 3
} PC_Comm_StateEnum;

// 核心数据结构体
typedef struct {
    PC_Comm_StateEnum state;             // 当前连接状态
    uint32_t          last_update_time;  // 上次接收到有效数据的时间戳 (用于判断离线)
    
    // 暂存空间：存放通过了CRC校验的13字节有效数据
    uint8_t raw_data_payload[PC_COMM_DATA_LEN]; 
} PC_Comm_DataTypeDef;


// 命令码枚举
typedef enum {
    PC_CMD_GIMBAL          = 0x01,
    PC_CMD_CHASSIS         = 0x02,
    PC_CMD_POWER_HEAT      = 0x10,
    PC_CMD_DAMAGE_HEALTH   = 0x11,
    PC_CMD_SHOOT_STATUS    = 0x12,
    PC_CMD_FIELD_BUFF      = 0x13
} PC_Comm_CmdID_e;

// 必须使用单字节对齐，防止单片机为了内存访问效率自动在中间插入空白字节
#pragma pack(push, 1)

// ========== 控制数据类 (上位机/视觉端发送的控制指令) ==========

// 0x01: 云台控制
typedef struct {
    uint8_t reserved;
    float   pitch;
    float   yaw;
} PC_Recv_Gimbal_t;

// 0x02: 底盘控制
typedef struct {
    int16_t vx;
    int16_t vy;
    int16_t wz;
} PC_Recv_Chassis_t;

// ========== 裁判系统数据类 (状态反馈和比赛数据) ==========

// 0x10: 功率与热量
typedef struct {
    uint16_t chassis_power_limit;
    uint16_t shooter_barrel_heat_limit;
    uint16_t shooter_barrel_cooling_value;
    uint16_t buffer_energy;
    uint16_t shooter_17mm_barrel_heat;
    uint16_t shooter_42mm_barrel_heat;
} PC_Recv_PowerHeat_t;

// 0x11: 伤害与血量
typedef struct {
    uint16_t current_hp;
    uint8_t  armor_id : 4;            // 位域：低 4 位
    uint8_t  hp_deduction_reason : 4; // 位域：高 4 位
} PC_Recv_Damage_t;

// 0x12: 射击反馈
typedef struct {
    uint8_t  launching_frequency;
    float    initial_speed;
    uint16_t allowance_17mm;
    uint16_t allowance_42mm;
} PC_Recv_Shoot_t;

// 0x13: 场地与增益
typedef struct {
    uint32_t rfid_status;
    uint8_t  recovery_buff;
    uint16_t cooling_buff;
    uint8_t  defence_buff;
    uint8_t  vulnerability_buff;
} PC_Recv_FieldBuff_t;

// ========== 联合结构体 - 控制数据集合 ==========
typedef struct {
    PC_Recv_Gimbal_t     gimbal;      // 云台控制数据
    PC_Recv_Chassis_t    chassis;     // 底盘控制数据
} PC_ControlData_t;

// ========== 联合结构体 - 裁判系统数据集合 ==========
typedef struct {
    PC_Recv_PowerHeat_t   power_heat;  // 功率与热量数据
    PC_Recv_Damage_t      damage;      // 伤害与血量数据
    PC_Recv_Shoot_t       shoot;       // 射击反馈数据
    PC_Recv_FieldBuff_t   field_buff;  // 场地与增益数据
} PC_RefereeData_t;

// ========== 总数据结构体 - 所有解析后的数据 ==========
typedef struct {
    PC_ControlData_t   control;     // 控制数据集合
    PC_RefereeData_t   referee;     // 裁判系统数据集合
} PC_ParsedData_t;

#pragma pack(pop)


extern PC_Comm_DataTypeDef PC_Comm_Data;
extern PC_ParsedData_t PC_Parsed;  // 解析后的总数据结构体

// 函数声明
void PC_Comm_Init(void);
void PC_Comm_ResetData(void);
uint8_t PC_Comm_CalculateCRC8(const uint8_t *data, uint16_t len);
uint8_t PC_Comm_VerifyChecksum(uint8_t *buff);
void PC_Comm_DecodePacket(uint8_t *buff, uint16_t rxdatalen);
void PC_Comm_RXCallback(UART_HandleTypeDef* huart);
PC_Comm_DataTypeDef* PC_Comm_GetDataPtr(void);
PC_ParsedData_t* PC_Comm_GetParsedDataPtr(void);  // 获取解析后的数据指针

#endif

#ifdef __cplusplus
}
#endif
