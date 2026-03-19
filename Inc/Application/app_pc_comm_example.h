/*
 * Project      : Infantry_Neptune
 * file         : app_pc_comm_example.h
 * Description  : PC 通信（上位机/视觉端）应用层示例代码头文件
 * Date         : 202X-XX-XX
 */

#ifndef APP_PC_COMM_EXAMPLE_H
#define APP_PC_COMM_EXAMPLE_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stdint.h"
#include "periph_pc_comm.h"

// 目标数据结构（根据实际需要修改）
typedef struct {
    float linear_x;
    float linear_y;
    float linear_z;
    float angular_x;
    float angular_y;
    float angular_z;
} TargetData;

// 函数声明
void PC_Comm_ExtractTargetData(TargetData* target);
uint8_t PC_Comm_IsOnline(void);
PC_Comm_StateEnum PC_Comm_GetState(void);

#ifdef __cplusplus
}
#endif

#endif
