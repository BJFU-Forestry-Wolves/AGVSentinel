# PC 通信模块 - 嵌套结构体使用说明

## 📋 概述

为了方便数据管理和访问，PC 通信模块采用了**分层嵌套结构体**设计，将所有接收到的数据分为两大类：

1. **控制数据 (Control Data)** - 来自上位机/视觉端的实时控制指令
2. **裁判系统数据 (Referee Data)** - 比赛状态、硬件反馈等信息

---

## 🏗️ 数据结构层次

```
PC_ParsedData_t (总结构体)
├── PC_ControlData_t (控制数据集合)
│   ├── PC_Recv_Gimbal_t (云台控制)
│   │   ├── reserved
│   │   ├── pitch (float)
│   │   └── yaw (float)
│   └── PC_Recv_Chassis_t (底盘控制)
│       ├── vx (int16_t)
│       ├── vy (int16_t)
│       └── wz (int16_t)
│
└── PC_RefereeData_t (裁判系统数据集合)
    ├── PC_Recv_PowerHeat_t (功率与热量)
    │   ├── chassis_power_limit
    │   ├── shooter_barrel_heat_limit
    │   ├── shooter_barrel_cooling_value
    │   ├── buffer_energy
    │   ├── shooter_17mm_barrel_heat
    │   └── shooter_42mm_barrel_heat
    ├── PC_Recv_Damage_t (伤害与血量)
    │   ├── current_hp
    │   ├── armor_id (4 bits)
    │   └── hp_deduction_reason (4 bits)
    ├── PC_Recv_Shoot_t (射击反馈)
    │   ├── launching_frequency
    │   ├── initial_speed
    │   ├── allowance_17mm
    │   └── allowance_42mm
    └── PC_Recv_FieldBuff_t (场地 Buff)
        ├── rfid_status
        ├── recovery_buff
        ├── cooling_buff
        ├── defence_buff
        └── vulnerability_buff
```

---

## 💡 使用方法

### 方法一：直接访问结构体成员

```c
#include "periph_pc_comm.h"

void Control_Task(void const * argument) {
    // 获取解析后的数据指针
    PC_ParsedData_t* pc_data = PC_Comm_GetParsedDataPtr();
    
    while (1) {
        // ===== 读取控制数据 =====
        
        // 1. 云台控制
        float pitch_angle = pc_data->control.gimbal.pitch;
        float yaw_angle = pc_data->control.gimbal.yaw;
        Gimbal_SetTargetAngle(pitch_angle, yaw_angle);
        
        // 2. 底盘控制
        int16_t vx = pc_data->control.chassis.vx;
        int16_t vy = pc_data->control.chassis.vy;
        int16_t wz = pc_data->control.chassis.wz;
        Chassis_SetSpeed(vx, vy, wz);
        
        // ===== 读取裁判系统数据 =====
        
        // 3. 功率和热量监控
        uint16_t power_limit = pc_data->referee.power_heat.chassis_power_limit;
        uint16_t heat_limit = pc_data->referee.power_heat.shooter_barrel_heat_limit;
        
        // 4. 血量状态
        uint16_t current_hp = pc_data->referee.damage.current_hp;
        if (current_hp < 100) {
            // 低血量警告
            LED_Red_On();
        }
        
        // 5. 射击状态
        uint16_t ammo_17mm = pc_data->referee.shoot.allowance_17mm;
        uint16_t ammo_42mm = pc_data->referee.shoot.allowance_42mm;
        
        // 6. Buff 状态
        uint8_t has_recovery = pc_data->referee.field_buff.recovery_buff;
        if (has_recovery) {
            // 有回血增益
        }
        
        osDelay(10); // 10ms 周期
    }
}
```

---

### 方法二：通过全局变量直接访问

```c
#include "periph_pc_comm.h"

// 在 periph_pc_comm.h 中已声明 extern
extern PC_ParsedData_t PC_Parsed;

void Some_Function(void) {
    // 直接访问全局变量
    float pitch = PC_Parsed.control.gimbal.pitch;
    float yaw = PC_Parsed.control.gimbal.yaw;
    
    int16_t vx = PC_Parsed.control.chassis.vx;
    int16_t vy = PC_Parsed.control.chassis.vy;
    
    // ... 使用数据
}
```

---

### 方法三：局部缓存（推荐用于高频访问）

```c
#include "periph_pc_comm.h"

void High_Frequency_Task(void) {
    // 缓存到局部变量，减少指针解引用
    PC_ParsedData_t parsed_data = *PC_Comm_GetParsedDataPtr();
    
    // 使用局部变量访问
    float pitch = parsed_data.control.gimbal.pitch;
    float yaw = parsed_data.control.gimbal.yaw;
    
    // 这样访问速度更快，适合控制循环
}
```

---

## 🎯 完整应用示例

### 示例 1：云台手动控制

```c
#include "periph_pc_comm.h"
#include "module_gimbal.h"

void Gimbal_Control_Task(void const * argument) {
    PC_ParsedData_t* pc_data;
    
    while (1) {
        pc_data = PC_Comm_GetParsedDataPtr();
        
        // 检查是否有新的控制数据
        if (PC_Comm_Data.state == PC_COMM_STATE_CONNECTED) {
            // 从 PC 控制数据中读取云台角度
            float target_pitch = pc_data->control.gimbal.pitch;
            float target_yaw = pc_data->control.gimbal.yaw;
            
            // 限制角度范围
            target_pitch = CONSTRAIN(target_pitch, -30.0f, 30.0f);
            target_yaw = CONSTRAIN(target_yaw, -180.0f, 180.0f);
            
            // 设置云台目标角度
            GimbalPitch_SetPitchRef(target_pitch);
            GimbalYaw_SetYawRef(target_yaw);
        }
        
        osDelay(5); // 5ms 控制周期
    }
}
```

---

### 示例 2：底盘遥控 + 功率限制

```c
#include "periph_pc_comm.h"
#include "module_chassis.h"

void Chassis_Control_Task(void const * argument) {
    PC_ParsedData_t* pc_data;
    float current_power_limit;
    
    while (1) {
        pc_data = PC_Comm_GetParsedDataPtr();
        
        if (PC_Comm_Data.state == PC_COMM_STATE_CONNECTED) {
            // 读取速度指令
            int16_t vx = pc_data->control.chassis.vx;
            int16_t vy = pc_data->control.chassis.vy;
            int16_t wz = pc_data->control.chassis.wz;
            
            // 读取功率限制
            current_power_limit = (float)pc_data->referee.power_heat.chassis_power_limit;
            
            // 应用功率限制
            Chassis_SetPowerLimit(current_power_limit);
            
            // 设置底盘速度
            Chassis_SetChassisRef((float)vx, (float)vy, (float)wz);
        }
        
        osDelay(5); // 5ms 控制周期
    }
}
```

---

### 示例 3：射击管理系统

```c
#include "periph_pc_comm.h"
#include "module_shoot.h"

void Shoot_Manage_Task(void const * argument) {
    PC_ParsedData_t* pc_data;
    uint16_t ammo_total;
    
    while (1) {
        pc_data = PC_Comm_GetParsedDataPtr();
        
        if (PC_Comm_Data.state == PC_COMM_STATE_CONNECTED) {
            // 计算总弹药量
            ammo_total = pc_data->referee.shoot.allowance_17mm + 
                        pc_data->referee.shoot.allowance_42mm;
            
            // 根据剩余弹药调整射击策略
            if (ammo_total < 10) {
                // 弹药不足，限制射击
                Shooter_ChangeFeederMode(Feeder_FINISH);
            }
            else if (ammo_total < 50) {
                // 弹药紧张，单发模式
                Shooter_ChangeFeederMode(Feeder_SINGLE);
            }
            else {
                // 弹药充足，连发模式
                Shooter_ChangeFeederMode(Feeder_FAST_CONTINUE);
            }
            
            // 监控枪口热量
            uint16_t heat_17mm = pc_data->referee.power_heat.shooter_17mm_barrel_heat;
            uint16_t heat_42mm = pc_data->referee.power_heat.shooter_42mm_barrel_heat;
            
            if (heat_17mm > 800 || heat_42mm > 800) {
                // 过热警告
                Shooter_CoolDown();
            }
        }
        
        osDelay(10); // 10ms 管理周期
    }
}
```

---

### 示例 4：血量监控与自动规避

```c
#include "periph_pc_comm.h"
#include "app_autoaim.h"

void Health_Monitor_Task(void const * argument) {
    PC_ParsedData_t* pc_data;
    uint8_t last_armor_id = 0;
    uint32_t hit_time = 0;
    
    while (1) {
        pc_data = PC_Comm_GetParsedDataPtr();
        
        if (PC_Comm_Data.state == PC_COMM_STATE_CONNECTED) {
            // 监控当前血量
            uint16_t current_hp = pc_data->referee.damage.current_hp;
            
            // 检测是否受到攻击
            if (pc_data->referee.damage.hp_deduction_reason != 0) {
                uint8_t hit_armor = pc_data->referee.damage.armor_id;
                
                if (hit_armor != last_armor_id) {
                    // 新受到攻击
                    hit_time = HAL_GetTick();
                    last_armor_id = hit_armor;
                    
                    // 根据受击装甲板调整姿态
                    switch (hit_armor) {
                        case 0: // 前装甲
                            AutoAim_Retreat();
                            break;
                        case 1: // 右装甲
                            AutoAim_TurnLeft();
                            break;
                        case 2: // 后装甲
                            AutoAim_Forward();
                            break;
                        case 3: // 左装甲
                            AutoAim_TurnRight();
                            break;
                    }
                }
            }
            
            // 低血量保护
            if (current_hp < 50) {
                // 进入防御模式
                Chassis_SetChassisMode(Chassis_DEFEND);
            }
        }
        
        osDelay(5); // 5ms 监控周期
    }
}
```

---

## 📊 数据更新频率建议

| 数据类型 | 建议更新频率 | 说明 |
|:-------:|:----------:|------|
| 云台控制 | 100-200Hz | 需要快速响应 |
| 底盘控制 | 100-200Hz | 需要快速响应 |
| 功率热量 | 10-50Hz | 变化较慢 |
| 血量状态 | 10-50Hz | 事件驱动 |
| 射击状态 | 50-100Hz | 中等频率 |
| Buff 状态 | 10Hz | 变化很慢 |

---

## ⚠️ 注意事项

### 1. 线程安全

```c
// ❌ 错误示范：在中断中直接访问
void Some_ISR(void) {
    float pitch = PC_Parsed.control.gimbal.pitch; // 危险！
}

// ✅ 正确做法：关闭中断或使用互斥锁
void Safe_Access(void) {
    __disable_irq();
    float pitch = PC_Parsed.control.gimbal.pitch;
    __enable_irq();
}
```

### 2. 数据有效性检查

```c
// ✅ 始终检查连接状态
if (PC_Comm_Data.state == PC_COMM_STATE_CONNECTED) {
    // 只有连接正常时才使用数据
    float pitch = PC_Parsed.control.gimbal.pitch;
} else {
    // 连接丢失，使用默认值或安全值
    float pitch = 0.0f;
}
```

### 3. 内存对齐

```c
// 结构体已使用 #pragma pack(push, 1) 单字节对齐
// 不要随意更改结构体顺序，可能导致解析错误
```

---

## 🔧 调试技巧

### 打印数据结构大小

```c
printf("PC_ParsedData_t size: %d bytes\r\n", sizeof(PC_ParsedData_t));
printf("PC_ControlData_t size: %d bytes\r\n", sizeof(PC_ControlData_t));
printf("PC_RefereeData_t size: %d bytes\r\n", sizeof(PC_RefereeData_t));
```

### 监控数据更新

```c
static uint32_t last_update = 0;
if (PC_Comm_Data.last_update_time != last_update) {
    printf("New data received!\r\n");
    printf("Gimbal: pitch=%.2f, yaw=%.2f\r\n", 
           PC_Parsed.control.gimbal.pitch, 
           PC_Parsed.control.gimbal.yaw);
    last_update = PC_Comm_Data.last_update_time;
}
```

---

## 📝 总结

**优势:**
- ✅ **结构清晰** - 控制数据和状态数据分离
- ✅ **易于维护** - 添加新字段只需修改对应子结构体
- ✅ **类型安全** - 编译器可以检查类型错误
- ✅ **访问方便** - 统一的访问接口

**使用要点:**
1. 优先使用 `PC_Comm_GetParsedDataPtr()` 获取指针
2. 始终检查 `PC_Comm_Data.state` 确保连接正常
3. 注意数据更新频率，避免过度访问
4. 在多线程环境中注意线程安全

---

**文档版本:** v1.0  
**更新日期:** 2026-03-14
