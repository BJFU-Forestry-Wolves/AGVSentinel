/*
 *  Project      : Polaris
 * 
 *  file         : cha_gimbal_ctrl.c
 *  Description  : This file contains Gimbal Pitch control function
 *  LastEditors  : Polaris
 *  Date         : 2021-05-04 20:53:31
 *  LastEditTime : 2023-05-05 16:27:28
 */


#include "cmsis_os.h"
#include "sys_const.h"
#include "module_gimbal.h"
#include "protocol_common.h"
#include "app_ins.h"
#include "periph_motor.h"
#include "sys_const.h"
#include "periph_DMmotor.h"

static float GimbalYaw_Normalize360(float ang) {
    while (ang < 0.0f) ang += 360.0f;
    while (ang >= 360.0f) ang -= 360.0f;
    return ang;
}

GimbalPitch_GimbalPitchTypeDef GimbalPitch_gimbalPitchControlData;
GimbalYaw_GimbalYawTypeDef GimbalYaw_gimbalYawControlData;

static PID_PIDTypeDef GimbalSmallYaw_spdPID;
static PID_PIDParamTypeDef GimbalSmallYaw_spdPIDParam;
static PID_PIDTypeDef GimbalSmallYaw_angPID;
static PID_PIDParamTypeDef GimbalSmallYaw_angPIDParam;

/* 上电时 IMU 的 YawTotalAngle 初值常带固定偏置（例如 -90° 对齐），若 yaw_ref 固定为 0 会导致大 yaw 每次上电都“追 0°”转一段。
 * 这里在第一次进入闭环控制时，把 yaw_ref 锁到当前 IMU 角度，避免上电自转。 */
static uint8_t GimbalYaw_need_align_ref = 1;//上电/首次进入闭环时的对齐标志
/**
  * @brief      Gimbal pitch control initialization
  * @param      NULL
  * @retval     NULL
  */
void GimbalPitch_InitGimbalPitch() {
    GimbalPitch_GimbalPitchTypeDef *gimbalpitch = GimbalPitch_GetGimbalPitchPtr();

    gimbalpitch->control_state = 1;
    gimbalpitch->output_state = 1;
    gimbalpitch->pitch_ref = 0;
    gimbalpitch->pitch_count = 0;

    PID_InitPIDParam(&gimbalpitch->spdPIDParam, Const_GimbalPitchSpdParam[0][0], Const_GimbalPitchSpdParam[0][1], Const_GimbalPitchSpdParam[0][2], Const_GimbalPitchSpdParam[0][3], 
                    Const_GimbalPitchSpdParam[0][4], Const_GimbalPitchSpdParam[1][0], Const_GimbalPitchSpdParam[1][1], Const_GimbalPitchSpdParam[2][0], Const_GimbalPitchSpdParam[2][1], 
                    Const_GimbalPitchSpdParam[3][0], Const_GimbalPitchSpdParam[3][1], PID_POSITION);
    PID_InitPIDParam(&gimbalpitch->angPIDParam, Const_GimbalPitchAngParam[0][0], Const_GimbalPitchAngParam[0][1], Const_GimbalPitchAngParam[0][2], Const_GimbalPitchAngParam[0][3], 
                    Const_GimbalPitchAngParam[0][4], Const_GimbalPitchAngParam[1][0], Const_GimbalPitchAngParam[1][1], Const_GimbalPitchAngParam[2][0], Const_GimbalPitchAngParam[2][1], 
                    Const_GimbalPitchAngParam[3][0], Const_GimbalPitchAngParam[3][1], PID_POSITION);                      
}

/**
  * @brief      Gimbal yaw control initialization
  * @param      NULL
  * @retval     NULL
  */
void GimbalYaw_InitGimbalYaw() {
    GimbalYaw_GimbalYawTypeDef *gimbalyaw = GimbalYaw_GetGimbalYawPtr();

    gimbalyaw->control_state = 1;
    gimbalyaw->output_state = 1;
    gimbalyaw->yaw_ref = 0;
    gimbalyaw->yaw_count = 0;
    GimbalYaw_need_align_ref = 1;

    /* 大yaw6020 */
    Motor_SetMotorOutput(&Motor_Big_YawMotor, 0.0f);

    PID_InitPIDParam(&gimbalyaw->spdPIDParam, Const_GimbalYawSpdParam[0][0], Const_GimbalYawSpdParam[0][1], Const_GimbalYawSpdParam[0][2], Const_GimbalYawSpdParam[0][3], 
                    Const_GimbalYawSpdParam[0][4], Const_GimbalYawSpdParam[1][0], Const_GimbalYawSpdParam[1][1], Const_GimbalYawSpdParam[2][0], Const_GimbalYawSpdParam[2][1], 
                    Const_GimbalYawSpdParam[3][0], Const_GimbalYawSpdParam[3][1], PID_POSITION);
    PID_InitPIDParam(&gimbalyaw->angPIDParam, Const_GimbalYawAngParam[0][0], Const_GimbalYawAngParam[0][1], Const_GimbalYawAngParam[0][2], Const_GimbalYawAngParam[0][3], 
                    Const_GimbalYawAngParam[0][4], Const_GimbalYawAngParam[1][0], Const_GimbalYawAngParam[1][1], Const_GimbalYawAngParam[2][0], Const_GimbalYawAngParam[2][1], 
                    Const_GimbalYawAngParam[3][0], Const_GimbalYawAngParam[3][1], PID_POSITION);                     

    // 小 yaw：使用同一套参数做“固定角度锁死”（目标逻辑角 = 0）
    PID_InitPIDParam(&gimbalyaw->GimbalSmallYaw_spdPIDParam, Const_GimbalYawSpdParam[0][0], Const_GimbalYawSpdParam[0][1], Const_GimbalYawSpdParam[0][2], Const_GimbalYawSpdParam[0][3],
                    Const_GimbalYawSpdParam[0][4], Const_GimbalYawSpdParam[1][0], Const_GimbalYawSpdParam[1][1], Const_GimbalYawSpdParam[2][0], Const_GimbalYawSpdParam[2][1],
                    Const_GimbalYawSpdParam[3][0], Const_GimbalYawSpdParam[3][1], PID_POSITION);
    PID_InitPIDParam(&gimbalyaw->GimbalSmallYaw_angPIDParam, Const_GimbalYawAngParam[0][0], Const_GimbalYawAngParam[0][1], Const_GimbalYawAngParam[0][2], Const_GimbalYawAngParam[0][3],
                    Const_GimbalYawAngParam[0][4], Const_GimbalYawAngParam[1][0], Const_GimbalYawAngParam[1][1], Const_GimbalYawAngParam[2][0], Const_GimbalYawAngParam[2][1],
                    Const_GimbalYawAngParam[3][0], Const_GimbalYawAngParam[3][1], PID_POSITION);
}

/**
  * @brief      Get the pointer of gimbal control object
  * @param      NULL
  * @retval     Pointer to gimbal control object
  */
GimbalPitch_GimbalPitchTypeDef* GimbalPitch_GetGimbalPitchPtr() {
    return &GimbalPitch_gimbalPitchControlData;
}
/**
  * @brief      Get the pointer of gimbal control object
  * @param      NULL
  * @retval     Pointer to gimbal control object
  */
GimbalYaw_GimbalYawTypeDef* GimbalYaw_GetGimbalYawPtr() {
    return &GimbalYaw_gimbalYawControlData;
}

/** 
 * @brief 读取云台“大 yaw 电机(6020)”编码器的原始角度（0~360°）。
 * @note  这个角度是电机编码器直接给出的 limited_angle，未做任何“朝向对齐/零位校准”。
 *        如果机械正前方对应的原始角不是 0°，这里读出来就会带一个固定偏置。
 *        这个偏置需要在 `sys_const.c` 里通过 `Const_GimbalBigYawAngleOffset` 记录。
 */
float GimbalYaw_GetEncoderRawDeg(void) {
    return Motor_Big_YawMotor.encoder.limited_angle;
}

/**
 * @brief 读取云台“大 yaw 电机(6020)”的“逻辑角/校准角”（0~360°）。
 * @note  逻辑角 = 原始角 - 零位偏置(Const_GimbalBigYawAngleOffset)，再归一化到 [0, 360)。
 *        含义是：当你把云台物理朝向“定义的正前方”时，把当时的原始角填进 offset，
 *        之后 `GimbalYaw_GetEncoderLogicalDeg()` 在“正前方”就会返回接近 0°。
 *        这样做是为了让控制/调参使用一个与“正前方对齐”的角度坐标系，而不是编码器原始坐标系。
 */
float GimbalYaw_GetEncoderLogicalDeg(void) {
    return GimbalYaw_Normalize360(Motor_Big_YawMotor.encoder.limited_angle - Const_GimbalBigYawAngleOffset);
}

/**
  * @brief      Set the gimbal control output calculation enabled state
  * @param      state: Enabled, 1 is enabled, 0 is disabled
  * @retval     NULL
  */
void GimbalPitch_SetGimbalPitchControlState(uint8_t state) {
    GimbalPitch_GimbalPitchTypeDef *gimbalPitch = GimbalPitch_GetGimbalPitchPtr();

    gimbalPitch->control_state = state;
}
/**
  * @brief      Set the gimbal control output calculation enabled state
  * @param      state: Enabled, 1 is enabled, 0 is disabled
  * @retval     NULL
  */
void GimbalYaw_SetGimbalYawControlState(uint8_t state) {
    GimbalYaw_GimbalYawTypeDef *gimbalYaw = GimbalYaw_GetGimbalYawPtr();

    gimbalYaw->control_state = state;
}


/**
  * @brief      Set gimbal control output enable status
  * @param      state: Enabled, 1 is enabled, 0 is disabled
  * @retval     NULL
  */
void GimbalPitch_SetGimbalPitchOutputState(uint8_t state) {
    GimbalPitch_GimbalPitchTypeDef *gimbalPitch = GimbalPitch_GetGimbalPitchPtr();

    gimbalPitch->output_state = state;
}
/**
  * @brief      Set gimbal control output enable status
  * @param      state: Enabled, 1 is enabled, 0 is disabled
  * @retval     NULL
  */
void GimbalYaw_SetGimbalYawOutputState(uint8_t state) {
    GimbalYaw_GimbalYawTypeDef *gimbalYaw = GimbalYaw_GetGimbalYawPtr();

    gimbalYaw->output_state = state;
}



/**
  * @brief      Set the target value of gimbal pitch
  * @param      pitch_ref: gimbal pitch target value
  * @retval     NULL
  */
void GimbalPitch_SetPitchRef(float pitch_ref) {
    GimbalPitch_GimbalPitchTypeDef *gimbalpitch = GimbalPitch_GetGimbalPitchPtr();
    
    // 遥控/视觉增量先累加到 pitch_ref，再对 pitch_ref 本身做机械角度限幅，//这里是因为没有对pitch_ref限位,导致到机械限位之后还在累加积分
    // 避免长时间推到限位时内部目标无限积分，导致反向需要“还账”很久才开始动作。
    gimbalpitch->pitch_ref += pitch_ref;
    if (gimbalpitch->pitch_ref > Const_PITCH_UMAXANGLE) {
        gimbalpitch->pitch_ref = Const_PITCH_UMAXANGLE;
    } else if (gimbalpitch->pitch_ref < Const_PITCH_DMAXANGLE) {
        gimbalpitch->pitch_ref = Const_PITCH_DMAXANGLE;
    }
}
/**
  * @brief      Set the target value of gimbal yaw
  * @param      yaw_ref: gimbal yaw target value
  * @retval     NULL
  */
void GimbalYaw_SetYawRef(float yaw_ref) {
    GimbalYaw_GimbalYawTypeDef *gimbalyaw = GimbalYaw_GetGimbalYawPtr();
    
    gimbalyaw->yaw_ref = yaw_ref;
}                                                             //Pitch??????Yaw?????

/**
* @brief      Pitch轴角度限位
* @param      ref:输入的俯仰角目标值
* @retval     限位后的俯仰角目标值
*/
float Gimbal_LimitPitch(float ref) {
    Protocol_DataTypeDef *buscomm = Protocol_GetBusDataPtr();
    GimbalPitch_GimbalPitchTypeDef *gimbalpitch = GimbalPitch_GetGimbalPitchPtr();

    float pitch_umaxangle;
    if (buscomm->cha_mode  == Cha_Gyro) {
        pitch_umaxangle = Const_PITCH_UMAXANGLE_GRYO;
    }
    else {
        pitch_umaxangle = Const_PITCH_UMAXANGLE;
    }
/*如果当前俯仰角已经超过向上最大阈值，且输入的目标值 ref > 0（想继续向上转），则触发限位
如果当前俯仰角已经低于向下最大阈值，且用户输入的目标值 ref < 0（想继续向下转），则触发限位*/
    if (((PID_GetPIDRef(&gimbalpitch->angPID) > pitch_umaxangle) && (ref > 0)) ||
        ((PID_GetPIDRef(&gimbalpitch->angPID) < Const_PITCH_DMAXANGLE) && (ref < 0)))
        return 0.0f;
        // Out of depression set maximum ref
    else return ref;
}


/**
* @brief      Yaw angle limit
* @param      ref: Yaw set ref
* @retval     Limited ywa ref
*/
float Gimbal_LimitYaw(float ref) {
    Protocol_DataTypeDef *buscomm = Protocol_GetBusDataPtr();
    INS_INSTypeDef *ins = INS_GetINSPtr();

	if (buscomm->cha_mode  == Cha_Gyro)
        return ref;
	else if (((ins->YawTotalAngle - buscomm->yaw_ref < -Const_YAW_MAXANGLE) && (ref > 0)) || 
             ((ins->YawTotalAngle - buscomm->yaw_ref >  Const_YAW_MAXANGLE) && (ref < 0))) 
        return 0.0f;
    else return ref;
}

/**
* @brief      Set pitch ref
* @param      ref: Yaw set ref
* @retval     NULL
*/
void Gimbal_SetPitchRef(float ref) {
    GimbalPitch_GimbalPitchTypeDef *gimbal = GimbalPitch_GetGimbalPitchPtr();

    gimbal->pitch_ref += ref;
}

/**
  * @brief      Setting IMU yaw position feedback
  * @param      imu_yaw_position_fdb: IMU Yaw Position feedback
  * @retval     NULL
  */
void GimbalYaw_SetIMUYawPositionFdb(float imu_yaw_position_fdb) {
    GimbalYaw_GimbalYawTypeDef *gimbalyaw = GimbalYaw_GetGimbalYawPtr();

    gimbalyaw->yaw_position_fdb = imu_yaw_position_fdb;
}


/**
  * @brief      Setting IMU yaw speed feedback
  * @param      imu_yaw_speed_fdb: IMU Yaw Speed feedback
  * @retval     NULL
  */
void GimbalYaw_SetIMUYawSpeedFdb(float imu_yaw_speed_fdb) {
    GimbalYaw_GimbalYawTypeDef *gimbalyaw = GimbalYaw_GetGimbalYawPtr();

    gimbalyaw->yaw_speed_fdb = imu_yaw_speed_fdb;
}


/**
  * @brief      Control function of gimbal pitch
  * @param      NULL
  * @retval     NULL
  */
void GimbalPitch_Control() {
    GimbalPitch_GimbalPitchTypeDef *gimbalpitch = GimbalPitch_GetGimbalPitchPtr();
    INS_INSTypeDef *ins = INS_GetINSPtr();

    if (gimbalpitch->control_state != 1) return;

    // 恢复为“简单限幅 + 直接写 DM 目标”的方案：
    // pitch_ref 按角度(deg)，先按照原有 PITCH 上下限做机械限幅，再转为 rad 写入 DM4310。
    (void)ins;
    float target_deg = gimbalpitch->pitch_ref;
    if (target_deg > Const_PITCH_UMAXANGLE) target_deg = Const_PITCH_UMAXANGLE;
    if (target_deg < Const_PITCH_DMAXANGLE) target_deg = Const_PITCH_DMAXANGLE;

    motor[Motor1].ctrl.pos_set = target_deg * 3.1415926f / 180.0f;
    // 保留 vel/tor 为 0，由 kp/kd 决定刚度与阻尼
    motor[Motor1].ctrl.vel_set = 0.0f;
    motor[Motor1].ctrl.tor_set = 0.0f;
}
/**
  * @brief      Control function of gimbal yaw���������������̶����� 0�㣨��ǰ��
  */
void GimbalYaw_Control() {
    GimbalYaw_GimbalYawTypeDef *gimbalyaw = GimbalYaw_GetGimbalYawPtr();
    INS_INSTypeDef *ins = INS_GetINSPtr();
    Protocol_DataTypeDef *buscomm = Protocol_GetBusDataPtr();

    if (gimbalyaw->control_state != 1) return;

    /* 第一次进入闭环：把 yaw_ref 对齐到 IMU 当前角度，避免上电追零自转 */
    if (GimbalYaw_need_align_ref) {
        gimbalyaw->yaw_ref = ins->YawTotalAngle;
        if (buscomm != NULL) {
            buscomm->yaw_ref = ins->YawTotalAngle;
        }
        GimbalYaw_need_align_ref = 0;
    }

    // 完全对齐 Omni：yaw 角度环用 YawTotalAngle，速度环用 Gyro[Z_INS]
    PID_SetPIDRef(&gimbalyaw->angPID, gimbalyaw->yaw_ref);
    PID_SetPIDFdb(&gimbalyaw->angPID, ins->YawTotalAngle);
    PID_CalcPID(&gimbalyaw->angPID, &gimbalyaw->angPIDParam);

    PID_SetPIDRef(&gimbalyaw->spdPID, PID_GetPIDOutput(&gimbalyaw->angPID));
    PID_SetPIDFdb(&gimbalyaw->spdPID, ins->Gyro[Z_INS]);
    PID_CalcPID(&gimbalyaw->spdPID, &gimbalyaw->spdPIDParam);

    Motor_SetMotorOutput(&Motor_Big_YawMotor, PID_GetPIDOutput(&gimbalyaw->spdPID));

    // 小 yaw：上电后锁死在固定角度（通过 Const_GimbalSmallYawAngleOffset 配置）
    {
        // 参考底盘舵向 6020：只用角度环输出，并用“最短路径误差 + 虚拟 fdb”避免 0/360 跳变导致高频抖
        float small_ref_deg = 0.0f; // 锁死目标逻辑角 = 0
        float small_fdb_deg = GimbalYaw_Normalize360(Motor_Small_YawMotor.encoder.limited_angle - Const_GimbalSmallYawAngleOffset);
        float err = small_ref_deg - small_fdb_deg;
        while (err > 180.0f)  err -= 360.0f;
        while (err < -180.0f) err += 360.0f;
        float fdb_for_pid = small_ref_deg - err; // 使 ref - fdb = err

        PID_SetPIDRef(&gimbalyaw->GimbalSmallYaw_angPID, small_ref_deg);
        PID_SetPIDFdb(&gimbalyaw->GimbalSmallYaw_angPID, fdb_for_pid);
        PID_CalcPID(&gimbalyaw->GimbalSmallYaw_angPID, &gimbalyaw->GimbalSmallYaw_angPIDParam);

        Motor_SetMotorOutput(&Motor_Small_YawMotor, PID_GetPIDOutput(&gimbalyaw->GimbalSmallYaw_angPID));
    }
}

void GimbalYaw_Output(void) {
    GimbalYaw_GimbalYawTypeDef *gimbalyaw = GimbalYaw_GetGimbalYawPtr();
    if (gimbalyaw->output_state != 1) {
        Motor_SetMotorOutput(&Motor_Big_YawMotor, 0.0f);
        Motor_SetMotorOutput(&Motor_Small_YawMotor, 0.0f);
    }
    Motor_SendMotorGroupOutput(&Motor_Big_YawMotors);
}

/**
  * @brief      Gimbal pitch output function
  * @param      NULL
  * @retval     NULL
  */
void GimbalPitch_Output() {
    GimbalPitch_GimbalPitchTypeDef *gimbalpitch = GimbalPitch_GetGimbalPitchPtr();
    GimbalYaw_GimbalYawTypeDef *gimbalyaw = GimbalYaw_GetGimbalYawPtr();
	
    // Pitch 由达妙 DM4310 在 app_gimbal 任务中通过 dm_motor_ctrl_send() 下发，这里不再发 DJI Pitch 组
    (void)gimbalpitch;
    (void)gimbalyaw;
}
///**
//  * @brief      Gimbal yaw output function
//  * @param      NULL
//  * @retval     NULL
//  */
//void GimbalYaw_Output() {
//    GimbalYaw_GimbalYawTypeDef *gimbalyaw = GimbalYaw_GetGimbalYawPtr();

//    if (gimbalyaw->output_state != 1) {
//		Motor_SetMotorOutput(&Motor_YawMotor, 0);
//	}
//    Motor_SendMotorGroupOutput(&Motor_YawMotors);
//}
