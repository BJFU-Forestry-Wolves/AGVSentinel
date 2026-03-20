/*
 *  Project      : Polaris Robot 
 *  
 *  FilePath     : periph_motor.c
 *  Description  : This file contains motor control function
 *  LastEditors  : Polaris
 *  Date         : 2023-01-23 00:43:48
 *  LastEditTime : 2023-05-05 17:59:23
 */


#include "periph_motor.h"
#include "sys_const.h"
#include "stdio.h"
#include "stdlib.h"

/********** VOLATILE USER CODE **********/

const uint32_t Const_Motor_MOTOR_OFFLINE_TIME = 200;
const uint32_t Const_Motor_MOTOR_TX_EXTID = 0x01;
const uint32_t Const_Motor_MOTOR_TX_DLC = 8;
const uint32_t Const_Motor_MOTOR_RX_DLC = 8;

Motor_MotorGroupTypeDef *Motor_groupHandle[MOTOR_GROUP_NUM];

Motor_MotorGroupTypeDef Motor_ChassisMotors;
Motor_MotorGroupTypeDef Motor_ChassisSteerMotors;
Motor_MotorGroupTypeDef Motor_PitchMotors;        
Motor_MotorGroupTypeDef Motor_ShootMotors;
Motor_MotorGroupTypeDef Motor_FeedMotors;
Motor_MotorGroupTypeDef Motor_Big_YawMotors;  
Motor_MotorGroupTypeDef Motor_Small_YawMotors;  
Motor_MotorGroupTypeDef Motor_DM_Motors;        


Motor_MotorTypeDef Motor_ChassisFontRightMotor;
Motor_MotorTypeDef Motor_ChassisFontLeftMotor;
Motor_MotorTypeDef Motor_ChassisBackLeftMotor;
Motor_MotorTypeDef Motor_ChassisBackRightMotor;
Motor_MotorTypeDef Motor_ChassisFontRightSteerMotor;
Motor_MotorTypeDef Motor_ChassisFontLeftSteerMotor;
Motor_MotorTypeDef Motor_ChassisBackLeftSteerMotor;
Motor_MotorTypeDef Motor_ChassisBackRightSteerMotor;
Motor_MotorTypeDef Motor_PitchMotor;		//Pitch达妙电机
Motor_MotorTypeDef Motor_Big_YawMotor;      //大YawGM6020(挂下板can1,id5)
Motor_MotorTypeDef Motor_Small_YawMotor;	//小YawGM6020(挂上板can2,id2)
Motor_MotorTypeDef Motor_ShootLeftMotor;
Motor_MotorTypeDef Motor_ShootRightMotor;
Motor_MotorTypeDef Motor_FeedMotor;


/**
  * @brief      Motor encoder decoding callback function
  * @param      canid: CAN Handle number
  * @param      stdid: CAN identifier
  * @param      rxdata: CAN rx data buff
  * @retval     NULL
  */
void Motor_EncoderDecodeCallback(CAN_HandleTypeDef* phcan, uint32_t stdid, uint8_t rxdata[], uint32_t len) {
    for (int i = 0; i < MOTOR_GROUP_NUM; i++) {
        for (int j = 0; j < Motor_groupHandle[i]->motor_num; j++) {
            if (Motor_groupHandle[i]->motor_handle[j] == NULL) continue;
            if ((phcan == Motor_groupHandle[i]->can_handle) 
                  && (stdid == Motor_groupHandle[i]->motor_handle[j]->id) && phcan != NULL) {
                Motor_groupHandle[i]->motor_handle[j]->callback(Motor_groupHandle[i]->motor_handle[j], rxdata, len);
            }
        }
    }
}


/********** VOLATILE USER CODE END **********/


/**
  * @brief      Initialize all motors
  * @param      NULL
  * @retval     NULL
  */
void Motor_InitAllMotors() {
	
    // 云台 Yaw：两颗 6020 共用 CAN2 的 0x1FF 帧
    // 0x1FF 数据槽映射: [0]=0x205, [1]=0x206, [2]=0x207, [3]=0x208
    // 大 yaw: 0x207 -> motor_handle[2]; 小 yaw: 0x206 -> motor_handle[1]
    Motor_groupHandle[0] = &Motor_Big_YawMotors;
    Motor_InitMotorGroup(&Motor_Big_YawMotors, Motor_TYPE_RM6020, 3, &hcan2, 0x1FF);
    Motor_InitMotor(&Motor_Big_YawMotor, Motor_TYPE_RM6020, 0x207, 0.1, gm6020_encoder_callback);
    Motor_InitMotor(&Motor_Small_YawMotor, Motor_TYPE_RM6020, 0x206, 0.1, gm6020_encoder_callback);
    Motor_Big_YawMotors.motor_handle[1] = &Motor_Small_YawMotor;
    Motor_Big_YawMotors.motor_handle[2] = &Motor_Big_YawMotor;


	
	
    Motor_groupHandle[1] = &Motor_ShootMotors;//发射机构电机组(包含两个3508摩擦轮和一个2006拨弹电机)
    Motor_InitMotorGroup(&Motor_ShootMotors, Motor_TYPE_RM3508, 3, &hcan2, 0x200);
    Motor_InitMotor(&Motor_ShootRightMotor, Motor_TYPE_RM3508, 0x201, 0.1, rm3508_encoder_callback);
    Motor_InitMotor(&Motor_ShootLeftMotor, Motor_TYPE_RM3508, 0x202, 0.1, rm3508_encoder_callback);
	Motor_InitMotor(&Motor_FeedMotor, Motor_TYPE_RM2006, 0X203, 0.1, rm2006_encoder_callback);

	Motor_ShootMotors.motor_handle[0] = &Motor_ShootRightMotor;
    Motor_ShootMotors.motor_handle[1] = &Motor_ShootLeftMotor;
	Motor_ShootMotors.motor_handle[2] = &Motor_FeedMotor;

	
	
	
	Motor_groupHandle[2] = &Motor_ChassisMotors;//底盘轮向电机组(四个轮向3508)
    Motor_InitMotorGroup(&Motor_ChassisMotors, Motor_TYPE_RM3508, 4, &hcan1, 0x200);
    Motor_InitMotor(&Motor_ChassisBackRightMotor, Motor_TYPE_RM3508, 0x201, 0.1, rm3508_encoder_callback);   /* �Һ� id1 */
    Motor_InitMotor(&Motor_ChassisBackLeftMotor,  Motor_TYPE_RM3508, 0x202, 0.1, rm3508_encoder_callback);   /* ��� id2 */
    Motor_InitMotor(&Motor_ChassisFontRightMotor, Motor_TYPE_RM3508, 0x203, 0.1, rm3508_encoder_callback);   /* ��ǰ id3 */
    Motor_InitMotor(&Motor_ChassisFontLeftMotor,  Motor_TYPE_RM3508, 0x204, 0.1, rm3508_encoder_callback);   /* ��ǰ id4 */
    Motor_ChassisMotors.motor_handle[0] = &Motor_ChassisBackRightMotor;
    Motor_ChassisMotors.motor_handle[1] = &Motor_ChassisBackLeftMotor;
    Motor_ChassisMotors.motor_handle[2] = &Motor_ChassisFontRightMotor;
    Motor_ChassisMotors.motor_handle[3] = &Motor_ChassisFontLeftMotor;

	Motor_groupHandle[3] = &Motor_ChassisSteerMotors;//底盘舵向电机组(四个舵向6020)
    Motor_InitMotorGroup(&Motor_ChassisSteerMotors, Motor_TYPE_RM6020, 4, &hcan1, 0x1FF);
    Motor_InitMotor(&Motor_ChassisBackRightSteerMotor, Motor_TYPE_RM6020, 0x205, 0.1, gm6020_encoder_callback);   /* �Һ� id1 */
    Motor_InitMotor(&Motor_ChassisBackLeftSteerMotor,  Motor_TYPE_RM6020, 0x206, 0.1, gm6020_encoder_callback);   /* ��� id2 */
    Motor_InitMotor(&Motor_ChassisFontRightSteerMotor, Motor_TYPE_RM6020, 0x207, 0.1, gm6020_encoder_callback);   /* ��ǰ id3 */
    Motor_InitMotor(&Motor_ChassisFontLeftSteerMotor,  Motor_TYPE_RM6020, 0x208, 0.1, gm6020_encoder_callback);   /* ��ǰ id4 */
    Motor_ChassisSteerMotors.motor_handle[0] = &Motor_ChassisBackRightSteerMotor;
    Motor_ChassisSteerMotors.motor_handle[1] = &Motor_ChassisBackLeftSteerMotor;
    Motor_ChassisSteerMotors.motor_handle[2] = &Motor_ChassisFontRightSteerMotor;
    Motor_ChassisSteerMotors.motor_handle[3] = &Motor_ChassisFontLeftSteerMotor;
	
	
	// Motor_DM_Motors (Pitch/达妙等) 后续再接入：未初始化前不要加入 Motor_groupHandle，避免遍历时访问未初始化 motor_num/can_handle

	
		
}


/**
  * @brief      Initialize the motor
  * @param      pmotor: Pointer to motor object
  * @param      type: Type of motor (pwm or can)
  * @param      callback: Motor callback function
  * @retval     NULL
  */
void Motor_InitMotor(Motor_MotorTypeDef* pmotor, Motor_MotorTypeEnum type, uint16_t id, float fil_param, 
                     Motor_EncoderCallbackFuncTypeDef callback) {
    if (pmotor == NULL) return;
    pmotor->last_update_time = 0;
    pmotor->type = type;
    pmotor->id = id;			
    pmotor->init = 0;			
    pmotor->is_online = 0;
    pmotor->output = 0;
    pmotor->callback = callback;
    Filter_LowPassInit(fil_param, &pmotor->fdb_fil_param);
}


/**
  * @brief      Initialization of motor group
  * @param      pgroup: Pointer to motor group
  * @param      type: Type of motor (pwm or can)
  * @param      motor_num: Number of motor group
  * @param      phcan: Pointer of can handle
  * @param      stdid: Motor id
  * @retval     NULL
  */
void Motor_InitMotorGroup(Motor_MotorGroupTypeDef* pgroup, Motor_MotorTypeEnum type, uint8_t motor_num, CAN_HandleTypeDef* phcan, uint16_t stdid) {
    if (pgroup == NULL) return;
    pgroup->motor_num = motor_num;
    pgroup->type = type;

    if (phcan == NULL) return;
    pgroup->can_handle = phcan;
    Can_InitTxHeader(&(pgroup->can_header), stdid, Const_Motor_MOTOR_TX_EXTID, Const_Motor_MOTOR_TX_DLC);

    for (int i = 0; i < 4; ++i) {
        pgroup->motor_handle[i] = NULL;
    }
}


/**
  * @brief      Set output
  * @param      pmotor: Pointer to motor object
  * @param      pparam: Pointer to motor parameter object
  * @retval     NULL
  */
void Motor_SetMotorOutput(Motor_MotorTypeDef* pmotor, float output) {
    pmotor->output = output;//?? PID ???????????????output???
}


/**
  * @brief      Get motor output value
  * @param      pmotor: Pointer to motor object
  * @retval     Output value
  */
uint16_t Motor_GetMotorOutput(Motor_MotorTypeDef* pmotor) {
    if (pmotor == NULL) return 0;
    if (pmotor->type == Motor_TYPE_NOT_CONNECTED) return 0;

    int16_t ret = 0;
    if (pmotor->type == Motor_TYPE_RM3508 || pmotor->type == Motor_TYPE_RM2006 || pmotor->type ==Motor_TYPE_RM6020) {
        ret = (int16_t)(pmotor->output * 1000.0f);               // output / 10.0f * 10000.0f
        return (uint16_t)ret;
    }
    return 0;
}


/**
  * @brief      Transmitter output
  * @param      pgroup: Pointer to the motor group to send
  * @retval     NULL
  */
void Motor_SendMotorGroupOutput(Motor_MotorGroupTypeDef *pgroup) {
    if (pgroup == NULL) return;//
    uint8_t txdata[64];
	memset(txdata, 0, 64);
    txdata[0] = (uint8_t)(Motor_GetMotorOutput(pgroup->motor_handle[0]) >> 8);
    txdata[1] = (uint8_t)Motor_GetMotorOutput(pgroup->motor_handle[0]);
    txdata[2] = (uint8_t)(Motor_GetMotorOutput(pgroup->motor_handle[1]) >> 8);
    txdata[3] = (uint8_t)Motor_GetMotorOutput(pgroup->motor_handle[1]);
    txdata[4] = (uint8_t)(Motor_GetMotorOutput(pgroup->motor_handle[2]) >> 8);
    txdata[5] = (uint8_t)Motor_GetMotorOutput(pgroup->motor_handle[2]);
    txdata[6] = (uint8_t)(Motor_GetMotorOutput(pgroup->motor_handle[3]) >> 8);
    txdata[7] = (uint8_t)Motor_GetMotorOutput(pgroup->motor_handle[3]);
    Can_SendMessage(pgroup->can_handle, &(pgroup->can_header), txdata);
}


void Motor_SendMotorGroupsOutput() {
    for (int i = 0; i < MOTOR_GROUP_NUM; i++) {
        Motor_SendMotorGroupOutput(Motor_groupHandle[i]);
    }
}


/**
  * @brief      Judge whether any motor is offline
  * @param      NULL
  * @retval     Offline or not (1 is yes, 0 is no)
  */
uint8_t Motor_IsAnyMotorOffline() {
    for (int i = 0; i < MOTOR_GROUP_NUM; ++i) {
        for (int j = 0; j < 4; ++j) {
            if (!Motor_IsMotorOffline(Motor_groupHandle[i]->motor_handle[j])) {
                return 0;
            }
        }
    }
    return 1;
}


/**
  * @brief      Judge whether the motor is offline
  * @param      pmotor: Pointer to motor object
  * @retval     Offline or not (1 is yes, 0 is no)
  */
uint8_t Motor_IsMotorOffline(Motor_MotorTypeDef* pmotor) {
    if (pmotor == NULL) return 0;
    if (pmotor->type == Motor_TYPE_NOT_CONNECTED) return 0;
    uint32_t now = HAL_GetTick();
    return (now - pmotor->last_update_time) < Const_Motor_MOTOR_OFFLINE_TIME;
}


/********** RM2006,RM3508,GM6020电机回调函数 **********/

/**
  * @brief      rm2006 motor encoder callback
  * @param      pmotor: Pointer to motor object
  * @retval     NULL
  */
void rm2006_encoder_callback(Motor_MotorTypeDef *pmotor, uint8_t rxbuff[], uint32_t len) {
    if (pmotor == NULL) return;
    if (len != 8) return;

    pmotor->encoder.last_angle = pmotor->init == 1 ? pmotor->encoder.angle : (rxbuff[0] << 8 | rxbuff[1]);
    pmotor->encoder.angle   = (float)((int16_t)((uint16_t)rxbuff[0] << 8 | (uint16_t)rxbuff[1]));
    pmotor->encoder.speed   = (float)((int16_t)((uint16_t)rxbuff[2] << 8 | (uint16_t)rxbuff[3])) / 36.0f;
    pmotor->encoder.current = (float)((int16_t)((uint16_t)rxbuff[4] << 8 | (uint16_t)rxbuff[5]));
    pmotor->encoder.temp = 0; 
    pmotor->init = 1; 

    int diff = pmotor->encoder.angle - pmotor->encoder.last_angle;      
    if (diff < -4096)           
        pmotor->encoder.round_count++;
    else if (diff > 4096)       
        pmotor->encoder.round_count--;
    // Calculate the shaft angle because the reduction ratio needs to be divided by 36
    pmotor->encoder.consequent_angle = (float)pmotor->encoder.round_count * 10.0f + 
                                       (float)pmotor->encoder.angle / 8192.0f * 10.0f;
    if (pmotor->encoder.round_count > 10000) {
        pmotor->encoder.consequent_angle -= 10 * pmotor->encoder.round_count; 
        pmotor->encoder.round_count = 0;
    }
    if (pmotor->encoder.round_count < -10000) {
        pmotor->encoder.consequent_angle += 10 * pmotor->encoder.round_count; 
        pmotor->encoder.round_count = 0;
    }
    if (pmotor->encoder.limited_angle < pmotor->encoder.init_offset - 180 && pmotor->encoder.init_offset >= 180)
        pmotor->encoder.limited_angle += 360;
    else if (pmotor->encoder.limited_angle > pmotor->encoder.init_offset + 180 && pmotor->encoder.init_offset < 180)
        pmotor->encoder.limited_angle -= 360;
    pmotor->last_update_time = HAL_GetTick(); 
}


/**
  * @brief      Gimbal motor encoder callback
  * @param      pmotor: Pointer to motor object
  * @retval     NULL
  */
void gm6020_encoder_callback(Motor_MotorTypeDef *pmotor, uint8_t rxbuff[], uint32_t len) {
    // Calculate angle difference and number of cycles
    int diff = pmotor->encoder.angle - pmotor->encoder.last_angle;      //The increase of mechanical angle is positive
    if (diff < -5500)           // Make a positive turn
        pmotor->encoder.round_count++;
    else if (diff > 5500)       // Turn around in the opposite direction
        pmotor->encoder.round_count--;

    pmotor->encoder.last_angle = pmotor->init == 1 ? pmotor->encoder.angle : (rxbuff[0] << 8 | rxbuff[1]);
    pmotor->encoder.angle   = (float)((int16_t)((uint16_t)rxbuff[0] << 8 | (uint16_t)rxbuff[1]));
    pmotor->encoder.speed   = (float)((int16_t)((uint16_t)rxbuff[2] << 8 | (uint16_t)rxbuff[3]));
    pmotor->encoder.current = (float)((int16_t)((uint16_t)rxbuff[4] << 8 | (uint16_t)rxbuff[5]));
    pmotor->encoder.temp = 0; 
    pmotor->init = 1; 

    // Calculate continuous angle
    pmotor->encoder.consequent_angle = (float)pmotor->encoder.round_count * 360.0f + 
                                       (float)pmotor->encoder.angle / 8192.0f * 360.0f;
    pmotor->encoder.limited_angle = (float)pmotor->encoder.angle / 8192.0f * 360.0f;
    pmotor->last_update_time = HAL_GetTick();
}


/**
  * @brief      Gimbal motor encoder callback
  * @param      pmotor: Pointer to motor object
  * @retval     NULL
  */
void rm3508_encoder_callback(Motor_MotorTypeDef *pmotor, uint8_t rxbuff[], uint32_t len) {
    if (pmotor == NULL) return;
    

    // Assign a value to the previous angle and get the latest angle
    pmotor->encoder.last_angle = pmotor->init == 1 ? pmotor->encoder.angle : (rxbuff[0] << 8 | rxbuff[1]);
    pmotor->encoder.angle   = (float)((int16_t)((uint16_t)rxbuff[0] << 8 | (uint16_t)rxbuff[1]));
    pmotor->encoder.speed   = (float)((int16_t)((uint16_t)rxbuff[2] << 8 | (uint16_t)rxbuff[3])) / 19.0f;
    pmotor->encoder.current = (float)((int16_t)((uint16_t)rxbuff[4] << 8 | (uint16_t)rxbuff[5]));
    pmotor->encoder.temp    = (float)((int16_t)((uint16_t)rxbuff[6]));
    
    int diff = pmotor->encoder.angle - pmotor->encoder.last_angle;      
    if (diff < -4096)           
        pmotor->encoder.round_count++;
    else if (diff > 4096)       
        pmotor->encoder.round_count--;
    // Calculate the shaft angle because the reduction ratio needs to be divided by 19
    pmotor->encoder.consequent_angle = (float)pmotor->encoder.round_count * 360.0f / 19.0f + 
                                       (float)pmotor->encoder.angle / 8192.0f * 360.0f / 19.0f;
    if (pmotor->encoder.round_count > 10000) {
        pmotor->encoder.consequent_angle -= 10 * pmotor->encoder.round_count; 
        pmotor->encoder.round_count = 0;
    }
    if (pmotor->encoder.round_count < -10000) {
        pmotor->encoder.consequent_angle += 10 * pmotor->encoder.round_count; 
        pmotor->encoder.round_count = 0;
    }
    if (pmotor->encoder.limited_angle < pmotor->encoder.init_offset - 180 && pmotor->encoder.init_offset >= 180)
        pmotor->encoder.limited_angle += 360;
    else if (pmotor->encoder.limited_angle > pmotor->encoder.init_offset + 180 && pmotor->encoder.init_offset < 180)
        pmotor->encoder.limited_angle -= 360;	
	
    pmotor->last_update_time = HAL_GetTick(); 
}

