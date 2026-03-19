/*
 *  Project      : Polaris
 * 
 *  file         : app_remote.c
 *  Description  : This file contains Remote control function
 *  LastEditors  : Polaris
 *  Date         : 2021-05-04 20:53:31
 *  LastEditTime : 2023-05-07 11:26:15
 */


#include "sys_const.h"
#include "protocol_common.h"
#include "app_remote.h"
#include "app_referee.h"
#include "module_shoot.h"
#include "module_gimbal.h"
#include "module_chassis.h"
#include "module_referee.h"
#include "periph_servo.h"
#include "cmsis_os.h"
#include "app_autoaim.h"
#define REMOTE_TASK_PERIOD  1	//定义遥控器任务的执行周期为 1ms
#define ENCODER_LIMIT 500		//定义编码器角度的突变阈值为 500
uint32_t game_state_cnt;
Remote_RemoteControlTypeDef Remote_remoteControlData;//遥控器控制的核心状态参数
Math_SlopeParamTypeDef Remote_ChassisFBSlope;
float last_encoder_angle=0;		//上一周期读取的编码器角度
float encoder_angle=0;			//这周期读取的编码器角度
float get_abs(float a){			//自定义函数,返回绝对值
    if(a>=0){
		return a;
		}
		else return(0-a);

}
float limit_siqu(float last_angle,float angle){			//编码器角度突变过滤函数
       if(get_abs((int16_t)(angle-last_angle))<=ENCODER_LIMIT){ 
			 return last_angle;}
	     else{
			 return angle;
			 }

}

/**
  * @brief          Remote task
  * @param          NULL
  * @retval         NULL
  */
void Remote_Task(void const * argument) {

    forever {
        Remote_ControlCom();
      osDelay(REMOTE_TASK_PERIOD);
    }
}


/**
  * @brief      Remote Control Init
  * @param      NULL
  * @retval     NULL
  */
void Remote_RemotrControlInit() {
    Remote_RemoteControlTypeDef *control_data = Remote_GetControlDataPtr();
    
    Math_InitSlopeParam(&Remote_ChassisFBSlope, MOUSE_CHASSIS_ACCELERATE, MOUSE_CHASSIS_ACCELERATE);
}


/**
  * @brief      Gets the pointer to the remote control data object
  * @param      NULL
  * @retval     Pointer to remote control data object
  */
Remote_RemoteControlTypeDef* Remote_GetControlDataPtr() {
    return &Remote_remoteControlData;
}


/**
* @brief      Remote control command
* @param      NULL
* @retval     NULL
*/
Remote_RemoteDataTypeDef *testdata;
void Remote_ControlCom() {
    Shoot_StatusTypeDef *shooter = Shooter_GetShooterControlPtr();
    Remote_RemoteControlTypeDef *control_data = Remote_GetControlDataPtr();
    Remote_RemoteDataTypeDef *data = Remote_GetRemoteDataPtr();
		testdata = data;
    control_data->pending = 1;

    switch (data->remote.s[0]) {
    /*    左侧三位开关控制模式   */
        case Remote_SWITCH_UP: {
            /* 左侧三位开关为上时是常规遥控模式 (右上底盘NULL,右中XTL,右下FOLLOW)*/
            Remote_RemoteProcess();
            break;
        }
        case Remote_SWITCH_MIDDLE: {
            /* 左侧三位开关为中时是固定给底盘SEP模式,根据右侧三位开关不同位置切换供弹射击模式 */
            Remote_RemoteShooterModeSet();
            break;
        }
        case Remote_SWITCH_DOWN: {
            /*左侧三位开关为下时也是常规遥控模式(右上底盘NULL,右中XTL,右下FOLLOW)  */
           Remote_RemoteProcess();
            break;
        }
        default:
            break;
    }

    control_data->pending = 0;
}


/**
* @brief      Mouse shoot mode set
* @param      NULL
* @retval     NULL
*/
int test_count;
void Remote_MouseShooterModeSet() {
    Remote_RemoteDataTypeDef *data = Remote_GetRemoteDataPtr();
    Shoot_StatusTypeDef *shooter = Shooter_GetShooterControlPtr();

    // Prevent launching without opening the friction wheel
//    if ((shooter->shooter_mode != Shoot_REFEREE) || (fabs(Motor_ShootLeftMotor.encoder.speed) <= 30) || (fabs(Motor_ShootRightMotor.encoder.speed) <= 30)) {
//        Shooter_ChangeFeederMode(Feeder_FINISH);
//        return;
//    }
    if ((fabs(Motor_ShootLeftMotor.encoder.speed) <= 30) || (fabs(Motor_ShootRightMotor.encoder.speed) <= 30)) {
        Shooter_ChangeFeederMode(Feeder_FINISH);
        return;
    }

    static int count_mouse_L = 0;
    if (data->mouse.l == 1) {
        count_mouse_L++;
        if (count_mouse_L >= 50) {
            Shooter_ChangeFeederMode(Feeder_FAST_CONTINUE);
            count_mouse_L = 50;
        }
    }
    else {
        if (0 < count_mouse_L && count_mouse_L < 50) {
            Shooter_SingleShootReset();
            Shooter_ChangeFeederMode(Feeder_SINGLE);
        }
        else Shooter_ChangeFeederMode(Feeder_FINISH);
        count_mouse_L = 0;
    }
		
		test_count = count_mouse_L;
}


/**
* @brief      Remote shoot mode set
* @param      NULL
* @retval     NULL
*/
void Remote_RemoteShooterModeSet() {


		Remote_RemoteDataTypeDef *data = Remote_GetRemoteDataPtr();
		Shoot_StatusTypeDef* shooter = Shooter_GetShooterControlPtr();
		

    switch (data->remote.s[1]) {
    /*      left switch control mode   */
        case Remote_SWITCH_UP: {
            /* 左中前提下右上是射击供弹模式NULL  */
            Shooter_ChangeShooterMode(Shoot_NULL);
            Shooter_ChangeFeederMode(Feeder_NULL);
            break;
        }
        case Remote_SWITCH_MIDDLE: {
            /* 左中前提下右中是切换射击供弹模式  */
            Shooter_ChangeShooterMode(Shoot_FAST);
					if(get_shoot_msg(&visionDataGet)==1)
					{
					  Shooter_ChangeFeederMode(Feeder_FAST_CONTINUE);
					}
					else
					{
            Shooter_ChangeFeederMode(Feeder_LOW_CONTINUE);
					}
            break;
        }
        case Remote_SWITCH_DOWN: {
            /* 左中前提下右下是切换射击快速供弹模式  */
            Shooter_ChangeShooterMode(Shoot_FAST);
            Shooter_ChangeFeederMode(Feeder_FAST_CONTINUE);
//            if ((PID_GetPIDFdb(&shooter->shootLeftPID) >= 30) && (PID_GetPIDFdb(&shooter->shootRightPID) <= -30)) {
//                Shooter_ChangeFeederMode(Feeder_REFEREE);
//            }
//            else Shooter_ChangeFeederMode(Feeder_FINISH);
            break;
        }
        default:
            break;
    }
		
    GimbalPitch_GimbalPitchTypeDef *gimbalpitch = GimbalPitch_GetGimbalPitchPtr();
    GimbalYaw_GimbalYawTypeDef *gimbalyaw = GimbalYaw_GetGimbalYawPtr();
		gimbalpitch->output_state = 1;
		gimbalyaw->output_state = 1;
    Protocol_DataTypeDef *buscomm = Protocol_GetBusDataPtr();
    buscomm->yaw_ref += Gimbal_LimitYaw((float)data->remote.ch[2] * -Const_WHEELLEG_REMOTE_YAW_GAIN + (float)visionDataGet.yaw_angle.yaw_predict *0.01f*0.004f);
		GimbalYaw_SetYawRef(buscomm->yaw_ref);
    float pitch_ref;
    pitch_ref = (float)data->remote.ch[3] * REMOTE_PITCH_ANGLE_TO_REF+ (float)visionDataGet.pitch_angle.pitch_predict*0.01f*0.002f;
    GimbalPitch_SetPitchRef(Gimbal_LimitPitch(-pitch_ref));
		
		Chassis_SetChassisMode(Chassis_SEP);
		Chassis_SetChassisRef((float)data->remote.ch[1]  , (float)data->remote.ch[0] , 0);
		
		
}


/**
* @brief      Remote control process
* @param      NULL
* @retval     NULL
*/
void Remote_RemoteProcess() {
    Protocol_DataTypeDef *buscomm = Protocol_GetBusDataPtr();
    Remote_RemoteDataTypeDef *data = Remote_GetRemoteDataPtr();
	  TargetData *dh_target = Dh_GetDhDataPtr();

    GimbalPitch_GimbalPitchTypeDef *gimbalpitch = GimbalPitch_GetGimbalPitchPtr();
    GimbalYaw_GimbalYawTypeDef *gimbalyaw = GimbalYaw_GetGimbalYawPtr();
		gimbalpitch->output_state = 0;
		gimbalyaw->output_state = 0;
		
    switch (data->remote.s[1]) {
    /*     右开关决定模式   */
        case Remote_SWITCH_UP: {//当左开关为上,右开关为上时,空输出NULL
					  Shooter_ChangeShooterMode(Shoot_NULL);
            Shooter_ChangeFeederMode(Feeder_NULL);
	          gimbalpitch->output_state = 0;
	          gimbalyaw->output_state = 0;
						Chassis_SetChassisMode(Chassis_NULL);
					Chassis_SetChassisRef(0.0f, 0.0f, 0.0f);
//				buscomm->yaw_ref += Gimbal_LimitYaw((float)data->remote.ch[2] * -Const_WHEELLEG_REMOTE_YAW_GAIN + (float)visionDataGet.yaw_angle.yaw_predict *0.01f*0.004f);
//GimbalYaw_SetYawRef(buscomm->yaw_ref);
//float pitch_ref;
//pitch_ref = (float)data->remote.ch[3] * REMOTE_PITCH_ANGLE_TO_REF; //(float)visionDataGet.pitch_angle.pitch_predict*0.01f*0.002f;
//GimbalPitch_SetPitchRef(Gimbal_LimitPitch(-pitch_ref));
            break;
        }

//				case Remote_SWITCH_MIDDLE:  
//				{
//	       gimbalpitch->output_state = 1;
//	       gimbalyaw->output_state = 1;
//	       Chassis_SetChassisMode(Chassis_FOLLOW);
//				 last_encoder_angle=encoder_angle;
//				 encoder_angle=(float)(Motor_YawMotor.encoder.limited_angle - CHASSIS_YAW_ANGLE_OFFSET);
//	       Chassis_SetChassisRef((float)dh_target->linear_x*Const_Chasiss_Navigate_linear_x  , -(float)dh_target->linear_y*Const_Chasiss_Navigate_linear_y ,limit_siqu(last_encoder_angle,encoder_angle)); //(float)(Motor_YawMotor.encoder.limited_angle - CHASSIS_YAW_ANGLE_OFFSET));
//         buscomm->yaw_ref += Gimbal_LimitYaw((float)dh_target->angular_z * Const_Chasiss_Navigate_angular_z ); 
//         float pitch_ref;
//         pitch_ref = Gimbal_LimitYaw((float)data->remote.ch[3] * REMOTE_PITCH_ANGLE_TO_REF + (float)visionDataGet.yaw_angle.yaw_predict *0.01f*0.004f); //(float)visionDataGet.pitch_angle.pitch_predict*0.01f*0.002f;
//         GimbalPitch_SetPitchRef(Gimbal_LimitPitch(-pitch_ref)); 
//				break;
//				}
												// 注释掉的代码（已废弃）
//    case Remote_SWITCH_MIDDLE:
//        {
//	      gimbalpitch->output_state = 1;
//	      gimbalyaw->output_state = 1;
//	      Chassis_SetChassisMode(Chassis_FOLLOW);
//				last_encoder_angle=encoder_angle;
//				encoder_angle=(float)(Motor_YawMotor.encoder.limited_angle - CHASSIS_YAW_ANGLE_OFFSET);
//	      Chassis_SetChassisRef((float)data->remote.ch[1] , (float)data->remote.ch[0] ,limit_siqu(last_encoder_angle,encoder_angle)); //(float)(Motor_YawMotor.encoder.limited_angle - CHASSIS_YAW_ANGLE_OFFSET));
//        buscomm->yaw_ref += Gimbal_LimitYaw((float)data->remote.ch[2] * -Const_WHEELLEG_REMOTE_YAW_GAIN + (float)visionDataGet.yaw_angle.yaw_predict *0.01f*0.004f);
//		    GimbalYaw_SetYawRef(buscomm->yaw_ref);
//        float pitch_ref;
//        pitch_ref = (float)data->remote.ch[3] * REMOTE_PITCH_ANGLE_TO_REF; //(float)visionDataGet.pitch_angle.pitch_predict*0.01f*0.002f;
//        GimbalPitch_SetPitchRef(Gimbal_LimitPitch(-pitch_ref));  
//					break;
//        }// 注释掉的代码 2（跟随模式）
				
		case Remote_SWITCH_MIDDLE: //左上前提下,右中时小陀螺模式(自转视觉自扫描)
			{
//			if(Referee_RefereeData.game_state==0x04)
//			{
			  Shooter_ChangeShooterMode(Shoot_FAST);
				if(Referee_RefereeData.game_state==4)
				{
				game_state_cnt++;
				}
		    float yaw_autoadd;
		    static float pitch_autoadd = 20.0f;
			static int add = 0;
			uint8_t state;
			uint8_t last_state;
				if(get_shoot_msg(&visionDataGet)==1)	
				{ 
				 Shooter_ChangeFeederMode(Feeder_FAST_CONTINUE);
				 gimbalpitch->output_state = 1;
	       gimbalyaw->output_state = 1;			
	       Chassis_SetChassisMode(Chassis_XTL);
				 last_encoder_angle=encoder_angle;
				 encoder_angle=(float)(Motor_Big_YawMotor.encoder.limited_angle - CHASSIS_YAW_ANGLE_OFFSET);
	       Chassis_SetChassisRef((float)data->remote.ch[1], (float)data->remote.ch[0], CHASSIS_XTL_WZ);	
				 buscomm->yaw_ref += Gimbal_LimitYaw((float)visionDataGet.yaw_angle.yaw_predict *0.01f*0.004f);
         GimbalYaw_SetYawRef(buscomm->yaw_ref);
				 float pitch_ref;
         pitch_ref = (float)visionDataGet.pitch_angle.pitch_predict*0.01f*0.002f*(-1); //(float)visionDataGet.pitch_angle.pitch_predict*0.01f*0.002f;
         GimbalPitch_SetPitchRef(Gimbal_LimitPitch(pitch_ref)); 					 
				}
				else if(get_shoot_msg(&visionDataGet)==0)
       {
         Shooter_ChangeFeederMode(Feeder_FINISH);
         /* 进入云台分离模式 SEP */
         gimbalpitch->output_state = 1;
         gimbalyaw->output_state = 1;
         Chassis_SetChassisMode(Chassis_XTL);
         last_encoder_angle = encoder_angle;
         encoder_angle = (float)(Motor_Big_YawMotor.encoder.limited_angle - CHASSIS_YAW_ANGLE_OFFSET);
         /* 云台分离模式下 vx/vy 由遥控器 ch[1]/ch[0]控制，wz 固定为 CHASSIS_XTL_WZ */
         Chassis_SetChassisRef((float)data->remote.ch[1], (float)data->remote.ch[0], CHASSIS_XTL_WZ);
         {
           yaw_autoadd = Const_YAW_AUTOADD;
           if(gimbalpitch->angPID.fdb < -5.0f)
             pitch_autoadd = Const_PITCH_AUTOADD;
           else if(gimbalpitch->angPID.fdb > 5.0f)
             pitch_autoadd = -Const_PITCH_AUTOADD;
           else if(pitch_autoadd == 0.0f)
             pitch_autoadd = -Const_PITCH_AUTOADD;
         }
         buscomm->yaw_ref += Gimbal_LimitYaw(yaw_autoadd * -Const_Vision_YAW_GAIN);
         GimbalYaw_SetYawRef(buscomm->yaw_ref);
         float pitch_ref;
         pitch_ref = pitch_autoadd * Const_Vision_PITCH_GAIN;
         GimbalPitch_SetPitchRef(Gimbal_LimitPitch(pitch_ref));
         /* 检测导航输入，如果有有效输入则切换到 SEP 模式，使用 dh_target 的线速度/角速度进行导航 */
         {
           float dx = (float)dh_target->linear_x, dy = (float)dh_target->linear_y, dz = (float)dh_target->angular_z;
           const float nav_thresh = 0.01f;
           if (dx*dx + dy*dy + dz*dz > nav_thresh * nav_thresh)
           {
             Chassis_SetChassisMode(Chassis_SEP);
             Chassis_SetChassisRef(dx * Const_Chasiss_Navigate_linear_x,
                                   -dy * Const_Chasiss_Navigate_linear_y,
                                   dz * Const_Chasiss_Navigate_angular_z);
           }
         }
       }
					break;
        }// 自动瞄准模式（视觉自瞄）
								
//        case Remote_SWITCH_DOWN: {
//            /* left switch down is slow shooting   */
//	          gimbalpitch->output_state = 1;
//	          gimbalyaw->output_state = 1;
//		        Chassis_SetChassisYawAngle(Motor_YawMotor.encoder.limited_angle,CHASSIS_YAW_ANGLE_OFFSET);
//		        Chassis_SetChassisMode(Chassis_FOLLOW);
//		        Chassis_SetChassisRef((float)data->remote.ch[1]  , (float)data->remote.ch[0] , limit_siqu(last_encoder_angle,encoder_angle));
//						buscomm->yaw_ref += Gimbal_LimitYaw((float)data->remote.ch[2] * -Const_WHEELLEG_REMOTE_YAW_GAIN + (float)visionDataGet.yaw_angle.yaw_predict *0.01f*0.004f);
//		GimbalYaw_SetYawRef(buscomm->yaw_ref);
//    float pitch_ref;
//    pitch_ref = (float)data->remote.ch[3] * REMOTE_PITCH_ANGLE_TO_REF; //(float)visionDataGet.pitch_angle.pitch_predict*0.01f*0.002f;
//    GimbalPitch_SetPitchRef(Gimbal_LimitPitch(-pitch_ref));
//							
//            break;
//        }// 注释掉的代码 3（慢速射击模式）
				case Remote_SWITCH_DOWN: {
            /* left switch down is slow shooting   */
	      gimbalpitch->output_state = 1;
	      gimbalyaw->output_state = 1;
	      Chassis_SetChassisMode(Chassis_FOLLOW);
				last_encoder_angle=encoder_angle;
				encoder_angle=(float)(Motor_Big_YawMotor.encoder.limited_angle - CHASSIS_YAW_ANGLE_OFFSET);
	      Chassis_SetChassisRef((float)data->remote.ch[1] , (float)data->remote.ch[0] ,limit_siqu(last_encoder_angle,encoder_angle)); //(float)(Motor_YawMotor.encoder.limited_angle - CHASSIS_YAW_ANGLE_OFFSET));
        buscomm->yaw_ref += Gimbal_LimitYaw((float)data->remote.ch[2] * -Const_WHEELLEG_REMOTE_YAW_GAIN + (float)visionDataGet.yaw_angle.yaw_predict *0.01f*0.004f);
		    GimbalYaw_SetYawRef(buscomm->yaw_ref);
        float pitch_ref;
        pitch_ref = (float)data->remote.ch[3] * REMOTE_PITCH_ANGLE_TO_REF; //(float)visionDataGet.pitch_angle.pitch_predict*0.01f*0.002f;
        GimbalPitch_SetPitchRef(Gimbal_LimitPitch(-pitch_ref));
            break;
        }// 手动控制模式（跟随底盘）

        default:
            break;
    }
	
    
}


/**
* @brief      KeyMouse control process
* @param      NULL
* @retval     NULL
*/
void Remote_KeyMouseProcess() { 
    Remote_RemoteDataTypeDef *data = Remote_GetRemoteDataPtr();
    Remote_RemoteControlTypeDef *control_data = Remote_GetControlDataPtr();
    Shoot_StatusTypeDef *shooter = Shooter_GetShooterControlPtr();
    GimbalPitch_GimbalPitchTypeDef *gimbal = GimbalPitch_GetGimbalPitchPtr();
    Protocol_DataTypeDef *buscomm = Protocol_GetBusDataPtr();
    
	
		//chassis control
		float chassis_vx;
		float chassis_vy;
		
		if(data->key.w == 1)
		{
			chassis_vx = 200.0f;
		}	
		else if(data->key.s == 1)
		{
			chassis_vx = -200.0f;
		}
		else
		{
			chassis_vx = 0.0f;
		}
		
		if(data->key.a == 1)
		{
			chassis_vy = -200.0f;
		}
		else if(data->key.d == 1)
		{
			chassis_vy = 200.0f;
		}
		else
		{
			chassis_vy = 0.0f;
		}
		
		if(data->key.shift == 1)
		{
		  Chassis_SetChassisYawAngle(Motor_Big_YawMotor.encoder.limited_angle,CHASSIS_YAW_ANGLE_OFFSET);
		  Chassis_SetChassisMode(Chassis_XTL);
		  Chassis_SetChassisRef(chassis_vx  , chassis_vy , CHASSIS_XTL_WZ);
		}
		else if(data->key.shift == 0)
		{
			Chassis_SetChassisMode(Chassis_FOLLOW);
			Chassis_SetChassisRef(chassis_vx  , chassis_vy , (float)(Motor_Big_YawMotor.encoder.limited_angle - CHASSIS_YAW_ANGLE_OFFSET));	
		}

		if(data->key.q == 1)
		{
			Servo_SetServoAngle(&Servo_ammoContainerCapServo, 90);
		}
		else if(data->key.q == 0)
		{
			Servo_SetServoAngle(&Servo_ammoContainerCapServo, 0);
		}
	
		
		//autoaim control
		float autoaim_yaw;
		float autoaim_pitch;
		if(data->mouse.r == 1)
		{
			autoaim_yaw = (float)visionDataGet.yaw_angle.yaw_predict *0.01f*0.004f;
			autoaim_pitch = (float)visionDataGet.pitch_angle.pitch_predict*0.01f*0.002f;
		}
		else if(data->mouse.r == 0)
		{
			autoaim_yaw = 0.0f;
			autoaim_pitch = 0.0f;
		}
		
		
		//gimbal control
    GimbalPitch_GimbalPitchTypeDef *gimbalpitch = GimbalPitch_GetGimbalPitchPtr();
    GimbalYaw_GimbalYawTypeDef *gimbalyaw = GimbalYaw_GetGimbalYawPtr();
		gimbalpitch->output_state = 1;
		gimbalyaw->output_state = 1;
    buscomm->yaw_ref += Gimbal_LimitYaw((float)data->mouse.x * -MOUSE_YAW_ANGLE_TO_FACT + autoaim_yaw);
		GimbalYaw_SetYawRef(buscomm->yaw_ref);
    float pitch_ref;
    pitch_ref = ((float)data->mouse.y * -MOUSE_PITCH_ANGLE_TO_FACT - autoaim_pitch);
    GimbalPitch_SetPitchRef(Gimbal_LimitPitch(-pitch_ref));

		//shoot control(fric)
    if (data->key.f == 1)      
        Shooter_ChangeShooterMode(Shoot_FAST);
    if (data->key.g == 1)      
        Shooter_ChangeShooterMode(Shoot_NULL);

}
