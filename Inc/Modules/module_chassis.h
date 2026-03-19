#ifndef MODULE_CHASSIS
#define MODULE_CHASSIS

#ifdef __cplusplus
extern "C" {
#endif

#include "periph_motor.h"
#include "alg_math.h"
#include "alg_pid.h"

typedef enum {
		Chassis_NULL = 0u,
		Chassis_SEP = 1u,
		Chassis_FOLLOW = 2u,
		Chassis_XTL = 3u
}Chassis_ModeEnum;	
	
typedef struct {
    float chassis_ref;   //速度期望值                               
    float chassis_position_fdb;//位置反馈值                         
    float chassis_speed_fdb;  //速度反馈值                      
    uint8_t chassis_ref_limit_status;//底盘限位标志位                   
    uint8_t chassis_count;

    uint8_t control_state;//控制状态                          
    uint8_t output_state;                          
    uint8_t pending_state;                          

    PID_PIDTypeDef spdPID;//
    PID_PIDParamTypeDef spdPIDParam;//速度pid参数配置
    PID_PIDTypeDef angPID;
    PID_PIDParamTypeDef angPIDParam;//角度pid参数配置

} Chassis_ChassisTypeDef;

typedef struct {
	  Chassis_ModeEnum chassis_mode;
	
		float Chassis_Vx;
		float Chassis_Vy;
		float Chassis_Wz;
	
		float Chassis_Yaw_Angle;//原始偏航角，来自c板陀螺仪,范围是 0° ~ 360°													
		float Chassis_Yaw_Rad;
		
		/* 舵轮：四轮舵向目标角(deg 0~360)与轮向目标速度 */
		float Chassis_FontRight_AngleRef;
		float Chassis_FontLeft_AngleRef;
		float Chassis_BackLeft_AngleRef;
		float Chassis_BackRight_AngleRef;
	
		float Chassis_FontRight_SpeedRef;
		float Chassis_FontLeft_SpeedRef;
		float Chassis_BackLeft_SpeedRef;
		float Chassis_BackRight_SpeedRef;
	
} Chassis_StatusTypeDef;

extern Chassis_ChassisTypeDef Chassis_ControlData[4];
extern Chassis_StatusTypeDef Chassis_StatusData;
/* 四路舵向当前编码器角(deg) [右后,左后,右前,左前]，校准时把四轮掰朝前后看此数组并填入 Const_ChassisSteerAngleOffset */
extern float Chassis_SteerAngleDeg[4];

//舵向电机输出方向
#ifndef CHASSIS_STEER0_SIGN
#define CHASSIS_STEER0_SIGN   (1)
#endif
#ifndef CHASSIS_STEER1_SIGN
#define CHASSIS_STEER1_SIGN   (1)
#endif
#ifndef CHASSIS_STEER2_SIGN
#define CHASSIS_STEER2_SIGN   (1)
#endif	
#ifndef CHASSIS_STEER3_SIGN
#define CHASSIS_STEER3_SIGN   (1)
#endif

/* 轮向电机(3508)方向：反了的那路设为 -1（顺序 [右后,左后,右前,左前]）；现在四个轮安装方向一致，但都与编码器实际正转方向不一致,所以全部设为 -1 */
#ifndef CHASSIS_WHEEL0_SIGN
#define CHASSIS_WHEEL0_SIGN   (-1)
#endif
#ifndef CHASSIS_WHEEL1_SIGN
#define CHASSIS_WHEEL1_SIGN   (-1)
#endif
#ifndef CHASSIS_WHEEL2_SIGN
#define CHASSIS_WHEEL2_SIGN   (-1)
#endif
#ifndef CHASSIS_WHEEL3_SIGN
#define CHASSIS_WHEEL3_SIGN   (-1)
#endif

void Chassis_InitChassis(void);

Chassis_ChassisTypeDef* Chassis_ChassisPtr(void);
void Chassis_SetChassisControlState(uint8_t state);
void Chassis_SetChassisOutputState(uint8_t state);

Chassis_StatusTypeDef* Chassis_StatusPtr(void);
void Chassis_SetChassisMode(Chassis_ModeEnum mode);
void Chassis_SetChassisYawAngle(float yaw_angle,float yaw_angle_offset);
void Chassis_SetChassisRef(float RC_Vx,float RC_Vy,float RC_Wz);

void Chasssis_SetChasssisFontRightRef(float chassisfontright_ref);
void Chassis_SetChassisFontLeftRef(float chassisfontleft_ref);
void Chassis_SetChassisBackLeftRef(float chassisbackleft_ref);
void Chassis_SetChassisBackRightRef(float chassisbackright_ref);

void Chassis_Control(void);
void Chassis_Output(void);


#endif
	
#ifdef __cplusplus
}
#endif

