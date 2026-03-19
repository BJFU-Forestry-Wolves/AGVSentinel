#include "cmsis_os.h"
#include "sys_const.h"
#include "module_chassis.h"
#include "math.h"
#include "alg_quaternionEKF.h"

/*
1. 底盘坐标系（修改核心）：
   - 原点：底盘几何中心
   - x轴：向前为正（↑）
   - y轴：向右为正（→）
   - 角速度wz：逆时针（CCW）为正，顺时针（CW）为负

2. 轮子坐标（相对于底盘中心，适配新坐标系）：
   - 右后轮(0)：(-L, +W) （x前（后），y右）
   - 左后轮(1)：(-L, -W) （x前（后），y左）
   - 右前轮(2)：(+L, +W) （x前，y右）
   - 左前轮(3)：(+L, -W) （x前，y左）
   注：L=前后轴距（中心到轮的前后距离），W=左右轮距（中心到轮的左右距离）

3. 速度公式（适配新坐标系）：
   - 平移速度：vx（x轴，前/后）、vy（y轴，左/右）
   - 自转切向速度：v_rot_x = wz * wheel_y；v_rot_y = -wz * wheel_x
*/

typedef struct {
    float v;    /* 速度大小（任意比例单位，与 vx,vy 同一量纲即可） */
    float yaw;  /* 底盘坐标系下的方向角，单位：deg，0° 朝前，逆时针为正 */
} Swerve_Vec;

#define SWERVE_PI 3.1415926f

Chassis_ChassisTypeDef Chassis_ControlData[4];
Chassis_StatusTypeDef Chassis_StatusData;
float Chassis_SteerAngleDeg[4] = {0};  /* 四路舵向 limited_angle，便于校准时观察 */

/* angle to [0, 360) */
static float Swerve_NormalizeAngle360(float ang) {
    float a = ang;
    while (a >= 360.0f) a -= 360.0f;
    while (a < 0.0f)   a += 360.0f;
    return a;
}

/* err to [-180, 180] */
static float Swerve_WrapAngleError(float err) {
    while (err > 180.0f)  err -= 360.0f;
    while (err < -180.0f) err += 360.0f;
    return err;
}

/* 向量相加：输入为底盘坐标系下的两个速度向量，角度单位为 deg，返回 a + b */
static Swerve_Vec Swerve_AddVector(const Swerve_Vec *a, const Swerve_Vec *b) {
    Swerve_Vec out;
    float a_rad = a->yaw * SWERVE_PI / 180.0f;//将a向量的yaw角转换为弧度制
    float b_rad = b->yaw * SWERVE_PI / 180.0f;//将b向量的yaw角转换为弧度制
    // 适配新坐标系：x=前，y=右
    float x = a->v * cosf(a_rad) + b->v * cosf(b_rad);
    float y = a->v * sinf(a_rad) + b->v * sinf(b_rad);
    out.v = sqrtf(x * x + y * y);
    out.yaw = atan2f(y, x) * 180.0f / SWERVE_PI;  /* [-180, 180]，0°=前，逆时针为正 */
    return out;
}

/* 计算舵角最短旋转距离（单位 deg），并在必要时允许翻转 180° 同时反向轮速
 * yaw_now_360, yaw_set_360 均为 [0,360) 角度；返回的 shortest_distance ∈ [-180,180]，且不超过 90°
 * reverflag 输出为 +1 或 -1，表示是否需要反向轮速 */
static float Swerve_CalcShortestDistance(float yaw_now_360, float yaw_set_360, float *reverflag) {
    float clockwise_distance = fmodf((yaw_set_360 - yaw_now_360 + 360.0f), 360.0f);
    float counter_clockwise_distance = 360.0f - clockwise_distance;
    float reverse_distance = fabsf(fmodf(yaw_set_360 - yaw_now_360 + 180.0f, 360.0f)) - 180.0f;

    float shortest_distance = clockwise_distance;
    if (counter_clockwise_distance < shortest_distance) {
        shortest_distance = -counter_clockwise_distance;
    }

    *reverflag = 1.0f;
    /* 如果正反两个方向的最短路径都超过 90°，则考虑翻转 180° 再走最短路径 */
    if (fabsf(shortest_distance) > 90.0f) {
        float flipped_yaw_now = yaw_now_360 + 180.0f;
        if (flipped_yaw_now >= 360.0f)
            flipped_yaw_now -= 360.0f;

        if (clockwise_distance > counter_clockwise_distance) {
            /* 未翻转前正向大于反向，则翻转后取“正向距离” */
            reverse_distance = fmodf((yaw_set_360 - flipped_yaw_now + 360.0f), 360.0f);
        } else {
            /* 相反情况取反向距离 */
            reverse_distance = -fmodf(flipped_yaw_now - yaw_set_360 + 360.0f, 360.0f);
        }
        *reverflag = -1.0f;
        shortest_distance = reverse_distance;
    }
    return shortest_distance;
}

/* 舵向角目标平滑：高速时跟得快，低速时跟得慢，避免轻推摇杆瞬间转 90° 的跳动 */
#define CHASSIS_STEER_ANGLE_SMOOTH_K_FAST   (0.12f)	  /* 高速运行状态平滑系数*/
#define CHASSIS_STEER_ANGLE_SMOOTH_K_SLOW   (0.0008f)  /* 低速运行状态平滑系数,轻微拨杆时更慢，转向更丝滑 */
#define CHASSIS_STEER_SMOOTH_SPEED_THRESH   (0.01f)    /* 速度低于此用 K_SLOW，轻微拨杆算低速 */
#define CHASSIS_STEER_MAX_DEG_PER_CYCLE     (1.0f)    /* 每周期最大转角(deg)，限制瞬时跳动 */
static float s_prev_steer_ang_ref[4] = {0};

//底盘初始化
void Chassis_InitChassis(void) {
    Chassis_ChassisTypeDef *chassis = Chassis_ChassisPtr();
    Chassis_StatusTypeDef *s = Chassis_StatusPtr();
    s->chassis_mode = Chassis_NULL;
    s->Chassis_FontRight_AngleRef = 0.0f;
    s->Chassis_FontLeft_AngleRef  = 0.0f;
    s->Chassis_BackLeft_AngleRef  = 0.0f;
    s->Chassis_BackRight_AngleRef = 0.0f;
    s->Chassis_FontRight_SpeedRef = 0.0f;
    s->Chassis_FontLeft_SpeedRef  = 0.0f;
    s->Chassis_BackLeft_SpeedRef  = 0.0f;
    s->Chassis_BackRight_SpeedRef = 0.0f;
    for (int i = 0; i < 4; i++)
        s_prev_steer_ang_ref[i] = 0.0f;
    for (int i = 0; i < 4; i++) {
        chassis[i].control_state = 1;
        chassis[i].output_state = 1;
        chassis[i].chassis_ref = 0;
        chassis[i].chassis_count = 0;
    }

    //上电时八个电机的输出output全给0，避免电机疯转 
    Motor_SetMotorOutput(&Motor_ChassisBackRightSteerMotor, 0.0f);
    Motor_SetMotorOutput(&Motor_ChassisBackLeftSteerMotor,  0.0f);
    Motor_SetMotorOutput(&Motor_ChassisFontRightSteerMotor, 0.0f);
    Motor_SetMotorOutput(&Motor_ChassisFontLeftSteerMotor,  0.0f);
    Motor_SetMotorOutput(&Motor_ChassisBackRightMotor, 0.0f);
    Motor_SetMotorOutput(&Motor_ChassisBackLeftMotor,  0.0f);
    Motor_SetMotorOutput(&Motor_ChassisFontRightMotor, 0.0f);
    Motor_SetMotorOutput(&Motor_ChassisFontLeftMotor,  0.0f);
    for (int i = 0; i < 4; i++) {
        PID_ClearPID(&chassis[i].angPID);
        PID_ClearPID(&chassis[i].spdPID);
    }

    PID_InitPIDParam(&chassis[0].spdPIDParam, Const_ChassisBackRightSpdParam[0][0], Const_ChassisBackRightSpdParam[0][1], Const_ChassisBackRightSpdParam[0][2], Const_ChassisBackRightSpdParam[0][3],
                    Const_ChassisBackRightSpdParam[0][4], Const_ChassisBackRightSpdParam[1][0], Const_ChassisBackRightSpdParam[1][1], Const_ChassisBackRightSpdParam[2][0], Const_ChassisBackRightSpdParam[2][1],
                    Const_ChassisBackRightSpdParam[3][0], Const_ChassisBackRightSpdParam[3][1], PID_POSITION);
    PID_InitPIDParam(&chassis[0].angPIDParam, Const_ChassisBackRightAngParam[0][0], Const_ChassisBackRightAngParam[0][1], Const_ChassisBackRightAngParam[0][2], Const_ChassisBackRightAngParam[0][3],
                    Const_ChassisBackRightAngParam[0][4], Const_ChassisBackRightAngParam[1][0], Const_ChassisBackRightAngParam[1][1], Const_ChassisBackRightAngParam[2][0], Const_ChassisBackRightAngParam[2][1],
                    Const_ChassisBackRightAngParam[3][0], Const_ChassisBackRightAngParam[3][1], PID_POSITION);

    PID_InitPIDParam(&chassis[1].spdPIDParam, Const_ChassisBackLeftSpdParam[0][0], Const_ChassisBackLeftSpdParam[0][1], Const_ChassisBackLeftSpdParam[0][2], Const_ChassisBackLeftSpdParam[0][3],
                    Const_ChassisBackLeftSpdParam[0][4], Const_ChassisBackLeftSpdParam[1][0], Const_ChassisBackLeftSpdParam[1][1], Const_ChassisBackLeftSpdParam[2][0], Const_ChassisBackLeftSpdParam[2][1],
                    Const_ChassisBackLeftSpdParam[3][0], Const_ChassisBackLeftSpdParam[3][1], PID_POSITION);
    PID_InitPIDParam(&chassis[1].angPIDParam, Const_ChassisBackLeftAngParam[0][0], Const_ChassisBackLeftAngParam[0][1], Const_ChassisBackLeftAngParam[0][2], Const_ChassisBackLeftAngParam[0][3],
                    Const_ChassisBackLeftAngParam[0][4], Const_ChassisBackLeftAngParam[1][0], Const_ChassisBackLeftAngParam[1][1], Const_ChassisBackLeftAngParam[2][0], Const_ChassisBackLeftAngParam[2][1],
                    Const_ChassisBackLeftAngParam[3][0], Const_ChassisBackLeftAngParam[3][1], PID_POSITION);

    PID_InitPIDParam(&chassis[2].spdPIDParam, Const_ChassisFontRightSpdParam[0][0], Const_ChassisFontRightSpdParam[0][1], Const_ChassisFontRightSpdParam[0][2], Const_ChassisFontRightSpdParam[0][3],
                    Const_ChassisFontRightSpdParam[0][4], Const_ChassisFontRightSpdParam[1][0], Const_ChassisFontRightSpdParam[1][1], Const_ChassisFontRightSpdParam[2][0], Const_ChassisFontRightSpdParam[2][1],
                    Const_ChassisFontRightSpdParam[3][0], Const_ChassisFontRightSpdParam[3][1], PID_POSITION);
    PID_InitPIDParam(&chassis[2].angPIDParam, Const_ChassisFontRightAngParam[0][0], Const_ChassisFontRightAngParam[0][1], Const_ChassisFontRightAngParam[0][2], Const_ChassisFontRightAngParam[0][3],
                    Const_ChassisFontRightAngParam[0][4], Const_ChassisFontRightAngParam[1][0], Const_ChassisFontRightAngParam[1][1], Const_ChassisFontRightAngParam[2][0], Const_ChassisFontRightAngParam[2][1],
                    Const_ChassisFontRightAngParam[3][0], Const_ChassisFontRightAngParam[3][1], PID_POSITION);

    PID_InitPIDParam(&chassis[3].spdPIDParam, Const_ChassisFontLeftSpdParam[0][0], Const_ChassisFontLeftSpdParam[0][1], Const_ChassisFontLeftSpdParam[0][2], Const_ChassisFontLeftSpdParam[0][3],
                    Const_ChassisFontLeftSpdParam[0][4], Const_ChassisFontLeftSpdParam[1][0], Const_ChassisFontLeftSpdParam[1][1], Const_ChassisFontLeftSpdParam[2][0], Const_ChassisFontLeftSpdParam[2][1],
                    Const_ChassisFontLeftSpdParam[3][0], Const_ChassisFontLeftSpdParam[3][1], PID_POSITION);
    PID_InitPIDParam(&chassis[3].angPIDParam, Const_ChassisFontLeftAngParam[0][0], Const_ChassisFontLeftAngParam[0][1], Const_ChassisFontLeftAngParam[0][2], Const_ChassisFontLeftAngParam[0][3],
                    Const_ChassisFontLeftAngParam[0][4], Const_ChassisFontLeftAngParam[1][0], Const_ChassisFontLeftAngParam[1][1], Const_ChassisFontLeftAngParam[2][0], Const_ChassisFontLeftAngParam[2][1],
                    Const_ChassisFontLeftAngParam[3][0], Const_ChassisFontLeftAngParam[3][1], PID_POSITION);
}

Chassis_ChassisTypeDef* Chassis_ChassisPtr(void) {
    return Chassis_ControlData;
}

Chassis_StatusTypeDef* Chassis_StatusPtr(void) {
    return &Chassis_StatusData;
}

void Chassis_SetChassisControlState(uint8_t state) {
    Chassis_ChassisTypeDef *chassis = Chassis_ChassisPtr();
    for (int i = 0; i < 4; i++)
        chassis[i].control_state = state;
}

void Chassis_SetChassisOutputState(uint8_t state) {
    Chassis_ChassisTypeDef *chassis = Chassis_ChassisPtr();
    for (int i = 0; i < 4; i++)
        chassis[i].output_state = state;
}

void Chassis_SetChassisMode(Chassis_ModeEnum mode) {
    Chassis_StatusTypeDef *s = Chassis_StatusPtr();
    s->chassis_mode = mode;
}

void Chassis_SetChassisYawAngle(float yaw_angle, float yaw_angle_offset) {
    Chassis_StatusTypeDef *s = Chassis_StatusPtr();
    if (yaw_angle >= 0.0f && yaw_angle <= yaw_angle_offset)//当输入角度在[0, yaw_angle_offset]范围内
        s->Chassis_Yaw_Angle = yaw_angle - yaw_angle_offset + 360.0f;//减去偏移量后还得+360,保持角度在(0,360)之内
    else if (yaw_angle > yaw_angle_offset && yaw_angle <= 360.0f)//当输入角度在(yaw_angle_offset, 360]范围内
        s->Chassis_Yaw_Angle = yaw_angle - yaw_angle_offset;//直接减去偏移量
    if (s->Chassis_Yaw_Angle >= 180.0f && s->Chassis_Yaw_Angle <= 360.0f)
        s->Chassis_Yaw_Angle -= 360.0f;
    s->Chassis_Yaw_Rad = Math_Angle2Rad(s->Chassis_Yaw_Angle);
}

/* 修改核心：适配前=X正、右=Y正的坐标系，调整平移/自转向量计算 */
static void Swerve_VectorSolveWheel(float wheel_x, float wheel_y,
    float vx, float vy, float wz,
    float cur_angle_deg, int wheel_idx,
    float *angle_deg, float *speed_ref) {

    /* 平移部分：底盘左手坐标系（x=前，y=右）下的合平移向量 */
    Swerve_Vec trans;
    trans.v = sqrtf(vx * vx + vy * vy);
    // 0°=前，atan2(右向速度vy, 前向速度vx) 计算方向角
    trans.yaw = (trans.v > 1e-4f) ? (atan2f(vy, vx) * 180.0f / SWERVE_PI) : 0.0f;

    /* 自转部分：适配左手坐标系的切向速度公式（v_rot_x=前向，v_rot_y=右向） */
	float v_rot_x = wz * wheel_y;    	// 前向切向速度（x轴）	wheel_x:轮子在底盘坐标系中的x坐标
    float v_rot_y = -wz * wheel_x;   	// 右向切向速度（y轴）	wheel_y:轮子在底盘坐标系中的y坐标
    Swerve_Vec rot;
    rot.v   = sqrtf(v_rot_x * v_rot_x + v_rot_y * v_rot_y);
    rot.yaw = (rot.v > 1e-4f) ? (atan2f(v_rot_y, v_rot_x) * 180.0f / SWERVE_PI) : 0.0f;

		/* 软件层面对1（左后）、2（右前）号轮子的自转速度反向 (现在不知道为什么一晚上回来不需要了,所以不用负号了)*/
    if (wheel_idx == 1 || wheel_idx == 2) {
        rot.v = rot.v;
    }

    /* 合速度向量 = 平移向量 + 自转向量（新坐标系下） */
    Swerve_Vec total = Swerve_AddVector(&trans, &rot);

    /* 当前与目标舵向角的最短路径（不超过 90°），并在必要时翻转 180° + 反向轮速 */
    float yaw_now_360 = Swerve_NormalizeAngle360(cur_angle_deg);
    float yaw_set_360 = Swerve_NormalizeAngle360(total.yaw);
    float reverflag = 1.0f;
    float yaw_diff_360 = Swerve_CalcShortestDistance(yaw_now_360, yaw_set_360, &reverflag);

    float new_angle = yaw_now_360 + yaw_diff_360;
    new_angle = Swerve_NormalizeAngle360(new_angle);

    *angle_deg = new_angle;
    *speed_ref = total.v * reverflag;
}

void Chassis_SetChassisRef(float RC_Vx, float RC_Vy, float RC_Wz) {
    Chassis_StatusTypeDef *s = Chassis_StatusPtr();
    const float L = CHASSIS_SWERVE_L;  // 前后轴距（中心到轮的前后距离）
    const float W = CHASSIS_SWERVE_W;  // 左右轮距（中心到轮的左右距离）
    float vx = 0, vy = 0, wz = 0;

    switch (s->chassis_mode) {
        case Chassis_NULL:
            s->Chassis_FontRight_AngleRef = 0;
            s->Chassis_FontLeft_AngleRef  = 0;
            s->Chassis_BackLeft_AngleRef  = 0;
            s->Chassis_BackRight_AngleRef = 0;
            s->Chassis_FontRight_SpeedRef = 0;
            s->Chassis_FontLeft_SpeedRef  = 0;
            s->Chassis_BackLeft_SpeedRef  = 0;
            s->Chassis_BackRight_SpeedRef = 0;
            return;
        case Chassis_SEP:
            // RC_Vx=前/后（X轴），RC_Vy=左/右（Y轴）
            vx = RC_Vx * REMOTE_CHASSIS_VX_GAIN;
            vy = RC_Vy * REMOTE_CHASSIS_VY_GAIN;
            wz = RC_Wz * REMOTE_CHASSIS_SEP_WZ_GAIN;
            break;
        case Chassis_FOLLOW:
            vx = RC_Vx * REMOTE_CHASSIS_VX_GAIN;
            vy = RC_Vy * REMOTE_CHASSIS_VY_GAIN;
            wz = RC_Wz * REMOTE_CHASSIS_FOLLOW_WZ_GAIN;
            if (wz > REMOTE_CHASSIS_FOLLOW_WZ_MAX) wz = REMOTE_CHASSIS_FOLLOW_WZ_MAX;
            else if (wz < -REMOTE_CHASSIS_FOLLOW_WZ_MAX) wz = -REMOTE_CHASSIS_FOLLOW_WZ_MAX;
            break;
       
        case Chassis_XTL: {
            // 适配新坐标系的世界系→车身系转换公式
			s->Chassis_Yaw_Rad = (QEKF_INS.YawTotalAngle * SWERVE_PI )/ 180.0f;
			
            float vx_world = RC_Vx * REMOTE_CHASSIS_VX_GAIN;  // 世界系前向（X）
            float vy_world = RC_Vy * REMOTE_CHASSIS_VY_GAIN;  // 世界系右向（Y）
            

            float c = cosf(s->Chassis_Yaw_Rad); // 
            float s_yaw = sinf(s->Chassis_Yaw_Rad); 
            // 新坐标系转换公式：前=X，右=Y
            vx =  vx_world * c - vy_world * s_yaw; // 车身前向（X）
            vy = vx_world * s_yaw + vy_world * c; // 车身右向（Y）
			wz = RC_Wz * REMOTE_CHASSIS_FOLLOW_WZ_GAIN;//逆时针旋转方向
			
            break;
        }
        default:
            return;
    }
    s->Chassis_Vx = vx;
    s->Chassis_Vy = vy;
    s->Chassis_Wz = wz;

    /* 当前舵向角：逻辑角 = 编码器 - 零位偏移，使四轮安装方向不一致时 0° 均为“朝前” */
    float cur_br = Swerve_NormalizeAngle360(Motor_ChassisBackRightSteerMotor.encoder.limited_angle   - Const_ChassisSteerAngleOffset[0]);
    float cur_bl = Swerve_NormalizeAngle360(Motor_ChassisBackLeftSteerMotor.encoder.limited_angle   - Const_ChassisSteerAngleOffset[1]);
    float cur_fr = Swerve_NormalizeAngle360(Motor_ChassisFontRightSteerMotor.encoder.limited_angle  - Const_ChassisSteerAngleOffset[2]);
    float cur_fl = Swerve_NormalizeAngle360(Motor_ChassisFontLeftSteerMotor.encoder.limited_angle    - Const_ChassisSteerAngleOffset[3]);

    /* 零速时保持当前舵向角，不归位到 0°，避免松杆后四轮齐转到一个方向 */
    const float zero_speed_thresh = 0.01f;
    if (fabsf(vx) < zero_speed_thresh && fabsf(vy) < zero_speed_thresh && fabsf(wz) < zero_speed_thresh) {
        s->Chassis_BackRight_AngleRef = cur_br;
        s->Chassis_BackLeft_AngleRef  = cur_bl;
        s->Chassis_FontRight_AngleRef = cur_fr;
        s->Chassis_FontLeft_AngleRef  = cur_fl;
        s->Chassis_FontRight_SpeedRef = 0.0f;
        s->Chassis_FontLeft_SpeedRef  = 0.0f;
        s->Chassis_BackLeft_SpeedRef  = 0.0f;
        s->Chassis_BackRight_SpeedRef = 0.0f;
        return;
    }

    /* 修改核心：传入新坐标系的轮子坐标（前=X，右=Y） */
    // 右后(0)：x=-L（后），y=+W（右）
    Swerve_VectorSolveWheel(-L,  W, vx, vy, wz, cur_br, 0,
        &s->Chassis_BackRight_AngleRef, &s->Chassis_BackRight_SpeedRef);
    // 左后(1)：x=-L（后），y=-W（左）
    Swerve_VectorSolveWheel(-L, -W, vx, vy, wz, cur_bl, 1,
        &s->Chassis_BackLeft_AngleRef,  &s->Chassis_BackLeft_SpeedRef);
    // 右前(2)：x=+L（前），y=+W（右）
    Swerve_VectorSolveWheel( L,  W, vx, vy, wz, cur_fr, 2,
        &s->Chassis_FontRight_AngleRef, &s->Chassis_FontRight_SpeedRef);
    // 左前(3)：x=+L（前），y=-W（左）
    Swerve_VectorSolveWheel( L, -W, vx, vy, wz, cur_fl, 3,
        &s->Chassis_FontLeft_AngleRef,  &s->Chassis_FontLeft_SpeedRef);
}

void Chassis_Control(void) {
    Chassis_ChassisTypeDef *chassis = Chassis_ChassisPtr();
    Chassis_StatusTypeDef *s = Chassis_StatusPtr();

    for (int i = 0; i < 4; i++)
        if (chassis[i].control_state != 1) return;

    /* NULL 模式不跑闭环，强制八路输出为 0，避免上电 6020 舵向疯转 */
    if (s->chassis_mode == Chassis_NULL) {
        Motor_SetMotorOutput(&Motor_ChassisBackRightSteerMotor, 0.0f);
        Motor_SetMotorOutput(&Motor_ChassisBackLeftSteerMotor,  0.0f);
        Motor_SetMotorOutput(&Motor_ChassisFontRightSteerMotor, 0.0f);
        Motor_SetMotorOutput(&Motor_ChassisFontLeftSteerMotor,  0.0f);
        Motor_SetMotorOutput(&Motor_ChassisBackRightMotor, 0.0f);
        Motor_SetMotorOutput(&Motor_ChassisBackLeftMotor,  0.0f);
        Motor_SetMotorOutput(&Motor_ChassisFontRightMotor, 0.0f);
        Motor_SetMotorOutput(&Motor_ChassisFontLeftMotor,  0.0f);
        return;
    }

    float ang_ref[4], ang_fdb[4];
    //steer[]为舵向电机数组,wheel[]为轮向电机数组
    Motor_MotorTypeDef *steer[] = { &Motor_ChassisBackRightSteerMotor, &Motor_ChassisBackLeftSteerMotor, &Motor_ChassisFontRightSteerMotor, &Motor_ChassisFontLeftSteerMotor };
    Motor_MotorTypeDef *wheel[] = { &Motor_ChassisBackRightMotor, &Motor_ChassisBackLeftMotor, &Motor_ChassisFontRightMotor, &Motor_ChassisFontLeftMotor };

    /* 舵向反馈用逻辑角（编码器 - 零位），初始化舵向电机使四轮向电机方向对齐 */
    ang_fdb[0] = Swerve_NormalizeAngle360(Motor_ChassisBackRightSteerMotor.encoder.limited_angle  - Const_ChassisSteerAngleOffset[0]);
    ang_fdb[1] = Swerve_NormalizeAngle360(Motor_ChassisBackLeftSteerMotor.encoder.limited_angle   - Const_ChassisSteerAngleOffset[1]);
    ang_fdb[2] = Swerve_NormalizeAngle360(Motor_ChassisFontRightSteerMotor.encoder.limited_angle  - Const_ChassisSteerAngleOffset[2]);
    ang_fdb[3] = Swerve_NormalizeAngle360(Motor_ChassisFontLeftSteerMotor.encoder.limited_angle - Const_ChassisSteerAngleOffset[3]);
    /* 供校准时观察：Chassis_SteerAngleDeg[i] = 该轮当前 limited_angle */
    Chassis_SteerAngleDeg[0] = Motor_ChassisBackRightSteerMotor.encoder.limited_angle;
    Chassis_SteerAngleDeg[1] = Motor_ChassisBackLeftSteerMotor.encoder.limited_angle;
    Chassis_SteerAngleDeg[2] = Motor_ChassisFontRightSteerMotor.encoder.limited_angle;
    Chassis_SteerAngleDeg[3] = Motor_ChassisFontLeftSteerMotor.encoder.limited_angle;

    ang_ref[0] = s->Chassis_BackRight_AngleRef;
    ang_ref[1] = s->Chassis_BackLeft_AngleRef;
    ang_ref[2] = s->Chassis_FontRight_AngleRef;
    ang_ref[3] = s->Chassis_FontLeft_AngleRef;
    /* 低速时用更小平滑系数 + 每周期转角上限，避免轻推摇杆时舵向瞬间跳 90° */
	const float L = CHASSIS_SWERVE_L;  // 前后轴距（中心到轮的前后距离）
    const float W = CHASSIS_SWERVE_W;  // 左右轮距（中心到轮的左右距离）
    float speed_mag = sqrtf(s->Chassis_Vx * s->Chassis_Vx + s->Chassis_Vy * s->Chassis_Vy) + fabsf(s->Chassis_Wz) * sqrtf(L * L + W * W);
    float smooth_k = (speed_mag < CHASSIS_STEER_SMOOTH_SPEED_THRESH) ? CHASSIS_STEER_ANGLE_SMOOTH_K_SLOW : CHASSIS_STEER_ANGLE_SMOOTH_K_FAST;
    for (int i = 0; i < 4; i++) {
        float err = Swerve_WrapAngleError(ang_ref[i] - s_prev_steer_ang_ref[i]);
        float delta = smooth_k * err;
        if (delta > CHASSIS_STEER_MAX_DEG_PER_CYCLE) delta = CHASSIS_STEER_MAX_DEG_PER_CYCLE;
        else if (delta < -CHASSIS_STEER_MAX_DEG_PER_CYCLE) delta = -CHASSIS_STEER_MAX_DEG_PER_CYCLE;
        s_prev_steer_ang_ref[i] += delta;
        s_prev_steer_ang_ref[i] = Swerve_NormalizeAngle360(s_prev_steer_ang_ref[i]);
        ang_ref[i] = s_prev_steer_ang_ref[i];
    }
    float spd_ref[] = { s->Chassis_BackRight_SpeedRef, s->Chassis_BackLeft_SpeedRef, s->Chassis_FontRight_SpeedRef, s->Chassis_FontLeft_SpeedRef };

    /* 舵向角环：用“虚拟反馈”使 PID 始终看到最短路径误差 */
    for (int i = 0; i < 4; i++) {
        float err = Swerve_WrapAngleError(ang_ref[i] - ang_fdb[i]);  /* 最短路径误差 [-180, 180] */
        float fdb_for_pid = ang_ref[i] - err;   /* 虚拟 fdb：使 ref - fdb = err */
        PID_SetPIDRef(&chassis[i].angPID, ang_ref[i]);
        PID_SetPIDFdb(&chassis[i].angPID, fdb_for_pid);
        PID_CalcPID(&chassis[i].angPID, &chassis[i].angPIDParam);
        float out = PID_GetPIDOutput(&chassis[i].angPID);
        if (i == 0) out *= CHASSIS_STEER0_SIGN;
        if (i == 1) out *= CHASSIS_STEER1_SIGN;
        if (i == 2) out *= CHASSIS_STEER2_SIGN;
        if (i == 3) out *= CHASSIS_STEER3_SIGN;
        Motor_SetMotorOutput(steer[i], out);

        float wheel_sign = (i == 0) ? CHASSIS_WHEEL0_SIGN : (i == 1) ? CHASSIS_WHEEL1_SIGN : (i == 2) ? CHASSIS_WHEEL2_SIGN : CHASSIS_WHEEL3_SIGN;
        PID_SetPIDRef(&chassis[i].spdPID, spd_ref[i]);
        PID_SetPIDFdb(&chassis[i].spdPID, wheel[i]->encoder.speed * wheel_sign);
        PID_CalcPID(&chassis[i].spdPID, &chassis[i].spdPIDParam);
        float wheel_out = PID_GetPIDOutput(&chassis[i].spdPID) * wheel_sign;
        Motor_SetMotorOutput(wheel[i], wheel_out);
    }
}

void Chassis_Output(void) {
    Chassis_ChassisTypeDef *chassis = Chassis_ChassisPtr();
    for (int i = 0; i < 4; i++)
        if (chassis[i].output_state != 1) return;
    Motor_SendMotorGroupOutput(&Motor_ChassisMotors);
    Motor_SendMotorGroupOutput(&Motor_ChassisSteerMotors);
}

void Chasssis_SetChasssisFontRightRef(float chassisfontright_ref) { (void)chassisfontright_ref; }
void Chassis_SetChassisFontLeftRef(float chassisfontleft_ref)  { (void)chassisfontleft_ref; }
void Chassis_SetChassisBackLeftRef(float chassisbackleft_ref)  { (void)chassisbackleft_ref; }
void Chassis_SetChassisBackRightRef(float chassisbackright_ref) { (void)chassisbackright_ref; }