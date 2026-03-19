/*
 *  Project      : Polaris
 * 
 *  file         : module_referee.c
 *  Description  : Polaris
 *  LastEditors  : Polaris
 *  Date         : 2021-05-04 20:53:31
 *  LastEditTime : 2023-05-05 11:18:25
 */


#include "module_referee.h"

#include "periph_referee.h"
#include "cmsis_os.h"




uint8_t referee_setup_flag = 0;


/********** Drawing Constants **********/

// 关于图层：图层0 ~ 9，高图层遮盖低图层
// 对于经常更新的分图层功能，建议前景图层使用3，背景图层使用2
// 其他功能在不产生遮挡的情况下建议使用图层2

// 关于坐标：左下角为 (0, 0)，水平方向为 X，垂直方向为 Y

const uint8_t AIM_LINE_LAYER        = 2;
const Draw_Color AIM_LINE_COLOR     = Draw_COLOR_GREEN;
const uint8_t AIM_LINE_LINE_MODE    = 3;
const uint8_t AIM_LINE_LINE_NUM     = 3 + 1;
const uint16_t AIM_LINES[AIM_LINE_LINE_MODE][AIM_LINE_LINE_NUM][6] = {  
    // ID, Width, X1, Y1, X2, Y2
    {       // Mode 0: 15 m/s
        {0x101, 2, 960, 500, 960, 620},     // Vertical Line
        {0x102, 4, 850, 600, 950, 600},     // Horizontal Line 1
        {0x103, 2, 850, 560, 950, 560},     // Horizontal Line 2
        {0x104, 2, 870, 520, 930, 520}      // Horizontal Line 3
    }, {    // Mode 1: 18 m/s
        {0x101, 2, 960, 500, 960, 620},     // Vertical Line
        {0x102, 4, 850, 600, 950, 600},     // Horizontal Line 1
        {0x103, 2, 850, 540, 950, 540},     // Horizontal Line 2
        {0x104, 2, 870, 500, 930, 500}      // Horizontal Line 3
    }, {    // Mode 2: 30 m/s
        {0x101, 2, 960, 500, 960, 620},     // Vertical Line
        {0x102, 4, 850, 600, 950, 600},     // Horizontal Line 1
        {0x103, 2, 850, 580, 950, 580},     // Horizontal Line 2
        {0x104, 2, 870, 560, 930, 560}      // Horizontal Line 3
    }
};

const uint8_t CROSSHAIR_LAYER       = 2;
const Draw_Color CROSSHAIR_COLOR    = Draw_COLOR_GREEN;
const uint16_t CROSSHAIR[5]         = {0x201, 2, 960, 560, 10};  // ID, Width, X, Y, R

const uint8_t WIDTH_MARK_LAYER      = 2;
const Draw_Color WIDTH_MARK_COLOR   = Draw_COLOR_YELLOW;
const uint16_t WIDTH_MARK_GYRO[2][6] = {      // ID, Width, X0, Y0, X1, Y1
    {0x301, 2, 660, 400, 660, 200},     // Left Mark Line, Normal
    {0x302, 2, 1260, 400, 1260, 200},     // Right Mark Line, Normal
};
const uint16_t WIDTH_MARK_NORMAL[2][6] = {
    {0x301, 2, 660, 400, 960, 200},     // Left Mark Line, Gyro Mode
    {0x302, 2, 1260, 400, 960, 200},     // Right Mark Line, Gyro Mode
};

const uint8_t CAP_STATE_LAYER[2]    = {3, 2};   // Foreground, Background
const Draw_Color CAP_STATE_COLOR[5] = {
    Draw_COLOR_WHITE,           // Background
    Draw_COLOR_GREEN,           // Text
    Draw_COLOR_GREEN,           // Foreground, Full (50% ~ 100%)
    Draw_COLOR_YELLOW,          // Foreground, Insufficient (10% ~ 50%)
    Draw_COLOR_ORANGE           // Foreground, Empty (0% ~ 10%)
};
const uint16_t CAP_STATE[4]         = {6, 960, 240, 40};     // Width, X, Y, R
const uint16_t CAP_STATE_CIRCLE     = 0x401;            // Background Circle ID
const uint16_t CAP_STATE_ARC        = 0x402;            // Foreground Arc ID
const uint16_t CAP_STATE_TEXT[5]    = {0x403, 20, 2, 900, 240};     // ID, Font Size, Width, X, Y
const char *CAP_STATE_TEXT_STR      = "CAP";

const uint8_t PITCH_METER_LAYER     = 2;
const Draw_Color PITCH_METER_COLOR  = Draw_COLOR_GREEN;
const uint16_t PITCH_METER_TEXT[5]  = {0x501, 20, 2, 1500, 540};     // ID, Font Size, Width, X, Y
const char *PITCH_METER_TEXT_STR    = "PITCH:";
const uint16_t PITCH_METER_VALUE[6] = {0x502, 20, 3, 2, 1200, 540};  // ID, Font Size, Precision, Width, X, Y

const uint8_t AIM_MODE_LAYER        = 2;
const Draw_Color AIM_MODE_COLOR     = Draw_COLOR_GREEN;
const uint16_t AIM_MODE_TEXT[5]     = {0x501, 20, 2, 600, 840};     // ID, Font Size, Width, X, Y
const uint16_t AIM_MODE_VALUE_TEXT[5]  = {0x501, 20, 2, 1000, 840};     // ID, Font Size, Width, X, Y
const char *AIM_MODE_TEXT_STR       = "AIM_MODE:";
const char *NORMAL_AIM_TEXT_STR     = "NORMAL";
const char *ARMOR_AIM_TEXT_STR      = "ARMOR";
const char *BIG_BUFF_AIM_TEXT_STR   = "BIG_BUF";
const char *SMALL_BUFF_AIM_TEXT_STR = "SMALL_BUF";

const uint8_t CHASSIS_MODE_LAYER    = 2;
const Draw_Color CHASSIS_MODE_COLOR = Draw_COLOR_GREEN;
const uint16_t CHASSIS_MODE_TEXT[5] = {0x501, 20, 2, 600, 540};     // ID, Font Size, Width, X, Y
const uint16_t CHASSIS_MODE_VALUE_TEXT[5]  = {0x501, 20, 2, 1000, 540};     // ID, Font Size, Width, X, Y
const char *CHASSIS_MODE_TEXT_STR  = "CHASSIS_MODE:";
const char *NORMAL_RUN_TEXT_STR    = "NORMAL";
const char *GYRO_RUN_TEXT_STR      = "GYRO";



/********** END OF Drawing Constants **********/


Referee_DrawDataTypeDef Referee_DrawData;



/**
  * @brief      设置车宽线模式
  * @param      mode: 车宽线模式（1为小陀螺，0为普通）
  * @retval     无
  */
void Referee_SetWidthMode(uint8_t mode) {
    Referee_DrawDataTypeDef *draw = &Referee_DrawData;
    draw->width_mode = mode;
}


/**
  * @brief      设置瞄准线模式
  * @param      mode: 瞄准线模式（0 ~ 2对应弹速 15,18,30 m/s）
  * @retval     无
  */
void Referee_SetAimMode(uint8_t mode) {
    Referee_DrawDataTypeDef *draw = &Referee_DrawData;
    if (mode > 2) return;
    draw->aim_mode = mode;
}


/**
  * @brief      设置电容电量
  * @param      state: 电容电量（0 ~ 100，单位百分比）
  * @retval     无
  */
void Referee_SetCapState(uint8_t state) {
    Referee_DrawDataTypeDef *draw = &Referee_DrawData;
    draw->cap_state = state;
}


/**
  * @brief      设置Pitch倾角
  * @param      angle: Pitch倾角
  * @retval     无
  */
void Referee_SetPitchAngle(float angle) {
    Referee_DrawDataTypeDef *draw = &Referee_DrawData;
    draw->pitch_angle = angle;
}


/**
  * @brief      瞄准线绘制：初始化阶段
  * @param      无
  * @retval     无
  */
void Referee_SetupAimLine() {
    // draw_cnt: 4
    Referee_DrawDataTypeDef *draw = &Referee_DrawData;
    draw->aim_mode_last = draw->aim_mode;
    const uint16_t (*aim_lines)[6] = AIM_LINES[draw->aim_mode];
    for (int i = 0; i < AIM_LINE_LINE_NUM; ++i) {
        Draw_AddLine(aim_lines[i][0], AIM_LINE_LAYER, AIM_LINE_COLOR, aim_lines[i][1], aim_lines[i][2], aim_lines[i][3], aim_lines[i][4], aim_lines[i][5]);
    }
}


/**
  * @brief      瞄准线绘制：更新阶段
  * @param      无
  * @retval     无
  */
void Referee_UpdateAimLine() {
    // draw_cnt: 4 when mode changed, 0 when mode not change
    Referee_DrawDataTypeDef *draw = &Referee_DrawData;
    if (draw->aim_mode_last == draw->aim_mode) return;
    draw->aim_mode_last = draw->aim_mode;
    const uint16_t (*aim_lines)[6] = AIM_LINES[draw->aim_mode];
    for (int i = 0; i < AIM_LINE_LINE_NUM; ++i) {
        Draw_AddLine(aim_lines[i][0], AIM_LINE_LAYER, AIM_LINE_COLOR, aim_lines[i][1], aim_lines[i][2], aim_lines[i][3], aim_lines[i][4], aim_lines[i][5]);
    }
}


/**
  * @brief      准心绘制：初始化阶段
  * @param      无
  * @retval     无
  */
void Referee_SetupCrosshair() {
    // draw_cnt: 1
    Draw_AddCircle(CROSSHAIR[0], CROSSHAIR_LAYER, CROSSHAIR_COLOR, CROSSHAIR[1], CROSSHAIR[2], CROSSHAIR[3], CROSSHAIR[4]);
}


/**
  * @brief      准心绘制：更新阶段
  * @param      无
  * @retval     无
  */
void Referee_UpdateCrosshair() {
    // nothing
}


/**
  * @brief      车宽线绘制：初始化阶段
  * @param      无
  * @retval     无
  */
void Referee_SetupWidthMark() {
    // draw_cnt: 2
    Referee_DrawDataTypeDef *draw = &Referee_DrawData;
    draw->width_mode_last = draw->width_mode;
    const uint16_t (*mark)[6] = (draw->width_mode == 1) ? WIDTH_MARK_NORMAL : WIDTH_MARK_GYRO;
    for (int i = 0; i < 2; ++i) {
        Draw_AddLine(mark[i][0], WIDTH_MARK_LAYER, WIDTH_MARK_COLOR, mark[i][1], mark[i][2], mark[i][3], mark[i][4], mark[i][5]);
    }
}


/**
  * @brief      车宽线绘制：更新阶段
  * @param      无
  * @retval     无
  */
void Referee_UpdateWidthMark() {
    // draw_cnt: 2 when mode changed, 0 when mode not change
    Referee_DrawDataTypeDef *draw = &Referee_DrawData;
    if (draw->width_mode_last == draw->width_mode) return;
    draw->width_mode_last = draw->width_mode;
    const uint16_t (*mark)[6] = (draw->width_mode == 1) ? WIDTH_MARK_NORMAL : WIDTH_MARK_GYRO;
    for (int i = 0; i < 2; ++i) {
        Draw_ModifyLine(mark[i][0], WIDTH_MARK_LAYER, WIDTH_MARK_COLOR, mark[i][1], mark[i][2], mark[i][3], mark[i][4], mark[i][5]);
    }
}


/**
  * @brief      电容状态绘制：初始化阶段
  * @param      无
  * @retval     无
  */
void Referee_SetupCapState() {
    // draw_cnt: 2
    Referee_DrawDataTypeDef *draw = &Referee_DrawData;
    
    Draw_AddCircle(CAP_STATE_CIRCLE, CAP_STATE_LAYER[1], CAP_STATE_COLOR[0], CAP_STATE[0], CAP_STATE[1], CAP_STATE[2], CAP_STATE[3]);
    
    int value = draw->cap_state;
    value = (int) (-draw->pitch_angle + 10) * 2;
    
    Draw_Color color;
    if (value > 100)
        return;
    else if (value >= 50) 
        color = CAP_STATE_COLOR[2];
    else if (value >= 20)
        color = CAP_STATE_COLOR[3];
    else
        color = CAP_STATE_COLOR[4];
    
    uint16_t start_angle = 0;
    uint16_t end_angle = 0;
    if (value > 0 && value <= 100)
        end_angle = (uint16_t) (360.0 * value / 100.0);
    
    Draw_AddArc(CAP_STATE_ARC, CAP_STATE_LAYER[0], color, start_angle, end_angle, CAP_STATE[0], CAP_STATE[1], CAP_STATE[2], CAP_STATE[3], CAP_STATE[3]);
}


/**
  * @brief      电容状态绘制：更新阶段
  * @param      无
  * @retval     无
  */
void Referee_UpdateCapState() {
    // draw_cnt: 1
    Referee_DrawDataTypeDef *draw = &Referee_DrawData;

    int value = draw->cap_state;
//    value = (int) (-draw->pitch_angle + 10) * 2;
    
    Draw_Color color;
    if (value > 100)
        return;
    else if (value >= 50) 
        color = CAP_STATE_COLOR[2];
    else if (value >= 20)
        color = CAP_STATE_COLOR[3];
    else
        color = CAP_STATE_COLOR[4];
    
    uint16_t start_angle = 0;
    uint16_t end_angle = 1;
    if (value > 0 && value <= 100)
        end_angle = (uint16_t) (360.0f * value / 100.0f);
    
    Draw_ModifyArc(CAP_STATE_ARC, CAP_STATE_LAYER[0], color, start_angle, end_angle, CAP_STATE[0], CAP_STATE[1], CAP_STATE[2], CAP_STATE[3], CAP_STATE[3]);
}


/**
  * @brief      Pitch倾角计绘制：初始化阶段
  * @param      无
  * @retval     无
  */
void Referee_SetupPitchMeter() {
    // draw_cnt: 1
    Referee_DrawDataTypeDef *draw = &Referee_DrawData;
    float value = -draw->pitch_angle;
    Draw_AddFloat(PITCH_METER_VALUE[0], PITCH_METER_LAYER, PITCH_METER_COLOR, PITCH_METER_VALUE[1], PITCH_METER_VALUE[2], PITCH_METER_VALUE[3], PITCH_METER_VALUE[4], PITCH_METER_VALUE[5], value);
}


/**
  * @brief      Pitch倾角计绘制：更新阶段
  * @param      无
  * @retval     无
  */
void Referee_UpdatePitchMeter() {
    // draw_cnt: 1
    Referee_DrawDataTypeDef *draw = &Referee_DrawData;
    float value = -draw->pitch_angle;
    Draw_ModifyFloat(PITCH_METER_VALUE[0], PITCH_METER_LAYER, PITCH_METER_COLOR, PITCH_METER_VALUE[1], PITCH_METER_VALUE[2], PITCH_METER_VALUE[3], PITCH_METER_VALUE[4], PITCH_METER_VALUE[5], value);
    //Draw_ModifyInt(PITCH_METER_VALUE[0], PITCH_METER_LAYER, PITCH_METER_COLOR, PITCH_METER_VALUE[1], PITCH_METER_VALUE[3], PITCH_METER_VALUE[4], PITCH_METER_VALUE[5], (int32_t) (value * 1000));
}


/**
  * @brief      设置底盘和自瞄模式
  * @param      auto_aim_mode: 自瞄模式（0 ~ 3对应 无自瞄、装甲板自瞄、小能量自瞄、大能量自瞄）
  * @param      cha_mode: 底盘模式 （0 ~ 1对应 正常底盘运动 、 小陀螺模式）
  * @retval     无
  */
void Referee_SetMode(uint8_t auto_aim_mode, uint8_t cha_mode) {
    Referee_DrawDataTypeDef *draw = &Referee_DrawData;
    if (auto_aim_mode <= 3)
        draw->auto_aim_mode = auto_aim_mode;
    if (cha_mode <= 1)
        draw->cha_mode = cha_mode;
}


/**
  * @brief      模式显示绘制：初始化阶段
  * @param      无
  * @retval     无
  */
void Referee_SetupModeDisplay() {
    Referee_DrawDataTypeDef *draw = &Referee_DrawData;
    draw->auto_aim_mode_last = draw->auto_aim_mode;
    draw->cha_mode_last = draw->cha_mode;
    
    Draw_AddString(AIM_MODE_VALUE_TEXT[0], AIM_MODE_LAYER, AIM_MODE_COLOR, AIM_MODE_VALUE_TEXT[1], AIM_MODE_VALUE_TEXT[2], AIM_MODE_VALUE_TEXT[3], AIM_MODE_VALUE_TEXT[4], NORMAL_AIM_TEXT_STR);    
    Draw_AddString(CHASSIS_MODE_VALUE_TEXT[0], CHASSIS_MODE_LAYER, CHASSIS_MODE_COLOR, CHASSIS_MODE_VALUE_TEXT[1], CHASSIS_MODE_VALUE_TEXT[2], CHASSIS_MODE_VALUE_TEXT[3], CHASSIS_MODE_VALUE_TEXT[4], NORMAL_RUN_TEXT_STR);
}


/**
  * @brief      模式显示绘制：更新阶段
  * @param      无
  * @retval     无
  */
void Referee_UpdateModeDisplay() {
    Referee_DrawDataTypeDef *draw = &Referee_DrawData;
    if (draw->auto_aim_mode_last != draw->auto_aim_mode) {
        draw->auto_aim_mode_last = draw->auto_aim_mode;
        switch (draw->auto_aim_mode) {
            case 0:
                Draw_AddString(AIM_MODE_VALUE_TEXT[0], AIM_MODE_LAYER, AIM_MODE_COLOR, AIM_MODE_VALUE_TEXT[1], AIM_MODE_VALUE_TEXT[2], AIM_MODE_VALUE_TEXT[3], AIM_MODE_VALUE_TEXT[4], NORMAL_AIM_TEXT_STR);      
                break;
            case 1:
                Draw_AddString(AIM_MODE_VALUE_TEXT[0], AIM_MODE_LAYER, AIM_MODE_COLOR, AIM_MODE_VALUE_TEXT[1], AIM_MODE_VALUE_TEXT[2], AIM_MODE_VALUE_TEXT[3], AIM_MODE_VALUE_TEXT[4], ARMOR_AIM_TEXT_STR);
                break;
            case 2:
                Draw_AddString(AIM_MODE_VALUE_TEXT[0], AIM_MODE_LAYER, AIM_MODE_COLOR, AIM_MODE_VALUE_TEXT[1], AIM_MODE_VALUE_TEXT[2], AIM_MODE_VALUE_TEXT[3], AIM_MODE_VALUE_TEXT[4], BIG_BUFF_AIM_TEXT_STR);
                break;
            case 3:
                Draw_AddString(AIM_MODE_VALUE_TEXT[0], AIM_MODE_LAYER, AIM_MODE_COLOR, AIM_MODE_VALUE_TEXT[1], AIM_MODE_VALUE_TEXT[2], AIM_MODE_VALUE_TEXT[3], AIM_MODE_VALUE_TEXT[4], SMALL_BUFF_AIM_TEXT_STR);
                break;
            default:
                break;    
        }
    }

    if (draw->cha_mode_last != draw->cha_mode) {
        draw->cha_mode_last = draw->cha_mode;
        switch (draw->cha_mode) {
            case 0:
                Draw_AddString(CHASSIS_MODE_VALUE_TEXT[0], CHASSIS_MODE_LAYER, CHASSIS_MODE_COLOR, CHASSIS_MODE_VALUE_TEXT[1], CHASSIS_MODE_VALUE_TEXT[2], CHASSIS_MODE_VALUE_TEXT[3], CHASSIS_MODE_VALUE_TEXT[4], NORMAL_RUN_TEXT_STR);            
                break;
            case 1:
                Draw_AddString(CHASSIS_MODE_VALUE_TEXT[0], CHASSIS_MODE_LAYER, CHASSIS_MODE_COLOR, CHASSIS_MODE_VALUE_TEXT[1], CHASSIS_MODE_VALUE_TEXT[2], CHASSIS_MODE_VALUE_TEXT[3], CHASSIS_MODE_VALUE_TEXT[4], GYRO_RUN_TEXT_STR);
                break;
            default:
                break;
        }
    }
}


/**
  * @brief      错误显示绘制：初始化阶段
  * @param      无
  * @retval     无
  */
void Referee_SetupErrorDisplay() {
    
}


/**
  * @brief      错误显示绘制：更新阶段
  * @param      无
  * @retval     无
  */
void Referee_UpdateErrorDisplay() {
    
}


/**
  * @brief      各功能初始化阶段文字绘制
  * @param      无
  * @retval     无
  */
void Referee_SetupAllString() {
    // cmd_cnt: 2
    //Referee_RefereeDataTypeDef *Referee = &Referee_DrawData;
    
    Draw_AddString(CAP_STATE_TEXT[0], CAP_STATE_LAYER[1], CAP_STATE_COLOR[1], CAP_STATE_TEXT[1], CAP_STATE_TEXT[2], CAP_STATE_TEXT[3], CAP_STATE_TEXT[4], CAP_STATE_TEXT_STR);
    Draw_AddString(PITCH_METER_TEXT[0], PITCH_METER_LAYER, PITCH_METER_COLOR, PITCH_METER_TEXT[1], PITCH_METER_TEXT[2], PITCH_METER_TEXT[3], PITCH_METER_TEXT[4], PITCH_METER_TEXT_STR);

    Draw_AddString(AIM_MODE_TEXT[0], AIM_MODE_LAYER, AIM_MODE_COLOR, AIM_MODE_TEXT[1], AIM_MODE_TEXT[2], AIM_MODE_TEXT[3], AIM_MODE_TEXT[4], AIM_MODE_TEXT_STR);
    Draw_AddString(CHASSIS_MODE_TEXT[0], CHASSIS_MODE_LAYER, CHASSIS_MODE_COLOR, CHASSIS_MODE_TEXT[1], CHASSIS_MODE_TEXT[2], CHASSIS_MODE_TEXT[3], CHASSIS_MODE_TEXT[4], CHASSIS_MODE_TEXT_STR);
}

uint8_t syj = 0;

/**
  * @brief      初始化各绘制功能
  * @param      无
  * @retval     无
  */
void Referee_Setup() {     
    static int last_time = -1000;
    int now = HAL_GetTick();
    if (now - last_time < 1000) return;
    last_time = now;    
    
	/// syj

	
	if (!syj) {
		Draw_ClearAll();                    // cmd_cnt: 1, total_cmd_cnt: 1
		Referee_SetupAimLine();            // draw_cnt: 4
		Referee_SetupCrosshair();          // draw_cnt: 1
		syj++;
	}
	else if (syj == 1) {
		Referee_SetupWidthMark();          // draw_cnt: 2, send(7), total_cmd_cnt: 2
		Referee_SetupCapState();           // draw_cnt: 2
		Referee_SetupPitchMeter();         // draw_cnt: 1
		syj++;
	}
	else if (syj == 2) {
		// Referee_SetupModeDisplay();        // draw_cnt: 2
		// Referee_SetupErrorDisplay();       // draw_cnt: 0, send(2)send(1), total_cmd_cnt: 4
		syj++;
	}
	else if (syj >= 3) {
		//Referee_SetupAllString();          // cmd_cnt: 2, total_cmd_cnt: 6
		syj = 0;
	}
    Referee_DrawingBufferFlush();       // useless since string cmd sent previously
    referee_setup_flag = 1;
}


/**
  * @brief      更新各绘制功能
  * @param      无
  * @retval     无
  */
void Referee_Update() {                
    Referee_UpdateAimLine();           // draw_cnt: if bullet speed changed 4, else 0
    Referee_UpdateCrosshair();         // draw_cnt: 0
    Referee_UpdateWidthMark();         // draw_cnt: if gyro mode changed 2, else 0
    Referee_UpdateCapState();          // draw_cnt: 1
    Referee_UpdatePitchMeter();        // draw_cnt: 1
    Referee_UpdateModeDisplay();       // draw_cnt: 0
    Referee_UpdateErrorDisplay();      // draw_cnt: 0
    
    Referee_DrawingBufferFlush();       // max draw_cnt: 8, cmd_cnt:2
                                        // min draw_cnt: 2, cmd_cnt:1
}
