#ifndef APP_AUTOAIM_H
#define APP_AUTOAIM_H

#ifdef __cplusplus
extern "C" {
#endif 

#include "cmsis_os.h"
#include "FreeRTOS.h"
#include "periph_autoaim.h"
#include "periph_referee.h"
#include "struct_typedef.h"
#include "app_ins.h"
#define USART1_MAX_RECV_LEN		400					//最大接收缓存字节数
#define USART1_MAX_SEND_LEN		400					//最大发送缓存字节数

extern uint8_t  USART1_RX_BUF[USART1_MAX_RECV_LEN];
extern uint16_t USART1_RX_STA;
extern int Armor_R_Flag_Before;
extern int Armor_R_Flag_Behind;
extern int autoaim_mode_flag;
extern uint8_t ChariqotRecognition_data[2][30];	
	
#define ChariotRecognition_data_dma_buf_len   30u
#define ChariotRecognition_data_len           15u
#define A_Size  60
#define Dis_buf_Size 10
#define Sendtosight_len 14
#define BUFFER_SIZE 5
#define SHOOT_MSG_BUF_SIZE 5


typedef struct {
	uint8_t head;       //帧头
	uint8_t color;      //颜色
	uint8_t mode;       //模式
	uint8_t robotID;    //机器人ID
	
	union yaw_INS {     //YAW轴陀螺仪数据 （低位优先发送）
		int16_t yaw_gyro;
		uint8_t yaw_angle_send[2];
	}yaw_angle;
	union pitch_INS {   //PITCH轴陀螺仪数据 （低位优先发送）
		int16_t pitch_gyro;
		uint8_t pitch_angle_send[2];
	}pitch_angle;
	union yaw_Acc {     //YAW轴陀螺仪加速度（低位优先发送）
		int16_t yaw_acc;
		uint8_t yaw_acc_send[2];
	}yaw_acc_data;
	union pitch_Acc {   //PITCH轴陀螺仪加速度 （低位优先发送）
		int16_t pitch_acc;
		uint8_t pitch_acc_send[2];
	}pitch_acc_data;
	
	uint8_t shoot_speed;//子弹速度
	uint8_t CRCcode;    //CRC校验 8校验
	uint8_t end;        //尾帧
}VisionDataSend_Typedef;
//陀螺仪相关数据均为float，需原始数据*100，转化成int16_t

/* 从视觉组获取的信息 信息一共14个字符 */
typedef struct {
	uint8_t head;              //头帧[0]
	uint8_t detect_signal;     //装甲板数量or是否识别到能量机关装甲板[1]
	uint8_t shoot_msg;         //开火命令 0 or 1[2]
	union yaw_coordinate {     //YAW轴增加量（低位优先发送）[3...4]
		int16_t yaw_predict;
		uint8_t yaw_angle_get[2];
	}yaw_angle;
	
	union pitch_coordinate {   //PITCH轴增加量（低位优先发送）[5...6]
		int16_t pitch_predict;
		uint8_t pitch_angle_get[2];
	}pitch_angle;
	
	union x_predict {          //预测坐标x （低位优先发送）[7...8]
		int16_t x;
		uint8_t x_get[2];
	}x_data;
	
	union y_predict {          //预测坐标y （低位优先发送）[9...10]
		int16_t y;
		uint8_t y_get[2];
	}y_data;
	
	union depth_predict {      //深度（低位优先发送）[11...12]
		int16_t depth;
		uint8_t depth_get[2];
	}depth_data;
	
	uint8_t CRCcode;           //CRC校验 8[13]
	uint8_t end;               //尾帧 [14]
	
}VisionDataGet_Typedef;






extern VisionDataSend_Typedef visionDataSend;					    //发送给视觉组的数据
extern VisionDataGet_Typedef  visionDataGet;               //接收来自视觉组的数据
extern uint8_t RXdata[15];                               //感觉没啥用   /***/
void SendVisionData(VisionDataSend_Typedef* RAW_Data);
extern void GetVisionData(VisionDataGet_Typedef* myVisionDataGet ,uint8_t RAW_Data[30]);
void Tidy_send_vision(VisionDataSend_Typedef *visionDataSend);

//***************************************************vision_uart end***************************************//

typedef enum
{
  closedistance,
	middledistance,
	longdistance
}State_distance;
extern State_distance state_distacne;

typedef struct CRringBuffer_t
{
	float ringBuf[BUFFER_SIZE];
	int16_t tailPosition;
	float lineBuf[BUFFER_SIZE-1];
	float errBuf[BUFFER_SIZE-2];
	float err_Average;
	float predict_Val;
	int16_t lost_COUNT;
	int16_t out_Point;
}CRringBuffer_t;

extern uint8_t ChariotRecognition_data[2][ChariotRecognition_data_dma_buf_len];
extern uint32_t usart1_this_time_rx_len;
extern float last_ChariotRecognition_pitch;
extern float ChariotRecognition_pitch;
extern float last_ChariotRecognition_yaw;
extern float ChariotRecognition_yaw;

extern uint16_t last_Target_Distance;
extern uint16_t Target_Distance;
extern uint16_t Distance_Buf[Dis_buf_Size];
extern uint8_t Dis_Buf_Index;
extern uint8_t Pitch_Add_Angle;
extern CRringBuffer_t CR_ringBuffer;
extern int  GM_Rotete_flag_Before;
extern int  GM_Rotete_flag_Behind;
extern int camera_send_flag;

extern float CR_yaw_Angle[20];
extern uint8_t CR_yaw_Angle_Index;
extern uint8_t CR_yaw_Angle_CNT;
extern int Last_CameraDetectTarget_Flag;
extern int CameraDetectTarget_Flag;
extern float yaw_add_little;
extern float pitch_add_little;
extern int count_Sendtosight;

void Sendtosightway(int value);
//void usart1_Init(u32 bound);
//void ChariotRecognition_Mes_Process(uint8_t *p);

extern void shoot_speed_sending_culculate(void);
extern void SendVisionData_Init(void);
extern void MyUART_Init(void);
extern void SendVisionData(VisionDataSend_Typedef* RAW_Data);

extern void Tidy_send_vision(VisionDataSend_Typedef *visionDataSend);

extern uint8_t get_shoot_msg(VisionDataGet_Typedef *visionDataGet);
#endif
