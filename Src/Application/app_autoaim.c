#include "app_autoaim.h"

//--------------------------------------------------------------------------------------------------------
//																						外部变量和函数
extern UART_HandleTypeDef huart1;
extern DMA_HandleTypeDef hdma_usart1_rx;
extern DMA_HandleTypeDef hdma_usart1_tx;

//extern shoot_control_t shoot_control;
//extern gimbal_control_t gimbal_control;
extern INS_INSTypeDef INS;
extern fp32 INS_angle[3];
extern fp32 INS_accel[3];
extern bool_t gimbal_cmd_to_chassis_stop(void);
extern int shoot_speed_add_little;
//--------------------------------------------------------------------------------------------------------



//--------------------------------------------------------------------------------------------------------
//																						变量定义
__align(8) uint8_t USART1_TX_BUF[USART1_MAX_SEND_LEN]; 	 //发送缓冲,最大USART1_MAX_SEND_LEN(400)字节
uint8_t USART1_RX_BUF[USART1_MAX_RECV_LEN];      //接收缓冲,最大USART_REC_LEN(400)个字节.
uint16_t USART1_RX_STA=0;						 //接收状态标记	

uint32_t usart1_this_time_rx_len = 0;              //USART1收到的数据个数
uint8_t ChariqotRecognition_data[2][ChariotRecognition_data_dma_buf_len];   //DMA接收妙算数据的双缓存数组
int32_t ChariotRecognitionTemp[2] = {0,0};      //解析角度内容的数组
int16_t ChariotRecognitionDirection[2];  //接受摄像头传来的大装甲数据
int16_t Chariot_Rec_Dir_rotate[2];       //接受摄像头传来的大装甲和小装甲数据
int CameraDetectTarget_Flag = 0;     //摄像头发现目标标志

int TempShootingFlag=0;      //发弹标志,修改该标志，可选择是否发弹

float last_ChariotRecognition_pitch = 0.0f;        //上一个pitch角度值
float ChariotRecognition_pitch = 0.0f;             //pitch角度值
float last_ChariotRecognition_yaw = 0.0f;          //上一个yaw轴角度值
float ChariotRecognition_yaw = 0.0f;               //yaw角度值

float YawCurrentPositionSave   = 0.0f;     //保存当前Yaw轴位置
float PitchCurrentPositionSave = 0.0f;     //保存当前Pitch轴位置

uint16_t last_Target_Distance = 0;         //上次摄像头与目标的距离
uint16_t Target_Distance = 150;            //摄像头与目标的距离

uint16_t Distance_buf[10];     //距离缓冲区
uint8_t Dis_Buf_Index = 0;
uint8_t Pitch_Add_Angle = 0;
uint8_t enter_CNT = 0;
int Armor_R_Flag_Before=0;
int Armor_R_Flag_Behind=0;

int GM_Rotete_flag_Before=0;        //前固定摄像头识别目标
int GM_Rotete_flag_Behind=0;        //后固定摄像头识别目标
int Time_count=0;

CRringBuffer_t CR_ringBuffer;

float CR_yaw_Angle[20];
uint8_t CR_yaw_Angle_Index = 0;
uint8_t CR_yaw_Angle_CNT   = 0;
int8_t loop_j;

char Sendtosight[Sendtosight_len];            //发送给视觉

int friction_wheel_count = 0;
float kalman_yaw = 0;
float kalman_pitch = 0;
float kalman_yaw_feedforward = 0;
uint8_t update_flag = 1;
int Last_CameraDetectTarget_Flag=0;
float E_TEST=0;
float E_TEST1=0;
float E_TEST2=0;
float E_TEST3=0;
int camera_send_flag = 0;

int autoaim_mode_flag
	= 0;   //自瞄状态开启/关闭标志
//--------------------------------------------------------------------------------------------------------




//--------------------------------------------------------------------------------------------------------
//																						串口处理器函数
//void USART1_IRQHandler(void)
//{  
//	 if(huart1.Instance->SR & UART_FLAG_RXNE)//接收到数据
//    {
//        __HAL_UART_CLEAR_PEFLAG(&huart1);
//    }

//    else if(USART1->SR & UART_FLAG_IDLE)
//    {
//        static uint8_t this_time_rx_len = 0;
//			
//        __HAL_UART_CLEAR_PEFLAG(&huart1);

//        if ((hdma_usart1_rx.Instance->CR & DMA_SxCR_CT) == RESET)
//        {
//            /* Current memory buffer used is Memory 0 */

//            //disable DMA
//            //失效DMA
//            __HAL_DMA_DISABLE(&hdma_usart1_rx);

//            //get receive data length, length = set_data_length - remain_length
//            //获取接收数据长度,长度 = 设定长度 - 剩余长度
//            this_time_rx_len = ChariotRecognition_data_dma_buf_len - hdma_usart1_rx.Instance->NDTR;//NDTR=15 buf_len =27,rxlen=12 然后就不接收了 应该把buf改成30


//            //reset set_data_lenght
//            //重新设定数据长度
//            hdma_usart1_rx.Instance->NDTR = (uint16_t)ChariotRecognition_data_dma_buf_len;

//            //set memory buffer 1
//            //设定缓冲区1
//            hdma_usart1_rx.Instance->CR |= DMA_SxCR_CT;
//            
//            //enable DMA
//            //使能DMA
//            __HAL_DMA_ENABLE(&hdma_usart1_rx);
//	   
//            if(this_time_rx_len == ChariotRecognition_data_len )
//            {
//							GetVisionData(&visionDataGet ,ChariqotRecognition_data[0]);
//            }
//        }
//        else
//        {
//            /* Current memory buffer used is Memory 1 */
//            //disable DMA
//            //失效DMA
//            __HAL_DMA_DISABLE(&hdma_usart1_rx);

//            //get receive data length, length = set_data_length - remain_length
//            //获取接收数据长度,长度 = 设定长度 - 剩余长度
//            this_time_rx_len = ChariotRecognition_data_dma_buf_len - hdma_usart1_rx.Instance->NDTR;//NDTR=21 

//            //reset set_data_lenght
//            //重新设定数据长度
//           hdma_usart1_rx.Instance->NDTR = (uint16_t)ChariotRecognition_data_dma_buf_len;

//            //set memory buffer 0
//            //设定缓冲区0
//            hdma_usart1_rx.Instance->CR &= ~(DMA_SxCR_CT);
//            
//            //enable DMA
//            //使能DMA
//            __HAL_DMA_ENABLE(&hdma_usart1_rx);

//            if(this_time_rx_len == ChariotRecognition_data_len)
//            {
//							GetVisionData(&visionDataGet ,ChariqotRecognition_data[0]);
//            }
//        }
//    }

//}

//--------------------------------------------------------------------------------------------------------



//--------------------------------------------------------------------------------------------------------
//																							机器人ID，机器人等级，射速......
State_distance state_distacne=closedistance;

extern uint8_t get_robot_id(void);
extern uint8_t get_robot_level(void);

int count_Sendtosight = 0;
//extern int windmill_mode;
float yaw_add_little = 0;
float pitch_add_little = 0;

int shoot_num = 0;
float real_shoot_speed_save[5] = {1.36,1.36,1.36,1.36,1.36};
float remaining_shoot_num = 0;
float last_remaining_shoot_num = 0;
float average_shoot_speed = 1.36;
float last_shoot_speed ;
//--------------------------------------------------------------------------------------------------------

//--------------------------------------------------------------------------------------------------------
//             求5次发弹的平均值，实时更新,暂时没有写好，完善图传链路时修改
void shoot_speed_sending_culculate(void)
{
//	remaining_shoot_num = get_bullet_remaining_num();
  remaining_shoot_num = 1.36f;
	if(last_remaining_shoot_num - remaining_shoot_num)
	{
		shoot_num++;
	}
	
	if(shoot_num)
	{
//		real_shoot_speed_save[shoot_num%5] = get_bullet_speed();
		real_shoot_speed_save[shoot_num%5] = 1.36f;
	}
	
	average_shoot_speed = (real_shoot_speed_save[0] + real_shoot_speed_save[1] + real_shoot_speed_save[2] + real_shoot_speed_save[3] + real_shoot_speed_save[4]) / 5;	
}
//--------------------------------------------------------------------------------------------------------





VisionDataSend_Typedef visionDataSend;					    //发送给视觉组的数据
VisionDataGet_Typedef  visionDataGet;               //接收来自视觉组的数据
uint8_t RXdata[15];                                 //接收视觉组的数组
//             发送数据初始化
void SendVisionData_Init(void)
{
	visionDataSend.head 					         = 0x53;		   //帧头
	visionDataSend.color 					         = 0x02;		   //初始颜色 0为己方红色 1为己方蓝色 2为己方红蓝双色（调试模式）
	visionDataSend.mode 					         = 0x01;		   //模式 0为自瞄 1为小能量机关 2为大能量机关 3为打击哨兵模式 4为小陀螺模式 5为录像模式 6为无人机模式 7为哨兵模式 8为雷达模式 其余情况默认为自瞄模式
	visionDataSend.robotID 					       = 0x03;		   //当前机器人ID 0为英雄 1为工程 2为步兵 3为无人机 4为哨兵
	visionDataSend.yaw_angle.yaw_gyro   	 = 5 ;			   //yaw轴陀螺仪角度   初始值
	visionDataSend.pitch_angle.pitch_gyro	 = 6.0;			   //pitch轴陀螺仪角度 初始值
	visionDataSend.yaw_acc_data.yaw_acc 	 = 0;					 //yaw轴速度				  初始值
	visionDataSend.pitch_acc_data.pitch_acc= 0;					 //pitch轴速度		    初始值
	visionDataSend.shoot_speed             = 0;		       //弹丸发射速度 			初始值
	visionDataSend.CRCcode                 = 0;          //CRC校验
	visionDataSend.end 						         = 0x45;		   //帧尾（某不愿意透露姓名的Q姓队长规定）
}
//             收发数据的串口初始化
void MyUART_Init(void)
{
	SendVisionData_Init();
}
//                向视觉发送数据 
void SendVisionData(VisionDataSend_Typedef* RAW_Data)		//调用该函数发送视觉信息给视觉组
{
	uint8_t data_buffer[14];
	static uint8_t delay_num;
	delay_num++;
	while(delay_num ==10)
	{
	data_buffer[0]  = RAW_Data->head;												//帧头
	data_buffer[1]  = RAW_Data->color;								   	  //颜色 0为己方红色 1为己方蓝色 2为己方红蓝双色（调试模式）
	data_buffer[2]  = RAW_Data->mode;												//模式 0为自瞄 1为小能量机关 2为大能量机关 3为打击哨兵模式 4为小陀螺模式 5为录像模式 6为无人机模式 7为哨兵模式 8为雷达模式 其余情况默认为自瞄模式
	data_buffer[3]  = RAW_Data->robotID;										//当前机器人ID 0为英雄 1为工程 2为步兵 3为无人机 4为哨兵
  data_buffer[4]  = RAW_Data->yaw_angle.yaw_angle_send[0];
	data_buffer[5]  = RAW_Data->yaw_angle.yaw_angle_send[1];
	data_buffer[6]  = RAW_Data->pitch_angle.pitch_angle_send[0];
	data_buffer[7]  = RAW_Data->pitch_angle.pitch_angle_send[1];
	data_buffer[8]  = RAW_Data->yaw_acc_data.yaw_acc_send[0];
	data_buffer[9]  = RAW_Data->yaw_acc_data.yaw_acc_send[1];
	data_buffer[10] = RAW_Data->pitch_acc_data.pitch_acc_send[0];
	data_buffer[11] = RAW_Data->pitch_acc_data.pitch_acc_send[1];
	data_buffer[12] = RAW_Data->shoot_speed;
	data_buffer[13] = RAW_Data->end;								//帧尾
	HAL_UART_Transmit(&huart1, data_buffer, 15, 0xff);//通过DMA一次性发送数据														//调用API发送
	delay_num =0;
	}

#ifdef UART_DEBUG_MODE
	printf("\n----------SendVisionData Start-------\n\n");
	printf("head:%x  end: %x \n", RAW_Data->head, RAW_Data->end);
	printf("color:%d  mode: %d ID: %d\n", RAW_Data->color, RAW_Data->mode, RAW_Data->MyID);
	printf("yaw_Angle:%f  pitch_Angle: %f \n", RAW_Data->yaw_Angle, RAW_Data->pitch_Angle);
	printf("yaw_velocity:%d  pitch_velocity: %d \n", RAW_Data->yaw_velocity, RAW_Data->pitch_velocity);
	printf("bullet_velocity:%d  \n", RAW_Data->bullet_velocity);
	printf("\n---------SendVisionData Finish-------\n");
#endif
}
//    处理从视觉接收的数据
//    myVisionDataGet    和   RAW_Data
void GetVisionData(VisionDataGet_Typedef* myVisionDataGet ,uint8_t RAW_Data[30])			//调用该函数获取视觉信息，视觉信息会传到 形参结构体 里
{
	myVisionDataGet->head = RAW_Data[0];
	myVisionDataGet->detect_signal = RAW_Data[1];
	myVisionDataGet->shoot_msg = RAW_Data[2];
	myVisionDataGet->yaw_angle.yaw_angle_get[0] = RAW_Data[3];
	myVisionDataGet->yaw_angle.yaw_angle_get[1] = RAW_Data[4];
  myVisionDataGet->pitch_angle.pitch_angle_get[0] = RAW_Data[5];
	myVisionDataGet->pitch_angle.pitch_angle_get[1] = RAW_Data[6];
  myVisionDataGet->x_data.x_get[0] = RAW_Data[7];
	myVisionDataGet->x_data.x_get[1] = RAW_Data[8];
	myVisionDataGet->y_data.y_get[0] = RAW_Data[9];
	myVisionDataGet->y_data.y_get[1] = RAW_Data[10];
	myVisionDataGet->depth_data.depth_get[0] = RAW_Data[11];
	myVisionDataGet->depth_data.depth_get[1] = RAW_Data[12];
	myVisionDataGet->CRCcode = RAW_Data[13];
	myVisionDataGet->end = RAW_Data[14];
	
	if(RAW_Data[0] == 0)  myVisionDataGet->head = 0;
 if(RAW_Data[1] == 0)  myVisionDataGet->detect_signal = 0;
 if(RAW_Data[2] == 0)  myVisionDataGet->shoot_msg =0;
 if(RAW_Data[3] == 0)  myVisionDataGet->yaw_angle.yaw_angle_get[0] = 0;
 if(RAW_Data[4] == 0)  myVisionDataGet->yaw_angle.yaw_angle_get[1] = 0;
 if(RAW_Data[5] == 0)  myVisionDataGet->pitch_angle.pitch_angle_get[0] = 0;
 if(RAW_Data[6] == 0)  myVisionDataGet->pitch_angle.pitch_angle_get[1] = 0;
 if(RAW_Data[7] == 0)  myVisionDataGet->x_data.x_get[0] = 0;
 if(RAW_Data[8] == 0)  myVisionDataGet->x_data.x_get[1] = 0;
 if(RAW_Data[9] == 0)  myVisionDataGet->y_data.y_get[0] = 0;
 if(RAW_Data[10] == 0)  myVisionDataGet->y_data.y_get[1] = 0;
 if(RAW_Data[11] == 0)  myVisionDataGet->depth_data.depth_get[0] = 0;
 if(RAW_Data[12] == 0)  myVisionDataGet->depth_data.depth_get[1] = 0;
 if(RAW_Data[13] == 0)  myVisionDataGet->CRCcode = 0;
 if(RAW_Data[14] == 0)  myVisionDataGet->end = 0;

#ifdef UART_DEBUG_MODE
	printf("\n------------GetVisionData-----------\n\n");
	printf("head:%x  end: %x \n", myVisionDataGet->head, myVisionDataGet->end);
	printf("VisionFlag:%x  VisionShoot: %x \n", myVisionDataGet->VisionFlag, myVisionDataGet->VisionShoot);
	printf("yaw:%d  pitch: %d \n", myVisionDataGet->RES_Yaw, myVisionDataGet->RES_Pitch);
	printf("depth:%x  QHH_CRC: %x \n", myVisionDataGet->depth, myVisionDataGet->QHH_CRC);
	printf("\n---------GetVisionData Finish-------\n");
#endif
}

void Tidy_send_vision(VisionDataSend_Typedef *visionDataSend)
{
	 Referee_RefereeDataTypeDef *referee = Referee_GetRefereeDataPtr();
	//visionDataSend.color 					         = 0x02;		//初始颜色 0为己方红色 1为己方蓝色 2为己方红蓝双色（调试模式）
	//visionDataSend.mode 					         = 0x01;		//模式 0为自瞄 1为小能量机关 2为大能量机关 3为打击哨兵模式 4为小陀螺模式 5为录像模式 6为无人机模式 7为哨兵模式 8为雷达模式 其余情况默认为自瞄模式
	visionDataSend->yaw_angle.yaw_gyro      = (int16_t)(INS.Yaw *100);
	visionDataSend->pitch_angle.pitch_gyro  = (int16_t)(INS.Roll* 100);
	visionDataSend->yaw_acc_data.yaw_acc 	  = (int16_t)(INS.Accel[0] * 100);
	visionDataSend->pitch_acc_data.pitch_acc= (int16_t)(INS.Accel[2] * 100);
	
	if(referee->bullet_speed*10>255||referee->bullet_speed<0||referee->bullet_speed==0)
	last_shoot_speed = 0;
	else if(referee->bullet_speed*10!=0||referee->bullet_speed*10<255)
	last_shoot_speed = referee->bullet_speed;
	visionDataSend->shoot_speed              =(uint8_t)(last_shoot_speed*10.0);
	
	visionDataSend->robotID                  =(uint8_t)referee->robot_id;
	
//  visionDataSend.CRCcode                 = 0;       //CRC校验;
}




//是否识别到目标的滑动窗口滤波函数
uint8_t get_shoot_msg(VisionDataGet_Typedef *visionDataGet)
{
	uint8_t shoot_msg = 1;//默认识别到目标
	static uint8_t shoot_msg_buf[SHOOT_MSG_BUF_SIZE] = {0};//定义静态缓存数组：存储最近SHOOT_MSG_BUF_SIZE帧的shoot_msg值
	shoot_msg_buf[0] = visionDataGet->shoot_msg;//最新帧数据入队：把当前视觉的shoot_msg存入缓存数组第0位
	for(uint8_t shoot_msg_buf_num = 1;shoot_msg_buf_num < SHOOT_MSG_BUF_SIZE; shoot_msg_buf_num++)
	{
		shoot_msg_buf[shoot_msg_buf_num] = shoot_msg_buf[shoot_msg_buf_num - 1];//第n位 = 第n-1位的值
	}
	for(uint8_t shoot_msg_buf_num = 0;shoot_msg_buf_num < SHOOT_MSG_BUF_SIZE; shoot_msg_buf_num++)
	{
		shoot_msg = shoot_msg * shoot_msg_buf[shoot_msg_buf_num];//所有5个缓存值“相乘”，判断是否连续识别到目标
	}
	return shoot_msg;//返回滤波后的结果：1=连续识别到目标，0=未连续识别,只有所有buf值都是1，结果才是1；只要有一个0，结果就为0
}
