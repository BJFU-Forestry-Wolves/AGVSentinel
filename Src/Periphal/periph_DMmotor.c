#include "periph_DMmotor.h"
#include "stm32f4xx_hal.h"

// 本地定义 MIT KP/KD 映射范围（与 Omni 示例保持一致）
#define KP_MIN 0.0f
#define KP_MAX 500.0f
#define KD_MIN 0.0f
#define KD_MAX 5.0f

DMmotor_t motor[num];

static void canx_send_data(CAN_HandleTypeDef* hcan, uint16_t stdid, uint8_t data[8], uint8_t len) {
    if (hcan == NULL) return;
    if (len != 8) return;

    CAN_TxHeaderTypeDef tx;
    tx.StdId = stdid;
    tx.ExtId = 0;
    tx.IDE = CAN_ID_STD;
    tx.RTR = CAN_RTR_DATA;
    tx.DLC = 8;
    tx.TransmitGlobalTime = DISABLE;

    uint32_t mailbox;
    (void)HAL_CAN_AddTxMessage(hcan, &tx, data, &mailbox);
}

int float_to_uint(float x_float, float x_min, float x_max, int bits) {
    float span = x_max - x_min;
    float offset = x_min;
    if (x_float > x_max) x_float = x_max;
    if (x_float < x_min) x_float = x_min;
    return (int)((x_float - offset) * ((float)((1 << bits) - 1)) / span);
}

float uint_to_float(int x_int, float x_min, float x_max, int bits) {
    float span = x_max - x_min;
    float offset = x_min;
    return ((float)x_int) * span / ((float)((1 << bits) - 1)) + offset;
}

void dm_motor_init(void) {
    motor[Motor1].id = 0x01;
    motor[Motor1].mst_id = 0x00;
    motor[Motor1].tmp.read_flag = 1;
    motor[Motor1].ctrl.mode = mit_mode;
    motor[Motor1].ctrl.pos_set = 0.32f;
    motor[Motor1].ctrl.vel_set = 6.5f;
    motor[Motor1].ctrl.kp_set = 120.0f;
    motor[Motor1].ctrl.kd_set = 3.0f;
    motor[Motor1].ctrl.tor_set = 3.0f;
    motor[Motor1].ctrl.cur_set = 0.0f;
    motor[Motor1].tmp.PMAX = 12.5f;
    motor[Motor1].tmp.VMAX = 30.0f;
    motor[Motor1].tmp.TMAX = 10.0f;
}

void dm_motor_clear_para(DMmotor_t *motor) {
    motor->ctrl.kd_set = 0;
    motor->ctrl.kp_set = 0;
    motor->ctrl.pos_set = 0;
    motor->ctrl.vel_set = 0;
    motor->ctrl.tor_set = 0;
    motor->ctrl.cur_set = 0;
}

void enable_motor_mode(CAN_HandleTypeDef* hcan, uint16_t motor_id, uint16_t mode_id) {
    uint8_t data[8];
    uint16_t id = motor_id + mode_id;
    data[0]=0xFF; data[1]=0xFF; data[2]=0xFF; data[3]=0xFF;
    data[4]=0xFF; data[5]=0xFF; data[6]=0xFF; data[7]=0xFC;
    canx_send_data(hcan, id, data, 8);
}

void disable_motor_mode(CAN_HandleTypeDef* hcan, uint16_t motor_id, uint16_t mode_id) {
    uint8_t data[8];
    uint16_t id = motor_id + mode_id;
    data[0]=0xFF; data[1]=0xFF; data[2]=0xFF; data[3]=0xFF;
    data[4]=0xFF; data[5]=0xFF; data[6]=0xFF; data[7]=0xFD;
    canx_send_data(hcan, id, data, 8);
}

void dm_motor_enable(CAN_HandleTypeDef* hcan, DMmotor_t *motor) {
    switch(motor->ctrl.mode) {
        case mit_mode: enable_motor_mode(hcan, motor->id, MIT_MODE); break;
        case pos_mode: enable_motor_mode(hcan, motor->id, POS_MODE); break;
        case spd_mode: enable_motor_mode(hcan, motor->id, SPD_MODE); break;
        case psi_mode: enable_motor_mode(hcan, motor->id, PSI_MODE); break;
        default: break;
    }
}

void dm_motor_disable(CAN_HandleTypeDef* hcan, DMmotor_t *motor) {
    switch(motor->ctrl.mode) {
        case mit_mode: disable_motor_mode(hcan, motor->id, MIT_MODE); break;
        case pos_mode: disable_motor_mode(hcan, motor->id, POS_MODE); break;
        case spd_mode: disable_motor_mode(hcan, motor->id, SPD_MODE); break;
        case psi_mode: disable_motor_mode(hcan, motor->id, PSI_MODE); break;
        default: break;
    }
    dm_motor_clear_para(motor);
}

void mit_ctrl(CAN_HandleTypeDef* hcan, DMmotor_t *motor, uint16_t motor_id, float pos, float vel, float kp, float kd, float tor) {
    uint8_t data[8];
    uint16_t pos_tmp, vel_tmp, kp_tmp, kd_tmp, tor_tmp;
    uint16_t id = motor_id + MIT_MODE;

    pos_tmp = float_to_uint(pos, -motor->tmp.PMAX, motor->tmp.PMAX, 16);
    vel_tmp = float_to_uint(vel, -motor->tmp.VMAX, motor->tmp.VMAX, 12);
    tor_tmp = float_to_uint(tor, -motor->tmp.TMAX, motor->tmp.TMAX, 12);
    kp_tmp  = float_to_uint(kp,  KP_MIN, KP_MAX, 12);
    kd_tmp  = float_to_uint(kd,  KD_MIN, KD_MAX, 12);

    data[0] = (pos_tmp >> 8);
    data[1] = pos_tmp;
    data[2] = (vel_tmp >> 4);
    data[3] = ((vel_tmp & 0xF) << 4) | (kp_tmp >> 8);
    data[4] = kp_tmp;
    data[5] = (kd_tmp >> 4);
    data[6] = ((kd_tmp & 0xF) << 4) | (tor_tmp >> 8);
    data[7] = tor_tmp;
    canx_send_data(hcan, id, data, 8);
}

void dm_motor_ctrl_send(CAN_HandleTypeDef* hcan, DMmotor_t *motor) {
    switch(motor->ctrl.mode) {
        case mit_mode:
            mit_ctrl(hcan, motor, motor->id, motor->ctrl.pos_set, motor->ctrl.vel_set, motor->ctrl.kp_set, motor->ctrl.kd_set, motor->ctrl.tor_set);
            break;
        default:
            break;
    }
}

void dm_motor_fbdata(DMmotor_t *motor, uint8_t *rx_data) {
    motor->para.id = (rx_data[0]) & 0x0F;
    motor->para.state = (rx_data[0]) >> 4;
    motor->para.p_int = (rx_data[1] << 8) | rx_data[2];
    motor->para.v_int = (rx_data[3] << 4) | (rx_data[4] >> 4);
    motor->para.t_int = ((rx_data[4] & 0xF) << 8) | rx_data[5];
    motor->para.pos = uint_to_float(motor->para.p_int, -motor->tmp.PMAX, motor->tmp.PMAX, 16);
    motor->para.vel = uint_to_float(motor->para.v_int, -motor->tmp.VMAX, motor->tmp.VMAX, 12);
    motor->para.tor = uint_to_float(motor->para.t_int, -motor->tmp.TMAX, motor->tmp.TMAX, 12);
    motor->para.Tmos = (float)(rx_data[6]);
    motor->para.Tcoil = (float)(rx_data[7]);
}

void dm_motor_detect(DMmotor_t *motor)
{
	if(motor->para.state==1)
		return;
	else
	{
	  dm_motor_enable(&hcan1, &motor[Motor1]);	
	}
}