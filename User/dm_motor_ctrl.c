#include "dm_motor_drv.h"
#include "dm_motor_ctrl.h"
#include "string.h"
#include "stdbool.h"

motor_t L_motor[E_JOINT_NUM];
motor_t R_motor[E_JOINT_NUM];

/**
************************************************************************
* @brief:      	dm4310_motor_init: DM4310�����?������
* @param:      	void
* @retval:     	void
* @details:    	��?��1��DM4310�???��������?�?����?���??��
*               ����ID������??������??����?��
************************************************************************
**/
void dm_motor_init(void)
{
    // ��?��Motor1��Motor2�?���?

    // PID

    memset(&L_motor[E_ROLL], 0, sizeof(L_motor[E_ROLL]));
    memset(&L_motor[E_PITCH], 0, sizeof(L_motor[E_PITCH]));
    memset(&L_motor[E_KNEE], 0, sizeof(L_motor[E_KNEE]));
    memset(&L_motor[E_WHEEL], 0, sizeof(L_motor[E_WHEEL]));

    memset(&R_motor[E_ROLL], 0, sizeof(R_motor[E_ROLL]));
    memset(&R_motor[E_PITCH], 0, sizeof(R_motor[E_PITCH]));
    memset(&R_motor[E_KNEE], 0, sizeof(R_motor[E_KNEE]));
    memset(&R_motor[E_WHEEL], 0, sizeof(R_motor[E_WHEEL]));
    dm_motor_clear_para(R_motor);
    dm_motor_clear_para(L_motor);
    L_motor[E_ROLL].ctrl.kp_set = 30.0f;
    L_motor[E_ROLL].ctrl.kd_set = 0.0f;
    L_motor[E_PITCH].ctrl.kp_set = 5.0f;
    L_motor[E_PITCH].ctrl.kd_set = 0.0f;
    L_motor[E_KNEE].ctrl.kp_set = 10.0f;
    L_motor[E_KNEE].ctrl.kd_set = 0.0f;

    R_motor[E_ROLL].ctrl.kp_set = 30.0f;
    R_motor[E_ROLL].ctrl.kd_set = 0.0f;
    R_motor[E_PITCH].ctrl.kp_set = 5.0f;
    R_motor[E_PITCH].ctrl.kd_set = 0.0f;
    R_motor[E_KNEE].ctrl.kp_set = 10.0f;
    R_motor[E_KNEE].ctrl.kd_set = 0.0f;

    // L_motor[E_ROLL].ctrl.kp_set = 0;
    // L_motor[E_ROLL].ctrl.kd_set = 0;
    // L_motor[E_PITCH].ctrl.kp_set = 0;
    // L_motor[E_PITCH].ctrl.kd_set = 0;
    // L_motor[E_KNEE].ctrl.kp_set = 0;
    // L_motor[E_KNEE].ctrl.kd_set = 0;
    // R_motor[E_ROLL].ctrl.kp_set = 0;
    // R_motor[E_ROLL].ctrl.kd_set = 0;
    // R_motor[E_PITCH].ctrl.kp_set = 0;
    // R_motor[E_PITCH].ctrl.kd_set = 0;
    // R_motor[E_KNEE].ctrl.kp_set = 0;
    // R_motor[E_KNEE].ctrl.kd_set = 0;

    L_motor[E_ROLL].hdcan = &hfdcan2;
    L_motor[E_PITCH].hdcan = &hfdcan2;
    L_motor[E_KNEE].hdcan = &hfdcan2;
    L_motor[E_WHEEL].hdcan = &hfdcan2;

    R_motor[E_ROLL].hdcan = &hfdcan1;
    R_motor[E_PITCH].hdcan = &hfdcan1;
    R_motor[E_KNEE].hdcan = &hfdcan1;
    R_motor[E_WHEEL].hdcan = &hfdcan1;

    L_motor[E_ROLL].id = 0x05;
    L_motor[E_ROLL].mst_id = 0x00;
    L_motor[E_ROLL].tmp.read_flag = 1;
    L_motor[E_ROLL].ctrl.mode = mit_mode;
    L_motor[E_ROLL].ctrl.vel_set = 0;
    L_motor[E_ROLL].ctrl.pos_set = 0;
    L_motor[E_ROLL].ctrl.tor_set = 0;
    L_motor[E_ROLL].ctrl.cur_set = 0;
    L_motor[E_ROLL].tmp.PMAX = 12.5f;
    L_motor[E_ROLL].tmp.VMAX = 30.0f;
    L_motor[E_ROLL].tmp.TMAX = 10.0f;

    R_motor[E_ROLL].id = 0x05;
    R_motor[E_ROLL].mst_id = 0x00; // ?��������?�?����?R���
    R_motor[E_ROLL].tmp.read_flag = 1;
    R_motor[E_ROLL].ctrl.mode = mit_mode;
    R_motor[E_ROLL].ctrl.vel_set = 0.0f;
    R_motor[E_ROLL].ctrl.pos_set = 0.0f;
    R_motor[E_ROLL].ctrl.tor_set = 0.0f;
    R_motor[E_ROLL].ctrl.cur_set = 0;
    R_motor[E_ROLL].tmp.PMAX = 12.5f;
    R_motor[E_ROLL].tmp.VMAX = 30.0f;
    R_motor[E_ROLL].tmp.TMAX = 10.0f;

    L_motor[E_PITCH].id = 0x01;
    L_motor[E_PITCH].mst_id = 0x00; // ?��������?�?����?L_motor
    L_motor[E_PITCH].tmp.read_flag = 1;
    L_motor[E_PITCH].ctrl.mode = mit_mode;
    L_motor[E_PITCH].ctrl.vel_set = 0.0f;
    L_motor[E_PITCH].ctrl.pos_set = 0.0f;
    L_motor[E_PITCH].ctrl.tor_set = 0.0f;
    L_motor[E_PITCH].ctrl.cur_set = 0;
    L_motor[E_PITCH].tmp.PMAX = 12.5f;
    L_motor[E_PITCH].tmp.VMAX = 30.0f;
    L_motor[E_PITCH].tmp.TMAX = 10.0f;

    R_motor[E_PITCH].id = 0x01;
    R_motor[E_PITCH].mst_id = 0x00; // ?��������?�?����?R_motor
    R_motor[E_PITCH].tmp.read_flag = 1;
    R_motor[E_PITCH].ctrl.mode = mit_mode;
    R_motor[E_PITCH].ctrl.vel_set = 0.0f;
    R_motor[E_PITCH].ctrl.pos_set = 0.0f;
    R_motor[E_PITCH].ctrl.tor_set = 0.0f;
    R_motor[E_PITCH].ctrl.cur_set = 0;
    R_motor[E_PITCH].tmp.PMAX = 12.5f;
    R_motor[E_PITCH].tmp.VMAX = 30.0f;
    R_motor[E_PITCH].tmp.TMAX = 10.0f;

    L_motor[E_KNEE].id = 0x02;
    L_motor[E_KNEE].mst_id = 0x00; // ?��������?�?����?����
    L_motor[E_KNEE].tmp.read_flag = 1;
    L_motor[E_KNEE].ctrl.mode = mit_mode;
    L_motor[E_KNEE].ctrl.vel_set = 0.0f;
    L_motor[E_KNEE].ctrl.pos_set = 0.0f;
    L_motor[E_KNEE].ctrl.tor_set = 0.0f;
    L_motor[E_KNEE].ctrl.cur_set = 0;
    L_motor[E_KNEE].tmp.PMAX = 12.5f;
    L_motor[E_KNEE].tmp.VMAX = 30.0f;
    L_motor[E_KNEE].tmp.TMAX = 10.0f;

    R_motor[E_KNEE].id = 0x02;
    R_motor[E_KNEE].mst_id = 0x00; // ?��������?�?����?R���
    R_motor[E_KNEE].tmp.read_flag = 1;
    R_motor[E_KNEE].ctrl.mode = mit_mode;
    R_motor[E_KNEE].ctrl.vel_set = 0.0f;
    R_motor[E_KNEE].ctrl.pos_set = 0.0f;
    R_motor[E_KNEE].ctrl.tor_set = 0.0f;
    R_motor[E_KNEE].ctrl.cur_set = 0;
    R_motor[E_KNEE].tmp.PMAX = 12.5f;
    R_motor[E_KNEE].tmp.VMAX = 30.0f;
    R_motor[E_KNEE].tmp.TMAX = 10.0f;

    L_motor[E_WHEEL].id = 0x03;
    L_motor[E_WHEEL].mst_id = 0x00; // ?��������?�?����?L_motor
    L_motor[E_WHEEL].tmp.read_flag = 1;
    L_motor[E_WHEEL].ctrl.mode = mit_mode;
    L_motor[E_WHEEL].ctrl.vel_set = 0.0f;
    L_motor[E_WHEEL].ctrl.pos_set = 0.0f;
    L_motor[E_WHEEL].ctrl.tor_set = 0.0f;
    L_motor[E_WHEEL].ctrl.cur_set = 0;
    L_motor[E_WHEEL].tmp.PMAX = 12.5f;
    L_motor[E_WHEEL].tmp.VMAX = 30.0f;
    L_motor[E_WHEEL].tmp.TMAX = 10.0f;

    R_motor[E_WHEEL].id = 0x03;
    R_motor[E_WHEEL].mst_id = 0x00; // ?��������?�?����?R_motor
    R_motor[E_WHEEL].tmp.read_flag = 1;
    R_motor[E_WHEEL].ctrl.mode = mit_mode;
    R_motor[E_WHEEL].ctrl.vel_set = 0.0f;
    R_motor[E_WHEEL].ctrl.pos_set = 0.0f;
    R_motor[E_WHEEL].ctrl.tor_set = 0.0f;
    R_motor[E_WHEEL].ctrl.cur_set = 0;
    R_motor[E_WHEEL].tmp.PMAX = 12.5f;
    R_motor[E_WHEEL].tmp.VMAX = 30.0f;
    R_motor[E_WHEEL].tmp.TMAX = 10.0f;
}
/**
************************************************************************
* @brief:      	read_aldata: ��?��������??�����������?
* @param:      	motor_t����������?��
* @retval:     	void
* @details:    	��?��?�?����
************************************************************************
**/
void read_aldata(motor_t *motor)
{
    switch (motor->tmp.read_flag)
    {
    case 1:
        read_motor_data(motor->id, RID_UV_VALUE);
        break; // UV_Value
    case 2:
        read_motor_data(motor->id, RID_KT_VALUE);
        break; // KT_Value
    case 3:
        read_motor_data(motor->id, RID_OT_VALUE);
        break; // OT_Value
    case 4:
        read_motor_data(motor->id, RID_OC_VALUE);
        break; // OC_Value
    case 5:
        read_motor_data(motor->id, RID_ACC);
        break; // ACC
    case 6:
        read_motor_data(motor->id, RID_DEC);
        break; // DEC
    case 7:
        read_motor_data(motor->id, RID_MAX_SPD);
        break; // MAX_SPD
    case 8:
        read_motor_data(motor->id, RID_MST_ID);
        break; // MST_ID
    case 9:
        read_motor_data(motor->id, RID_ESC_ID);
        break; // ESC_ID
    case 10:
        read_motor_data(motor->id, RID_TIMEOUT);
        break; // TIMEOUT
    case 11:
        read_motor_data(motor->id, RID_CMODE);
        break; // CTRL_MODE
    case 12:
        read_motor_data(motor->id, RID_DAMP);
        break; // Damp
    case 13:
        read_motor_data(motor->id, RID_INERTIA);
        break; // Inertia
    case 14:
        read_motor_data(motor->id, RID_HW_VER);
        break; // Rsv1
    case 15:
        read_motor_data(motor->id, RID_SW_VER);
        break; // sw_ver
    case 16:
        read_motor_data(motor->id, RID_SN);
        break; // Rsv2
    case 17:
        read_motor_data(motor->id, RID_NPP);
        break; // NPP
    case 18:
        read_motor_data(motor->id, RID_RS);
        break; // Rs
    case 19:
        read_motor_data(motor->id, RID_LS);
        break; // Ls
    case 20:
        read_motor_data(motor->id, RID_FLUX);
        break; // Flux
    case 21:
        read_motor_data(motor->id, RID_GR);
        break; // Gr
    case 22:
        read_motor_data(motor->id, RID_PMAX);
        break; // PMAX
    case 23:
        read_motor_data(motor->id, RID_VMAX);
        break; // VMAX
    case 24:
        read_motor_data(motor->id, RID_TMAX);
        break; // TMAX
    case 25:
        read_motor_data(motor->id, RID_I_BW);
        break; // I_BW
    case 26:
        read_motor_data(motor->id, RID_KP_ASR);
        break; // KP_ASR
    case 27:
        read_motor_data(motor->id, RID_KI_ASR);
        break; // KI_ASR
    case 28:
        read_motor_data(motor->id, RID_KP_APR);
        break; // KP_APR
    case 29:
        read_motor_data(motor->id, RID_KI_APR);
        break; // KI_APR
    case 30:
        read_motor_data(motor->id, RID_OV_VALUE);
        break; // OV_Value
    case 31:
        read_motor_data(motor->id, RID_GREF);
        break; // GREF
    case 32:
        read_motor_data(motor->id, RID_DETA);
        break; // Deta
    case 33:
        read_motor_data(motor->id, RID_V_BW);
        break; // V_BW
    case 34:
        read_motor_data(motor->id, RID_IQ_CL);
        break; // IQ_c1
    case 35:
        read_motor_data(motor->id, RID_VL_CL);
        break; // VL_c1
    case 36:
        read_motor_data(motor->id, RID_CAN_BR);
        break; // can_br
    case 37:
        read_motor_data(motor->id, RID_SUB_VER);
        break; // sub_ver
    case 38:
        read_motor_data(motor->id, RID_U_OFF);
        break; // u_off
    case 39:
        read_motor_data(motor->id, RID_V_OFF);
        break; // v_off
    case 40:
        read_motor_data(motor->id, RID_K1);
        break; // k1
    case 41:
        read_motor_data(motor->id, RID_K2);
        break; // k2
    case 42:
        read_motor_data(motor->id, RID_M_OFF);
        break; // m_off
    case 43:
        read_motor_data(motor->id, RID_DIR);
        break; // dir
    case 44:
        read_motor_data(motor->id, RID_P_M);
        break; // pm
    case 45:
        read_motor_data(motor->id, RID_X_OUT);
        break; // xout
    }
}
/**
************************************************************************
* @brief:      	receive_motor_data: ���?�����?�������?
* @param:      	motor_t����������?��
* @param:      	data�����?�����
* @retval:     	void
* @details:    	��?��?���?��?�����?
************************************************************************
**/
void receive_motor_data(motor_t *motor, uint8_t *data)
{
    if (motor->tmp.read_flag == 0)
        return;

    float_type_u y;

    if (data[2] == 0x33)
    {
        uint16_t rid_value = data[3];
        y.b_val[0] = data[4];
        y.b_val[1] = data[5];
        y.b_val[2] = data[6];
        y.b_val[3] = data[7];

        switch (rid_value)
        {
        case RID_UV_VALUE:
            motor->tmp.UV_Value = y.f_val;
            motor->tmp.read_flag = 2;
            break;
        case RID_KT_VALUE:
            motor->tmp.KT_Value = y.f_val;
            motor->tmp.read_flag = 3;
            break;
        case RID_OT_VALUE:
            motor->tmp.OT_Value = y.f_val;
            motor->tmp.read_flag = 4;
            break;
        case RID_OC_VALUE:
            motor->tmp.OC_Value = y.f_val;
            motor->tmp.read_flag = 5;
            break;
        case RID_ACC:
            motor->tmp.ACC = y.f_val;
            motor->tmp.read_flag = 6;
            break;
        case RID_DEC:
            motor->tmp.DEC = y.f_val;
            motor->tmp.read_flag = 7;
            break;
        case RID_MAX_SPD:
            motor->tmp.MAX_SPD = y.f_val;
            motor->tmp.read_flag = 8;
            break;
        case RID_MST_ID:
            motor->tmp.MST_ID = y.u_val;
            motor->tmp.read_flag = 9;
            break;
        case RID_ESC_ID:
            motor->tmp.ESC_ID = y.u_val;
            motor->tmp.read_flag = 10;
            break;
        case RID_TIMEOUT:
            motor->tmp.TIMEOUT = y.u_val;
            motor->tmp.read_flag = 11;
            break;
        case RID_CMODE:
            motor->tmp.cmode = y.u_val;
            motor->tmp.read_flag = 12;
            break;
        case RID_DAMP:
            motor->tmp.Damp = y.f_val;
            motor->tmp.read_flag = 13;
            break;
        case RID_INERTIA:
            motor->tmp.Inertia = y.f_val;
            motor->tmp.read_flag = 14;
            break;
        case RID_HW_VER:
            motor->tmp.hw_ver = y.u_val;
            motor->tmp.read_flag = 15;
            break;
        case RID_SW_VER:
            motor->tmp.sw_ver = y.u_val;
            motor->tmp.read_flag = 16;
            break;
        case RID_SN:
            motor->tmp.SN = y.u_val;
            motor->tmp.read_flag = 17;
            break;
        case RID_NPP:
            motor->tmp.NPP = y.u_val;
            motor->tmp.read_flag = 18;
            break;
        case RID_RS:
            motor->tmp.Rs = y.f_val;
            motor->tmp.read_flag = 19;
            break;
        case RID_LS:
            motor->tmp.Ls = y.f_val;
            motor->tmp.read_flag = 20;
            break;
        case RID_FLUX:
            motor->tmp.Flux = y.f_val;
            motor->tmp.read_flag = 21;
            break;
        case RID_GR:
            motor->tmp.Gr = y.f_val;
            motor->tmp.read_flag = 22;
            break;
        case RID_PMAX:
            motor->tmp.PMAX = y.f_val;
            motor->tmp.read_flag = 23;
            break;
        case RID_VMAX:
            motor->tmp.VMAX = y.f_val;
            motor->tmp.read_flag = 24;
            break;
        case RID_TMAX:
            motor->tmp.TMAX = y.f_val;
            motor->tmp.read_flag = 25;
            break;
        case RID_I_BW:
            motor->tmp.I_BW = y.f_val;
            motor->tmp.read_flag = 26;
            break;
        case RID_KP_ASR:
            motor->tmp.KP_ASR = y.f_val;
            motor->tmp.read_flag = 27;
            break;
        case RID_KI_ASR:
            motor->tmp.KI_ASR = y.f_val;
            motor->tmp.read_flag = 28;
            break;
        case RID_KP_APR:
            motor->tmp.KP_APR = y.f_val;
            motor->tmp.read_flag = 29;
            break;
        case RID_KI_APR:
            motor->tmp.KI_APR = y.f_val;
            motor->tmp.read_flag = 30;
            break;
        case RID_OV_VALUE:
            motor->tmp.OV_Value = y.f_val;
            motor->tmp.read_flag = 31;
            break;
        case RID_GREF:
            motor->tmp.GREF = y.f_val;
            motor->tmp.read_flag = 32;
            break;
        case RID_DETA:
            motor->tmp.Deta = y.f_val;
            motor->tmp.read_flag = 33;
            break;
        case RID_V_BW:
            motor->tmp.V_BW = y.f_val;
            motor->tmp.read_flag = 34;
            break;
        case RID_IQ_CL:
            motor->tmp.IQ_cl = y.f_val;
            motor->tmp.read_flag = 35;
            break;
        case RID_VL_CL:
            motor->tmp.VL_cl = y.f_val;
            motor->tmp.read_flag = 36;
            break;
        case RID_CAN_BR:
            motor->tmp.can_br = y.u_val;
            motor->tmp.read_flag = 37;
            break;
        case RID_SUB_VER:
            motor->tmp.sub_ver = y.u_val;
            motor->tmp.read_flag = 38;
            break;
        case RID_U_OFF:
            motor->tmp.u_off = y.f_val;
            motor->tmp.read_flag = 39;
            break;
        case RID_V_OFF:
            motor->tmp.v_off = y.f_val;
            motor->tmp.read_flag = 40;
            break;
        case RID_K1:
            motor->tmp.k1 = y.f_val;
            motor->tmp.read_flag = 41;
            break;
        case RID_K2:
            motor->tmp.k2 = y.f_val;
            motor->tmp.read_flag = 42;
            break;
        case RID_M_OFF:
            motor->tmp.m_off = y.f_val;
            motor->tmp.read_flag = 43;
            break;
        case RID_DIR:
            motor->tmp.dir = y.f_val;
            motor->tmp.read_flag = 44;
            break;
        case RID_P_M:
            motor->tmp.p_m = y.f_val;
            motor->tmp.read_flag = 45;
            break;
        case RID_X_OUT:
            motor->tmp.x_out = y.f_val;
            motor->tmp.read_flag = 0;
            break;
        }
    }
}

/**
************************************************************************
* @brief:      	fdcan1_rx_callback: CAN1���??�����
* @param:      	void
* @retval:     	void
* @details:    	����CAN1�����???������?��?���ID�����?�?����?�?�����
*               �����?�ID?0?������dm4310_fbdata��������Motor�?������?�
************************************************************************
**/
void fdcan1_rx_callback(void)
{
    uint16_t rec_id1;
    uint8_t rx_data[8] = {0};
    fdcanx_receive(&hfdcan1, &rec_id1, rx_data);
    switch ((rx_data[0]) & 0x0F)
    {
    case 0x05:
        dm_motor_fbdata(&R_motor[E_ROLL], rx_data);
        receive_motor_data(&R_motor[E_ROLL], rx_data);
        break;
    case 0x01:
        dm_motor_fbdata(&R_motor[E_PITCH], rx_data);
        receive_motor_data(&R_motor[E_PITCH], rx_data);
        break;
    case 0x02:
        dm_motor_fbdata(&R_motor[E_KNEE], rx_data);
        receive_motor_data(&R_motor[E_KNEE], rx_data);
        break;
    case 0x03:
        dm_motor_fbdata(&R_motor[E_WHEEL], rx_data);
        receive_motor_data(&R_motor[E_WHEEL], rx_data);
        break;
    }
}

/**
************************************************************************
* @brief:      	fdcan2_rx_callback: CAN2���??�����
* @param:      	void
* @retval:     	void
* @details:    	����CAN2�����???������?��?���ID�����?�?����?�?�����
*               �����?�ID?0?������dm4310_fbdata��������Motor�?������?�
************************************************************************
**/
void fdcan2_rx_callback(void)
{
    uint16_t rec_id2;
    uint8_t rx_data[8] = {0};
    fdcanx_receive(&hfdcan2, &rec_id2, rx_data);
    switch ((rx_data[0]) & 0x0F)
    {
    case 0x05:
        dm_motor_fbdata(&L_motor[E_ROLL], rx_data);
        receive_motor_data(&L_motor[E_ROLL], rx_data);
        break;
    case 0x01:
        dm_motor_fbdata(&L_motor[E_PITCH], rx_data);
        receive_motor_data(&L_motor[E_PITCH], rx_data);
        break;
    case 0x02:
        dm_motor_fbdata(&L_motor[E_KNEE], rx_data);
        receive_motor_data(&L_motor[E_KNEE], rx_data);
        break;
    case 0x03:
        dm_motor_fbdata(&L_motor[E_WHEEL], rx_data);
        receive_motor_data(&L_motor[E_WHEEL], rx_data);
        break;
    }
}
