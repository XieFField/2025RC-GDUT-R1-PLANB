/**
 * @file Unitree_GO.cpp
 * @author XieFField
 * @brief ����GO�ؽڵ�������ļ�
 * @attention ����������GO�ؽڵ��M_8010��Э�������ָ��ӣ�������ǰ����RM�������
 *            �������ʹ������Ľӿ�����ݣ������ҵ���һ���ļ�
 * @date 2025-5-27
 */

 #include "Unitree_GO.h"
 #include "data_pool.h"
// ��Ҫһ������id���ж����ĸ�ģ��ĺ�������Ϊ���ܻ��ж��ģ��id
// ����չ֡��������λ��ģ��id������Ӧ�����÷�װ����Ȼʵ����ֻ��1����

uint8_t CAN_To_RS485Module_IDCallBack(uint8_t module_ID)
{   
    if (module_ID == (CAN_To_RS485_Module_ID_0 >> 27)) // �����ģ��1

        return 0;
    else if (module_ID == (CAN_To_RS485_Module_ID_1 >> 27)) 
        return 1;
    else if (module_ID == (CAN_To_RS485_Module_ID_2 >> 27)) 
        return 2;
    else if (module_ID == (CAN_To_RS485_Module_ID_3 >> 27)) 
        return 3;

  return -1;
}

// �øú�����������һ��ģ���ϵ��ĸ����,�����ﴫ������֡�����н���
// ֻҪ����ֵ>=0����˵�������ģ���ϵĵ��
// ��ʱ��д����ID����Ϊ����ѧԺ������GO�����������д������
uint8_t GO_Motor_ID_CallBack(uint32_t motor_ID)
{
    if ((motor_ID & 0x4000000) != GO1_Rec_From_Module) // �ж��ǲ��Ƿ������ı���
        return -1;
    uint8_t temp_motor_id = (motor_ID & 0xF00);
    if (temp_motor_id == GO1_Motor_ID_0) 
        return 0;
    else if (temp_motor_id == GO1_Motor_ID_1) 
        return 1;
    else if (temp_motor_id == GO1_Motor_ID_2) 
        return 2;
    else if (temp_motor_id == GO1_Motor_ID_3) 
        return 3;
  return -2;
}

//���ػ�Ͽ��ƽӿ�
// 10 1 00 0000 01 0 00000000
uint8_t GO_M8010::GO_Motor_Standard_Ctrl(float ref_kpos, float ref_kspd,
                                         float ref_torque, float ref_speed,
                                         float ref_pos) 
{
    // �жϷ��͵�������û�б仯������б仯������Ҫ���·���Kpos �� Kspd
    if (_tool_Abs(this->real_ref_data.K_P - ref_kpos) > 0.0000001 ||
      _tool_Abs(this->real_ref_data.K_W - ref_kspd) > 0.0000001) 
    {
        // ��Ҫ�޸�Kpos �� Kspd
        this->GO_Set_Ref_KParam(ref_kpos, ref_kspd);
        this->GO_Set_Send_Extid(this->module_id << 27 | GO1_Send_To_Module |
                            GO1_Send_Data_Mode | GO1_Ctrl_Mode_2 | this->ID << 8 |
                            GO1_Default_Mode | GO1_Send_Null_1Bits |
                            GO1_Send_Null_1Byte);
    }
    else
    {
        this->Set_Ref_CtrlParam(ref_torque, ref_speed, ref_pos);
        // ˵��Kpos ��
        // Kspdû�б仯������Ҫ��ʹ�ÿ���ģʽ11����һ�Σ�ֱ��ʹ�ÿ���ģʽ10����
        this->GO_Set_Send_Extid(this->module_id << 27 | GO1_Send_To_Module |
                         GO1_Send_Data_Mode | GO1_Ctrl_Mode_1 | this->ID << 8 |
                         GO1_FOC_Closer_Mode | GO1_Send_Null_1Bits |
                         GO1_Send_Null_1Byte);
    }
    this->CanMsg_Process(this->can_tx_for_motor);

    xQueueSend(CAN2_TxPort, &this->can_tx_for_motor, portMAX_DELAY); //ֻдCAN2����Ϊ��ֻ��CAN2���ͣ��������Խ�һ����װ
    return 0;
}

uint8_t GO_M8010::GO_Motor_Speed_Ctrl(float refspeed, float K_p)
{
    if (_tool_Abs(this->real_ref_data.K_W - K_p) >= 0.0000001) {
    // ��Ҫ�޸�Kspd,�ٶ�ģʽK_pos����Ϊ0
    this->GO_Set_Ref_KParam(0, K_p);
    this->GO_Set_Send_Extid(this->module_id << 27 | GO1_Send_To_Module |
                         GO1_Send_Data_Mode | GO1_Ctrl_Mode_2 | this->ID << 8 |
                         GO1_Default_Mode | GO1_Send_Null_1Bits |
                         GO1_Send_Null_1Byte);
    } 
    else 
    {
        // ˵��Kspdû�б仯������Ҫ��ʹ�ÿ���ģʽ11����һ�Σ�ֱ��ʹ�ÿ���ģʽ1����
        // �ٶ�ģʽref_T ����Ϊ0
        this->GO_Set_Send_Extid(this->module_id << 27 | GO1_Send_To_Module |
                            GO1_Send_Data_Mode | GO1_Ctrl_Mode_1 | this->ID << 8 |
                            GO1_FOC_Closer_Mode | GO1_Send_Null_1Bits |
                            GO1_Send_Null_1Byte);
        this->Set_Ref_CtrlParam(0.0f, refspeed, 0.0f);
    }

    this->CanMsg_Process(this->can_tx_for_motor);
    xQueueSend(CAN2_TxPort, &this->can_tx_for_motor, portMAX_DELAY);
    this->last_ref_kspd = K_p;
    return 0;
}

uint8_t GO_M8010::GO_Motor_Damping_Ctrl(float k_dampping)
{
    if (_tool_Abs(this->real_ref_data.K_W - k_dampping) > 0.0000001) {
        this->GO_Set_Ref_KParam(0, k_dampping);
        this->GO_Set_Send_Extid(this->module_id << 27 | GO1_Send_To_Module |
                            GO1_Send_Data_Mode | GO1_Ctrl_Mode_2 | this->ID << 8 |
                            GO1_Default_Mode | GO1_Send_Null_1Bits |
                            GO1_Send_Null_1Byte);
    } 
    else 
    {
        /* ����������������0 */
        this->Set_Ref_CtrlParam(0.0f, 0.0f, 0.0f);
        this->GO_Set_Send_Extid(this->module_id << 27 | GO1_Send_To_Module |
                            GO1_Send_Data_Mode | GO1_Ctrl_Mode_1 | this->ID << 8 |
                            GO1_FOC_Closer_Mode | GO1_Send_Null_1Bits |
                            GO1_Send_Null_1Byte);
    }
    this->CanMsg_Process(this->can_tx_for_motor);

        xQueueSend(CAN2_TxPort, &this->can_tx_for_motor, portMAX_DELAY);
    return 0;
}

uint8_t GO_M8010::GO_Motor_No_Tarque_Ctrl()
{
    // �жϵ�ǰ��������K_pos��K_spd�ǲ���0�����ǵĻ�����Ҫ���·���Kpos �� Kspd
    if (_tool_Abs(this->real_ref_data.K_P - 0) > 0.0000001 ||
        _tool_Abs(this->real_ref_data.K_W - 0) > 0.0000001) {
        // ��Ҫ�޸�Kpos �� Kspd
        this->GO_Set_Ref_KParam(0.0f, 0.0f);
        this->GO_Set_Send_Extid(this->module_id << 27 | GO1_Send_To_Module |
                            GO1_Send_Data_Mode | GO1_Ctrl_Mode_2 | this->ID << 8 |
                            GO1_Default_Mode | GO1_Send_Null_1Bits |
                            GO1_Send_Null_1Byte);
    } else {
        this->Set_Ref_CtrlParam(0.0f, 0.0f, 0.0f);
        // ˵��Kpos ��
        // Kspdû�б仯������Ҫ��ʹ�ÿ���ģʽ11����һ�Σ�ֱ��ʹ�ÿ���ģʽ10����
        this->GO_Set_Send_Extid(this->module_id << 27 | GO1_Send_To_Module |
                            GO1_Send_Data_Mode | GO1_Ctrl_Mode_1 | this->ID << 8 |
                            GO1_FOC_Closer_Mode | GO1_Send_Null_1Bits |
                            GO1_Send_Null_1Byte);
    }
    this->CanMsg_Process(this->can_tx_for_motor);
    xQueueSend(CAN2_TxPort, &this->can_tx_for_motor, portMAX_DELAY);

  return 0;
}

// ����������ߵ������ش�Kpos �� Kspd
uint8_t GO_M8010::GO_Motor_ReadBack_Ctrl() {
  // ���Ϳ���ģʽ��12
    this->GO_Set_Send_Extid(this->module_id << 27 | GO1_Send_To_Module |
                        GO1_Send_Data_Mode | GO1_Ctrl_Mode_3 | this->ID << 8 |
                        GO1_FOC_Closer_Mode | GO1_Send_Null_1Bits |
                        GO1_Send_Null_1Byte);
    this->CanMsg_Process(this->can_tx_for_motor);
    xQueueSend(CAN2_TxPort, &this->can_tx_for_motor, portMAX_DELAY);
  return 0;
}

void GO_M8010::GO_Motor_STOP() 
{

    if (_tool_Abs(this->real_ref_data.K_P - 0) > 0.0000001 ||
        _tool_Abs(this->real_ref_data.K_W - 0.005) > 0.0000001) 
    {
        // ��Ҫ�޸�Kpos �� Kspd
        this->GO_Set_Ref_KParam(0, 0.005);
        this->GO_Set_Send_Extid(this->module_id << 27 | GO1_Send_To_Module |
                            GO1_Send_Data_Mode | GO1_Ctrl_Mode_2 | this->ID << 8 |
                            GO1_Default_Mode | GO1_Send_Null_1Bits |
                            GO1_Send_Null_1Byte);
    } 
    else 
    {
        this->Set_Ref_CtrlParam(0.0f, 0.0f, this->real_cur_data.Pos);
        this->GO_Set_Send_Extid(this->module_id << 27 | GO1_Send_To_Module |
                            GO1_Send_Data_Mode | GO1_Ctrl_Mode_1 | this->ID << 8 |
                            GO1_FOC_Closer_Mode | GO1_Send_Null_1Bits |
                            GO1_Send_Null_1Byte);
    }
    this->CanMsg_Process(this->can_tx_for_motor);

    xQueueSend(CAN2_TxPort, &this->can_tx_for_motor, portMAX_DELAY);
}

// �������棬�Ƚ�ʣ����չ֡�е������ó������жϽ���ģʽ��Ȼ���ٽ��ж�Ӧ�����ݶν���
void GO_M8010::update_GO1(uint8_t can_rx_data[], uint32_t data_id) 
{
    // �ȸ��ݵ�λ1 ���ж���ʲô��������ģʽ
    if ((data_id & 0x3000000) == GO1_Rec_Data_Mode1) // ��������ģʽ1
    {
        // ����Ҫ�������һλ�ǲ���-128�����жϵ�����ޱ���
        if ((uint16_t)(data_id & 0xFF) == -128) 
        {
            // ��������������ģ�
            this->errorType_Get((data_id & 0xFF0000) >>
                                16); // ���д�����Ž��������忴������������
        } 
        else 
        {
            // ����������������ģ���ʱ�ǵô洢�¶�
            this->air_pressure_parameters_Get((uint16_t)((data_id & 0xFF0000) >> 16));
            this->temperature_get((uint16_t)(data_id & 0xFF));
        }
        // ���۵���Ƿ񱨴����ᷢ�ͻ������ݶ�
        this->cur_rec_data.T = (int16_t)(can_rx_data[7] << 8 | can_rx_data[6]);
        this->cur_rec_data.W = (int16_t)(can_rx_data[5] << 8 | can_rx_data[4]);
        this->cur_rec_data.Pos =
            (int32_t)(can_rx_data[3] << 24 | can_rx_data[2] << 16 |
                    can_rx_data[1] << 8 | can_rx_data[0]);
    } 
    else if ((data_id & 0x3000000) == GO1_Rec_Data_Mode2) // ��������ģʽ2
    {
        // ��Ӧָ���·��� ģʽ12 ����ȡKpos �� Kspdģʽ�����ֻ����Kpos �� Kspd
        this->cur_rec_data.K_P = (int16_t)(can_rx_data[3] << 8 | can_rx_data[2]);
        this->cur_rec_data.K_W = (int16_t)(can_rx_data[1] << 8 | can_rx_data[0]);
    }
    this->processRxMsg();
}
