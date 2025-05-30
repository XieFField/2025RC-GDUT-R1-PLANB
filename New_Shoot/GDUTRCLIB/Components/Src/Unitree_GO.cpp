/**
 * @file Unitree_GO.cpp
 * @author XieFField
 * @brief 宇树GO关节电机驱动文件
 * @attention 由于宇树的GO关节点击M_8010的协议特殊又复杂，很难与前面像RM电机或者
 *            本杰明和达妙电机的接口相兼容，所以我单开一个文件
 * @date 2025-5-27
 */

 #include "Unitree_GO.h"
 #include "data_pool.h"
// 需要一个根据id来判断是哪个模块的函数，因为可能会有多个模块id
// 而扩展帧拆开来那两位是模块id，所以应该做好封装，虽然实际上只有1个。

uint8_t CAN_To_RS485Module_IDCallBack(uint8_t module_ID)
{   
    if (module_ID == (CAN_To_RS485_Module_ID_0 >> 27)) // 如果是模块1

        return 0;
    else if (module_ID == (CAN_To_RS485_Module_ID_1 >> 27)) 
        return 1;
    else if (module_ID == (CAN_To_RS485_Module_ID_2 >> 27)) 
        return 2;
    else if (module_ID == (CAN_To_RS485_Module_ID_3 >> 27)) 
        return 3;

  return -1;
}

// 用该函数来区分是一个模块上的哪个电机,从这里传进数据帧，进行解析
// 只要返回值>=0，就说明是这个模块上的电机
// 暂时先写两个ID，因为整个学院就两个GO电机，所以先写这两个
uint8_t GO_Motor_ID_CallBack(uint32_t motor_ID)
{
    if ((motor_ID & 0x4000000) != GO1_Rec_From_Module) // 判断是不是发回来的报文
        return -1;
    uint8_t temp_motor_id = (motor_ID & 0xF00);
    if (temp_motor_id == GO1_Motor_ID_0) 
        return 0;
    else if (temp_motor_id == GO1_Motor_ID_1) 
        return 1;
    else if (temp_motor_id == GO1_Motor_ID_2) 
        return 2;
  return -2;
}

//力矩混合控制接口
// 10 1 00 0000 01 0 00000000
uint8_t GO_M8010::GO_Motor_Standard_Ctrl(float ref_kpos, float ref_kspd,
                                         float ref_torque, float ref_speed,
                                         float ref_pos) 
{
    // 判断发送的数据有没有变化，如果有变化，就需要重新发送Kpos 和 Kspd
    if (_tool_Abs(this->real_ref_data.K_P - ref_kpos) > 0.0000001 ||
      _tool_Abs(this->real_ref_data.K_W - ref_kspd) > 0.0000001) 
    {
        // 想要修改Kpos 和 Kspd
        this->GO_Set_Ref_KParam(ref_kpos, ref_kspd);
        this->GO_Set_Send_Extid(this->module_id << 27 | GO1_Send_To_Module |
                            GO1_Send_Data_Mode | GO1_Ctrl_Mode_2 | this->ID << 8 |
                            GO1_Default_Mode | GO1_Send_Null_1Bits |
                            GO1_Send_Null_1Byte);
    }
    else
    {
        this->Set_Ref_CtrlParam(ref_torque, ref_speed, ref_pos);
        // 说明Kpos 和
        // Kspd没有变化，不需要再使用控制模式11发送一次，直接使用控制模式10发送
        this->GO_Set_Send_Extid(this->module_id << 27 | GO1_Send_To_Module |
                         GO1_Send_Data_Mode | GO1_Ctrl_Mode_1 | this->ID << 8 |
                         GO1_FOC_Closer_Mode | GO1_Send_Null_1Bits |
                         GO1_Send_Null_1Byte);
    }
    this->CanMsg_Process(this->can_tx_for_motor);

    xQueueSend(CAN2_TxPort, &this->can_tx_for_motor, portMAX_DELAY); //只写CAN2是因为我只用CAN2发送，后续可以进一步封装
    return 0;
}

uint8_t GO_M8010::GO_Motor_Speed_Ctrl(float refspeed, float K_p)
{
    if (_tool_Abs(this->real_ref_data.K_W - K_p) >= 0.0000001) {
    // 想要修改Kspd,速度模式K_pos必须为0
    this->GO_Set_Ref_KParam(0, K_p);
    this->GO_Set_Send_Extid(this->module_id << 27 | GO1_Send_To_Module |
                         GO1_Send_Data_Mode | GO1_Ctrl_Mode_2 | this->ID << 8 |
                         GO1_Default_Mode | GO1_Send_Null_1Bits |
                         GO1_Send_Null_1Byte);
    } 
    else 
    {
        // 说明Kspd没有变化，不需要再使用控制模式11发送一次，直接使用控制模式1发送
        // 速度模式ref_T 必须为0
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
        /* 其他控制量必须置0 */
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
    // 判断当前的数据中K_pos和K_spd是不是0，不是的话就需要重新发送Kpos 和 Kspd
    if (_tool_Abs(this->real_ref_data.K_P - 0) > 0.0000001 ||
        _tool_Abs(this->real_ref_data.K_W - 0) > 0.0000001) {
        // 想要修改Kpos 和 Kspd
        this->GO_Set_Ref_KParam(0.0f, 0.0f);
        this->GO_Set_Send_Extid(this->module_id << 27 | GO1_Send_To_Module |
                            GO1_Send_Data_Mode | GO1_Ctrl_Mode_2 | this->ID << 8 |
                            GO1_Default_Mode | GO1_Send_Null_1Bits |
                            GO1_Send_Null_1Byte);
    } else {
        this->Set_Ref_CtrlParam(0.0f, 0.0f, 0.0f);
        // 说明Kpos 和
        // Kspd没有变化，不需要再使用控制模式11发送一次，直接使用控制模式10发送
        this->GO_Set_Send_Extid(this->module_id << 27 | GO1_Send_To_Module |
                            GO1_Send_Data_Mode | GO1_Ctrl_Mode_1 | this->ID << 8 |
                            GO1_FOC_Closer_Mode | GO1_Send_Null_1Bits |
                            GO1_Send_Null_1Byte);
    }
    this->CanMsg_Process(this->can_tx_for_motor);
    xQueueSend(CAN2_TxPort, &this->can_tx_for_motor, portMAX_DELAY);

  return 0;
}

// 发送命令告诉电机，请回传Kpos 和 Kspd
uint8_t GO_M8010::GO_Motor_ReadBack_Ctrl() {
  // 发送控制模式是12
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
        // 想要修改Kpos 和 Kspd
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


