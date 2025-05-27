/**
 * @file Unitree_GO.h
 * @author XieFField 
 * @brief ������ú�����������������Ƚ�����һЩ�������ҵ�����һ���ļ����ŵ���ķ�װ������Ȼ�Ǽ̳�Motor_Base,��֤�ӿ�ͳһ
 *  ĿǰGO_M8010��id�Ǳ�����Ϊ1�����ƺ���Ҳ�������޸�
 *  Ŀǰ��ʱ��GO���������װ�����������пջὫ���Ż���motor.h������з�װ
 *  ��ʱû�к������������ͳһ�Ľӿڣ������пջ��Ż���������Ҫ�ȵ����
 * @date 2025-5-25
 * ************************************************************************************************************
*/

#ifndef UNITREE_GO_H
#define UNITREE_GO_H

#ifdef __cplusplus
extern "C" {
    #include "string.h"
#endif 

#ifdef __cplusplus
}
#endif

#include "motor.h"


typedef enum
{
    NORMAL_WORKING = 0,
    OVER_HEATED,   // ����
    OVER_CURRENT,  // ����
    OVER_VOLTAGE,  // ��ѹ
    WRONG_ENCODER, // ����������
}GO_M8010_Error_E;          //GO���

typedef struct motor_format_ref_data{
  int16_t T = 0;   // �����ؽ����Ť�� unit: N.m
  int16_t W = 0;   // �����ؽ�����ٶ� unit: rad/s
  int32_t Pos = 0; // �����ؽ����λ�� unit: rad
  int16_t K_P = 0; // �����ؽڸն�ϵ��
  int16_t K_W = 0; // �����ؽ�����ϵ��
} motor_format_ref_data;

typedef struct motor_real_ref_data{
  float T;   // �����ؽڵ�������� �������������أ�unit��N.m
  float W;   // �����ؽ��ٶȣ����������ٶȣ���rad/s��
  float Pos; // �����ؽ�λ�ã�rad��
  float K_P; // �ؽڸն�ϵ��
  float K_W; // �ؽ��ٶ�ϵ��

} motor_real_ref_data;

/*-----------------------------------macro------------------------------------*/
/*
    ��չ֡��ɣ��Ӹ�λ����λ��
    ָ���·���ģ��id | �·�ָʾλ | ��������ָʾλ | ����ģʽλ | Ŀ����id |
   ����ģʽ | Ԥ��λ | Ԥ��λ

    ģ���ϴ���
 */

/* ��λ2bits��Ϊid��ʶ�� */
/* ģ��id 2bits �������3��CANתRS485ģ�� */
#define CAN_To_RS485_Module_ID_0 (0 << 27)
#define CAN_To_RS485_Module_ID_1 (1 << 27)
#define CAN_To_RS485_Module_ID_2 (2 << 27)
#define CAN_To_RS485_Module_ID_3 (3 << 27)

/* ָ���·�Ϊ0 ģ���ϴ�Ϊ1,ռ1bit */
#define GO1_Send_To_Module (0 << 26)
#define GO1_Rec_From_Module (1 << 26)

/* ��������ָʾλ,ռ2bits */
#define GO1_Send_Data_Mode (0 << 24)
#define GO1_Rec_Data_Mode1 (1 << 24)
#define GO1_Rec_Data_Mode2 (2 << 24)

/* ---------------------��λ3--------------------- */
/* ����ģʽλ ռ1Byte */
#define GO1_Ctrl_Mode_1 (10 << 16)
#define GO1_Ctrl_Mode_2 (11 << 16)
#define GO1_Ctrl_Mode_3 (12 << 16)
#define GO1_Ctrl_Mode_4 (13 << 16)

/* ---------------------��λ2--------------------- */
/* Ԥ��λ */
#define GO1_Send_Null_1Bits (0 << 15) // 1bit�ı���λ

/* �������ģʽ ռ3bits */
#define GO1_Default_Mode (0 << 12)        // ����
#define GO1_FOC_Closer_Mode (1 << 12)     // FOC�ջ�
#define GO1_Encoder_calibration (2 << 12) // ������У׼ģʽ
// ʣ��0x3~0x7 ȫΪԤ��λ

/* Ŀ����id ������������15��GO1����������ϣ�ռ4bits */
#define GO1_Motor_ID_0 (0 << 8)
#define GO1_Motor_ID_1 (1 << 8)
#define GO1_Motor_ID_2 (2 << 8)
#define GO1_Motor_ID_3 (3 << 8)
#define GO1_Motor_ID_4 (4 << 8)
#define GO1_Motor_ID_5 (5 << 8)
#define GO1_Motor_ID_6 (6 << 8)
#define GO1_Motor_ID_7 (7 << 8)
#define GO1_Motor_ID_8 (8 << 8)
#define GO1_Motor_ID_9 (9 << 8)
#define GO1_Motor_ID_10 (10 << 8)
#define GO1_Motor_ID_11 (11 << 8)
#define GO1_Motor_ID_12 (12 << 8)
#define GO1_Motor_ID_13 (13 << 8)
#define GO1_Motor_ID_14 (14 << 8)
#define GO1_Motor_ID_15 (15 << 8)

/* ---------------------��λ1--------------------- */
/* ����Ԥ��λ,ռ1Byte */
#define GO1_Send_Null_1Byte (0 << 0) // 1Byte�ı���λ

/*----------------------------------function----------------------------------*/

//����GOϵ�йؽڵ��
class GO_M8010 : public Motor_Speed
{
public:
    virtual MOTOR_FLAG GET_MOTOR_FLAG() const {return GO_MOTOR;}
    GO_M8010(uint8_t id, uint32_t module_id)
     : Motor_Speed(id){} //���id
    virtual ~GO_M8010(){}
    
    //GO_M8010�ĵ�����º���
    void update_GO1(uint8_t can_rx_data[], uint32_t data_id);
    
    /** 
     * @brief ��λ��Ͽ��ƽӿ�
     *        ÿ����һ�Σ��ͻ᷵��һ�����ݣ���Ҫ�������ò��ܻ�ȡ�������   
     * @attention ע�⣬�����λ�ÿ��ƺ�λ��ģʽ��ͬ���������ݵ�ǰλ����Ϊ0�㣨��ʱ���pos��һ����0�ȣ���
     *            Ȼ��ȥ��ת����
     */
    uint8_t GO_Motor_Standard_Ctrl(float ref_kpos, float ref_kspd,
                                 float ref_torque, float ref_speed,
                                 float ref_pos);
    
    /** 
     * @brief λ��ģʽ�ӿڣ����Ա�����λ�ã�
     *        ��������λ�ã�����ֱ��ת������λ��
     * @attention  pd���������������Ŀ��λ�ñ仯�ϴ���ô�������������Ҳ��ܴ�
     *             Ч���������ת������λ�ã�����������λ�ã�
     *             ע��GO1���õ��ǵ�Ȧ����ֵ������������ὫȦ������
     */                           
    uint8_t GO_Motor_Pos_Ctrl(float ref_pos, float K_p, float K_d);
    
    /**
     * @brief �ٶ�ģʽ�ӿ�
     *       ���������ٶȣ����������Ὣ���ȶ���һ���̶����ٶ�
     *       ��ʱ��һ�����ٶȵ�p������
     */
    uint8_t GO_Motor_Speed_Ctrl(float refspeed, float K_p);

    /**
   * @brief ����ģʽ�ӿ�
   *        ����Ч������һ������һ����Ч�������ǵ��������λ�ã����������ǻᶯ
   *
   * @param k_dampping
   * @return uint8_t
   */
  uint8_t GO_Motor_Damping_Ctrl(float k_dampping);

  /**
   * @brief ������ģʽ�ӿ�
   *        �����ת������������С���ϵ�֮ǰ���о�ûʲô��
   *        ������Գ䵱ʧ�ܵ��ģʽ���������ģʽ
   *        ��Ϊ����ӿڵı����ǽ����п�������0
   *
   * @return uint8_t
   */
  uint8_t GO_Motor_No_Tarque_Ctrl();

  /* K_p K_w ���ݻش��ӿ� */
  /**
   * @brief  ��ȡ����ش���Kpos �� Kspd
   *         �����ض�������õ�����Իض����õ�Kpos �� Kspd
   *
   * @return uint8_t
   */
  uint8_t GO_Motor_ReadBack_Ctrl();

  /**
   * @brief ���ͣת
   *
   */
    void GO_Motor_STOP();

    /* ��������ֵ,Kpos & Kspd */
    void GO_Set_Ref_KParam(float ref_kpos, float ref_kspd)
    {
        this->Data_PreProcess_for_KParam(ref_kpos, ref_kspd);
    }

    /* ���ÿ������ο�ֵ */
    void Set_Ref_CtrlParam(float ref_torque, float ref_speed, float ref_pos)
    {
        Data_PreProcess_for_CtrlParam(ref_torque, ref_speed, ref_pos);
    }

    /*��������Ԥ����*/
    void Data_PreProcess_for_KParam(float ref_kpos, float ref_kspd)
    {
        /* ��������ʵֵ�����޷� */
        // �������ؽڸն�ϵ���޷�
        motor_constraint(&(ref_kpos), static_cast<float>(0.0f), static_cast<float>(25.599f));
        // �������ؽ�����ϵ���޷�
        motor_constraint(&(ref_kspd), static_cast<float>(0.0f), static_cast<float>(25.599f));

        this->real_ref_data.K_P = ref_kpos;
        this->real_ref_data.K_W = ref_kspd;

        this->ref_send_data.K_P =
            (uint16_t)(ref_kpos * 1280.0f); // 1280 = 32768/256
        this->ref_send_data.K_W =
            (uint16_t)(ref_kspd * 1280.0f); // 1280 = 32768/256
    }
    void Data_PreProcess_for_CtrlParam(float ref_torque, float ref_speed, float ref_pos)
    {
        /* ������ֵ���м��ٱ����� --- ���ؽ��ٶ�ӳ�䵽���ת���ٶ��ٷ��� */
        this->real_ref_data.T = ref_torque * GET_MOTOR_DESCRIPTION();
        this->real_ref_data.W = ref_speed * GET_MOTOR_DESCRIPTION();
        this->real_ref_data.Pos = ref_pos * GET_MOTOR_DESCRIPTION();

        motor_constraint(&(this->real_ref_data.T), static_cast<float>(-127.99f),
                        static_cast<float>(127.99f));
        motor_constraint(&(this->real_ref_data.W), static_cast<float>(-804.00f),
                        static_cast<float>(804.00f));
        motor_constraint(&(this->real_ref_data.Pos), static_cast<float>(-411774.0f),
                        static_cast<float>(411774.0f));

        this->ref_send_data.T =
            (uint16_t)(this->real_ref_data.T *
                    256.0f); // ����256�����������ý�С�Ĵ�����
        this->ref_send_data.W =
            (uint16_t)(this->real_ref_data.W * 256.0f / (2 * PI));
        this->ref_send_data.Pos =
            (uint32_t)(this->real_ref_data.Pos * 32768 / (2 * PI));
    }

    /*CAN֡�������ݴ��*/
    virtual void CanMsg_Process(CAN_TxMsg &CAN_TxMsg) override
    {
        // ����ģʽλΪ10
      // @todo:"���ģʽ�£�ÿ��һ�ξͻ��յ�һ�ε����������"�����ôʵ�֣�
      if ((CAN_TxMsg.id & 0xFF0000) == GO1_Ctrl_Mode_1) 
      {
            CAN_TxMsg.len = 8;
            CAN_TxMsg.data[7] = (this->ref_send_data.T >> 8) & 0xFF;
            CAN_TxMsg.data[6] = this->ref_send_data.T & 0xFF;
            CAN_TxMsg.data[5] = (this->ref_send_data.W >> 8) & 0xFF;
            CAN_TxMsg.data[4] = this->ref_send_data.W & 0xFF;
            CAN_TxMsg.data[3] = (this->ref_send_data.Pos >> 24) & 0xFF;
            CAN_TxMsg.data[2] = (this->ref_send_data.Pos >> 16) & 0xFF;
            CAN_TxMsg.data[1] = (this->ref_send_data.Pos >> 8) & 0xFF;
            CAN_TxMsg.data[0] = this->ref_send_data.Pos & 0xFF;
        } 
        else if ((CAN_TxMsg.id & 0xFF0000) == GO1_Ctrl_Mode_2) 
        {
            // ����ģʽλΪ11 ���� Kpos �� Kspd
            CAN_TxMsg.data[7] = 0;
            CAN_TxMsg.data[6] = 0;
            CAN_TxMsg.data[5] = 0;
            CAN_TxMsg.data[4] = 0;
            CAN_TxMsg.data[3] = (this->ref_send_data.K_P >> 8) & 0xFF;
            CAN_TxMsg.data[2] = this->ref_send_data.K_P & 0xFF;
            CAN_TxMsg.data[1] = (this->ref_send_data.K_W >> 8) & 0xFF;
            CAN_TxMsg.data[0] = this->ref_send_data.K_W & 0xFF;
        }
        else if ((CAN_TxMsg.id & 0xFF0000) == GO1_Ctrl_Mode_3) 
        {
        // ����ģʽλΪ12
            memset(CAN_TxMsg.data, 0,sizeof(CAN_TxMsg.data)); // ֱ�����㷢��
        } 
        else if ((CAN_TxMsg.id & 0xFF0000) == GO1_Ctrl_Mode_4) 
        {
            // ����ģʽλΪ13
            // @todo:���ģʽ�£����Ϳ�����Ϣ�����ڵ���������򲻻������ݷ���
            CAN_TxMsg.data[7] = (this->ref_send_data.T >> 8) & 0xFF;
            CAN_TxMsg.data[6] = this->ref_send_data.T & 0xFF;
            CAN_TxMsg.data[5] = (this->ref_send_data.W >> 8) & 0xFF;
            CAN_TxMsg.data[4] = this->ref_send_data.W & 0xFF;
            CAN_TxMsg.data[3] = (this->ref_send_data.Pos >> 24) & 0xFF;
            CAN_TxMsg.data[2] = (this->ref_send_data.Pos >> 16) & 0xFF;
            CAN_TxMsg.data[1] = (this->ref_send_data.Pos >> 8) & 0xFF;
            CAN_TxMsg.data[0] = this->ref_send_data.Pos & 0xFF;
        }
    }

    /* ������Ž��� */
    void errorType_Get(uint8_t error_id) 
    {
        switch (error_id) 
        {
        case 0:
            this->error_type = NORMAL_WORKING;
            break;
        case 1:
            this->error_type = OVER_HEATED;
            break;
        case 2:
            this->error_type = OVER_CURRENT;
            break;
        case 3:
            this->error_type = OVER_VOLTAGE;
            break;
        case 4:
            this->error_type = WRONG_ENCODER;
            break;
        }
    }

    void air_pressure_parameters_Get(uint16_t data) //��ȡ��ѹ����(�о��ò���)
    {
        this->air_pressure_parameters = (float)data;
    }

    void temperature_get(uint16_t data)
    {
        this->temperature = (float)data;
    }

    void processRxMsg()
    {
        // ������յ�������
        this->real_cur_data.T =((float)this->cur_rec_data.T) / 256.0f / 
                                GET_MOTOR_DESCRIPTION(); // ����256����������ת��Ϊ����
        this->real_cur_data.W = (((float)this->cur_rec_data.W) / 256.0f) * 2 * PI /
                                GET_MOTOR_DESCRIPTION();
        this->real_cur_data.Pos = ((float)this->cur_rec_data.Pos) * 2 * PI / 32768 /
                                GET_MOTOR_DESCRIPTION();
    }

    //���÷��͵���չ֡id
    void GO_Set_Send_Extid(uint32_t extID)
    {
        this->can_tx_for_motor.id = extID;
    }
    
    /*���ݶ���*/
    /* �����͵�ǰ����ʵ�ٶ� */
    motor_real_ref_data real_ref_data = {};
    motor_real_ref_data real_cur_data = {};    
    /* �����͵�ǰ����ʽ��������� */
    motor_format_ref_data ref_send_data = {};
    motor_format_ref_data cur_rec_data = {};

    GO_M8010_Error_E error_type = NORMAL_WORKING; // ��������ʶ: 0.���� 1.���� 2.���� 3.��ѹ 4.����������  5-7.����

    float air_pressure_parameters = 0;
    int temperature;
    uint32_t module_id = 0;
    
    CAN_RxBuffer can_rx_for_motor;
    CAN_TxMsg can_tx_for_motor;
private:
    virtual uint8_t GET_MOTOR_DESCRIPTION() const { return 6.33; } //���ٱ�
    float last_ref_kpos = 0;
    float last_ref_kspd = 0;  
};

#endif // #define UNITREE_GO_H
