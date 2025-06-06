/**
 * @file chassis_task.cpp
 * @author Wu Jia
 * @brief 机构任务
 * @version 0.1
 * @date 2025-05-24
 * 
 * @copyright Copyright (c) 2024
 * 
 */
#include "chassis_task.h"
#include "speed_plan.h"


Omni_Chassis chassis(0.152/2.f, 0.442f/2.f, 3, 1.f); //底盘直径0.442m，轮子半径0.152m，底盘加速度0.5m/s^2
Launcher launch(560.f,-916.645996);
// float pos_set = 0;
// bool shoot_ready = false; 
CONTROL_T ctrl;
float target_angle = 0;
int i = 0;
void Chassis_Task(void *pvParameters)
{
//    PID pid;
//    pid.PID_Param_Init(12.0f, 0.015f, 0.0f, 16384.0f, 16384.0f, 10);
//    pid.PID_Mode_Init(0.1f, 0.0f, false, true);
    // static CONTROL_T ctrl;
    for(;;)
    {   
        if(xQueueReceive(Chassia_Port, &ctrl, pdTRUE) == pdPASS)
        {
        
            /*==底盘控制==*/
        
            if(ctrl.chassis_ctrl == CHASSIS_COM_MODE)
            {
                //普通控制模式
                chassis.Control(ctrl.twist);
            }
            else if(ctrl.chassis_ctrl == CHASSIS_CALIBRA_MODE)
            {
                //激光校准模式
                //还没做
                Robot_Twist_t twist = {0};
                chassis.Control(twist);
            }
            else if(ctrl.chassis_ctrl == CHASSIS_LOCK_RING_MODE)
            {
                //环锁定模式
                //还没做
                Robot_Twist_t twist = {0};
                chassis.Control(twist);
            }
            else if(ctrl.chassis_ctrl == CHASSIS_TOGGLE_RING_MODE)
            {
                //环切换模式
                //还没做
                Robot_Twist_t twist = {0};
                chassis.Control(twist);
            }
            else if(ctrl.chassis_ctrl == CHASSIS_OFF)
            {
                //底盘关闭
                Robot_Twist_t twist = {0};
                chassis.Control(twist);
            }
            else
            {
                Robot_Twist_t twist = {0};
                chassis.Control(twist);
            }
            /*===========*/

            /*==俯仰控制==*/
            if(ctrl.pitch_ctrl == PITCH_HAND_MODE)
            {
    //            if(ctrl.twist.pitch.column > 0.5f)
    //                target_angle = target_angle + 0.04f;
    //            else if(ctrl.twist.pitch.column<-0.5f)
    //                target_angle = target_angle - 0.04f;
    //            else {}
    //            launch.PitchControl(target_angle);
            }
            else if(ctrl.pitch_ctrl == PITCH_AUTO_MODE)
            {
    //            launch.Pitch_AutoCtrl(180);
            }
            else if(ctrl.pitch_ctrl == PITCH_CATCH_MODE)
            {
    //            launch.Pitch_AutoCtrl(300);
            }
            else if(ctrl.pitch_ctrl == PITCH_RESET_MODE)
            {
    //            launch.PitchControl(0);
            }
            /*===========*/

            /*==摩擦轮控制==*/
            if(ctrl.friction_ctrl == FRICTION_OFF_MODE)
            {
            //launch.ShootCtrl(false,false,0);
                launch.PushControl(false);
            }
            else if(ctrl.friction_ctrl == FRICTION_ON_MODE)
            {
                if(ctrl.shoot_ctrl == SHOOT_OFF)   
                    //launch.ShootCtrl(false,true,20000);
                    launch.PushControl(false);
                else
                    //launch.ShootCtrl(true,true,20000);
                    launch.PushControl(true);
            }
            /*=============*/

            /*==运球机构角度==*/
            if(ctrl.dri_angle_ctrl == DRIBBLE_ANGLE)
            {

            }
            else if(ctrl.dri_angle_ctrl == PLACE_ANGLE)
            {

            }
            else 
            {
                //DRIBBLE_ANGLE 运球角度
            }
            /*==============*/

            /*==运球摩擦带控制==*/
            if(ctrl.dribble_ctrl == DRIBBLE_OFF)
            {

            }
            else if(ctrl.dribble_ctrl == SUCK_BALL_MODE)//吸球
            {

            }
            else if(ctrl.dribble_ctrl == SPIT_BALL_MODE)//吐球
            {

            }
            else
            {
                //DRIBBLE_OFF 摩擦带停转
            }
            /*===================*/
            
            /*接球机构控制*/
            if(ctrl.catch_ball == CATCH_OFF)
            {

            }
            else if(ctrl.catch_ball == CATCH_ON)
            {

            }
            else
            {
                //CATCH_OFF 接球关闭
            }
        
            // launch.ShootCtrl(true,true,20000);
//            launch.PushControl(true);
            chassis.Motor_Control();
            launch.LaunchMotorCtrl();
        }
    
        osDelay(1);
    }
}


void PidParamInit(void)
{   
    chassis.Pid_Param_Init(0, 20.0f, 0.015f, 0.0f, 16384.0f, 16384.0f, 20); 
    chassis.Pid_Param_Init(1, 12.0f, 0.015f, 0.0f, 16384.0f, 16384.0f, 20); 
    chassis.Pid_Param_Init(2, 12.0f, 0.015f, 0.0f, 16384.0f, 16384.0f, 20); 

    chassis.Pid_Mode_Init(0, 0.1f, 0.0f, false, true);
    chassis.Pid_Mode_Init(1, 0.1f, 0.0f, false, true);
    chassis.Pid_Mode_Init(2, 0.1f, 0.0f, false, true);

    launch.Pid_Param_Init(0,12.0f, 0.015f, 0.0f, 16384.0f, 16384.0f, 0);
    launch.Pid_Mode_Init(0,0.1f, 0.0f, false, true);

    launch.Pid_Param_Init(1,12.0f, 0.015f, 0.0f, 10000.0f, 10000.0f, 0);        //RM2006推球
    launch.Pid_Mode_Init(1,0.1f, 0.0f, false, true);
}
