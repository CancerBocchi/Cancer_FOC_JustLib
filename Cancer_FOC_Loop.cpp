#include "Cancer_FOC.hpp"

void MY_FOC::FOC_LoopRun()
{
     if(ADC_Calibration_Flag)
    {
        switch (FOC_Loop_Select)
        {
        case FOC_CURRENT_LOOP:
            FOC_Loop_Current(Iq_REF);
            break;

        case FOC_SPEED_LOOP:
            FOC_Loop_Speed(Speed_REF);
            break;

        case FOC_POS_LOOP:
            FOC_Loop_Position(Pos_REF);
            break;
        }
        
    }
}

void MY_FOC::FOC_Loop_Position(float Pos_Ref)
{
    static uint8_t Loop_PSC = 50;
    static uint8_t Flag = 0;
    static float pos_out;
    static float theta_past;

    Position_Control.Ref = Pos_Ref;

    if(theta_past-theta > 6.0f)
        theta+=6.28f;
    else if (theta_past-theta < -6.0f)
        theta-=6.28f;
    
    theta_past = theta;

    if(Flag == Loop_PSC)
    {
        pos_out = Pos_PID_Controller(&Position_Control,theta);
        Flag = 0;
    }
    Flag ++;
    
    FOC_Loop_Speed(pos_out);

}

void MY_FOC::FOC_Loop_Speed(float Speed_Ref)
{
    static uint8_t Loop_PSC = 50;
    static uint8_t Flag = 0;
    static float speed_out;

    FOC_Angle_Get_Speed();
    Speed_Control.Ref = Speed_Ref;

    if(Flag == Loop_PSC)
    {
        speed_out = Pos_PID_Controller(&Speed_Control,speed);
        Flag = 0;
    }
    Flag++;

    FOC_Loop_Current(speed_out);
    // FOC_Angle_GetAngle();
    // FOC_Angle_Get_Speed();

    // theta_Park = theta * Pole_Pairs;
    // Park_D[1] = arm_sin_f32(theta_Park);   Park_D[0] = arm_cos_f32(theta_Park);

    // FOC_Cacu_Clark_Park(-raw_Current_A,-raw_Current_B,-raw_Current_C);
    // //反park变换验证

    // Voltage_Q = Pos_PID_Controller(&I_q,Current_Park_Q);
    // Voltage_D = Pos_PID_Controller(&I_d,Current_Park_D);

    // // Voltage_D = 0;

    // FOC_Cacu_RevPark();
    // //传入参数控制电机旋转
    // svpwm.SVPWM_Program(duty,Voltage_RevPark_alpha,Voltage_RevPark_beta);
    // FOC_Sys_ChangeDuty(duty);
    // FOC_Sys_Print_Vari();


    
}

void MY_FOC::FOC_Loop_Current(float I_q_Ref)
{
        //自己生成角度
        //  theta_Park += 2.0f*PI/400.0f;
        
        I_q.Ref = I_q_Ref;
        //电角度校准
        FOC_Angle_GetAngle();
        // SEGGER_RTT_printf(0,"%d\n", (int)(speed*1000));

        //获取电角度和park正反变换结构体
        theta_Park = theta * Pole_Pairs;
        Park_D[1] = arm_sin_f32(theta_Park);   Park_D[0] = arm_cos_f32(theta_Park);

        FOC_Cacu_Clark_Park(-raw_Current_A,-raw_Current_B,-raw_Current_C);
        //反park变换验证

        Voltage_Q = Pos_PID_Controller(&I_q,Current_Park_Q);
        Voltage_D = Pos_PID_Controller(&I_d,Current_Park_D);

        // Voltage_D = 0;
  
        FOC_Cacu_RevPark();
        //传入参数控制电机旋转
        svpwm.SVPWM_Program(duty,Voltage_RevPark_alpha,Voltage_RevPark_beta);
        FOC_Sys_ChangeDuty(duty);
        FOC_Sys_Print_Vari();

}
