#include "Cancer_FOC.hpp"


void MY_FOC::FOC_lab_OpenTest()
{
    //校准完成后运行程序
    if(ADC_Calibration_Flag)
    {
        //自己生成角度
#if OPEN_TEST == SELF_GENERATION
        theta_Park += 2.0f*PI/300.0f;
        FOC_Angle_GetAngle();
        //
        Park_D[1] = arm_sin_f32(theta_Park);   Park_D[0] = arm_cos_f32(theta_Park);

        Voltage_RevPark_alpha = 0.5f * Vdc * arm_cos_f32(theta_Park);
        Voltage_RevPark_beta  = 0.5f * Vdc * arm_sin_f32(theta_Park);

        //FOC_Cacu_Clark_Park(Current[0],Current[1],Current[2]);

        // FOC_Cacu_Clark_Park(raw_Current_C,raw_Current_A,raw_Current_B);

#elif OPEN_TEST == TARGET_ANGLE

        Voltage_RevPark_alpha = 0.1f * Vdc * arm_cos_f32(PI);
        Voltage_RevPark_beta  = 0.1f * Vdc * arm_sin_f32(PI);


#elif OPEN_TEST == DQ_INPUT
        FOC_Angle_GetAngle();
        FOC_Angle_Get_Speed();
        //获取电角度和park正反变换结构体
        theta_Park = theta * Pole_Pairs;

        // Park_D[1] = arm_sin_f32(theta_Park-PI*1/6);   Park_D[0] = arm_cos_f32(theta_Park-PI*1/6);
    

        // Park_D[1] = arm_sin_f32(2*PI-theta_Park);   Park_D[0] = arm_cos_f32(2*PI-theta_Park);
        Park_D[1] = sinf(theta_Park);   Park_D[0] = cosf(theta_Park);

        FOC_Cacu_Clark_Park(-raw_Current_A,-raw_Current_B,-raw_Current_C);
        //反park变换验证
        // if(Voltage_Q<20.0f)
        //     Voltage_Q += 0.0001f;
        // Voltage_Q = 17.50f;
        Voltage_D = 0.0f;

        FOC_Cacu_RevPark();

        // Voltage_RevPark_alpha = 0.2f * Vdc * arm_cos_f32(theta_Park + PI/4);
        // Voltage_RevPark_beta  = 0.2f * Vdc * arm_sin_f32(theta_Park + PI/4);

#endif
        
        //传入参数控制电机旋转
        svpwm.SVPWM_Program(duty,Voltage_RevPark_alpha,Voltage_RevPark_beta);
        FOC_Sys_ChangeDuty(duty);
        FOC_Sys_Print_Vari();

    }
}

void MY_FOC::FOC_lab_CurrentLoop()
{
    if(ADC_Calibration_Flag)
    {

         //电角度校准
        FOC_Angle_GetAngle();
        FOC_Angle_Get_Speed();
        static uint8_t Loop_PSC = 50;
        static uint8_t Flag = 0;

        if(Flag == Loop_PSC)
        {
            Speed_Control.Ref = Speed_REF;
            I_q.Ref = Pos_PID_Controller(&Speed_Control,speed);
            Flag = 0;
        }
        Flag++;

        //获取电角度和park正反变换结构体
        theta_Park = theta * Pole_Pairs;
        Park_D[1] = arm_sin_f32(theta_Park);   Park_D[0] = arm_cos_f32(theta_Park);

        FOC_Cacu_Clark_Park(-raw_Current_A,-raw_Current_B,-raw_Current_C);
        //反park变换验证
        // I_q.Ref = 200.0f;
        Voltage_Q = Pos_PID_Controller(&I_q,Current_Park_Q);
        Voltage_D = Pos_PID_Controller(&I_d,Current_Park_D);
  
        FOC_Cacu_RevPark();
        //传入参数控制电机旋转
        svpwm.SVPWM_Program(duty,Voltage_RevPark_alpha,Voltage_RevPark_beta);
        FOC_Sys_ChangeDuty(duty);
        FOC_Sys_Print_Vari();

    }
}