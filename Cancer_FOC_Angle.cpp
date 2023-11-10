#include "Cancer_FOC.hpp"

//校准电角度函数
void MY_FOC::FOC_Angle_Calibrate_the_electrical_angle()
{
    svpwm.SVPWM_Program(duty,5.0f,0);
    FOC_Sys_ChangeDuty(this->duty);
    HAL_Delay(50);
//读取两遍才能获得上一次的数据
    raw_angle = AS5048a_Read_Angle();

    this->theta_offset = raw_angle*2*PI/0x3fff;
    //防止读出的数据错误
    while(this->theta_offset > 2*PI)
        this->theta_offset = AS5048a_Read_Angle()*2*PI/0x3fff;

    HAL_Delay(10);
}

void MY_FOC::FOC_Angle_GetAngle()
{
    raw_angle = AS5048a_Read_Angle();
    theta = raw_angle*2*PI/0x3fff;
    if(theta < theta_offset)
    {
        theta = PI*2 - theta_offset + theta;
    }
    else if(theta >= theta_offset)
    {
        theta = theta - theta_offset;
    }
}

//读取速度函数
void MY_FOC::FOC_Angle_Get_Speed()
{
    static int conversion_T = 0;
    static int T_NUM = 50;
    static float theta_past;
    static float speed_past;

    if(conversion_T == T_NUM)
    {
        speed = (theta - theta_past)/(svpwm.Get_Ts()*T_NUM*2*PI);
        conversion_T = 0;

        if(fabsf(theta - theta_past) > 3.14f)
            speed = speed_past;
        else
            speed_past = speed;

        theta_past = theta;
    }

    // SEGGER_RTT_printf(0,"%d,%d\n",(int)(theta*1000),(int)(theta_past*1000));
    conversion_T++;
}
