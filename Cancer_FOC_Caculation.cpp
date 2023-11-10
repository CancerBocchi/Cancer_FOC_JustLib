#include "Cancer_FOC.hpp"

//运算部分
void MY_FOC::FOC_Cacu_Clark_Park(float Current_A,float Current_B,float Current_C)
{
    Current_Clark_alpha = Current_A - (Current_B + Current_C) * 0.5f;
    Current_Clark_beta  = (Current_B - Current_C) * Sqrt3_2;

    Current_Park_D =   Current_Clark_alpha*Park_D[0] + Current_Clark_beta*Park_D[1];
    Current_Park_Q =  -Current_Clark_alpha*Park_D[1] + Current_Clark_beta*Park_D[0];
}

void MY_FOC::FOC_Cacu_RevPark()
{
    Voltage_RevPark_alpha = Voltage_D * Park_D[0] - Voltage_Q * Park_D[1];
    Voltage_RevPark_beta  = Voltage_D * Park_D[1] + Voltage_Q * Park_D[0]; 
}

void MY_FOC::FOC_Cacu_First_order_low_pass_filter()
{
    for(int i = 0; i<3 ; i++)
    {
        raw_ADC_after_fliter[i][1] = (float)((1.0f - a) * (float)raw_ADC_after_fliter[i][0] + a*(float)raw_ADC_Value[i]);
        raw_ADC_after_fliter[i][0] = raw_ADC_after_fliter[i][1];
    }
}
