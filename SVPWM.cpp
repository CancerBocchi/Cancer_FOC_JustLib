#include "SVPWM.hpp"

//const member init
const int   SVPWM :: First   = 5;
const int   SVPWM :: Second  = 7;
const int   SVPWM :: Third   = 6;  
const int   SVPWM :: Fourth  = 2;
const int   SVPWM :: Fifth   = 0;
const int   SVPWM :: Sixth   = 1;

SVPWM::SVPWM(float Ts,float Vdc)
{
	this->m = Ts/Vdc/Sqrt3_2;
	this->Ts = Ts;
	this->Vdc = Vdc;
}

SVPWM::SVPWM()
{
	
}

SVPWM::~SVPWM()
{
	
}

uint8_t SVPWM::SVPWM_Position_Judgement()
{
    float Value1 = V_Input_Beta - Sqrt3 * V_Input_Alpha;
	float Value2 = V_Input_Beta + Sqrt3 * V_Input_Alpha;
	int Value[3],CodeValue;
	if(V_Input_Beta>0)
		Value[0] = 1;
	else
		Value[0] = 0;
	
	if(Value1>0)
		Value[1] = 1;
	else
		Value[1] = 0;
	
	if(Value2>0)
		Value[2] = 1;
	else
		Value[2] = 0;
	CodeValue = Value[0]*4 + Value[1]*2 + Value[2];
	return CodeValue;
}

void SVPWM::SVPWM_ChangeVdc(float Vdc)
{
	this->m = Ts*Sqrt3/Vdc;
	this->Ts = Ts;
	this->Vdc = Vdc;
}

bool SVPWM::SVPWM_Program(float *duty,float V_Alpha,float V_Beta)
{
	if(V_Alpha*V_Alpha+V_Beta*V_Beta > Vdc*Vdc)
	{
		return false;
	}

	V_Input_Alpha = V_Alpha;
	V_Input_Beta  = V_Beta;

    //扇区判断
	uint8_t pos = SVPWM_Position_Judgement();
	float Val_alpha = Sqrt3_2 * V_Input_Alpha;
	float Val_beta = 0.5 * V_Input_Beta;
	//占空比分配
	if(pos == First)
	{	
		T1 = m * (Val_alpha - Val_beta);
		T2 = m * V_Input_Beta;
		T0 = (Ts - T1 - T2)/2;
		
		duty[0] = (T0+T1+T2)/Ts;
		duty[1] = (T0+T2)/Ts;
		duty[2] = T0/Ts;
	}
	else if(pos == Second)
	{
		T1 = m * (Val_alpha  + Val_beta);
		T2 = m * (-Val_alpha + Val_beta);
		T0 = (Ts - T1 - T2)/2;
		
		duty[0] = (T0+T1)/Ts;
		duty[1] = (T0+T1+T2)/Ts;
		duty[2] = T0/Ts;
	}
	else if(pos == Third)
	{
		T1 = m *  V_Input_Beta;
		T2 = m * (-Val_alpha - Val_beta);
		T0 = (Ts - T1 - T2)/2;
		
		duty[0] = T0/Ts;
		duty[1] = (T0+T1+T2)/Ts;
		duty[2] = (T0+T2)/Ts;
	}
	else if(pos == Fourth)
	{
		T1 = - m * (Val_alpha - Val_beta);
		T2 = - m *  V_Input_Beta;
		T0 = (Ts - T1 - T2)/2;
		
		duty[0] = T0/Ts;
		duty[1] = (T0+T1)/Ts;
		duty[2] = (T0+T1+T2)/Ts;
	}
	else if(pos == Fifth)
	{
		T1 = - m * (Val_alpha  + Val_beta);
		T2 = - m * (-Val_alpha + Val_beta);
		T0 = (Ts - T1 - T2)/2;
		
		duty[0] = (T0+T2)/Ts;
		duty[1] = T0/Ts;
		duty[2] = (T0+T1+T2)/Ts;
	}
	else if(pos == Sixth)
	{
		T1 = - m *  V_Input_Beta;
		T2 = - m * (-Val_alpha - Val_beta);
		T0 = (Ts - T1 - T2)/2;
		
		duty[0] = (T0+T1+T2)/Ts;
		duty[1] = T0/Ts;
		duty[2] = (T0+T1)/Ts;
	}
	return true;
}

float SVPWM::Get_Ts()
{
	return Ts;
}
