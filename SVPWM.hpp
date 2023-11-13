/**
 * @file SVPWM.hpp
 * @author Cancer (you@domain.com)
 * @brief   输入：期望 alpha beta 轴的电压值（通过更改该值大小来调整实际矢量大小） 
 *          输出：三项占空比
 *          对象需要的参数：单位周期的时间，常系数为 m
 *          单纯算法 只给出占空比 没有输出到外设 
 * @version 0.1
 * @date 2023-07-06
 * 
 * @copyright Copyright (c) 2023
 * 
 */

#ifndef __SVPWM_HPP__
#define __SVPWM_HPP__

//#include "task.hpp"
#include "task.h"

#define Sqrt3 1.732050807f
#define Sqrt3_2 0.866025403f

class SVPWM
{
private:
//常量定义 加快运算
    static const int First;
    static const int Second;
    static const int Third;
    static const int Fourth;
    static const int Fifth;
    static const int Sixth;

//三相系统转化成 alpha 和 beta 轴后的电压
    float V_Input_Alpha;
    float V_Input_Beta;

//相关参数
    float Ts;//周期时间大小
    float T0;
    float T1;
    float T2;

    float m;//常系数 公式为：(2*Ts)/(sqrt(3)*Vdc)
    float Vdc;

    uint8_t SVPWM_Position_Judgement();
    void SVPWM_JudgeOverLoad();
public:
    SVPWM(float Ts,float Vdc);
    ~SVPWM();
    SVPWM();
    
    //传入duty得到占空比
    float Get_Ts();
    void SVPWM_ChangeVdc(float Vdc);//改变输入电压
    bool SVPWM_Program(float *duty,float V_Alpha,float V_Beta);
    
};

#endif
