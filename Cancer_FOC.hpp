/**
 * @file Cancer_FOC.hpp
 * @author your name (you@domain.com)
 * @brief 系统函数包含：初始化函数，foc启动时校准函数，系统运行函数
 *        计算函数包含：坐标系变换函数，滤波函数
 *        Loop中包含：环路运算函数
 * @version 0.1
 * @date 2023-10-19
 * 
 * @copyright Copyright (c) 2023
 * 
 */


#ifndef __CANCER_FOC_HPP__
#define __CANCER_FOC_HPP__

#include "task.hpp"
#include "SVPWM.hpp"
#include "SEGGER_RTT.h"
#include "Cancer_FOC_Defines.hpp"
#include "AS5048a.h"

struct FOC_Init_TypeDef
{
//SVPWM相关
    float SVPWM_Ts;
    float SVPWM_Vdc;
//外设接口
    volatile uint32_t *ccr1;
    volatile uint32_t *ccr2;
    volatile uint32_t *ccr3;
#if TIM_TYPE == TIM
    TIM_HandleTypeDef *htim;
#elif TIM_TYPE == HRTIM
    HRTIM_HandleTypeDef *hrtim;
#endif
    ADC_HandleTypeDef *hadc;
    UART_HandleTypeDef *huart;

//电机固有参数
    uint8_t Pole_Pairs;
};

enum FOC_Loop_State
{
    FOC_CURRENT_LOOP,
    FOC_SPEED_LOOP,
    FOC_POS_LOOP,
};


class MY_FOC
{
private:
//ADC 模块
    uint16_t            raw_ADC_Value[3];
    uint32_t            raw_ADC_Offset[3];
    int                 raw_Current_A;
    int                 raw_Current_B;
    int                 raw_Current_C;
    float               Current[3];
    ADC_HandleTypeDef*  hadc;

//数学运算模块
//真实的 alpha beta轴
    float Current_Clark_alpha;
    float Current_Clark_beta;
//真实的 Q 轴
    float Current_Park_D;
    float Current_Park_Q;
//闭环控制后输出的电压值
    float Park_D[2];//Park变换矩阵
    float Voltage_D;
    float Voltage_Q;
//这两组值应当输入SVPWM模块
    float Voltage_RevPark_alpha;
    float Voltage_RevPark_beta;
    
//定时器接口
    volatile uint32_t *ccr1;
    volatile uint32_t *ccr2;
    volatile uint32_t *ccr3;

#if TIM_TYPE == TIM
    TIM_HandleTypeDef *htim;
#elif TIM_TYPE == HRTIM
    HRTIM_HandleTypeDef *hhrtim;
#endif

    UART_HandleTypeDef *huart;
    uint8_t uart_rx_mem;
    uint8_t uart_rx_buffer[128];

//低通滤波
    float a;
    int raw_ADC_after_fliter[3][2];
//编码器传入的角度
    float theta_Park;
    float theta;
    float theta_offset;
    uint16_t raw_angle;
//SVPWM
    SVPWM svpwm;
    float duty[3];
//PID
    Pos_PID_t I_d;
    Pos_PID_t I_q;
    Pos_PID_t Speed_Control;
    Pos_PID_t Position_Control;

    float Iq_REF;
    float Speed_REF;
    float Pos_REF;

//固有参数
    uint8_t Pole_Pairs;
    uint8_t Vdc;

//输入ADC校准标志位
    bool ADC_Calibration_Flag;

//电机转速
    float speed;

    enum FOC_Loop_State FOC_Loop_Select;

//计算部分函数
    void FOC_Cacu_Clark_Park(float Current_A,float Current_B,float Current_C);
    void FOC_Cacu_RevPark();
    void FOC_Cacu_First_order_low_pass_filter();

//ADC部分函数
    bool FOC_Get_ADC_Offset();
    void FOC_ADC_Init();
    
//角度相关函数
    void FOC_Angle_Get_Speed();
    void FOC_Angle_Calibrate_the_electrical_angle();
    void FOC_Angle_GetAngle();

//sys接口函数
    void FOC_Sys_ChangeDuty(float* duty);
    void FOC_Sys_TIMER_Init();
    void FOC_Sys_PWM_Switch(bool onoff);
    void FOC_Sys_UART_Init();
    void FOC_Sys_UART_Command();

//loop函数
    void FOC_Loop_Speed(float Speed_Ref);
    void FOC_Loop_Current(float I_q_Ref);
    void FOC_Loop_Position(float Pos_Ref);

public:

//构造函数
    MY_FOC(FOC_Init_TypeDef Init_Struct);
    ~MY_FOC();

    void FOC_Sys_Init();//系统初始化 
    void FOC_Sys_Print_Vari();//系统打印调试函数

    void FOC_ADC_Program();
    //开环测试
    void FOC_lab_OpenTest();
    void FOC_lab_CurrentLoop();

    void FOC_LoopRun();

};

#endif
