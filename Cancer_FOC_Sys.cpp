#include "Cancer_FOC.hpp"


MY_FOC::MY_FOC(FOC_Init_TypeDef Init_Struct):svpwm(Init_Struct.SVPWM_Ts,Init_Struct.SVPWM_Vdc)
{
    this->ccr1 = Init_Struct.ccr1;
    this->ccr2 = Init_Struct.ccr2;
    this->ccr3 = Init_Struct.ccr3;

    this->Vdc  = Init_Struct.SVPWM_Vdc;
    this->Pole_Pairs = Init_Struct.Pole_Pairs;
    this->hadc = Init_Struct.hadc;

#if TIM_TYPE == TIM
    this->htim = Init_Struct.htim;
#elif TIM_TYPE == HRTIM
    this->hhrtim = Init_Struct.hrtim;
#endif

    this->huart = huart;

    raw_Current_C = 0;
    raw_Current_B = 0;
    raw_Current_A = 0;
    Current_Clark_alpha = 0.0f;
    Current_Clark_beta  = 0.0f;
    Current_Park_D = 0.0f;
    Current_Park_Q = 0.0f;
    Voltage_D = 0.0f;
    Voltage_Q = 0.0f;
    Voltage_RevPark_alpha = 0.0f;
    Voltage_RevPark_beta =  0.0f;
    ADC_Calibration_Flag = false;
    a = 0.80f;

    theta_Park = 0;

    Iq_REF = 100.0f;
    Speed_REF = 01.0f;
    Pos_REF = PI*3/2;


    Pos_PID_Init(&I_q,0.01f,0.003f,0);
    I_q.Output_Max = 16.0*Sqrt3;
    I_q.Output_Min = -16.0*Sqrt3;
    I_q.Value_I_Max = 2000.0f;

    Pos_PID_Init(&I_d,0.01f,0.003f,0);
    I_d.Output_Max = 6.0f;
    I_d.Output_Min = -6.0f;
    I_d.Value_I_Max = 2000.0f;
    I_d.Ref = 0.0f;

    Pos_PID_Init(&Speed_Control,100.0f,5.0f,0);
    Speed_Control.Output_Max = 700.0f;
    Speed_Control.Output_Min = -700.0f;
    Speed_Control.Value_I_Max = 200.0f;

    Pos_PID_Init(&Position_Control,10.0f,0.0f,10.0f);
    Position_Control.Output_Max =  4.0f;
    Position_Control.Output_Min = -4.0f;
    Position_Control.Value_I_Max = 2000.0f;

    FOC_Loop_Select = FOC_CURRENT_LOOP;
}

MY_FOC::~MY_FOC()
{
    
}

/**
 * @brief 系统初始化函数
 *          顺序：打开定时器pwm->编码器初始化->校准电角度->ADC初始化
 *          注意：电角度校准必须放到ADC初始化前面，否则ADC偏置获取会出错
 */
void MY_FOC::FOC_Sys_Init()
{
    //初始化串口调试
    FOC_Sys_UART_Init();
    //初始化编码器
    FOC_Sys_Encoder_Init();
    //初始化定时器
    FOC_Sys_TIMER_Init();
    //打开PWM
    FOC_Sys_PWM_Switch(true);
    //校准电角度
    FOC_Angle_Calibrate_the_electrical_angle();
    //关闭PWM 防止干扰ADC校准
    FOC_Sys_PWM_Switch(false);
    HAL_Delay(1);
    //ADC初始化
    FOC_ADC_Init(); 
}

void MY_FOC::FOC_Sys_Encoder_Init()
{
    AS5048a_Init();
}


void MY_FOC::FOC_Sys_ChangeDuty(float* duty)
{
#if TIM_TYPE == TIM
        htim->Instance->CCR3 = duty[0] * (htim->Instance->ARR+1);
        htim->Instance->CCR2 = duty[1] * (htim->Instance->ARR+1);
        htim->Instance->CCR1 = duty[2] * (htim->Instance->ARR+1);
#elif TIM_TYPE == HRTIM
        *ccr1 = duty[0] * hhrtim->Instance->sTimerxRegs[0].PERxR;
        *ccr2 = duty[1] * hhrtim->Instance->sTimerxRegs[1].PERxR;
        *ccr3 = duty[2] * hhrtim->Instance->sTimerxRegs[4].PERxR;
#endif  
}

void MY_FOC::FOC_Sys_TIMER_Init()
{
#if TIM_TYPE == TIM
    HAL_TIM_Base_Start_IT(this->htim);
    
#elif TIM_TYPE == HRTIM

    HAL_HRTIM_WaveformCounterStart_IT(hhrtim,HRTIM_TIMERID_TIMER_A|
                                            HRTIM_TIMERID_TIMER_B|
                                            HRTIM_TIMERID_TIMER_E);

#endif
}

void MY_FOC::FOC_Sys_PWM_Switch(bool onoff)
{
#if TIM_TYPE == HRTIM
    if(onoff)
    {
        HAL_HRTIM_WaveformOutputStart(hhrtim,HRTIM_OUTPUT_TA1
                                                |HRTIM_OUTPUT_TA2
                                                |HRTIM_OUTPUT_TB1
                                                |HRTIM_OUTPUT_TB2
                                                |HRTIM_OUTPUT_TE1
                                                |HRTIM_OUTPUT_TE2);
    }
    else
    {
        HAL_HRTIM_WaveformOutputStop(hhrtim,HRTIM_OUTPUT_TA1
                                                |HRTIM_OUTPUT_TA2
                                                |HRTIM_OUTPUT_TB1
                                                |HRTIM_OUTPUT_TB2
                                                |HRTIM_OUTPUT_TE1
                                                |HRTIM_OUTPUT_TE2);
    }
#elif TIM_TYPE == TIM
    if(onoff)
    {
        HAL_TIM_PWM_Start(this->htim,TIM_CHANNEL_1);
        HAL_TIMEx_PWMN_Start(this->htim,TIM_CHANNEL_1);
        HAL_TIM_PWM_Start(this->htim,TIM_CHANNEL_2);
        HAL_TIMEx_PWMN_Start(this->htim,TIM_CHANNEL_2);
        HAL_TIM_PWM_Start(this->htim,TIM_CHANNEL_3);
        HAL_TIMEx_PWMN_Start(this->htim,TIM_CHANNEL_3);
    }
    else
    {
        HAL_TIM_PWM_Stop(this->htim,TIM_CHANNEL_1);
        HAL_TIMEx_PWMN_Stop(this->htim,TIM_CHANNEL_1);
        HAL_TIM_PWM_Stop(this->htim,TIM_CHANNEL_2);
        HAL_TIMEx_PWMN_Stop(this->htim,TIM_CHANNEL_2);
        HAL_TIM_PWM_Stop(this->htim,TIM_CHANNEL_3);
        HAL_TIMEx_PWMN_Stop(this->htim,TIM_CHANNEL_3);

    }
#endif                                                                                     

}

void MY_FOC::FOC_Sys_UART_Init()
{
    HAL_UART_Receive_IT(huart,&uart_rx_mem,1);
}

void MY_FOC::FOC_Sys_UART_Command()
{

}
/**
 * @brief UART 接口 实时改变期望
 * 
 * @param code 
 */
void MY_FOC::FOC_Sys_UartDebug(uint8_t code)
{
    switch (code)
    {
        case '1':
            Pos_REF = 0;
            Block_UART_printf(huart,"Angle Set 0\n");
            break;
        case '2':
            Pos_REF = PI/3;
            Block_UART_printf(huart,"Angle Set 60\n");
            break;
        case '3':
            Pos_REF = PI*2/3;
            Block_UART_printf(huart,"Angle Set 120\n");
            break;
        case '4':
            Pos_REF = PI;
            Block_UART_printf(huart,"Angle Set 180\n");
            break;
        case '5':
            Pos_REF = PI*4/3;
            Block_UART_printf(huart,"Angle Set 240\n");
            break;
        case '6':
            Pos_REF = PI*5/3;
            Block_UART_printf(huart,"Angle Set 300\n");
            break;
    }
}

bool MY_FOC::FOC_Sys_ChangeREF(float value, uint8_t target)
{
    switch (target)
    {
    case 'I':
        Iq_REF = value;
        DMA_UART_printf(huart,"CancerFOC:Set I_ref = %.3f\n",value);
        break;
    case 'S':
        Speed_REF = value;
        DMA_UART_printf(huart,"CancerFOC:Set Speed_ref = %.3f\n",value);
        break;
    case 'P':
        Pos_REF = value;
        DMA_UART_printf(huart,"CancerFOC:Set Pos_ref = %.3f\n",value);
        break;  

    default:
        DMA_UART_printf(huart,"CancerFOC:Target Error\n");
        return false;
        break;
    }
    return true;
}

bool MY_FOC::FOC_Sys_ChangeMode(uint8_t mode)
{
    switch (mode)
    {
    case 'I':
        FOC_Loop_Select = FOC_CURRENT_LOOP;
        DMA_UART_printf(huart,"CancerFOC:Current Loop Selected\n");
        break;
    case 'S':
        FOC_Loop_Select = FOC_SPEED_LOOP;
        DMA_UART_printf(huart,"CancerFOC:Speed Loop Selected\n");
        break;
    case 'P':
        FOC_Loop_Select = FOC_POS_LOOP;
        DMA_UART_printf(huart,"CancerFOC:Pos Loop Selected\n");
        break;
    
    default:
        break;
    }
    return false;
}

void MY_FOC::FOC_Sys_Print_Vari()
{
#if PRINT_WAY == RTT_PRINT
    #if PRINT_TARGET == PRINT_RAW_CURRENT
        SEGGER_RTT_printf(0,"%d\n",raw_Current_A);
    #elif PRINT_TARGET == PRINT_CURRENT_PARK
        SEGGER_RTT_printf(0,"%d,%d\n", (int)(Current_Park_D),(int)(Current_Park_Q));
    #elif PRINT_TARGET == PRINT_CURRENT_CLARK
        SEGGER_RTT_printf(0,"%d,%d\n", (int)(Current_Clark_alpha*1000),(int)(Current_Clark_beta*1000));
    #elif PRINT_TARGET == PRINT_VOLTAGE_DQ
        SEGGER_RTT_printf(0,"%d,%d\n", (int)(Voltage_D*1000),(int)(Voltage_Q*1000));
    #elif PRINT_TARGET == PRINT_VOLTAGE_AB
        SEGGER_RTT_printf(0,"%d,%d\n", (int)(Voltage_RevPark_alpha*1000),(int)(Voltage_RevPark_beta*1000));
    #elif PRINT_TARGET == PRINT_SPEED
        SEGGER_RTT_printf(0,"%d,%d\n", (int)(speed*1000),(int)(theta*1000));
    #elif PRINT_TARGET == PRINT_THETA_AND_OFFSET
        SEGGER_RTT_printf(0,"%d\n", (int)(theta*1000));
    #elif PRINT_TARGET == PRINT_RAW_ADC_VALUE
        SEGGER_RTT_printf(0,"%d\n", raw_ADC_Value[2]);
    #elif PRINT_TARGET == PRINT_FLITER_VALUE
        SEGGER_RTT_printf(0,"%d,%d\n", raw_ADC_after_fliter[0][1],raw_ADC_after_fliter[1][1]);
    #elif PRINT_TARGET == PRINT_THETA_PARK
        SEGGER_RTT_printf(0,"%d\n", (int)(theta_Park*1000.0f));
    #elif PRINT_TARGET == PRINT_DUTY 
        SEGGER_RTT_printf(0,"%d,%d\n", (int)(duty[1]*1000),(int)(duty[0]*1000));
    #elif PRINT_TARGET == PRINT_RAWANGLE
        SEGGER_RTT_printf(0,"%d\n", (int)(raw_angle));
    #elif PRINT_TARGET == PRINT_ELECANGLE
        SEGGER_RTT_printf(0,"%d\n", (int)(theta_Park*1000));
    #elif PRINT_TARGET == PRINT_PID_Q
        SEGGER_RTT_printf(0,"%d,%d,%d\n", (int)(Current_Park_D*100),(int)(Current_Park_Q*100),(int)(I_q.Value_I*100));
    #elif PRINT_TARGET == PRINT_PID_SPEED
        SEGGER_RTT_printf(0,"%d,%d,%d\n", (int)(I_q.Ref*1000),(int)(Speed_Control.Error*1000),(int)(speed*100));
    #elif PRINT_TARGET == PRINT_PID_POS
        SEGGER_RTT_printf(0,"%d,%d,%d\n", (int)(Speed_Control.Ref*1000),(int)(Position_Control.Error*1000),(int)(theta*100));
    #elif PRINT_TARGET == PRINT_LOOP_VALUE
        SEGGER_RTT_printf(0,"%d,%d,%d\n", (int)(theta*100),(int)(speed*100),(int)(Current_Park_Q*100));

    #endif

#elif PRINT_WAY == UART_PRINT

    #if PRINT_TARGET == PRINT_RAW_CURRENT
        DMA_UART_printf(&huart1,"%d,%d,%d\n",raw_Current_A,raw_Current_B,raw_Current_C);
    #elif PRINT_TARGET == PRINT_CURRENT_PARK
        DMA_UART_printf(&huart1,"%.2f\n",Current_Park_Q);
    #elif PRINT_TARGET == PRINT_CURRENT_CLARK
        DMA_UART_printf(&huart1,"%.3f,%.3f\n", Current_Clark_alpha,Current_Clark_beta);
    #elif PRINT_TARGET == PRINT_VOLTAGE_DQ
    DMA_UART_printf(&huart1,"%.2f,%.2f\n", Voltage_D,Voltage_Q);
    #elif PRINT_TARGET == PRINT_VOLTAGE_AB
        DMA_UART_printf(&huart1,"%d,%d\n", (int)(Voltage_RevPark_alpha*1000),(int)(Voltage_RevPark_beta*1000));
    #elif PRINT_TARGET == PRINT_SPEED
        DMA_UART_printf(&huart1,"%.2f\n", speed);
    #elif PRINT_TARGET == PRINT_THETA_AND_OFFSET
        DMA_UART_printf(&huart1,"%.3f\n", theta);
    #elif PRINT_TARGET == PRINT_RAW_ADC_VALUE
        DMA_UART_printf(&huart1,"%d,%d,%d\n", raw_ADC_Value[0],raw_ADC_Value[1],raw_ADC_Value[2]);
    #elif PRINT_TARGET == PRINT_FLITER_VALUE
        DMA_UART_printf(&huart1,"%d,%d\n", raw_ADC_after_fliter[0][1],raw_ADC_after_fliter[1][1]);
    #elif PRINT_TARGET == PRINT_THETA_PARK
        DMA_UART_printf(&huart1,"%.3f\n", theta_Park*1000.0f);
    #elif PRINT_TARGET == PRINT_DUTY 
        DMA_UART_printf(&huart1,"%.3f,%.3f\n", duty[1],duty[0]*1000);
    #elif PRINT_TARGET == PRINT_RAWANGLE
        DMA_UART_printf(&huart1,"%d\n", raw_angle);
    #elif PRINT_TARGET == PRINT_ELECANGLE
        DMA_UART_printf(&huart1,"%.3f\n", theta_Park);
    #elif PRINT_TARGET == PRINT_PID_Q
        DMA_UART_printf(&huart1,"%.1f,%.1f,%.1f\n", Voltage_Q,I_q.Error,I_q.Value_I);
    #endif

#endif
}
