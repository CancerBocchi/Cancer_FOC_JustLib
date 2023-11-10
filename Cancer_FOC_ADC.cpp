#include "Cancer_FOC.hpp"

/**
 * @brief 获取采样电路偏置函数
 * 
 * @return true 
 * @return false 
 */
bool MY_FOC::FOC_Get_ADC_Offset()
{
    static uint16_t Flag = 0;
    if(Flag < 1000)
    {
        this->raw_ADC_Offset[0] += this->raw_ADC_Value[0];
        this->raw_ADC_Offset[1] += this->raw_ADC_Value[1];
        this->raw_ADC_Offset[2] += this->raw_ADC_Value[2];
        Flag++;
        return false;
    }
    else if(Flag >=1000)
    {
        if(!ADC_Calibration_Flag)
        {
            this->raw_ADC_Offset[0] = (int)((float)this->raw_ADC_Offset[0] / 1000.0f);
            this->raw_ADC_Offset[1] = (int)((float)this->raw_ADC_Offset[1] / 1000.0f);
            this->raw_ADC_Offset[2] = (int)((float)this->raw_ADC_Offset[2] / 1000.0f);
            ADC_Calibration_Flag = true;
            FOC_Sys_PWM_Switch(true);
        }
        return true;
    }
    return false;
}
/**
 * @brief ADC中断运行函数
 * 
 * 滤波->得到实际电流值->坐标系变换
 * 
 */
void MY_FOC::FOC_ADC_Program()
{

#if ADC_METHOD == ADC_JEC
    raw_ADC_Value[0] = (uint16_t)HAL_ADCEx_InjectedGetValue(hadc,ADC_INJECTED_RANK_1);
    raw_ADC_Value[1] = (uint16_t)HAL_ADCEx_InjectedGetValue(hadc,ADC_INJECTED_RANK_2);
    raw_ADC_Value[2] = (uint16_t)HAL_ADCEx_InjectedGetValue(hadc,ADC_INJECTED_RANK_3);
#endif

    if(FOC_Get_ADC_Offset())
    {
        FOC_Cacu_First_order_low_pass_filter();

        raw_Current_A = (raw_ADC_after_fliter[0][1] - raw_ADC_Offset[0]);
        raw_Current_B = (raw_ADC_after_fliter[1][1] - raw_ADC_Offset[1]);
        raw_Current_C = (raw_ADC_after_fliter[2][1] - raw_ADC_Offset[2]);

        raw_Current_C = raw_Current_C - raw_Current_A - raw_Current_B;

        // Current[0] = (float)raw_Current_A*3.0f/4096.0f;
        // Current[1] = (float)raw_Current_B*3.0f/4096.0f;
        // Current[2] = (float)raw_Current_C*3.0f/4096.0f;

    }
}

void MY_FOC::FOC_ADC_Init()
{
#ifdef STM32G474
    HAL_ADCEx_Calibration_Start(&hadc1, ADC_SINGLE_ENDED);
    HAL_Delay(1);
#endif

#if ADC_METHOD == ADC_REG
    HAL_ADC_Start_DMA(&hadc1,(uint32_t*)this->raw_ADC_Value,3);
#elif ADC_METHOD == ADC_JEC

    HAL_ADCEx_InjectedStart_IT(&hadc1);
    HAL_TIM_PWM_Start(this->htim,TIM_CHANNEL_4);

#endif

}
