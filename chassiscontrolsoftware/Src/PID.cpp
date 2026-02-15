/**
 * @file PID.cpp
 * @author gjc
 * @brief
 * @version 0.1
 * @date 2023-12-05
 *
 * @copyright USTC-RoboWalker (c) 2023
 *
 */

/* Includes ------------------------------------------------------------------*/
#include "PID.h"
#include "string.h"
#include "../Inc/math_process.h"
/* Private macros ------------------------------------------------------------*/

/* Private types -------------------------------------------------------------*/

/* Private variables ---------------------------------------------------------*/

/* Private function declarations ---------------------------------------------*/

/* Function prototypes -------------------------------------------------------*/

/**
 * @brief 构造函数
 * @note
 *
 * @param
 * @return
 */
Class_PID::Class_PID()
{
    /* 读变量 */
    Out = 0.0f;                            /*!< 输出值 */

    /* 写变量 */
    K_P = 0.0f;                            /*!< P参数 */
    K_I = 0.0f;                            /*!< I参数 */
    K_D = 0.0f;                            /*!< D参数 */
    K_F = 0.0f;                            /*!< 前馈参数 */
    I_Out_Max = 0;                         /*!< 积分限幅, 0为不限制 */
    Out_Max = 0;                           /*!< 输出限幅, 0为不限制 */
    I_Variable_Speed_A = 0.0f;             /*!< 变速积分定速内段阈值, 0为不限制 */
    I_Variable_Speed_B = 0.0f;             /*!< 变速积分变速区间, 0为不限制 */
    I_Separate_Threshold = 0.0f;           /*!< 积分分离阈值，需为正数, 0为不限制 */
    Target = 0.0f;                         /*!< 目标值 */
    Now = 0.0f;                            /*!< 当前值 */
    Dead_Zone = 0.0f;                      /*!< 死区, Error在其绝对值内不输出 */
    D_First = PID_D_First_DISABLE;         /*!< 微分先行 */
    Accurate_D = PID_Accurate_D_DISABLE;   /*!< 准确微分 */

    /* 读写变量 */
    Integral_Error = 0.0f;                 /*!< 积分值 */

    /* 内部变量 */
    Pre_Now = 0.0f;                        /*!< 之前的当前值 */
    Pre_Target = 0.0f;                     /*!< 之前的目标值 */
    Pre_Out = 0.0f;                        /*!< 之前的输出值 */
    Pre_Error = 0.0f;                      /*!< 前向误差 */
}


/**
 * @brief PID初始化
 *
 * @param __K_P         P参数
 * @param __K_I         I参数
 * @param __K_D         D参数
 * @param __K_F         前馈参数
 * @param __I_Out_Max   积分限幅
 * @param __Out_Max     输出限幅
 * @param __D_T         控制周期
 */
void Class_PID::Init(float __K_P, float __K_I, float __K_D, float __K_F, float __I_Out_Max,
    float __Out_Max, float __Dead_Zone,float __I_Variable_Speed_A, float __I_Variable_Speed_B,
    float __I_Separate_Threshold,uint8_t __D_First,uint8_t __Accurate_D)
{
K_P = __K_P;
K_I = __K_I;
K_D = __K_D;
K_F = __K_F;
I_Out_Max = __I_Out_Max;
Out_Max = __Out_Max;
Dead_Zone = __Dead_Zone;
I_Variable_Speed_A = __I_Variable_Speed_A;
I_Variable_Speed_B = __I_Variable_Speed_B;
I_Separate_Threshold = __I_Separate_Threshold;
D_First = __D_First;
Accurate_D = __Accurate_D;
Accurate_diff_items = 0.0f; 
}

/**
 * @brief PID清零
 * @note
 *
 * @param
 * @return
 */
void Class_PID::Clear(void)
{
    Integral_Error = 0.0f;                 /*!< 积分值 */
    Pre_Now = 0.0f;                        /*!< 之前的当前值 */
    Pre_Target = 0.0f;                     /*!< 之前的目标值 */
    Pre_Out = 0.0f;                        /*!< 之前的输出值 */
    Pre_Error = 0.0f;                      /*!< 前向误差 */
    Out = 0.0f;                            /*!< 输出值 */
    Target = 0.0f;                         /*!< 目标值 */
    Now = 0.0f;                            /*!< 当前值 */
    Accurate_diff_items = 0.0f;           /*!< 准确微分项 */

}

/**
 * @brief PID计算
 * @note 不考虑dt,dt直接体现在了K_P,K_I,K_D上
 *
 * @param
 * @return
 */
void Class_PID::Calculate()
{
    float p_out = 0.0f;     // P输出
    float i_out = 0.0f;     // I输出
    float d_out = 0.0f;     // D输出
    float f_out = 0.0f;     // F输出（前馈）
    float error;            //误差
    float abs_error;        //绝对值误差
    float speed_ratio;      //线性变速积分

    /* 计算误差 */
    error = Target - Now;
    abs_error = Math_Abs(error);

    /* 判断死区 */
    if(abs_error < Dead_Zone)
    {
        Target = Now;
        error = 0.0f;
        abs_error = 0.0f;
    }

    /* 计算p项 */
    p_out = K_P * error;

    /* 计算i项 */
    if(I_Variable_Speed_A == 0.0f && I_Variable_Speed_B == 0.0f)
    {
        /* 非变速积分 */
        speed_ratio = 1.0f;
    }
    else
    {
        /* 变速积分 */
        if(abs_error <= I_Variable_Speed_A)
        {
            speed_ratio = 1.0f;
        }
        else if(I_Variable_Speed_A < abs_error && abs_error < I_Variable_Speed_B)
        {
            speed_ratio = (I_Variable_Speed_B - abs_error) / (I_Variable_Speed_B - I_Variable_Speed_A);
        }
        if(abs_error >= I_Variable_Speed_B)
        {
            speed_ratio = 0.0f;
        }
    }
    /* 积分限幅 */
    if(I_Out_Max != 0.0f)
    {
        Integral_Error = limit_range_function(Integral_Error, -I_Out_Max / K_I, I_Out_Max / K_I);
    }
    if(I_Separate_Threshold == 0.0f)
    {
        /* 无积分分离 */
        Integral_Error += speed_ratio * error;
        i_out = K_I * Integral_Error;
    }
    else
    {
        /* 积分分离 */
        if(abs_error < I_Separate_Threshold)
        {
            Integral_Error += speed_ratio * error;
            i_out = K_I * Integral_Error;
        }
        else
        {
            Integral_Error = 0.0f;
            i_out = 0.0f;
        }
    }

    /* 计算d项 */
    if(Accurate_D == PID_Accurate_D_ENABLE)
    {
        /* 具有准确的微分项，不需要差分 */
        d_out = K_D * Accurate_diff_items;
    }
    else
    {
        if(D_First == PID_D_First_ENABLE)
        {
            /* 微分先行 */
            d_out = K_D * (Out - Pre_Out);
        }
        else
        {
            /* 无微分先行 */
            d_out = K_D * (error - Pre_Error);
        }
    }
    
    /* 计算前馈 */
    f_out = (Target - Pre_Target) * K_F;

    /* 计算输出 */
    Out = p_out + i_out + d_out + f_out;
    /* 输出限幅 */
    if(Out_Max != 0.0f)
    {
        Out = limit_range_function(Out, -Out_Max, Out_Max);
    }

    /* 更新数据 */
    Pre_Now = Now;
    Pre_Target = Target;
    Pre_Out = Out;
    Pre_Error = error;
}

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/
/**
 * @brief 
 * @note
 *
 * @param
 * @return
 */
