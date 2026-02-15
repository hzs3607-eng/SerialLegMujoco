/**
 * @file PID.h
 * @author gjc
 * @brief
 * @version 0.1
 * @date 2023-12-05
 *
 * @copyright USTC-RoboWalker (c) 2023
 *
 */
#ifndef PID_H
#define PID_H


#ifdef __cplusplus       
extern "C"{                      
#endif

/* Includes ------------------------------------------------------------------*/
#include <cstdint>
#include <cstring>
/* Exported macros -----------------------------------------------------------*/
//微分先行
#define PID_D_First_DISABLE 0
#define PID_D_First_ENABLE 1
//准确微分项
#define PID_Accurate_D_DISABLE 0
#define PID_Accurate_D_ENABLE 1
/* Exported types ------------------------------------------------------------*/
/* PID控制器 */
class Class_PID
{
public:
    /* 函数 */
    Class_PID();//构造函数
    void Init(float __K_P, float __K_I, float __K_D, float __K_F = 0.0f, float __I_Out_Max = 0.0f,
              float __Out_Max = 0.0f, float __Dead_Zone = 0.0f,
              float __I_Variable_Speed_A = 0.0f, float __I_Variable_Speed_B = 0.0f, float __I_Separate_Threshold = 0.0f,
              uint8_t __D_First = PID_D_First_DISABLE,uint8_t __Accurate_D = PID_Accurate_D_DISABLE);
    void Clear(void); // 清除积分项等内容
    void Calculate();

    inline float Get_Integral_Error();
    inline float Get_Out();
    inline void Set_K_P(float __K_P);
    inline void Set_K_I(float __K_I);
    inline void Set_K_D(float __K_D);
    inline void Set_K_F(float __K_F);
    inline void Set_I_Out_Max(float __I_Out_Max);
    inline void Set_Out_Max(float __Out_Max);
    inline void Set_I_Variable_Speed_A(float __Variable_Speed_I_A);
    inline void Set_I_Variable_Speed_B(float __Variable_Speed_I_B);
    inline void Set_I_Separate_Threshold(float __I_Separate_Threshold);
    inline void Set_Target(float __Target);
    inline void Set_Now(float __Now);
    inline void Set_Integral_Error(float __Integral_Error);
    inline void Set_Dead_Zone(float __Dead_Zone);
    inline void Set_Accurate_D(float __Accurate_diff_items);
    inline void Enable_Accurate_D(void);
protected:
    /* 读变量 */
    float Out;                       /*!< 输出值 */

    /* 写变量 */
    float K_P;                       /*!< P参数 */
    float K_I;                       /*!< I参数 */
    float K_D;                       /*!< D参数 */
    float K_F;                       /*!< 前馈参数 */
    float I_Out_Max;                 /*!< 积分限幅, 0为不限制 */
    float Out_Max;                   /*!< 输出限幅, 0为不限制 */
    float I_Variable_Speed_A;        /*!< 变速积分定速内段阈值, 0为不限制 */
    float I_Variable_Speed_B;        /*!< 变速积分变速区间, 0为不限制 */
    float I_Separate_Threshold;      /*!< 积分分离阈值，需为正数, 0为不限制 */
    float Target;                    /*!< 目标值 */
    float Now;                       /*!< 当前值 */
    float Dead_Zone;                 /*!< 死区, Error在其绝对值内不输出 */
    uint8_t D_First;                 /*!< 微分先行 */
    uint8_t Accurate_D;              /*!< 准确微分项 */
    float Accurate_diff_items;       /*!< 准确微分项 */

    /* 读写变量 */
    float Integral_Error;            /*!< 积分值 */

    /* 内部变量 */
    float Pre_Now;                   /*!< 之前的当前值 */
    float Pre_Target;                /*!< 之前的目标值 */
    float Pre_Out;                   /*!< 之前的输出值 */
    float Pre_Error;                 /*!< 前向误差 */
};

/* Exported variables --------------------------------------------------------*/

/* Exported function declarations --------------------------------------------*/
/* 接口函数定义 --------------------------------------------------------------------*/
/**
 * @brief 获取输出值
 *
 * @return float 输出值
 */
float Class_PID::Get_Integral_Error()
{
    return (Integral_Error);
}

/**
 * @brief 获取输出值
 *
 * @return float 输出值
 */
float Class_PID::Get_Out()
{
    return (Out);
}

/**
 * @brief 设定PID的P
 *
 * @param __K_P PID的P
 */
void Class_PID::Set_K_P(float __K_P)
{
    K_P = __K_P;
}

/**
 * @brief 设定PID的I
 *
 * @param __K_I PID的I
 */
void Class_PID::Set_K_I(float __K_I)
{
    K_I = __K_I;
}

/**
 * @brief 设定PID的D
 *
 * @param __K_D PID的D
 */
void Class_PID::Set_K_D(float __K_D)
{
    K_D = __K_D;
}

/**
 * @brief 设定前馈
 *
 * @param __K_D 前馈
 */
void Class_PID::Set_K_F(float __K_F)
{
    K_F = __K_F;
}

/**
 * @brief 设定积分限幅, 0为不限制
 *
 * @param __I_Out_Max 积分限幅, 0为不限制
 */
void Class_PID::Set_I_Out_Max(float __I_Out_Max)
{
    I_Out_Max = __I_Out_Max;
}

/**
 * @brief 设定输出限幅, 0为不限制
 *
 * @param __Out_Max 输出限幅, 0为不限制
 */
void Class_PID::Set_Out_Max(float __Out_Max)
{
    Out_Max = __Out_Max;
}

/**
 * @brief 设定定速内段阈值, 0为不限制
 *
 * @param __I_Variable_Speed_A 定速内段阈值, 0为不限制
 */
void Class_PID::Set_I_Variable_Speed_A(float __I_Variable_Speed_A)
{
    I_Variable_Speed_A = __I_Variable_Speed_A;
}

/**
 * @brief 设定变速区间, 0为不限制
 *
 * @param __I_Variable_Speed_B 变速区间, 0为不限制
 */
void Class_PID::Set_I_Variable_Speed_B(float __I_Variable_Speed_B)
{
    I_Variable_Speed_B = __I_Variable_Speed_B;
}

/**
 * @brief 设定积分分离阈值，需为正数, 0为不限制
 *
 * @param __I_Separate_Threshold 积分分离阈值，需为正数, 0为不限制
 */
void Class_PID::Set_I_Separate_Threshold(float __I_Separate_Threshold)
{
    I_Separate_Threshold = __I_Separate_Threshold;
}

/**
 * @brief 设定目标值
 *
 * @param __Target 目标值
 */
void Class_PID::Set_Target(float __Target)
{
    Target = __Target;
}

/**
 * @brief 设定当前值
 *
 * @param __Now 当前值
 */
void Class_PID::Set_Now(float __Now)
{
    Now = __Now;
}

/**
 * @brief 设定积分, 一般用于积分清零
 *
 * @param __Set_Integral_Error 积分值
 */
void Class_PID::Set_Integral_Error(float __Integral_Error)
{
    Integral_Error = __Integral_Error;
}

/**
 * @brief 设定死区, Error在其绝对值内不输出
 *
 * @param __Dead_Zone 死区, Error在其绝对值内不输出
 */
void Class_PID::Set_Dead_Zone(float __Dead_Zone)
{
    Dead_Zone = __Dead_Zone;
}

/**
 * @brief 设定准确微分项
 *
 * @param __Accurate_D 准确微分项
 */
void Class_PID::Set_Accurate_D(float __Accurate_diff_items)
{
    Accurate_diff_items = __Accurate_diff_items;
}

/**
 * @brief 设定准确微分项开关
 *
 * @param __Accurate_D 准确微分项开关
 */
void Class_PID::Enable_Accurate_D(void)
{
    Accurate_D = PID_Accurate_D_ENABLE;
}


#ifdef __cplusplus      
}                                          
#endif 

#endif

/************************ COPYRIGHT(C) USTC-ROBOWALKER **************************/

