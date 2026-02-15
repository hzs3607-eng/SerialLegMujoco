#pragma once

/* Includes ------------------------------------------------------------------*/
#include "PID.h"
#include "chassis_controller_params.h"
#include "chassis_controller_utils.h"
#include "math_process.h"
#include <cstring>                          // C字符串操作库
/* Classes --------------------------------------------------------------------*/
/**
 * @brief 底盘控制器执行器类
 * @details 实现机器人底盘的力矩控制，包含LQR控制算法、虚拟模型控制等
 */
class ChassisControllerActuator
{
    private:
        float M;                             // 机器人质量
        float Width;                         // 机器人宽度

        float Matrix_K[4][10];               // 三维动力学LQR控制矩阵

        Class_PID pid_mean_l0;               // 腿长均值PID控制器
        Class_PID pid_diff_l0;               // 腿长差值PID控制器
        float leg_compensate_torque_left;     // 左侧腿部力矩补偿
        float leg_compensate_torque_right;    // 右侧腿部力矩补偿

        float g_l, g_r;                      // 左右腿重力分量
        
        VMC vmc_l, vmc_r;                    // 左右腿的虚拟力矩

        float F_l_last, F_r_last;             // 上次的支持力

        ChassisControllerContext* context_;   // 底盘控制器上下文指针
        
        // Functions
        /**
         * @brief LQR控制矩阵计算
         * @details 计算用于LQR控制器的反馈矩阵
         */
        void kMatrixCalculation(void);
        
        /**
         * @brief 等效重力加速度计算
         * @details 计算机器人在倾斜状态下的等效重力分量
         */
        void equivalentGravityCalculation(void);
        
        /**
         * @brief LQR控制算法
         * @details 实现线性二次调节器控制算法
         */
        void calculateLQRTorque(void);
        
        /**
         * @brief 支持力计算
         * @details 计算机器人腿部对地面的支持力
         */
        void calculateSupportForce(void);
        
        /**
         * @brief 虚拟力转换至电机输出转矩
         * @details 将虚拟模型控制输出转换为实际电机转矩
         */
        void convertToMotorTorque(void);
        
        /**
         * @brief 获取功率限制因子
         * @param[in] left_wheel_torque 左轮力矩
         * @param[in] right_wheel_torque 右轮力矩
         * @return 返回功率限制因子
         * @details 根据轮子力矩计算功率限制因子
         */
        float getPowerLimitFactor(float left_wheel_torque, float right_wheel_torque);

    public:
        /**
         * @brief 构造函数，赋值
         * @param[in] context 底盘控制器上下文指针
         * @details 初始化执行器并关联控制上下文
         */
        explicit ChassisControllerActuator(ChassisControllerContext* context);
        
        /**
         * @brief 重启机器人状态
         * @details 将执行器状态重置为初始状态
         */
        void reset(void);
        
        /**
         * @brief 计算控制量
         * @details 执行主要的控制算法更新逻辑
         */
        void update(void);
};