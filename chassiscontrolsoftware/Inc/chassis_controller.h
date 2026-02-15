//
// Created by huzongsheng on 2026/1/22.
//
#pragma once

/* Includes ------------------------------------------------------------------*/
#include "chassis_controller_estimator.h"
#include "chassis_controller_fsm.h"
#include "chassis_controller_actuator.h"


#include <cstring>                            // C字符串操作库

/* Classes -------------------------------------------------------------------*/
/**
 * @brief 底盘控制器主类
 * @details 整合了状态估计、状态机控制和执行器功能，实现完整的机器人底盘控制
 */
class ChassisSoftWareControl
{
    private:
        ChassisControllerEstimator estimator_;  // 状态估计器实例
        ChassisControllerFSM fsm_;              // 有限状态机实例
        ChassisControllerActuator actuator_;    // 执行器实例
    public:
        ChassisControllerContext context_;      // 控制器上下文
        
        /**
         * @brief 构造函数
         * @details 初始化底盘控制器各组件
         */
        ChassisSoftWareControl(void);
        
        /**
         * @brief 重置控制器
         * @details 将控制器恢复到初始状态
         */
        void reset(void);
        
        /**
         * @brief 更新控制器
         * @details 执行控制器的主要更新逻辑
         */
        void update(void);




        /**
         * @brief 获取期望腿长
         * @return 返回期望腿长值
         */
        float getL0Exp(void);
        
        /**
         * @brief 获取底盘X方向速度
         * @return 返回底盘X方向速度
         */
        float getChassisVX(void);
        
        /**
         * @brief 获取底盘角速度
         * @return 返回底盘绕Z轴角速度
         */
        float getChassisVW(void);
        
        /**
         * @brief 获取底盘俯仰角
         * @return 返回底盘俯仰角
         */
        float getChassisPitch(void);
        
        /**
         * @brief 获取电机力矩
         * @return 返回电机力矩结构体
         */
        MotorTargetTorque getMotorTorque(void);
        
        /**
         * @brief 获取电机位置
         * @return 返回电机位置结构体
         */
        MotorTargetPos getMotorPos(void);
        
        /**
         * @brief 获取电机速度
         * @return 返回电机速度结构体
         */
        MotorTargetVel getMotorVel(void);
        
        /**
         * @brief 获取调试信息
         * @return 返回调试信息结构体
         */
        DebugInfo getDebugInfo(void);
        
        /**
         * @brief 速度限制
         * @param[in,out] raw_vx 输入输出的X方向速度
         * @param[in,out] raw_vw 输入输出的角速度
         * @details 对输入速度进行限制处理
         */
        void velLimit(float *raw_vx, float *raw_vw);
        
        /**
         * @brief 获取输出模式
         * @return 返回当前控制器输出模式
         */
        ControllerOutputMode getOutputMode(void);
        
        /**
         * @brief 控制云台
         * @return 返回是否控制云台
         */
        bool controlGimbal(void);
        
        /**
         * @brief 是否强制抬起
         * @return 返回是否处于强制抬起模式
         */
        bool isForceLiftup(void);
        
        /**
         * @brief 获取VMC状态
         * @return 返回虚拟模型控制状态
         */
        VMCState getState(void);
        
        /**
         * @brief 获取X方向速度增量限制
         * @return 返回X方向速度增量限制
         */
        float getDeltaXVelLimit(void);
        
        /**
         * @brief 获取偏航角速度增量限制
         * @return 返回偏航角速度增量限制
         */
        float getDeltaYawVelLimit(void);
        
        /**
         * @brief 获取X方向速度限制
         * @return 返回X方向速度限制
         */
        float getXVelLimit(void);
        
        /**
         * @brief 获取偏航角速度限制
         * @return 返回偏航角速度限制
         */
        float getYawVelLimit(void);
        
        /**
         * @brief 获取电机报告模式
         * @return 返回电机报告模式
         */
        uint16_t getMotorReportMode(void);
        
        /**
         * @brief 检查所有电机报告模式
         * @param[in] report_mode 期望的报告模式
         * @return 返回true表示所有电机都处于指定报告模式，否则返回false
         */
        bool isMotorReportModeAll(uint16_t report_mode);

        /**
         * @brief 更新外部命令
         * @details 更新来自外部的控制命令
         */
        void updateExternalCommand(void);
        
        /**
         * @brief 设置期望腿长
         * @param[in] l0_exp 期望腿长值
         */
        void setL0Exp(float l0_exp);
        
        /**
         * @brief 设置功率限制
         * @param[in] power_limit 功率限制值
         */
        void setPowerLimit(float power_limit);
        
        /**
         * @brief 触发跳跃命令
         * @details 启动跳跃动作序列
         */
        void triggerJumpCommand(void);
        
        /**
         * @brief 触发爬楼梯命令
         * @details 启动爬楼梯动作序列
         */
        void triggerClimbCommand(void);
        
        /**
         * @brief 触发抬起模式
         * @details 启动自动抬起模式
         */
        void triggerLiftUpMode(void);
        
        /**
         * @brief 触发强制抬起模式
         * @details 启动强制抬起模式
         */
        void triggerForceLiftUpMode(void);
        
        /**
         * @brief 触发手动抬起模式
         * @details 启动手动抬起模式
         */
        void triggerManualLiftUpMode(void);
};

extern  ChassisSoftWareControl chassis_SoftWareController;