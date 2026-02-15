/* Includes ------------------------------------------------------------------*/
#include "chassis_controller_params.h"
#include "chassis_controller_utils.h"
#include "math_process.h"
#include <cstring>                          // C字符串操作库
/* Classes -------------------------------------------------------------------*/

/**
 * @brief 底盘控制器有限状态机类
 * @details 实现机器人底盘控制的状态机管理，负责不同控制模式之间的切换和执行
 */
class ChassisControllerFSM
{
    private:
        float time_interval;                  // 控制时间间隔
        float Wid;                            // 机器人宽度参数

        uint8_t todo_reset_x;                 // 是否需要重置X轴位置标志
        uint16_t in_state_count;              // 当前状态持续计数
        uint16_t jump_landing_touchground_count; // 跳跃着陆触地计数
        bool todo_record_multiround_angle;    // 是否记录多圈角度标志

        float exp_inertial_acc;               // 期望惯性加速度

        float multi_round_angle_l, multi_round_angle_r;  // 左右电机多圈角度
        float round_angle_diff_l, round_angle_diff_r;    // 左右电机圈数差值

        float height_diff_last_;              // 上次高度差值
        bool height_diff_last_initialize_;    // 上次高度差值初始化标志

        ChassisControllerContext* context_;   // 底盘控制器上下文指针

        /**
         * @brief 切换状态机状态
         * @details 根据当前条件决定是否切换到新的控制状态
         */
        void switchFSM(void);
        
        /**
         * @brief 执行状态机动作
         * @details 根据当前状态执行相应的控制动作
         */
        void actFSM(void);
        
        /**
         * @brief 计算速度命令
         * @details 根据控制输入计算期望速度
         */
        void calculateVelocityCommand(void);
        
        /**
         * @brief 计算位置命令
         * @details 根据控制输入计算期望位置
         */
        void calculatePositionCommand(void);
        
        /**
         * @brief 计算高度命令
         * @details 根据控制输入计算期望高度
         */
        void calculateHeightCommand(void);
        
        /**
         * @brief 计算角度命令
         * @details 根据控制输入计算期望角度
         */
        void calculateThetaCommand(void);
        
        /**
         * @brief 目标高度计算
         * @param[out] length_diff 长度差值输出
         * @param[out] length_mean 长度均值输出
         * @param[out] length_diff_dot 长度差值导数输出
         * @param[in] input_l0 输入的腿长
         * @param[in] centrifugal_acc 离心加速度
         * @details 根据输入的腿长和离心加速度计算目标高度参数
         */
        void calculateTargetHeight(float *length_diff, float* length_mean, float *length_diff_dot, float input_l0, float centrifugal_acc);
        
        /**
         * @brief 更新位移
         * @details 根据当前运动状态更新累积位移
         */
        void updateDisplacement(void);
        
        /**
         * @brief 检查并减小位移
         * @details 检查位移是否超限，如超限则采取措施减小
         */
        void checkAndReduceDisplacement(void);
        
        /**
         * @brief 清除位移
         * @details 将累积位移清零
         */
        void clearDisplacement(void);
        
        /**
         * @brief 重新分配功率限制
         * @details 根据当前状态重新分配各电机的功率限制
         */
        void realocatePowerLimit(void);

        /**
         * @brief 执行力矩控制
         * @details 根据计算结果执行力矩控制模式
         */
        void actTorqueControl(void);
        
        /**
         * @brief 执行速度控制
         * @details 根据计算结果执行速度控制模式
         */
        void actVelControl(void);
        
        /**
         * @brief 执行位置控制
         * @details 根据计算结果执行位置控制模式
         */
        void actPosControl(void);

        /**
         * @brief 清除电机位置命令
         * @details 将电机位置命令设置为默认值
         */
        void clearMotorPosCommand(void);
        
        /**
         * @brief 清除电机速度命令
         * @details 将电机速度命令设置为默认值
         */
        void clearMotorVelCommand(void);
        
        /**
         * @brief 清除电机力矩命令
         * @details 将电机力矩命令设置为默认值
         */
        void clearMotorTorqueCommand(void);

        /**
         * @brief 检测异常状态
         * @return 返回true表示检测到异常，false表示正常
         * @details 检测系统是否存在异常运行状态
         */
        bool detectAbnormal(void);
        
        /**
         * @brief 检测爬楼梯状态
         * @return 返回true表示检测到爬楼梯，false表示平地行走
         * @details 检测机器人是否处于爬楼梯状态
         */
        bool detectClimbStair(void);

        /**
         * @brief 记录多圈角度
         * @details 记录电机的多圈绝对角度值
         */
        void recordMultiRoundAngle(void);

        /**
         * @brief 设置电机位置命令
         * @param[in] theta1_left 左侧电机1位置
         * @param[in] theta2_left 左侧电机2位置
         * @param[in] theta1_right 右侧电机1位置
         * @param[in] theta2_right 右侧电机2位置
         * @param[in] gimbal_pitch 云台俯仰角
         * @param[in] gimbal_yaw 云台偏航角
         * @details 将计算得到的位置命令发送给对应电机
         */
        void setMotorPosCommand(float theta1_left, float theta2_left, float theta1_right, float theta2_right, float gimbal_pitch, float gimbal_yaw);
        
        /**
         * @brief 设置电机速度命令
         * @param[in] theta1_left 左侧电机1速度
         * @param[in] theta2_left 左侧电机2速度
         * @param[in] theta1_right 右侧电机1速度
         * @param[in] theta2_right 右侧电机2速度
         * @details 将计算得到的速度命令发送给对应电机
         */
        void setMotorVelCommand(float theta1_left, float theta2_left, float theta1_right, float theta2_right);
        
        /**
         * @brief 逆运动学计算
         * @param[in] vmc_l0 虚拟模型控制腿长
         * @param[in] vmc_phi0 虚拟模型控制相位角
         * @param[out] theta1 输出的角度1
         * @param[out] theta2 输出的角度2
         * @details 将虚拟模型控制输出转换为实际关节角度
         */
        void backwardKinematic(float vmc_l0, float vmc_phi0, float* theta1, float* theta2);

    public:
        /**
         * @brief 构造函数
         * @param[in] context 底盘控制器上下文指针
         * @details 初始化状态机并关联控制上下文
         */
        ChassisControllerFSM(ChassisControllerContext* context);
        
        /**
         * @brief 重置状态机
         * @details 将状态机恢复到初始状态
         */
        void reset(void);
        
        /**
         * @brief 更新状态机
         * @details 执行状态机的主要更新逻辑
         */
        void update(void);
        
        /**
         * @brief 切换到指定状态
         * @param[in] state 目标控制器状态
         * @details 强制将状态机切换到指定的控制状态
         */
        void switchTo(ControllerState state);

        /**
         * @brief 获取输出模式
         * @return 返回当前控制器输出模式
         * @details 查询当前控制器的工作模式
         */
        ControllerOutputMode getOutputMode(void);
        
        /**
         * @brief 检查所有电机报告模式
         * @param[in] report_mode 期望的报告模式
         * @return 返回true表示所有电机都处于指定报告模式，否则返回false
         * @details 检查所有电机是否都处于相同的报告模式
         */
        bool isMotorReportModeAll(uint16_t report_mode);
};