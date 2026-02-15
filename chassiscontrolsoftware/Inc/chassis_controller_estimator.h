#pragma once

/* Includes ------------------------------------------------------------------*/

#include "chassis_controller_params.h"
#include "chassis_controller_utils.h"
#include <cstring>
#include "math_process.h"

/* Classes -------------------------------------------------------------------*/
/**
 * @brief 底盘控制器状态估计器类
 * @details 负责机器人底盘的状态估计，包括姿态、位置、速度等信息的计算
 *          主要功能包括：五连杆几何计算、打滑检测、离地检测、旋转矩阵计算等
 */
class ChassisControllerEstimator
{
    private:
        // 车体加速度（已去除重力影响）
        float chassis_acc_x;  ///< x轴加速度m/s²，已分离重力加速度
        float chassis_acc_y;  ///< y轴加速度m/s²，已分离重力加速度
        float chassis_acc_z;  ///< z轴加速度m/s²，已分离重力加速度
        
        float body_to_ground_matrix[9];    ///< 底盘到地面的旋转矩阵（3x3矩阵展开为一维数组）
        float ground_to_body_matrix[9];    ///< 地面到底盘的旋转矩阵（3x3矩阵展开为一维数组）

        float delta_velocity_wheel;        ///< 两轮与地面接触点的线速度差，用于打滑检测
        float delta_velocity_wheel_last;   ///< 上次两轮与地面接触点的线速度差，用于打滑检测
        uint8_t slip_flag;                 ///< 打滑标志位
        
        float time_interval;               ///< 时间间隔，单位秒

        float spring_model_A;              ///< 弹簧模型参数A
        float spring_model_B;              ///< 弹簧模型参数B
        
        // 低通滤波器相关变量
        float ckyf_speed_right_front_last;  ///< 右前CKYF电机速度上次值
        float ckyf_speed_right_back_last;   ///< 右后CKYF电机速度上次值
        float ckyf_speed_left_front_last;   ///< 左前CKYF电机速度上次值
        float ckyf_speed_left_back_last;    ///< 左后CKYF电机速度上次值
        float m3508_speed_left_last;        ///< 左侧M3508电机速度上次值
        float m3508_speed_right_last;       ///< 右侧M3508电机速度上次值
        float chassis_roll_speed_rad_last;  ///< 底盘翻滚角速度上次值
        float chassis_pitch_speed_rad_last; ///< 底盘俯仰角速度上次值
        float chassis_yaw_speed_rad_last;   ///< 底盘偏航角速度上次值
        float chassis_acc_x_last;           ///< x轴加速度上次值
        float chassis_acc_y_last;           ///< y轴加速度上次值
        float chassis_acc_z_last;           ///< z轴加速度上次值
        float ckyf_real_torque_left_back_last;   ///< 左后CKYF电机实际扭矩上次值
        float ckyf_real_torque_left_front_last;  ///< 左前CKYF电机实际扭矩上次值
        float ckyf_real_torque_right_back_last;  ///< 右后CKYF电机实际扭矩上次值
        float ckyf_real_torque_right_front_last; ///< 右前CKYF电机实际扭矩上次值

        // 离地检测相关变量
        float offground_detect_list_body_acc[OFF_GROUND_DETECT_LIST_LENGTH];     ///< 机身加速度历史记录列表
        float offground_detect_list_left_vmc_force[OFF_GROUND_DETECT_LIST_LENGTH];///< 左侧VMC力历史记录列表
        float offground_detect_list_right_vmc_force[OFF_GROUND_DETECT_LIST_LENGTH];///< 右侧VMC力历史记录列表
        float offground_detect_list_left_relative_vel[OFF_GROUND_DETECT_LIST_LENGTH];///< 左侧相对速度历史记录列表
        float offground_detect_list_right_relative_vel[OFF_GROUND_DETECT_LIST_LENGTH];///< 右侧相对速度历史记录列表
        uint8_t offground_detect_list_i;    ///< 离地检测循环队列索引
        float offground_root_vz_diff;       ///< 根部z轴速度差值
        float offground_left_vmc_impulse;   ///< 左侧VMC冲量
        float offground_right_vmc_impulse;  ///< 右侧VMC冲量
        
        ChassisControllerContext* context_; ///< 控制器上下文指针
        
        // 私有方法
        void stateCaculationWithoutX(void);         ///< 不依赖X轴的几何状态计算（五连杆几何参数解算）
        void stateCaculationWithX(void);            ///< 依赖X轴的运动状态计算（包含车体加速度计算）
        void slipDetection(void);                   ///< 打滑检测
        void offGroundDetection(void);              ///< 离地检测
        void calculateRotationMatrix(void);         ///< 旋转矩阵计算（地面系到车身坐标系及反向）
        void solvePentagon(Pentagon &p);            ///< 五连杆机构运动学解算
        void calculateGuideWheelHeight(void);       ///< 导轮高度计算

    public:
        /**
         * @brief 构造函数
         * @param context 控制器上下文指针
         */
        ChassisControllerEstimator(ChassisControllerContext* context);
        
        /**
         * @brief 重置状态
         * @details 将所有内部状态变量重置为初始值
         */
        void reset(void);
        
        /**
         * @brief 更新状态
         * @details 执行完整的状态估计流程
         */
        void update(void);
};

