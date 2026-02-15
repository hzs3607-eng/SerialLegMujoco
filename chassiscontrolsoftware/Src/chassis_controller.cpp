//
// Created by huzongsheng on 2026/1/22.
//
/* Includes ------------------------------------------------------------------*/
#include "../Inc/chassis_controller.h"



/* Global Variables ----------------------------------------------------------*/


ChassisSoftWareControl chassis_SoftWareController; // 底盘控制器全局实例

/* Functions -----------------------------------------------------------------*/

/**
 * @brief 底盘控制器构造函数
 * @details 初始化底盘控制器各组件，包括状态估计器、状态机、执行器以及各类电机和传感器
 */
ChassisSoftWareControl::ChassisSoftWareControl(void)
    :estimator_(&context_)        // 初始化状态估计器，传入上下文指针
    ,fsm_(&context_)              // 初始化有限状态机，传入上下文指针
    , actuator_(&context_)        // 初始化执行器，传入上下文指针

{
    reset(); // 重置控制器
}

/**
 * @brief 重置控制器
 * @details 将控制器恢复到初始状态，包括上下文、估计器、状态机和执行器
 */
void ChassisSoftWareControl::reset(void)
{
    context_.reset();     // 重置上下文
    estimator_.reset();   // 重置状态估计器
    fsm_.reset();         // 重置有限状态机
    actuator_.reset();    // 重置执行器
    fsm_.switchTo(FSM_LIFT_UP); // 初始状态设为抬起状态
}

/**
 * @brief 更新控制器
 * @details 执行控制器的主要更新逻辑，依次更新外部命令、状态估计、状态机和执行器
 */
void ChassisSoftWareControl::update(void)
{
    updateExternalCommand(); // 更新外部命令
    // 更新各模块
    estimator_.update();     // 更新状态估计器
    fsm_.update();           // 更新有限状态机
    actuator_.update();      // 更新执行器
}



/**
 * @brief 更新外部命令
 * @details 从底盘对象获取控制命令并更新到控制器上下文中
 */
void ChassisSoftWareControl::updateExternalCommand(void)
{
    // 从底盘对象获取控制命令
    // context_.external_command.velocity_x_plan = Chassis.velocity_x_plan; // X方向速度规划
    // context_.external_command.omega_plan = Chassis.omega_plan;           // 角速度规划
    // context_.external_command.angle_target = Chassis.angle_target;       // 角度目标

    // 检查并初始化期望腿长
    if (context_.external_command.l0_exp < 1e-3f) // 如果期望腿长小于极小值
    {
        context_.external_command.l0_exp = robot_param.l0_stable; // 设为稳定腿长
    }
    if (context_.external_command.l0_exp_raw < 1e-3f) // 如果原始期望腿长小于极小值
    {
        context_.external_command.l0_exp_raw = robot_param.l0_stable; // 设为稳定腿长
    }

    // 一阶低通滤波处理期望腿长
    context_.external_command.l0_exp = one_low_pass_function(0.01f, context_.external_command.l0_exp_raw, context_.external_command.l0_exp);

    // 根据底盘状态设置是否跟随云台
    // if (Chassis.state == CHASSIS_GYRO || Chassis.state == CHASSIS_NORMAL) // 如果是陀螺仪或普通状态
    // {
    //     context_.follow_gimbal = false; // 不跟随云台
    // }
    // else // 其他状态
    // {
    //     context_.follow_gimbal = true; // 跟随云台
    // }
}


/**
 * @brief 设置期望腿长
 * @param[in] l0_exp 期望腿长值
 */
void ChassisSoftWareControl::setL0Exp(float l0_exp)
{
    context_.external_command.l0_exp_raw = l0_exp; // 设置原始期望腿长
}


/**
 * @brief 设置功率限制
 * @param[in] power_limit 功率限制值
 */
void ChassisSoftWareControl::setPowerLimit(float power_limit)
{
    context_.power_control.power_limit = power_limit; // 设置功率限制
}

/**
 * @brief 触发跳跃命令
 * @details 启动跳跃动作序列
 */
void ChassisSoftWareControl::triggerJumpCommand(void)
{
    fsm_.switchTo(FSM_JUMP_STRETCHING); // 切换到跳跃伸展状态
}

/**
 * @brief 触发爬楼梯命令
 * @details 启动爬楼梯动作序列
 */
void ChassisSoftWareControl::triggerClimbCommand(void)
{
    fsm_.switchTo(FSM_CLIMBSTAIR_LIFTLEG); // 切换到爬楼梯抬腿状态
}

/**
 * @brief 触发抬起模式
 * @details 启动自动抬起模式
 */
void ChassisSoftWareControl::triggerLiftUpMode(void)
{
    fsm_.switchTo(FSM_LIFT_UP); // 切换到抬起状态
}

/**
 * @brief 触发强制抬起模式
 * @details 启动强制抬起模式
 */
void ChassisSoftWareControl::triggerForceLiftUpMode(void)
{
    fsm_.switchTo(FSM_FORCE_LIFT_UP); // 切换到强制抬起状态
}

/**
 * @brief 触发手动抬起模式
 * @details 启动手动抬起模式
 */
void ChassisSoftWareControl::triggerManualLiftUpMode(void)
{
    fsm_.switchTo(FSM_MANUAL_LIFT_UP); // 切换到手动抬起状态
}

/**
 * @brief 获取X方向速度限制
 * @return 返回X方向速度限制
 * @details 根据功率限制计算X方向最大允许速度
 */
float ChassisSoftWareControl::getXVelLimit(void) // 速度限制
{
    // For 100w
    float power_high = 100.0f;      // 高功率值
    float vel_allow_high = VEL_X_UPPER; // 高功率下的速度上限
    // For 45 w
    float power_low = 45.0f;        // 低功率值
    float vel_allow_low = VEL_X_MID; // 低功率下的速度中值

    // 根据当前功率限制插值得到允许速度
    float vel_allow = vel_allow_low + limit_range_function((vel_allow_high - vel_allow_low) * (context_.power_control.power_limit - power_low) / (power_high - power_low), 0.0f, 1.0f);

    return vel_allow;
}

/**
 * @brief 获取偏航角速度限制
 * @return 返回偏航角速度限制
 * @details 根据功率限制计算偏航角最大允许速度
 */
float ChassisSoftWareControl::getYawVelLimit(void)
{
    // For 100w
    float power_high = 100.0f;      // 高功率值
    float vel_allow_high = 15.0f;   // 高功率下的角速度上限
    // For 45 w
    float power_low = 45.0f;        // 低功率值
    float vel_allow_low = 12.0f;    // 低功率下的角速度中值

    // 根据当前功率限制插值得到允许角速度
    float vel_allow = vel_allow_low + limit_range_function((vel_allow_high - vel_allow_low) * (context_.power_control.power_limit - power_low) / (power_high - power_low), 0.0f, 1.0f);

    return vel_allow;
}

/**
 * @brief 获取期望腿长
 * @return 返回期望腿长值
 */
float ChassisSoftWareControl::getL0Exp(void)
{
    return context_.external_command.l0_exp_raw; // 返回原始期望腿长
}

/**
 * @brief 获取底盘X方向速度
 * @return 返回底盘X方向速度
 */
float ChassisSoftWareControl::getChassisVX(void)
{
    return context_.vmc_state.center_v_x; // 返回中心X方向速度
}

/**
 * @brief 获取底盘角速度
 * @return 返回底盘绕Z轴角速度
 */
float ChassisSoftWareControl::getChassisVW(void)
{
    return context_.imu_data.chassis_yaw_speed_rad; // 返回偏航角速度
}

/**
 * @brief 获取底盘俯仰角
 * @return 返回底盘俯仰角
 */
float ChassisSoftWareControl::getChassisPitch(void)
{
    return context_.imu_data.chassis_pitch_rad; // 返回俯仰角
}

/**
 * @brief 获取电机力矩
 * @return 返回电机力矩结构体
 */
MotorTargetTorque ChassisSoftWareControl::getMotorTorque(void)
{
    return context_.motor_TargetTorque; // 返回电机力矩
}

/**
 * @brief 获取电机位置
 * @return 返回电机位置结构体
 */
MotorTargetPos ChassisSoftWareControl::getMotorPos(void)
{
    return context_.motor_TargetPos; // 返回电机位置
}

/**
 * @brief 获取电机速度
 * @return 返回电机速度结构体
 */
MotorTargetVel ChassisSoftWareControl::getMotorVel(void)
{
    return context_.motor_TargetVel; // 返回电机速度
}

/**
 * @brief 获取调试信息
 * @return 返回调试信息结构体
 */
DebugInfo ChassisSoftWareControl::getDebugInfo(void)
{
    return context_.debug_info; // 返回调试信息
}

/**
 * @brief 速度限制
 * @param[in,out] vx 输入输出的X方向速度
 * @param[in,out] vw 输入输出的角速度
 * @details 对输入速度进行限制处理
 */
void ChassisSoftWareControl::velLimit(float *vx, float *vw)
{
    float vw_rad = degree2rad(*vw); // 将角速度转换为弧度
    // 计算最大允许惯性加速度
    float max_inertial_acc = INERTIAL_ACC_LIMIT_RATIO * 9.8f * robot_param.Wid / (context_.vmc_state.y_left + context_.vmc_state.y_right);
    // 计算当前期望惯性加速度
    float exp_inertial_acc = max_function(fabsf(vw_rad), fabsf(context_.vmc_state.center_omega_yaw)) * max_function(fabsf(*vx), fabsf(context_.vmc_state.center_v_x));
    float scale = 1.0f;
    if (exp_inertial_acc > max_inertial_acc){ // 如果期望惯性加速度超过限制
        scale = max_inertial_acc / exp_inertial_acc; // 计算缩放因子
    }
    *vx = powf(scale, 0.25f) * *vx; // 对X方向速度进行缩放
    *vw = powf(scale, 0.75f) * *vw; // 对角速度进行缩放
}

/**
 * @brief 获取VMC状态
 * @return 返回虚拟模型控制状态
 */
VMCState ChassisSoftWareControl::getState(void)
{
    return context_.vmc_state; // 返回VMC状态
}

/**
 * @brief 计算底盘在X方向上允许的最大速度增量
 * @return 返回X方向速度增量限制
 */
float ChassisSoftWareControl::getDeltaXVelLimit(void)
{
    // For 100w
    float power_high = 100.0f;      // 高功率值
    float allow_acc_high = 3.0f;    // 高功率下的加速度上限
    // For 45 w
    float power_low = 45.0f;        // 低功率值
    float allow_acc_low = 1.2f;     // 低功率下的加速度下限
    // 根据当前功率限制插值得到允许加速度
    float allow_acc = allow_acc_low + limit_range_function((allow_acc_high - allow_acc_low) * (context_.power_control.power_limit - power_low) / (power_high - power_low), 0.0f, 1.0f);
    return allow_acc * TIME_INTERVAL; // 返回速度增量限制（乘以时间间隔）
}

/**
 * @brief 计算底盘在Y方向上允许的最大速度增量
 * @return 返回Y方向速度增量限制
 */
float ChassisSoftWareControl::getDeltaYawVelLimit(void)
{
    // For 100w
    float power_high = 100.0f;      // 高功率值
    float allow_acc_high = 10.0f;   // 高功率下的角加速度上限
    // For 45 w
    float power_low = 45.0f;        // 低功率值
    float allow_acc_low = 8.0f;     // 低功率下的角加速度下限
    // 根据当前功率限制插值得到允许角加速度
    float allow_acc = allow_acc_low + limit_range_function((allow_acc_high - allow_acc_low) * (context_.power_control.power_limit - power_low) / (power_high - power_low), 0.0f, 1.0f);
    return allow_acc * TIME_INTERVAL; // 返回角速度增量限制（乘以时间间隔）
}

/**
 * @brief 获取电机报告模式
 * @return 返回电机报告模式
 */
uint16_t ChassisSoftWareControl::getMotorReportMode(void)
{
    // 检查所有电机是否都处于报告模式2
    if (context_.motor_report_mode.of_motor1 == 2 && context_.motor_report_mode.of_motor2 == 2 && context_.motor_report_mode.of_motor3 == 2 && context_.motor_report_mode.of_motor4 == 2)
        return 2; // 所有电机都在报告模式2
    else
        return 1; // 否则返回报告模式1
}

/**
 * @brief 检查所有电机报告模式
 * @param[in] report_mode 期望的报告模式
 * @return 返回true表示所有电机都处于指定报告模式，否则返回false
 */
bool ChassisSoftWareControl::isMotorReportModeAll(uint16_t report_mode)
{
    return fsm_.isMotorReportModeAll(report_mode); // 调用状态机的检查函数
}

/**
 * @brief 获取输出模式
 * @return 返回当前控制器输出模式
 */
ControllerOutputMode ChassisSoftWareControl::getOutputMode(void)
{
    return fsm_.getOutputMode(); // 调用状态机的输出模式函数
}

/**
 * @brief 控制云台
 * @return 返回是否控制云台
 */
bool ChassisSoftWareControl::controlGimbal(void)
{
    if (context_.controller_state == FSM_SELF_SAVE) // 如果处于自救状态
        return true; // 需要控制云台
    else
        return false; // 不需要控制云台
}

/**
 * @brief 是否强制抬起
 * @return 返回是否处于强制抬起模式
 */
bool ChassisSoftWareControl::isForceLiftup(void)
{
    if (context_.controller_state == FSM_FORCE_LIFT_UP) // 如果处于强制抬起状态
        return true; // 是强制抬起模式
    else
        return false; // 不是强制抬起模式
}