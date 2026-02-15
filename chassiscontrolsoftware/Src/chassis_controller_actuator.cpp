/* Includes ------------------------------------------------------------------*/
#include "../Inc/chassis_controller_actuator.h"


/* Functions -----------------------------------------------------------------*/

/**
 * @brief 底盘控制器执行器构造函数
 * @param[in] context 底盘控制器上下文指针
 * @details 初始化执行器相关参数和PID控制器
 */
ChassisControllerActuator::ChassisControllerActuator(ChassisControllerContext* context)
{
    /* 相关PID初始化 */
    pid_mean_l0.Init(PID_MEAN_L0_KP,PID_MEAN_L0_KI,PID_MEAN_L0_KD,0.0f,PID_MEAN_L0_MAX_I,PID_MEAN_L0_MAX); // 初始化腿长均值PID
    pid_mean_l0.Enable_Accurate_D(); // 启用精确微分
    pid_diff_l0.Init(PID_DIFF_L0_KP,PID_DIFF_L0_KI,PID_DIFF_L0_KD,0.0f,PID_DIFF_L0_MAX_I,PID_DIFF_L0_MAX); // 初始化腿长差值PID
    pid_diff_l0.Enable_Accurate_D(); // 启用精确微分

    M = robot_param.M;       // 机器人质量
    Width = robot_param.Wid; // 机器人宽度

    context_ = context;      // 关联控制上下文
}

/**
 * @brief 重启机器人状态
 * @details 将执行器状态重置为初始状态
 */
void ChassisControllerActuator::reset(void)
{
    /* 重启机器人状态 */
    pid_mean_l0.Clear(); // 清除腿长均值PID
    pid_diff_l0.Clear(); // 清除腿长差值PID

    leg_compensate_torque_left = 0.0f;   // 左侧腿部力矩补偿清零
    leg_compensate_torque_right = 0.0f;  // 右侧腿部力矩补偿清零

    g_l = GRAVITY_ACC / 2.0f;  // 左侧重力分量
    g_r = GRAVITY_ACC / 2.0f;  // 右侧重力分量

    vmc_l.F = 0.0f;           // 左腿虚拟力清零
    vmc_l.Digital_torque = 0.0f; // 左腿数字力矩清零
    vmc_l.Wheel_torque = 0.0f;   // 左轮力矩清零
    vmc_r.F = 0.0f;           // 右腿虚拟力清零
    vmc_r.Digital_torque = 0.0f; // 右腿数字力矩清零
    vmc_r.Wheel_torque = 0.0f;   // 右轮力矩清零

    F_l_last = 0.0f;  // 上次左腿支持力
    F_r_last = 0.0f;  // 上次右腿支持力

    for (int i = 0; i < 4; i++) // 清零LQR控制矩阵
    {
        for (int j = 0; j < 10; j++)
        {
            Matrix_K[i][j] = 0.0f;
        }
    }
}

/**
 * @brief 更新控制量
 * @details 执行主要的控制算法更新逻辑
 */
void ChassisControllerActuator::update(void)
{
    /* 计算控制量 */
    kMatrixCalculation();          // 计算LQR矩阵
    equivalentGravityCalculation(); // 计算等效重力
    calculateLQRTorque();          // 计算LQR力矩
    calculateSupportForce();       // 计算支持力
    convertToMotorTorque();        // 转换为电机力矩
}

/**
 * @brief 根据状态选择K矩阵
 * @details 根据当前控制状态选择合适的LQR反馈矩阵
 */
void ChassisControllerActuator::kMatrixCalculation(void)
{
    if (context_->controller_state == FSM_JUMP_FOLDING || context_->controller_state == FSM_JUMP_LANDING || context_->off_ground.off_ground_body) // 系统处于跳跃折叠、跳跃着陆或离地状态，使用离地时的 LQR 增益矩阵
    {
        for (int i=0; i<4; i++){
            for (int j=0; j<4; j++){
                Matrix_K[i][j] = 0.0f; // 清零前4列
            }
            for (int j=4; j<10; j++){
                Matrix_K[i][j] = Matrix_K_3d_offground[i][j-4]; // 使用离地状态的LQR矩阵
            }
        }
    }
    else // 正常状态，使用地面状态的LQR矩阵
    {
        for (int i=0; i<4; i++){
            for (int j=0; j<10; j++){
                // 根据当前腿长计算LQR矩阵系数
                Matrix_K[i][j] = Matrix_coefficients_forward[0][10*i+j] * context_->geometric_left.l0 * context_->geometric_left.l0 + Matrix_coefficients_forward[1][10 * i + j] * context_->geometric_right.l0 * context_->geometric_right.l0
                                 + Matrix_coefficients_forward[2][10*i+j] * context_->geometric_left.l0 * context_->geometric_right.l0 + Matrix_coefficients_forward[3][10 * i + j] * context_->geometric_left.l0
                                 +Matrix_coefficients_forward[4][10*i+j]*context_->geometric_right.l0 + Matrix_coefficients_forward[5][10 * i + j];
            }
        }
    }
}

/**
 * @brief 等效重力补偿
 * @details 计算机器人在倾斜状态下的等效重力分量
 */
void ChassisControllerActuator::equivalentGravityCalculation(void)
{
    // 计算惯性加速度补偿
    float inertial_acc = clip(0.0f, GRAVITY_ACC * INERTIAL_ACC_LIMIT_RATIO, - context_->vmc_state.center_omega_yaw * context_->vmc_state.center_v_x);
    if (fabsf(context_->imu_data.chassis_yaw_speed_rad) > HIGH_YAW_SPEED_THRESHOLD) // 如果偏航速度过高，则不使用惯性补偿
    {
        inertial_acc = 0.0f;
    }
    float wid_l = 0.5f * Width;  // 左侧宽度
    float wid_r = 0.5f * Width;  // 右侧宽度
    float mean_y = (context_->vmc_state.y_right + context_->vmc_state.y_left) / 2.0f; // 平均腿长
    float inertial_compensation_factor = 2.0f; // 惯性补偿因子
    if (!context_->off_ground.off_ground_body) // 如果机器人未离地
    {
        // 计算左右侧等效重力分量（考虑倾斜和惯性影响）
        g_l = GRAVITY_ACC * (wid_r * context_->imu_data.cos_roll_chassis - mean_y * context_->imu_data.sin_roll_chassis) / Width + inertial_acc * (wid_r * context_->imu_data.sin_roll_chassis + mean_y * context_->imu_data.cos_roll_chassis) / Width * inertial_compensation_factor;
        g_r = GRAVITY_ACC * (wid_l * context_->imu_data.cos_roll_chassis + mean_y * context_->imu_data.sin_roll_chassis) / Width - inertial_acc * (- wid_l * context_->imu_data.sin_roll_chassis + mean_y * context_->imu_data.cos_roll_chassis) / Width * inertial_compensation_factor;
    }
    else // 如果机器人离地
    {
        g_l = GRAVITY_ACC * (wid_r) / (Width);  // 简化计算
        g_r = GRAVITY_ACC * (wid_l) / (Width); }
    
}

/**
 * @brief LQR力矩计算
 * @details 实现线性二次调节器控制算法
 */
void ChassisControllerActuator::calculateLQRTorque(void)
{
    // 预处理：计算偏航角误差
    float yaw_error_temp = fmodf(context_->vmc_state.center_yaw - context_->internal_command.yaw_expectation + LQR_PI, 2.0f * LQR_PI);
    if (yaw_error_temp < 0.0f)  yaw_error_temp += 2.0f * LQR_PI;
    yaw_error_temp = yaw_error_temp - LQR_PI;
    yaw_error_temp = limit_range_function(yaw_error_temp, -PI/3.0f, PI/3.0f); // 限制偏航角误差范围

    // 计算状态误差数组
    float error[10];
    error[0] = context_->vmc_state.center_x - context_->internal_command.x_expectation;           // X位置误差
    error[1] = context_->vmc_state.center_v_x - context_->internal_command.desired_vel_x;       // X速度误差
    error[2] = yaw_error_temp;                                                                 // 偏航角误差
    error[3] = context_->vmc_state.center_omega_yaw - context_->internal_command.desired_vel_omega; // 偏航角速度误差
    error[4] = context_->vmc_state.angle_left - context_->internal_command.target_theta_l;     // 左侧角度误差
    error[5] = context_->vmc_state.omega_left;                                                  // 左侧角速度
    error[6] = context_->vmc_state.angle_right - context_->internal_command.target_theta_r;    // 右侧角度误差
    error[7] = context_->vmc_state.omega_right;                                                 // 右侧角速度
    error[8] = context_->vmc_state.phi;                                                        // phi角
    error[9] = context_->vmc_state.omega_phi;                                                   // phi角速度

    // 初始化控制量
    vmc_l.Wheel_torque = 0.0f;
    vmc_l.Digital_torque = 0.0f;
    vmc_r.Wheel_torque = 0.0f;
    vmc_r.Digital_torque = 0.0f;
    if (context_->controller_state == FSM_JUMP_FOLDING || context_->off_ground.off_ground_body) // 如果是跳跃收腿或离地状态
    {
        vmc_l.Wheel_torque = 0.0f;
        vmc_r.Wheel_torque = 0.0f;
        // 使用简单的PD控制
        vmc_l.Digital_torque = - 40.0f * (context_->vmc_state.angle_left)
                               - 4.0f * (context_->vmc_state.omega_left);
        vmc_r.Digital_torque = - 40.0f * (context_->vmc_state.angle_right)
                               - 4.0f * (context_->vmc_state.omega_right);
    }
    else { // 正常状态，使用LQR控制
        for (int i = 0; i < 10; i++)
        {
            vmc_l.Wheel_torque -= Matrix_K[0][i] * error[i]; // 左轮力矩
            vmc_l.Digital_torque -= Matrix_K[1][i] * error[i]; // 左数字力矩
            vmc_r.Wheel_torque -= Matrix_K[2][i] * error[i]; // 右轮力矩
            vmc_r.Digital_torque -= Matrix_K[3][i] * error[i]; // 右数字力矩
        }
    
        // 补偿离心力力偶矩
        vmc_l.Digital_torque += context_->geometric_left.com_x * context_->geometric_left.com_y * context_->geometric_left.m_leg * context_->vmc_state.center_omega_yaw * context_->vmc_state.center_omega_yaw;
        vmc_r.Digital_torque += context_->geometric_right.com_x * context_->geometric_right.com_y * context_->geometric_right.m_leg * context_->vmc_state.center_omega_yaw * context_->vmc_state.center_omega_yaw;
    }

    // 获取功率限制因子
    float power_limit_factor = getPowerLimitFactor(vmc_l.Wheel_torque, vmc_r.Wheel_torque);

    // 应用功率限制
    vmc_l.Wheel_torque *= power_limit_factor;
    vmc_l.Digital_torque *= power_limit_factor;
    vmc_r.Wheel_torque *= power_limit_factor;
    vmc_r.Digital_torque *= power_limit_factor;
}

/**
 * @brief 计算支持力
 * @details 计算机器人腿部对地面的支持力
 */
void ChassisControllerActuator::calculateSupportForce(void)
{
    // 根据状态动态调整PID参数
    if (context_->controller_state == FSM_JUMP_STRETCHING || context_->controller_state == FSM_JUMP_FOLDING) {
        // 动态调节均值腿长控制 PID 的微分项，随着腿长接近目标减小微分值
        float mean_kd_ratio = 1.0f - limit_range_function(fabsf(((context_->internal_command.target_y_l + context_->internal_command.target_y_r) - (context_->geometric_left.l0 + context_->geometric_right.l0)) / (L0_MAX - L0_JUMP_FOLDING) / 2.0f), 0.0f, 1.0f);
        pid_mean_l0.Set_K_P(PID_MEAN_L0_KP);  // 设置比例系数
        pid_mean_l0.Set_K_I(PID_MEAN_L0_KI);  // 设置积分系数
        pid_mean_l0.Set_K_D(PID_MEAN_L0_KD * mean_kd_ratio * 0.2f); // 设置微分系数（动态调整）
        pid_diff_l0.Set_K_P(PID_DIFF_L0_KP);  // 设置比例系数
        pid_diff_l0.Set_K_I(PID_DIFF_L0_KI);  // 设置积分系数
        pid_diff_l0.Set_K_D(PID_DIFF_L0_KD);  // 设置微分系数
        pid_mean_l0.Clear();  // 清除PID历史数据
        pid_diff_l0.Clear();  // 清除PID历史数据
    }
    else { // 正常状态PID参数
        pid_mean_l0.Set_K_P(PID_MEAN_L0_KP);
        pid_mean_l0.Set_K_I(PID_MEAN_L0_KI);
        pid_mean_l0.Set_K_D(PID_MEAN_L0_KD);
        pid_diff_l0.Set_K_P(PID_DIFF_L0_KP);
        pid_diff_l0.Set_K_I(PID_DIFF_L0_KI);
        pid_diff_l0.Set_K_D(PID_DIFF_L0_KD);
    }
    // 计算腿长控制量
    pid_mean_l0.Set_Target((context_->internal_command.target_y_l + context_->internal_command.target_y_r) / 2.0f); // 设置目标值（平均腿长）
    pid_mean_l0.Set_Now((context_->geometric_left.l0 + context_->geometric_right.l0) / 2.0f); // 设置当前值（平均腿长）
    pid_mean_l0.Set_Accurate_D((context_->internal_command.target_dot_y_l + context_->internal_command.target_dot_y_r) / 2.0f - (context_->vmc_state.speed_y_left + context_->vmc_state.speed_y_right) / 2.0f); // 设置精确微分
    pid_mean_l0.Calculate(); // 计算PID输出

    pid_diff_l0.Set_Target((context_->internal_command.target_y_l - context_->internal_command.target_y_r) / 2.0f); // 设置目标值（腿长差值）
    pid_diff_l0.Set_Now((context_->geometric_left.l0 - context_->geometric_right.l0) / 2.0f); // 设置当前值（腿长差值）
    pid_diff_l0.Set_Accurate_D( - context_->imu_data.chassis_roll_speed_rad); // 设置精确微分
    pid_diff_l0.Calculate(); // 计算PID输出

    // 计算腿部补偿力矩
    leg_compensate_torque_left = pid_mean_l0.Get_Out() + pid_diff_l0.Get_Out();  // 左侧补偿力矩
    leg_compensate_torque_right = pid_mean_l0.Get_Out() - pid_diff_l0.Get_Out(); // 右侧补偿力矩

    // 跳跃过程中的额外补偿
    double extra_mean_compensation = 0.0f;  // 额外均值补偿
    double extra_diff_compensation = 0.0f;  // 额外差值补偿
    if (context_->controller_state == FSM_JUMP_STRETCHING) { // 跳跃伸展状态
        extra_mean_compensation = 12.0f * ((context_->internal_command.target_y_l + context_->internal_command.target_y_r) - (context_->geometric_left.l0 + context_->geometric_right.l0)) / (L0_MAX - L0_MIN) / 2.0f;
        extra_diff_compensation = 40.0f * (context_->geometric_right.l0 - context_->geometric_left.l0) / (L0_MAX - L0_MIN) / 2.0f + 4.0f * (context_->vmc_state.speed_y_right - context_->vmc_state.speed_y_left);
    }
    else if (context_->controller_state == FSM_JUMP_FOLDING) { // 跳跃收腿状态
        extra_mean_compensation = 12.0f * ((context_->internal_command.target_y_l + context_->internal_command.target_y_r) - (context_->geometric_left.l0 + context_->geometric_right.l0)) / (L0_MAX - L0_MIN) / 2.0f;
        extra_diff_compensation = 40.0f * (context_->geometric_right.l0 - context_->geometric_left.l0) / (L0_MAX - L0_MIN) / 2.0f + 1.0f * (context_->vmc_state.speed_y_right - context_->vmc_state.speed_y_left);
    }

    // 应用额外补偿
    leg_compensate_torque_left += extra_mean_compensation / 2.0f + extra_diff_compensation / 2.0f;
    leg_compensate_torque_right += extra_mean_compensation / 2.0f - extra_diff_compensation / 2.0f;
    
    // 计算支持力
    bool not_use_gravity = (context_->off_ground.off_ground_body || context_->controller_state == FSM_JUMP_LANDING || context_->controller_state == FSM_JUMP_FOLDING); // 离地没有重力
    vmc_l.F = M*g_l*(1 - (not_use_gravity ? 1 : 0)) + M * GRAVITY_ACC / 2.0f * leg_compensate_torque_left; // 左腿支持力
    vmc_r.F = M*g_r*(1 - (not_use_gravity ? 1 : 0)) + M * GRAVITY_ACC / 2.0f * leg_compensate_torque_right; // 右腿支持力

    // 计算并应用腿长导致的力偏差修正
    double delta_F = DELTA_F_A * (context_->geometric_left.l0 + context_->geometric_right.l0) / 2.0f + DELTA_F_B;
    vmc_l.F -= delta_F / 2.0f; // 修正左侧力
    vmc_r.F += delta_F / 2.0f; // 修正右侧力
    // 减去弹簧补偿力
    vmc_l.F -= context_->off_ground.F_spring_vmc_l;
    vmc_r.F -= context_->off_ground.F_spring_vmc_r;
    // 输出力限幅
    if (context_->controller_state == FSM_JUMP_STRETCHING) { // 跳跃伸展状态
        vmc_l.F = limit_range_function(vmc_l.F, -0.0f, 300.0f);
        vmc_r.F = limit_range_function(vmc_r.F, -0.0f, 300.0f);
    }
    else if (context_->controller_state == FSM_JUMP_FOLDING) { // 跳跃收腿状态
        vmc_l.F = limit_range_function(vmc_l.F, -200.0f, -100.0f);
        vmc_r.F = limit_range_function(vmc_r.F, -200.0f, -100.0f);
    }
    else { // 其他状态
        vmc_l.F = limit_range_function(vmc_l.F, -200.0f, 300.0f);
        vmc_r.F = limit_range_function(vmc_r.F, -200.0f, 300.0f);
    }
    // 力滤波防止跳变
    if (context_->controller_state == FSM_JUMP_STRETCHING || context_->controller_state == FSM_JUMP_FOLDING || context_->controller_state == FSM_JUMP_LANDING)
    {
        vmc_l.F = F_LOWPASS_ALPHA * vmc_l.F + (1.0f - F_LOWPASS_ALPHA) * F_l_last; // 低通滤波
        vmc_r.F = F_LOWPASS_ALPHA * vmc_r.F + (1.0f - F_LOWPASS_ALPHA) * F_r_last; // 低通滤波
    }

    F_l_last = vmc_l.F; // 更新上次左腿支持力
    F_r_last = vmc_r.F; // 更新上次右腿支持力
}

/**
 * @brief 虚拟控制量转换至实际力矩
 * @details 将虚拟模型控制输出转换为实际电机转矩
 */
void ChassisControllerActuator::convertToMotorTorque(void)
{
    /* 虚拟控制量转换至实际力矩 */
    Real_torque control_r; // 右侧实际控制量
    control_r.T1 = context_->geometric_right.l0_over_theta1 * vmc_r.F + context_->geometric_right.phi0_over_theta1 * vmc_r.Digital_torque; // 右侧T1力矩
    control_r.T2 = context_->geometric_right.l0_over_theta2 * vmc_r.F + context_->geometric_right.phi0_over_theta2 * vmc_r.Digital_torque; // 右侧T2力矩

    Real_torque control_l; // 左侧实际控制量    
    control_l.T1 = context_->geometric_left.l0_over_theta1 * vmc_l.F + context_->geometric_left.phi0_over_theta1 * vmc_l.Digital_torque; // 左侧T1力矩
    control_l.T2 = context_->geometric_left.l0_over_theta2 * vmc_l.F + context_->geometric_left.phi0_over_theta2 * vmc_l.Digital_torque; // 左侧T2力矩

    /* 转换至电机输出转矩,并作相关限幅 */
    context_->motor_TargetTorque.ckyf_troque_right_front = limit_range_function(control_r.T2, -30.0f, 30.0f); // 右前CKYF电机力矩限幅
    context_->motor_TargetTorque.ckyf_troque_right_back = limit_range_function(control_r.T1, -30.0f, 30.0f);  // 右后CKYF电机力矩限幅
    context_->motor_TargetTorque.ckyf_troque_left_front = limit_range_function(-control_l.T2, -30.0f, 30.0f); // 左前CKYF电机力矩限幅（注意符号）
    context_->motor_TargetTorque.ckyf_troque_left_back = limit_range_function(-control_l.T1, -30.0f, 30.0f);  // 左后CKYF电机力矩限幅（注意符号）

    // 控制驱动轮（行走轮）的力矩限幅 —— 摩擦极限防打滑
    float max_allow_torque_r = max_function(50.0f, context_->off_ground.F_support_r) * FRICTION_COEF * robot_param.R_wheel; // 右侧最大允许力矩
    float max_allow_torque_l = max_function(50.0f, context_->off_ground.F_support_l) * FRICTION_COEF * robot_param.R_wheel; // 左侧最大允许力矩
    context_->motor_TargetTorque.m3508_troque_right = limit_range_function(vmc_r.Wheel_torque, -max_allow_torque_r, max_allow_torque_r); // 右侧M3508电机力矩限幅
    context_->motor_TargetTorque.m3508_troque_left = limit_range_function(-vmc_l.Wheel_torque, -max_allow_torque_l, max_allow_torque_l); // 左侧M3508电机力矩限幅
    

    /* 如果IMU掉线，力矩赋0 */
    if (!context_->imu_connected)
    {
        context_->motor_TargetTorque.ckyf_troque_right_front = 0.0f; // 清除右前CKYF电机力矩
        context_->motor_TargetTorque.ckyf_troque_right_back = 0.0f;  // 清除右后CKYF电机力矩
        context_->motor_TargetTorque.ckyf_troque_left_front = 0.0f;  // 清除左前CKYF电机力矩
        context_->motor_TargetTorque.ckyf_troque_left_back = 0.0f;   // 清除左后CKYF电机力矩
        context_->motor_TargetTorque.m3508_troque_right = 0.0f;      // 清除右侧M3508电机力矩
        context_->motor_TargetTorque.m3508_troque_left = 0.0f;       // 清除左侧M3508电机力矩
    }
}

/**
 * @brief 获取功率限制因子
 * @param[in] left_wheel_torque 左轮力矩
 * @param[in] right_wheel_torque 右轮力矩
 * @return 返回功率限制因子
 * @details 根据轮子力矩计算功率限制因子
 */
float ChassisControllerActuator::getPowerLimitFactor(float left_wheel_torque, float right_wheel_torque)
{
    // 计算轮子电流
    float left_wheel_current = left_wheel_torque / M3508_TORQUE_CONSTANT;
    float right_wheel_current = right_wheel_torque / M3508_TORQUE_CONSTANT;

    // 估算功率
    float estimated_power = powerModel(left_wheel_current, context_->power_control.left_wheel_omega, POWER_MODEL_C1, POWER_MODEL_C2, POWER_MODEL_C3, POWER_MODEL_C4, POWER_MODEL_C5, GEARBOX_RATIO) + 
                            powerModel(right_wheel_current, context_->power_control.right_wheel_omega, POWER_MODEL_C1, POWER_MODEL_C2, POWER_MODEL_C3, POWER_MODEL_C4, POWER_MODEL_C5, GEARBOX_RATIO);
    // powermodel为一个关于电流的二次方程
    float power_limit_factor;

    if (estimated_power < context_->power_control.power_limit) // 功率未达上限
    {
        power_limit_factor = 1.0f;    
    }
    else // 功率达上限，计算功率限制因子
    {
        // 构建二次方程系数
        float A = POWER_MODEL_C1 * (left_wheel_current * left_wheel_current + right_wheel_current * right_wheel_current);
        float B = POWER_MODEL_C2 * (left_wheel_current * context_->power_control.left_wheel_omega + right_wheel_current * context_->power_control.right_wheel_omega);
        float C = POWER_MODEL_C3 * (context_->power_control.left_wheel_omega * context_->power_control.left_wheel_omega + context_->power_control.right_wheel_omega * context_->power_control.right_wheel_omega)
                + POWER_MODEL_C4 * (fabsf(context_->power_control.left_wheel_omega) + fabsf(context_->power_control.right_wheel_omega))
                + POWER_MODEL_C5 
                - context_->power_control.power_limit;

        // 求解二次方程
        float solution_1 = (-B + sqrtf(B * B - 4.0f * A * C)) / (2.0f * A);
        float solution_2 = (-B - sqrtf(B * B - 4.0f * A * C)) / (2.0f * A);

        // 选择合适的解
        if (solution_1 > 0.0f && solution_1 <= 1.0f)
        {
            power_limit_factor = solution_1;
        }
        else if (solution_2 > 0.0f && solution_2 <= 1.0f)
        {
            power_limit_factor = solution_2;
        }
        else
        {
            power_limit_factor = 1.0f;    
        }
    }
    
    return power_limit_factor;
}