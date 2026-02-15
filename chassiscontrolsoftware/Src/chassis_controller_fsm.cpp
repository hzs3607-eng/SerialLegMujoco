/* Includes ------------------------------------------------------------------*/
#include "../Inc/chassis_controller_fsm.h"


/* Global Variables ----------------------------------------------------------*/


/* Functions -----------------------------------------------------------------*/
/**
 * @brief 底盘控制器有限状态机构造函数
 * @param[in] context 底盘控制器上下文指针
 * @details 初始化状态机相关参数，设置时间间隔和机器人宽度参数
 */
ChassisControllerFSM::ChassisControllerFSM(ChassisControllerContext* context)
{
    time_interval = TIME_INTERVAL;  // 时间间隔,单位s
    Wid = robot_param.Wid;          // 机器人宽度参数

    context_ = context;             // 关联控制上下文
}

/**
 * @brief 重置状态机
 * @details 将状态机的所有内部状态变量重置为初始值
 */
void ChassisControllerFSM::reset(void)
{
    todo_reset_x = 0;                           // 清除X轴重置标志
    in_state_count = 0;                         // 清除状态持续计数
    jump_landing_touchground_count = 0;         // 清除跳跃着陆触地计数
    exp_inertial_acc = 0.0f;                    // 清除期望惯性加速度
    multi_round_angle_l = 0.0f;                 // 清除左电机多圈角度
    multi_round_angle_r = 0.0f;                 // 清除右电机多圈角度
    round_angle_diff_l = 0.0f;                  // 清除左电机圈数差值
    round_angle_diff_r = 0.0f;                  // 清除右电机圈数差值
    height_diff_last_initialize_ = false;        // 未初始化上次高度差值
    todo_record_multiround_angle = false;       // 清除记录多圈角度标志
}

/**
 * @brief 更新状态机
 * @details 执行状态机的主要更新逻辑，包括状态切换和动作执行
 */
void ChassisControllerFSM::update(void)
{
    switchFSM();        // 切换状态机状态
    actFSM();           // 执行状态机动作
    in_state_count += 1; // 累加当前状态持续计数
}

/**
 * @brief 切换状态机状态
 * @details 根据当前条件决定是否切换到新的控制状态
 */
void ChassisControllerFSM::switchFSM(void)
{
    // 定义静止速度阈值
    float velocity_w_still_threshold = 0.001f;  // TODO: 移动到params.h
    float velocity_x_still_threshold = 0.001f;

    // 检查是否有运动指令
    bool x_command = fabsf(context_->internal_command.desired_vel_x) > velocity_x_still_threshold; // 是否有前后运动指令
    bool omega_command = fabsf(context_->internal_command.desired_vel_omega) > velocity_w_still_threshold; // 是否有旋转运动指令
    
    // 计算phi0角偏差
    float phi0_deviation = fabsf(context_->geometric_left.phi0 - PI / 2.0f) + fabsf(context_->geometric_right.phi0 - PI / 2.0f);

    // 根据当前状态执行不同的状态切换逻辑
    switch (context_->controller_state)
    {
    case FSM_STATIONARY: // 静止状态
        if (detectAbnormal()) // 如果检测到异常
        {
            switchTo(FSM_LIFT_UP); // 切换到抬起状态
        }
        else if(x_command) // 如果有前进指令
        {
            switchTo(FSM_DIFF_DRIVE); // 切换到差动驱动状态
        }
        else if (omega_command) // 如果有旋转指令
        {
            if (fabsf(context_->internal_command.desired_vel_omega) > 8.0f || !context_->follow_gimbal)
                switchTo(FSM_SPINNING); // 切换到旋转状态
        }
        break;

    case FSM_SPINNING: // 旋转状态
        if (detectAbnormal()) // 如果检测到异常
        {
            switchTo(FSM_LIFT_UP); // 切换到抬起状态
        }
        else if(x_command) // 如果有前进指令
        {
            switchTo(FSM_DIFF_DRIVE); // 切换到差动驱动状态
        }
        else if (!omega_command) // 如果没有旋转指令
        {
            switchTo(FSM_STATIONARY); // 切换到静止状态
        }
        break;

    case FSM_DIFF_DRIVE: // 差动驱动状态
        if (detectAbnormal()) // 如果检测到异常
        {
            switchTo(FSM_LIFT_UP); // 切换到抬起状态
        }
        else if (detectClimbStair()) // 如果检测到爬楼梯
        {
            switchTo(FSM_CLIMBSTAIR_LIFTLEG); // 切换到爬楼梯抬腿状态
        }
        else if (!x_command) // 如果没有前进指令
        {
            if (!omega_command) // 如果也没有旋转指令
            {
                switchTo(FSM_STATIONARY); // 切换到静止状态
            }
            else // 如果有旋转指令
            {
                switchTo(FSM_SPINNING); // 切换到旋转状态
            }
        }
        break;
        
    case FSM_CLIMBSTAIR_LIFTLEG: // 爬楼梯抬腿状态
        if ((context_->guide_wheel_height_diff.to_leftwheel < 0.3f && context_->guide_wheel_height_diff.to_rightwheel < 0.03f) || in_state_count > 500) // 通过导向轮高度差判断
        {
            switchTo(FSM_CLIMBSTAIR_FOLDLEG); // 切换到爬楼梯收腿状态
        }
        break;
        
    case FSM_CLIMBSTAIR_FOLDLEG: // 爬楼梯收腿状态
        if ((context_->geometric_left.phi0 > PI / 2.5f) || (context_->geometric_right.phi0 > PI / 2.5f)) // 如果phi0角大于阈值
        {
            switchTo(FSM_STATIONARY); // 切换到静止状态
            clearDisplacement(); // 清除位移
            context_->external_command.l0_exp_raw = L0_STABLE; // 设置期望腿长原始值为稳定值
            context_->external_command.l0_exp = L0_MIN; // 设置期望腿长为最小值
        }
        break;

    case FSM_JUMP_SQUATING: // 跳跃下蹲状态
        if (context_->vmc_state.y_left < L0_MIN + 0.02f && context_->vmc_state.y_right < L0_MIN + 0.02f) {
            switchTo(FSM_JUMP_STRETCHING); // 切换到跳跃伸展状态
        }
        /* code */
        break;

    case FSM_JUMP_STRETCHING: // 跳跃伸展状态
        if (context_->vmc_state.y_left > L0_MAX - 0.02f && context_->vmc_state.y_right > L0_MAX - 0.02f) {
            switchTo(FSM_JUMP_FOLDING); // 切换到跳跃收腿状态
        }
        /* code */
        break;

    case FSM_JUMP_FOLDING: // 跳跃收腿状态
        if ((in_state_count >= 200)) {
            switchTo(FSM_JUMP_LANDING); // 切换到跳跃着陆状态
        }
        /* code */
        break;

    case FSM_JUMP_LANDING: // 跳跃着陆状态
        if (!context_->off_ground.off_ground_body) // 如果机器人未离地
        {
            jump_landing_touchground_count += 1; // 增加触地计数
        }
        else // 如果机器人离地
        {
            jump_landing_touchground_count = 0; // 重置触地计数
        }
        if ((jump_landing_touchground_count >= 20)) {
            switchTo(FSM_DIFF_DRIVE); // 切换到差动驱动状态
        }
        break;

    case FSM_LIFT_UP: // 抬起状态
        if (context_->imu_data.g_z_body > LIFTUP_2_SELFSAVE_GRAVITY_THRESHOLD) // 如果Z轴重力加速度超过阈值
        {
            if (fabsf(context_->imu_data.chassis_roll_speed_rad) + fabsf(context_->imu_data.chassis_pitch_speed_rad) + fabsf(context_->imu_data.chassis_yaw_speed_rad) < LIFTUP_2_SELFSAVE_ANGVEL_THRESHOLD)
                switchTo(FSM_SELF_SAVE); // 切换到自救状态
        }
        else if (phi0_deviation < LIFTUP_2_STAND_THRESHOLD) // 如果phi0角偏差小于阈值
        {
            switchTo(FSM_DIFF_DRIVE); // 切换到差动驱动状态
            clearDisplacement(); // 清除位移
            context_->external_command.l0_exp_raw = L0_STABLE; // 设置期望腿长原始值为稳定值
            context_->external_command.l0_exp = L0_STABLE; // 设置期望腿长为稳定值
        }
        break;

    case FSM_SELF_SAVE: // 自救状态
        if (context_->imu_data.g_z_body < SELFSAVE_2_LIFTUP_THRESHOLD) // 如果Z轴重力加速度低于阈值
        {
            switchTo(FSM_LIFT_UP); // 切换到抬起状态
        }
        break;

    case FSM_FORCE_LIFT_UP: // 强制抬起状态
        break;

    case FSM_MANUAL_LIFT_UP: // 手动抬起状态
        break;

    default:
        break;
    }
}

/**
 * @brief 执行状态机动作
 * @details 根据当前状态执行相应的控制动作
 */
void ChassisControllerFSM::actFSM(void)
{
    if (todo_record_multiround_angle) { // 记录电机的多圈角度
        uint8_t desired_report_mode = (getOutputMode() == Output_Torque) ? 1 : 2;
        if (isMotorReportModeAll(desired_report_mode)) {
            recordMultiRoundAngle();
            todo_record_multiround_angle = false;
        }
    }

    switch (context_->controller_state)
    {
    case FSM_STATIONARY: // 静止状态
        actTorqueControl(); // 执行力矩控制
        updateDisplacement(); // 更新位置
        checkAndReduceDisplacement(); // 检查并减少位置误差
        break;

    case FSM_SPINNING: // 旋转状态
        actTorqueControl(); // 执行力矩控制
        clearDisplacement(); // 移动时，位置积分容易累积误差，因此每次都清零
        break;

    case FSM_DIFF_DRIVE: // 差动驱动状态
        actTorqueControl(); // 执行力矩控制
        clearDisplacement(); // 清除位移
        break;
        
    case FSM_CLIMBSTAIR_LIFTLEG: // 爬楼梯抬腿状态
        actVelControl(); // 执行速度控制
        break;
        
    case FSM_CLIMBSTAIR_FOLDLEG: // 爬楼梯收腿状态
        actVelControl(); // 执行速度控制
        break;

    case FSM_JUMP_SQUATING: // 跳跃下蹲状态
        actTorqueControl(); // 执行力矩控制
        clearDisplacement(); // 清除位移
        /* code */
        break;

    case FSM_JUMP_STRETCHING: // 跳跃伸展状态
        actTorqueControl(); // 执行力矩控制
        clearDisplacement(); // 清除位移
        /* code */
        break;

    case FSM_JUMP_FOLDING: // 跳跃收腿状态
        actTorqueControl(); // 执行力矩控制
        clearDisplacement(); // 清除位移
        /* code */
        break;

    case FSM_JUMP_LANDING: // 跳跃着陆状态
        actTorqueControl(); // 执行力矩控制
        clearDisplacement(); // 清除位移
        break;

    case FSM_LIFT_UP: // 抬起状态
        // actPosControl();
        actVelControl(); // 执行速度控制
        break;

    case FSM_SELF_SAVE: // 自救状态
        actPosControl(); // 执行位置控制
        break;
    
    case FSM_FORCE_LIFT_UP: // 强制抬起状态：如果腿长大于某个阈值，使用速度控制进行抬起，否则位控。
        if ((context_->vmc_state.y_left > 0.25f) || (context_->vmc_state.y_right > 0.25f)) {
            actVelControl(); // 执行速度控制
        }
        else {
            actPosControl(); // 执行位置控制
        }
        // actPosControl();
        break;

    case FSM_MANUAL_LIFT_UP: // 手动抬起状态
        actVelControl(); // 执行速度控制

    default:
        break;
    }
}


/**
 * @brief 切换到指定状态
 * @param[in] state 目标控制器状态
 * @details 强制将状态机切换到指定的控制状态，并进行特殊处理
 */
void ChassisControllerFSM::switchTo(ControllerState state)
{
    // 切换状态
    context_->controller_state = state;
    // 特殊处理
    switch (state)
    {
    case FSM_STATIONARY: // 静止状态
        todo_reset_x = 100; // 设置X轴重置标志
        break;
    case FSM_JUMP_LANDING: // 跳跃着陆状态
        jump_landing_touchground_count = 0; // 清除触地计数
        break;
    default:
        break;
    }
    in_state_count = 0; // 重置状态持续计数
    todo_record_multiround_angle = true; // 切换状态做重置
}

/**
 * @brief 记录多圈角度
 * @details 计算并记录电机的多圈绝对角度值
 */
void ChassisControllerFSM::recordMultiRoundAngle(void)
{
    // 计算当前由于电机初始化位置不同导致的整圈差异
    float angle_diff_l = context_->geometric_left.theta2 - context_->geometric_left.theta1;
    float angle_diff_r = context_->geometric_right.theta2 - context_->geometric_right.theta1;

    round_angle_diff_l = 0.0f;
    round_angle_diff_r = 0.0f;

    if (angle_diff_l > THETA2_MINUS_THETA1_MAX) {
        while (angle_diff_l - round_angle_diff_l > THETA2_MINUS_THETA1_MAX) {
            round_angle_diff_l +=  2.0f * LQR_PI;
        }
    }
    else if (angle_diff_l < THETA2_MINUS_THETA1_MIN) {
        while (angle_diff_l - round_angle_diff_l < THETA2_MINUS_THETA1_MIN) {
            round_angle_diff_l -=  2.0f * LQR_PI;
        }
    }
    if (angle_diff_r > THETA2_MINUS_THETA1_MAX) {
        while (angle_diff_r - round_angle_diff_r > THETA2_MINUS_THETA1_MAX) {
            round_angle_diff_r +=  2.0f * LQR_PI;
        }
    }
    else if (angle_diff_r < THETA2_MINUS_THETA1_MIN) {
        while (angle_diff_r - round_angle_diff_r < THETA2_MINUS_THETA1_MIN) {
            round_angle_diff_r -=  2.0f * LQR_PI;
        }
    }
    // 记录当前的多圈phi0角
    float phi0_left = ((context_->geometric_left.theta1 + round_angle_diff_l) + context_->geometric_left.theta2) / 2.0f;
    float phi0_right = ((context_->geometric_right.theta1 + round_angle_diff_r) + context_->geometric_right.theta2) / 2.0f;
    multi_round_angle_l = phi0_left - PI / 2.0f - atan2f(sinf(context_->geometric_left.phi0 - PI / 2.0f), cosf(context_->geometric_left.phi0 - PI / 2.0f));
    multi_round_angle_r = phi0_right - PI / 2.0f - atan2f(sinf(context_->geometric_right.phi0 - PI / 2.0f), cosf(context_->geometric_right.phi0 - PI / 2.0f));
}

/**
 * @brief 计算速度期望
 * @details 根据外部命令计算速度期望值，并考虑安全限制
 */
void ChassisControllerFSM::calculateVelocityCommand(void)
{
    context_->internal_command.desired_vel_omega = context_->external_command.omega_plan; // 设置期望角速度
    float raw_vx = context_->external_command.velocity_x_plan; // 获取原始X轴速度计划
    // 根据腿长调整X轴速度上限
    raw_vx = clip(0.0f, VEL_X_LOWER + (VEL_X_UPPER - VEL_X_LOWER) * min_function((L0_MAX - context_->external_command.l0_exp) / (L0_MAX - L0_STABLE), 1.0f), raw_vx);
    context_->internal_command.desired_vel_x = raw_vx; // 设置期望X轴速度

    // 计算最大允许惯性加速度
    float max_inertial_acc = INERTIAL_ACC_LIMIT_RATIO * 9.8f * Wid / (context_->vmc_state.y_left + context_->vmc_state.y_right);
    exp_inertial_acc = max_function(fabsf(context_->internal_command.desired_vel_omega), fabsf(context_->vmc_state.center_omega_yaw)) * max_function(fabsf(context_->internal_command.desired_vel_x), fabsf(context_->vmc_state.center_v_x));
    if (exp_inertial_acc > max_inertial_acc){
        float scale = sqrtf(max_inertial_acc / exp_inertial_acc);
        context_->internal_command.desired_vel_omega *= scale; // 缩放角速度
        context_->internal_command.desired_vel_x *= scale; // 缩放X轴速度
    }
}


/**
 * @brief 计算yaw角期望位置
 * @details 根据跟随模式或手动控制计算yaw角期望位置
 */
void ChassisControllerFSM::calculatePositionCommand(void)
{
    // 计算yaw角期望位置
    if (context_->follow_gimbal) // 如果跟随云台
    {
        context_->internal_command.yaw_expectation = context_->vmc_state.center_yaw + context_->external_command.angle_target; // 使用云台角度加上目标角度
    }
    else // 如果不跟随云台
    {
        context_->internal_command.yaw_expectation += context_->internal_command.desired_vel_omega * time_interval; // 根据角速度累加角度
        exp_inertial_acc = max_function(fabsf(context_->internal_command.desired_vel_omega), fabsf(context_->vmc_state.center_omega_yaw)) * max_function(fabsf(context_->internal_command.desired_vel_x), fabsf(context_->vmc_state.center_v_x));
        float max_allow_yaw_error = max_function(0.0f, 1.0f - fabsf(exp_inertial_acc)); // 计算最大允许yaw角误差
        max_allow_yaw_error = min_function(max_allow_yaw_error, max_function(0.0f, HIGH_YAW_SPEED_THRESHOLD - context_->vmc_state.center_omega_yaw));
        // 限制yaw角误差
        float yaw_expectation_temp = fmodf(context_->internal_command.yaw_expectation - context_->vmc_state.center_yaw + LQR_PI, 2.0f * LQR_PI);
        if (yaw_expectation_temp < 0.0f)  yaw_expectation_temp += 2.0f * LQR_PI;
        yaw_expectation_temp = yaw_expectation_temp - LQR_PI;
        context_->internal_command.yaw_expectation = context_->vmc_state.center_yaw + clip(0.0f, max_allow_yaw_error, yaw_expectation_temp);
    }
}

/**
 * @brief 计算目标高度
 * @details 根据当前状态和外部命令计算期望的高度
 */
void ChassisControllerFSM::calculateHeightCommand(void)
{
    float max_allow_l0 = L0_MAX; // 最大允许腿长
    if (context_->controller_state != FSM_SPINNING) // 如果不是旋转状态
    {
        exp_inertial_acc = max_function(fabsf(context_->internal_command.desired_vel_omega), fabsf(context_->vmc_state.center_omega_yaw)) * max_function(fabsf(context_->internal_command.desired_vel_x), fabsf(context_->vmc_state.center_v_x));
        if (exp_inertial_acc > INERTIAL_ACC_HEIGHT_LIMIT_THRESHOLD)
            max_allow_l0 = max_function(L0_SQUAT, L0_MAX - (exp_inertial_acc - INERTIAL_ACC_HEIGHT_LIMIT_THRESHOLD) / (2.0f * INERTIAL_ACC_HEIGHT_LIMIT_THRESHOLD) * (L0_MAX - L0_MIN)); // 根据惯性加速度调整最大允许腿长
    }
    float desired_l0 = min_function(max_allow_l0, context_->external_command.l0_exp); // 获取期望腿长

    if (context_->controller_state == FSM_DIFF_DRIVE || context_->controller_state == FSM_STATIONARY || context_->controller_state == FSM_SPINNING) // 不跳跃
    {
        if (context_->off_ground.off_ground_body) // 如果机器人离地
        {
            float mean_l0 = (context_->vmc_state.y_left + context_->vmc_state.y_right) / 2.0f; // 计算平均腿长
            float target_l0 = limit_range_function(mean_l0 + 0.02f, L0_MIN, L0_MAX); // 计算目标腿长
            context_->internal_command.target_y_l = target_l0; // 设置左腿目标高度
            context_->internal_command.target_y_r = target_l0; // 设置右腿目标高度
        }
        else // 如果机器人未离地
        {
            float inertial_acc_compensation = 0.0f; // 惯性加速度补偿
            if((fabsf(context_->internal_command.desired_vel_omega) > 0.001f) && context_->follow_gimbal) // 如果有旋转且跟随云台
            {
                float inertial_acc = context_->vmc_state.center_v_x * context_->vmc_state.center_omega_yaw; // 计算惯性加速度
                float abs_inertial_acc = fabsf(inertial_acc);
                if (abs_inertial_acc > INERTIAL_ACC_ROLL_COMPENSATE_UPPER) // 如果惯性加速度超过上限
                {
                    inertial_acc_compensation = INERTIAL_ACC_ROLL_COMPENSATE_LIMIT * inertial_acc / abs_inertial_acc;
                }
                else if (abs_inertial_acc > INERTIAL_ACC_ROLL_COMPENSATE_LOWER) // 如果惯性加速度在中范围内
                {
                    inertial_acc_compensation = INERTIAL_ACC_ROLL_COMPENSATE_LIMIT * inertial_acc / abs_inertial_acc * (abs_inertial_acc - INERTIAL_ACC_ROLL_COMPENSATE_LOWER) / (INERTIAL_ACC_ROLL_COMPENSATE_UPPER - INERTIAL_ACC_ROLL_COMPENSATE_LOWER);
                }
            }
            float length_diff, length_mean, length_diff_dot; // 长度差值、均值和导数
            calculateTargetHeight(&length_diff, &length_mean, &length_diff_dot, desired_l0, inertial_acc_compensation); // 计算目标高度参数
            context_->internal_command.target_y_l = length_mean + length_diff / 2.0f; // 设置左腿目标高度
            context_->internal_command.target_y_r = length_mean - length_diff / 2.0f; // 设置右腿目标高度
            
            // if (context_->off_ground.off_ground_l){
            //     context_->internal_command.target_dot_y_l = 1.0f;
            //     context_->internal_command.target_dot_y_r = 0.0f;
            // }
            // else if (context_->off_ground.off_ground_r){
            //     context_->internal_command.target_dot_y_l = 0.0f;
            //     context_->internal_command.target_dot_y_r = 1.0f;
            // }
            // else{
                // context_->internal_command.target_dot_y_l = length_diff_dot / 4.0f;
                // context_->internal_command.target_dot_y_r = -length_diff_dot / 4.0f;
                context_->internal_command.target_dot_y_l = 0.0f; // 设置左腿高度变化率
                context_->internal_command.target_dot_y_r = 0.0f; // 设置右腿高度变化率
            // }
        }
    }
    else if (context_->controller_state == FSM_JUMP_SQUATING) { // 如果是跳跃下蹲状态
        context_->internal_command.target_y_l = L0_MIN; // 设置左腿目标高度为最小值
        context_->internal_command.target_y_r = L0_MIN; // 设置右腿目标高度为最小值
        context_->internal_command.target_dot_y_l = 0.0f; // 设置左腿高度变化率为0
        context_->internal_command.target_dot_y_r = 0.0f; // 设置右腿高度变化率为0
    }
    else if (context_->controller_state == FSM_JUMP_STRETCHING) { // 如果是跳跃伸展状态
        context_->internal_command.target_y_l = L0_MAX; // 设置左腿目标高度为最大值
        context_->internal_command.target_y_r = L0_MAX; // 设置右腿目标高度为最大值
        context_->internal_command.target_dot_y_l = 0.0f; // 设置左腿高度变化率为0
        context_->internal_command.target_dot_y_r = 0.0f; // 设置右腿高度变化率为0
    }
    else if (context_->controller_state == FSM_JUMP_FOLDING) { // 如果是跳跃收腿状态
        context_->internal_command.target_y_l = L0_JUMP_FOLDING; // 设置左腿目标高度为跳跃收腿值
        context_->internal_command.target_y_r = L0_JUMP_FOLDING; // 设置右腿目标高度为跳跃收腿值
        context_->internal_command.target_dot_y_l = 0.0f; // 设置左腿高度变化率为0
        context_->internal_command.target_dot_y_r = 0.0f; // 设置右腿高度变化率为0
    }
    else if (context_->controller_state == FSM_JUMP_LANDING) { // 如果是跳跃着陆状态
        context_->internal_command.target_y_l = L0_STABLE; // 设置左腿目标高度为稳定值
        context_->internal_command.target_y_r = L0_STABLE; // 设置右腿目标高度为稳定值
        context_->internal_command.target_dot_y_l = 0.0f; // 设置左腿高度变化率为0
        context_->internal_command.target_dot_y_r = 0.0f; // 设置右腿高度变化率为0
    }
}

/**
 * @brief 计算目标theta角度
 * @details 根据当前腿长计算期望的theta角度
 */
void ChassisControllerFSM::calculateThetaCommand(void)
{
    float l0_low = L0_STABLE; // 低腿长值
    float target_theta_low = THETA_STABLE_LOW; // 低目标角度
    float l0_high = L0_MAX; // 高腿长值
    float target_theta_high = THETA_STABLE_HIGH; // 高目标角度
    // 根据左腿长度插值计算目标theta角度
    context_->internal_command.target_theta_l = target_theta_low + (target_theta_high - target_theta_low) * limit_range_function((context_->vmc_state.y_left - l0_low) / (l0_high - l0_low), 0.0f, 1.0f);
    // 根据右腿长度插值计算目标theta角度
    context_->internal_command.target_theta_r = target_theta_low + (target_theta_high - target_theta_low) * limit_range_function((context_->vmc_state.y_right - l0_low) / (l0_high - l0_low), 0.0f, 1.0f);
}

/**
 * @brief 目标高度计算
 * @param[out] length_diff 长度差值输出
 * @param[out] length_mean 长度均值输出
 * @param[out] length_diff_dot 长度差值导数输出
 * @param[in] input_l0 输入的腿长
 * @param[in] centrifugal_acc 离心加速度
 * @details 根据输入的腿长和离心加速度计算目标高度参数
 */
void ChassisControllerFSM::calculateTargetHeight(float *length_diff, float* length_mean, float *length_diff_dot, float input_l0, float centrifugal_acc)
{
    // ----------- 不考虑离心加速度的情况 -----------
    //  计算高度差
    float height_l = - Wid / 2 * context_->imu_data.sin_roll_chassis + context_->vmc_state.y_left * context_->imu_data.cos_roll_chassis; // 左侧高度
    float height_r = Wid / 2 * context_->imu_data.sin_roll_chassis + context_->vmc_state.y_right * context_->imu_data.cos_roll_chassis; // 右侧高度
    float height_diff = height_l - height_r; // 高度差
    if (!height_diff_last_initialize_) height_diff_last_ = height_diff; // 如果未初始化，则使用当前高度差
    *length_diff_dot = (height_diff - height_diff_last_) / time_interval; // 计算高度差变化率
    height_diff_last_ = height_diff; // 更新上次高度差
    height_diff_last_initialize_ = true; // 标记已初始化
    // 检查高度差是否超限
    if (height_diff > (L0_MAX - L0_MIN))
    {
        height_diff = L0_MAX - L0_MIN;
    }
    else if (height_diff < -(L0_MAX - L0_MIN))
    {
        height_diff = -(L0_MAX - L0_MIN);
    }
    //  检查左右腿高度均值期望是否超限
    if (input_l0 - (L0_MAX + L0_MIN) / 2.0f > (L0_MAX - L0_MIN) / 2.0f - fabsf(height_diff / 2.0f))
    {
        input_l0 = (L0_MAX + L0_MIN) / 2.0f + (L0_MAX - L0_MIN) / 2.0f - fabsf(height_diff / 2.0f);
    }
    else if (input_l0 - (L0_MAX + L0_MIN) / 2.0f < -((L0_MAX - L0_MIN) / 2.0f - fabsf(height_diff / 2.0f)))
    {
        input_l0 = (L0_MAX + L0_MIN) / 2.0f - (L0_MAX - L0_MIN) / 2.0f + fabsf(height_diff / 2.0f);
    }
    *length_mean = input_l0; // 输出长度均值

    // ------------- 考虑离心加速度的情况 -----------
    //  计算等效roll角（用重力加速度和离心加速度的合矢量代替重力加速度，进而计算roll角）
    float tan_delta_roll = centrifugal_acc / (9.8f); // 计算roll角正切值
    float sin_delta_roll = tan_delta_roll / sqrtf(1 + tan_delta_roll * tan_delta_roll); // 计算roll角正弦值
    float cos_delta_roll = 1 / sqrtf(1 + tan_delta_roll * tan_delta_roll); // 计算roll角余弦值

    float s_roll_equivalent = context_->imu_data.sin_roll_chassis * cos_delta_roll + context_->imu_data.cos_roll_chassis * sin_delta_roll; // 等效sin roll角
    float c_roll_equivalent = context_->imu_data.cos_roll_chassis * cos_delta_roll - context_->imu_data.sin_roll_chassis * sin_delta_roll; // 等效cos roll角

    float height_l_equivalent = - Wid / 2 * s_roll_equivalent + context_->vmc_state.y_left * c_roll_equivalent; // 等效左侧高度
    float height_r_equivalent = Wid / 2 * s_roll_equivalent + context_->vmc_state.y_right * c_roll_equivalent; // 等效右侧高度
    *length_diff = clip(0.0f, min_function(L0_MAX - input_l0, input_l0 - L0_MIN) * 2, height_l_equivalent - height_r_equivalent); // 输出长度差值，限制在合理范围内
    //  计算左右腿目标高度
    // *target_y_l = input_l0 + height_diff_equivalent / 2.0f;
    // *target_y_r = input_l0 - height_diff_equivalent / 2.0f;
}

/**
 * @brief 更新位移
 * @details 根据当前运动状态更新累积位移
 */
void ChassisControllerFSM::updateDisplacement(void)
{
    context_->internal_command.x_expectation += context_->internal_command.desired_vel_x * time_interval; // 根据X轴速度更新期望X位置
    context_->internal_command.x_expectation = clip(0.0f,0.5f,context_->internal_command.x_expectation); // 限制X期望位置在合理范围内
}

/**
 * @brief 检查并减小位移
 * @details 检查位移是否超限，如超限则采取措施减小
 */
void ChassisControllerFSM::checkAndReduceDisplacement(void)
{
    if ((todo_reset_x>0 && fabsf(context_->vmc_state.center_v_x) < 0.1f)) // 如果需要重置X且X轴速度较小
    {
        context_->vmc_state.center_x *= 0.95f; // 衰减实际X位置
        context_->internal_command.x_expectation *= 0.95f; // 衰减期望X位置
        todo_reset_x--; // 减少重置计数
        todo_reset_x = max_function(0, todo_reset_x); // 确保计数不为负
    }
}

/**
 * @brief 清除位移
 * @details 将累积位移清零
 */
void ChassisControllerFSM::clearDisplacement(void)
{
    context_->vmc_state.center_x = 0.0f; // 清除实际X位置
    context_->internal_command.x_expectation = 0.0f; // 清除期望X位置
}

/**
 * @brief 重新分配功率限制
 * @details 根据当前状态重新分配各电机的功率限制
 */
void ChassisControllerFSM::realocatePowerLimit(void)
{
    float compensation_factor = 1.0f; // 补偿因子
    // 判断是否为导论近地的异常状态
    if (context_->guide_wheel_height_diff.to_leftwheel < 0.05f || context_->guide_wheel_height_diff.to_rightwheel < 0.05f)
        compensation_factor = 2.0f; // 如是，考虑增大允许功率
    // 如是，考虑增大允许功率
    float minimum_required_torque_l = robot_param.M / 2.0f * GRAVITY_ACC * context_->vmc_state.y_left * context_->vmc_state.sin_theta_left; // 左侧最小所需扭矩
    float minimum_required_torque_r = robot_param.M / 2.0f * GRAVITY_ACC * context_->vmc_state.y_right * context_->vmc_state.sin_theta_right; // 右侧最小所需扭矩

    float minimum_reuiqred_current_l = minimum_required_torque_l / M3508_TORQUE_CONSTANT; // 左侧最小所需电流
    float minimum_reuiqred_current_r = minimum_required_torque_r / M3508_TORQUE_CONSTANT; // 右侧最小所需电流

    // 计算最小所需功率
    float minimum_required_power = powerModel(fabsf(minimum_reuiqred_current_l), fabsf(context_->power_control.left_wheel_omega), POWER_MODEL_C1, POWER_MODEL_C2, POWER_MODEL_C3, POWER_MODEL_C4, POWER_MODEL_C5, GEARBOX_RATIO) + 
                                    powerModel(fabsf(minimum_reuiqred_current_r), fabsf(context_->power_control.right_wheel_omega), POWER_MODEL_C1, POWER_MODEL_C2, POWER_MODEL_C3, POWER_MODEL_C4, POWER_MODEL_C5, GEARBOX_RATIO);


    context_->power_control.power_limit = max_function(context_->power_control.power_limit, compensation_factor * minimum_required_power); // 更新功率限制
}

/**
 * @brief 执行力矩控制
 * @details 根据计算结果执行力矩控制模式
 */
void ChassisControllerFSM::actTorqueControl(void)
{
    calculateVelocityCommand();   // 计算速度命令
    calculatePositionCommand();   // 计算位置命令
    calculateHeightCommand();     // 计算高度命令
    calculateThetaCommand();      // 计算角度命令
    realocatePowerLimit();        // 重新分配功率限制

    clearMotorPosCommand();       // 清除电机位置命令
    clearMotorVelCommand();       // 清除电机速度命令
}

/**
 * @brief 执行速度控制
 * @details 根据计算结果执行速度控制模式
 */
void ChassisControllerFSM::actVelControl(void)
{
    // 决定当前期望的phi0角和l0
    float vmc_l0_target, vmc_phi0_target_l, vmc_phi0_target_r;

    float target_dot_l0_l, target_dot_l0_r, target_dot_phi0_l, target_dot_phi0_r;
    if ((context_->controller_state == FSM_LIFT_UP) || (context_->controller_state == FSM_MANUAL_LIFT_UP) || (context_->controller_state == FSM_FORCE_LIFT_UP)) // 如果是抬起状态
    {
        vmc_phi0_target_l = PI / 2.0f; // 左侧phi0目标值
        vmc_phi0_target_r = PI / 2.0f; // 右侧phi0目标值
        vmc_l0_target = 0.18f; // 腿长目标值
        
        vmc_phi0_target_l += multi_round_angle_l; // 加上多圈角度
        vmc_phi0_target_r += multi_round_angle_r; // 加上多圈角度
        // 计算当前phi0角
        float phi0_left = ((context_->geometric_left.theta1 + round_angle_diff_l) + context_->geometric_left.theta2) / 2.0f;
        float phi0_right = ((context_->geometric_right.theta1 + round_angle_diff_r) + context_->geometric_right.theta2) / 2.0f;

        // 计算误差
        float phi0_error_l = vmc_phi0_target_l - phi0_left;
        float phi0_error_r = vmc_phi0_target_r - phi0_right;
        float l0_error_l = vmc_l0_target - context_->vmc_state.y_left;
        float l0_error_r = vmc_l0_target - context_->vmc_state.y_right;

        float error_limit = 0.1 * 0.1;
        phi0_error_l = clip(0.0f, error_limit / fabsf(l0_error_l), phi0_error_l);
        phi0_error_r = clip(0.0f, error_limit / fabsf(l0_error_r), phi0_error_r);

        float kp_l0 = 200.0f; // l0的P增益
        float kp_phi0 = 250.0f; // phi0的P增益

        target_dot_l0_l = kp_l0 * l0_error_l; // 左侧l0目标变化率
        target_dot_l0_r = kp_l0 * l0_error_r; // 右侧l0目标变化率
        target_dot_phi0_l = kp_phi0 * phi0_error_l; // 左侧phi0目标变化率
        target_dot_phi0_r = kp_phi0 * phi0_error_r; // 右侧phi0目标变化率
    }
    else if (context_->controller_state == FSM_CLIMBSTAIR_LIFTLEG) // 如果是爬楼梯抬腿状态
    {
        // vmc_l0_target = (context_->vmc_state.y_l + context_->vmc_state.y_r) / 2.0f;
        // vmc_phi0_target_l = 0.0f;
        // vmc_phi0_target_r = 0.0f;
        float v_y = 80.0f; // Y方向速度
        float v_x = -50.0f; // X方向速度

        target_dot_l0_l = (v_x) * context_->vmc_state.sin_theta_left - v_y * context_->vmc_state.cos_theta_left; // 左侧l0目标变化率
        target_dot_l0_r = (v_x) * context_->vmc_state.sin_theta_right - v_y * context_->vmc_state.cos_theta_right; // 右侧l0目标变化率
        target_dot_phi0_l = ((v_x) * context_->vmc_state.cos_theta_left + v_y * context_->vmc_state.sin_theta_left) / context_->vmc_state.y_left; // 左侧phi0目标变化率
        target_dot_phi0_r = ((v_x) * context_->vmc_state.cos_theta_right + v_y * context_->vmc_state.sin_theta_right) / context_->vmc_state.y_right; // 右侧phi0目标变化率
    }
    else if (context_->controller_state == FSM_CLIMBSTAIR_FOLDLEG) // 如果是爬楼梯收腿状态
    {
        // vmc_l0_target = (context_->vmc_state.y_l + context_->vmc_state.y_r) / 2.0f;
        // vmc_phi0_target_l = 0.0f;
        // vmc_phi0_target_r = 0.0f;
        float v_x = 50.0f; // X方向速度
        target_dot_l0_l = (v_x) * context_->vmc_state.sin_theta_left; // 左侧l0目标变化率
        target_dot_l0_r = (v_x) * context_->vmc_state.sin_theta_right; // 右侧l0目标变化率
        target_dot_phi0_l = ((v_x) * context_->vmc_state.cos_theta_left) / context_->vmc_state.y_left; // 左侧phi0目标变化率
        target_dot_phi0_r = ((v_x) * context_->vmc_state.cos_theta_right) / context_->vmc_state.y_right; // 右侧phi0目标变化率
        
        vmc_l0_target = 0.16f; // 腿长目标值
        float l0_error_l = vmc_l0_target - context_->vmc_state.y_left; // 左侧l0误差
        float l0_error_r = vmc_l0_target - context_->vmc_state.y_right; // 右侧l0误差
        float kp_l0 = 400.0f; // l0的P增益
        target_dot_l0_l += kp_l0 * l0_error_l; // 累加l0目标变化率
        target_dot_l0_r += kp_l0 * l0_error_r; // 累加l0目标变化率
    }

    // 转换为电机角速度期望
    float target_dot_theta1_l = context_->geometric_right.theta1_over_l0 * target_dot_l0_l + context_->geometric_left.theta1_over_phi0 * target_dot_phi0_l; // 左后电机目标角速度
    float target_dot_theta2_l = context_->geometric_right.theta2_over_l0 * target_dot_l0_l + context_->geometric_left.theta2_over_phi0 * target_dot_phi0_l; // 左前电机目标角速度
    float target_dot_theta1_r = context_->geometric_right.theta1_over_l0 * target_dot_l0_r + context_->geometric_left.theta1_over_phi0 * target_dot_phi0_r; // 右后电机目标角速度
    float target_dot_theta2_r = context_->geometric_right.theta2_over_l0 * target_dot_l0_r + context_->geometric_left.theta2_over_phi0 * target_dot_phi0_r; // 右前电机目标角速度

    setMotorVelCommand(target_dot_theta1_l, target_dot_theta2_l, target_dot_theta1_r, target_dot_theta2_r); // 设置电机速度命令

    clearMotorTorqueCommand(); // 清除电机力矩命令
    clearMotorPosCommand(); // 清除电机位置命令
}

/**
 * @brief 执行位置控制
 * @details 根据计算结果执行位置控制模式
 */
void ChassisControllerFSM::actPosControl(void)
{
    // 在自救时计算云台控制量。以下使用角度制
    float gimbal_pitch = 0.0f;  // 未使用
    float gimbal_yaw_target = 0.0f;
    // if (context_->controller_state == FSM_SELF_SAVE)
    if (context_->controller_state == FSM_SELF_SAVE || context_->controller_state == FSM_LIFT_UP)   // 暂时在LIFTUP时也控云台以调试
    {
        if (fabsf(context_->location_yaw) < 90.0f) // 如果yaw角小于90度
        {
            gimbal_yaw_target = 0.0f; // 设置目标yaw角为0
        }
        else // 如果yaw角大于等于90度
        {
            gimbal_yaw_target = 180.0f; // 设置目标yaw角为180度
        }
    }
    
    float error_gimbal_yaw = fmodf(gimbal_yaw_target -context_->location_yaw + 180.0f, 360.0f); // 计算yaw角误差
    if (error_gimbal_yaw < 0.0f)    error_gimbal_yaw += 360.0f; // 确保误差为正值
    error_gimbal_yaw -= 180.0f; // 调整误差范围
    
    float max_phi0_error = 0.5f; // 最大phi0误差
    // 决定当前期望的phi0角和l0
    float vmc_l0_target, vmc_phi0_target_l, vmc_phi0_target_r;
    if (context_->controller_state == FSM_LIFT_UP) // 如果是抬起状态
    {
        vmc_phi0_target_l = PI / 2.0f; // 左侧phi0目标值
        vmc_phi0_target_r = PI / 2.0f; // 右侧phi0目标值
        // vmc_l0_target = (fabsf(vmc_phi0_target - phi0_left) + fabsf(vmc_phi0_target - phi0_right)) > 0.3f ? 0.2f : 0.17f;
        vmc_l0_target = L0_MIN; // 腿长目标值设为最小值
        max_phi0_error = 0.1f; // 最大phi0误差
    }
    else if (context_->controller_state == FSM_FORCE_LIFT_UP) // 如果是强制抬起状态
    {
        vmc_phi0_target_l = PI / 2.0f; // 左侧phi0目标值
        vmc_phi0_target_r = PI / 2.0f; // 右侧phi0目标值
        vmc_l0_target = 0.18f; // 腿长目标值
        max_phi0_error = 0.3f; // 最大phi0误差
    }
    else if (context_->controller_state == FSM_CLIMBSTAIR_LIFTLEG) // 如果是爬楼梯抬腿状态
    {
        vmc_l0_target = 0.2f; // 腿长目标值
        vmc_phi0_target_l = 0.0f; // 左侧phi0目标值
        vmc_phi0_target_r = 0.0f; // 右侧phi0目标值
        max_phi0_error = 0.3f; // 最大phi0误差
    }
    else // 其他状态
    {
        vmc_l0_target = 0.39f; // 腿长目标值
        // error_gimbal_yaw 与 context_->imu_data.g_x_body同号先动右腿，异号先动左腿
        if (context_->imu_data.g_x_body > 0.0f) // 如果X轴重力分量大于0
        {
            if (error_gimbal_yaw > 25.0f) // 如果yaw角误差大于25度
            {
                vmc_phi0_target_l = PI / 2.0f; // 左侧phi0目标值
                vmc_phi0_target_r = - PI; // 右侧phi0目标值
            }
            else if (error_gimbal_yaw < -20.0f) // 如果yaw角误差小于-20度
            {
                vmc_phi0_target_l = - PI; // 左侧phi0目标值
                vmc_phi0_target_r = PI / 2.0f; // 右侧phi0目标值
            }
            else // 其他情况
            {
                vmc_phi0_target_l = - PI; // 左侧phi0目标值
                vmc_phi0_target_r = - PI; // 右侧phi0目标值
            }
        }
        else // 如果X轴重力分量小于等于0
        {
            if (error_gimbal_yaw > 25.0f) // 如果yaw角误差大于25度
            {
                vmc_phi0_target_l = 2.0f * PI; // 左侧phi0目标值
                vmc_phi0_target_r = PI * 0.5f; // 右侧phi0目标值
            }
            else if (error_gimbal_yaw < -20.0f) // 如果yaw角误差小于-20度
            {
                vmc_phi0_target_l = PI * 0.5f; // 左侧phi0目标值
                vmc_phi0_target_r = 2.0f * PI; // 右侧phi0目标值
            }
            else // 其他情况
            {
                vmc_phi0_target_l = 2.0f * PI; // 左侧phi0目标值
                vmc_phi0_target_r = 2.0f * PI; // 右侧phi0目标值
            }
        }
    }
    vmc_phi0_target_l += multi_round_angle_l; // 加上多圈角度
    vmc_phi0_target_r += multi_round_angle_r; // 加上多圈角度
    // 限制phi0角误差
    float phi0_left = ((context_->geometric_left.theta1 + round_angle_diff_l) + context_->geometric_left.theta2) / 2.0f; // 左侧phi0角
    float phi0_right = ((context_->geometric_right.theta1 + round_angle_diff_r) + context_->geometric_right.theta2) / 2.0f; // 右侧phi0角
    float target_theta1_left, target_theta2_left, target_theta1_right, target_theta2_right;
    backwardKinematic(vmc_l0_target, clip(phi0_left, max_phi0_error, vmc_phi0_target_l), &target_theta1_left, &target_theta2_left); // 逆运动学计算左侧目标角度
    backwardKinematic(vmc_l0_target, clip(phi0_right, max_phi0_error, vmc_phi0_target_r), &target_theta1_right, &target_theta2_right); // 逆运动学计算右侧目标角度

    target_theta1_left -= round_angle_diff_l; // 减去圈数差值
    target_theta1_right -= round_angle_diff_r; // 减去圈数差值

    // 位控量赋值
    setMotorPosCommand(target_theta1_left, target_theta2_left, target_theta1_right, target_theta2_right, gimbal_pitch, error_gimbal_yaw); // 设置电机位置命令
    clearMotorTorqueCommand(); // 清除电机力矩命令
    clearMotorVelCommand(); // 清除电机速度命令
}

/**
 * @brief 清除电机位置命令
 * @details 将电机位置命令设置为默认值
 */
void ChassisControllerFSM::clearMotorPosCommand(void)
{
    context_->motor_TargetPos.ckyf_pos_left_back = 0.0f;      // 左后CKYF电机位置命令
    context_->motor_TargetPos.ckyf_pos_left_front = 0.0f;     // 左前CKYF电机位置命令
    context_->motor_TargetPos.ckyf_pos_right_back = 0.0f;     // 右后CKYF电机位置命令
    context_->motor_TargetPos.ckyf_pos_right_front = 0.0f;    // 右前CKYF电机位置命令
    context_->motor_TargetPos.gimbal_pitch = 0.0f;            // 云台俯仰角命令
    context_->motor_TargetPos.gimbal_yaw = 0.0f;              // 云台偏航角命令
}

/**
 * @brief 清除电机速度命令
 * @details 将电机速度命令设置为默认值
 */
void ChassisControllerFSM::clearMotorVelCommand(void)
{
    context_->motor_TargetVel.ckyf_velocity_left_back = 0.0f;     // 左后CKYF电机速度命令
    context_->motor_TargetVel.ckyf_velocity_left_front = 0.0f;    // 左前CKYF电机速度命令
    context_->motor_TargetVel.ckyf_velocity_right_back = 0.0f;    // 右后CKYF电机速度命令
    context_->motor_TargetVel.ckyf_velocity_right_front = 0.0f;   // 右前CKYF电机速度命令
}

/**
 * @brief 清除电机力矩命令
 * @details 将电机力矩命令设置为默认值
 */
void ChassisControllerFSM::clearMotorTorqueCommand(void)
{
    context_->motor_TargetTorque.ckyf_troque_left_back = 0.0f;    // 左后CKYF电动力矩命令
    context_->motor_TargetTorque.ckyf_troque_left_front = 0.0f;   // 左前CKYF电动力矩命令
    context_->motor_TargetTorque.ckyf_troque_right_back = 0.0f;   // 右后CKYF电动力矩命令
    context_->motor_TargetTorque.ckyf_troque_right_front = 0.0f;  // 右前CKYF电动力矩命令
    context_->motor_TargetTorque.m3508_troque_left = 0.0f;        // 左侧M3508电动力矩命令
    context_->motor_TargetTorque.m3508_troque_right = 0.0f;       // 右侧M3508电动力矩命令
}

/**
 * @brief 检测异常状态
 * @return 返回true表示检测到异常，false表示正常
 * @details 检测系统是否存在异常运行状态
 */
bool ChassisControllerFSM::detectAbnormal(void)
{
    float left_wheel_to_center_x = fabsf(context_->vmc_state.sin_theta_left * context_->vmc_state.y_left); // 左轮到中心的X距离
    float right_wheel_to_center_x = fabsf(context_->vmc_state.sin_theta_right * context_->vmc_state.y_right); // 右轮到中心的X距离
    if (context_->imu_data.g_z_body > -9.8f / 2.0f || left_wheel_to_center_x > 0.25f || right_wheel_to_center_x > 0.25f) // 如果Z轴重力分量过大或轮子离中心太远
    {
        return true; // 检测到异常
    }
    else
    {
        return false; // 正常
    }
}

/**
 * @brief 检测爬楼梯状态
 * @return 返回true表示检测到爬楼梯，false表示平地行走
 * @details 检测机器人是否处于爬楼梯状态
 */
bool ChassisControllerFSM::detectClimbStair(void)
{
    float left_wheel_to_center_x = fabsf(context_->vmc_state.sin_theta_left * context_->vmc_state.y_left); // 左轮到中心的X距离
    float right_wheel_to_center_x = fabsf(context_->vmc_state.sin_theta_right * context_->vmc_state.y_right); // 右轮到中心的X距离

    if (context_->external_command.l0_exp > 0.34f && (left_wheel_to_center_x > 0.12f || right_wheel_to_center_x > 0.12f)) // 如果腿长较大且轮子离中心较远
    {
        return true; // 检测到爬楼梯
    }
    else
    {
        return false; // 平地行走
    }
}

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
void ChassisControllerFSM::setMotorPosCommand(float theta1_left, float theta2_left, float theta1_right, float theta2_right, float gimbal_pitch, float gimbal_yaw)
{
    context_->motor_TargetPos.ckyf_pos_left_back = -theta1_left;          // 左后CKYF电机位置命令（取反）
    context_->motor_TargetPos.ckyf_pos_left_front = -theta2_left + LQR_PI; // 左前CKYF电机位置命令（取反并加π）
    context_->motor_TargetPos.ckyf_pos_right_back = theta1_right;         // 右后CKYF电机位置命令
    context_->motor_TargetPos.ckyf_pos_right_front = theta2_right - LQR_PI; // 右前CKYF电机位置命令（减π）
    context_->motor_TargetPos.gimbal_pitch = gimbal_pitch;                // 云台俯仰角命令
    context_->motor_TargetPos.gimbal_yaw = gimbal_yaw;                    // 云台偏航角命令
}


/**
 * @brief 设置电机速度命令
 * @param[in] theta1_left 左侧电机1速度
 * @param[in] theta2_left 左侧电机2速度
 * @param[in] theta1_right 右侧电机1速度
 * @param[in] theta2_right 右侧电机2速度
 * @details 将计算得到的速度命令发送给对应电机
 */
void ChassisControllerFSM::setMotorVelCommand(float theta1_left, float theta2_left, float theta1_right, float theta2_right)
{
    context_->motor_TargetVel.ckyf_velocity_left_back = -theta1_left;     // 左后CKYF电机速度命令（取反）
    context_->motor_TargetVel.ckyf_velocity_left_front = -theta2_left;    // 左前CKYF电机速度命令（取反）
    context_->motor_TargetVel.ckyf_velocity_right_back = theta1_right;    // 右后CKYF电机速度命令
    context_->motor_TargetVel.ckyf_velocity_right_front = theta2_right;   // 右前CKYF电机速度命令
}

/**
 * @brief 逆运动学计算
 * @param[in] vmc_l0 虚拟模型控制腿长
 * @param[in] vmc_phi0 虚拟模型控制相位角
 * @param[out] theta1 输出的角度1
 * @param[out] theta2 输出的角度2
 * @details 将虚拟模型控制输出转换为实际关节角度
 */
void ChassisControllerFSM::backwardKinematic(float vmc_l0, float vmc_phi0, float* theta1, float* theta2)
{
    // 计算角度偏移，基于五连杆机构几何关系
    float angle_offset = acosf((robot_param.l1 * robot_param.l1 + vmc_l0 * vmc_l0 - robot_param.l3 * robot_param.l3) / (2 * robot_param.l1 * vmc_l0));
    *theta1 = vmc_phi0 - angle_offset; // 计算theta1角度
    *theta2 = vmc_phi0 + angle_offset; // 计算theta2角度
}


/**
 * @brief 检查所有电机报告模式
 * @param[in] report_mode 期望的报告模式
 * @return 返回true表示所有电机都处于指定报告模式，否则返回false
 * @details 检查所有电机是否都处于相同的报告模式
 */
bool ChassisControllerFSM::isMotorReportModeAll(uint16_t report_mode)
{
    // 检查四个电机的报告模式是否都符合要求
    if (context_->motor_report_mode.of_motor1 == report_mode && 
        context_->motor_report_mode.of_motor2 == report_mode && 
        context_->motor_report_mode.of_motor3 == report_mode && 
        context_->motor_report_mode.of_motor4 == report_mode)
        return true;
    else
        return false;
}

/**
 * @brief 获取输出模式
 * @return 返回当前控制器输出模式
 * @details 查询当前控制器的工作模式
 */
ControllerOutputMode ChassisControllerFSM::getOutputMode(void)
{
    if ((context_->controller_state == FSM_SELF_SAVE) || 
        ((context_->controller_state == FSM_FORCE_LIFT_UP) && 
         (context_->vmc_state.y_left < 0.25f) && 
         (context_->vmc_state.y_right < 0.25f))) // 如果是自救状态或强制抬起状态且腿长小于0.25m
    {
        return Output_Position; // 返回位置输出模式
    }
    else if ((context_->controller_state == FSM_LIFT_UP) || 
             (context_->controller_state == FSM_CLIMBSTAIR_LIFTLEG) || 
             (context_->controller_state == FSM_CLIMBSTAIR_FOLDLEG) || 
             (context_->controller_state == FSM_MANUAL_LIFT_UP)) // 如果是抬起状态、爬楼梯抬腿/收腿状态或手动抬起状态
    {
        return Output_Velocity; // 返回速度输出模式
    }
    else // 其他状态
    {
        return Output_Torque; // 返回力矩输出模式
    }
}
