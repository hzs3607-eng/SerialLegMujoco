/* Includes ------------------------------------------------------------------*/
#include "../Inc/chassis_controller_estimator.h"

//#include "arm_math.h"


/* External Variables --------------------------------------------------------*/


/* Functions -----------------------------------------------------------------*/
/**
 * @brief 构造函数，初始化状态估计器
 * @param context 控制器上下文指针
 * @details 初始化状态估计器对象，设置弹簧模型参数和时间间隔
 */
ChassisControllerEstimator::ChassisControllerEstimator(ChassisControllerContext* context)
{
    context_ = context;
    
    // 计算弹簧模型参数A和B
    // 公式：A = (l1*F1 - l2*F2) / (l1 - l2)
    // 公式：B = l1*l2*(F2 - F1) / (l1 - l2)
    spring_model_A = (robot_param.spring_l1 * robot_param.spring_F1 - robot_param.spring_l2 * robot_param.spring_F2) / (robot_param.spring_l1 - robot_param.spring_l2);
    spring_model_B = robot_param.spring_l1 * robot_param.spring_l2 * (robot_param.spring_F2 - robot_param.spring_F1) / (robot_param.spring_l1 - robot_param.spring_l2);
    
    time_interval = TIME_INTERVAL;//时间间隔,单位s
}

/**
 * @brief 重置状态
 * @details 将所有内部状态变量重置为初始值，清除所有历史数据
 *          包括打滑标志、低通滤波器变量和离地检测相关变量
 */
void ChassisControllerEstimator::reset(void)
{
    slip_flag = 0;
    
    // 重置低通滤波器相关变量
    ckyf_speed_right_front_last = 0.0f;
    ckyf_speed_right_back_last = 0.0f;
    ckyf_speed_left_front_last = 0.0f;
    ckyf_speed_left_back_last = 0.0f;
    // context_->motorRealStatus.wheel_velocity_left_last = 0.0f;
    // context_->motorRealStatus.wheel_velocity_right_last = 0.0f;
    chassis_roll_speed_rad_last = 0.0f;
    chassis_pitch_speed_rad_last = 0.0f;
    chassis_yaw_speed_rad_last = 0.0f;
    chassis_acc_x_last = 0.0f;
    chassis_acc_y_last = 0.0f;
    chassis_acc_z_last = 0.0f;
    ckyf_real_torque_left_back_last = 0.0f;
    ckyf_real_torque_left_front_last = 0.0f;
    ckyf_real_torque_right_back_last = 0.0f;
    ckyf_real_torque_right_front_last = 0.0f;

    // 清空离地检测相关的历史数据
    memset(&offground_detect_list_body_acc, 0, sizeof(offground_detect_list_body_acc));
    memset(&offground_detect_list_left_vmc_force, 0, sizeof(offground_detect_list_left_vmc_force));
    memset(&offground_detect_list_right_vmc_force, 0, sizeof(offground_detect_list_right_vmc_force));
    memset(&offground_detect_list_left_relative_vel, 0, sizeof(offground_detect_list_left_relative_vel));
    memset(&offground_detect_list_right_relative_vel, 0, sizeof(offground_detect_list_right_relative_vel));
    offground_detect_list_i = 0;
    offground_root_vz_diff = 0.0f;
    offground_left_vmc_impulse = 0.0f;
    offground_right_vmc_impulse = 0.0f;
}

/**
 * @brief 更新状态估计
 * @details 执行完整状态估计流程，按顺序调用各子功能模块
 *          依次执行旋转矩阵计算、几何状态计算、打滑检测、离地检测、运动状态计算和导轮高度计算
 */
void ChassisControllerEstimator::update(void)
{
    calculateRotationMatrix();    // 计算旋转矩阵
    stateCaculationWithoutX();   // 计算不依赖X轴的几何状态
    slipDetection();             // 执行打滑检测
    offGroundDetection();        // 执行离地检测
    stateCaculationWithX();      // 计算依赖X轴的运动状态
    calculateGuideWheelHeight(); // 计算导轮高度
}


/**
 * @brief 不依赖X轴的几何状态计算
 * @details 计算五连杆机构的几何参数，更新VMC模型的几何量
 *          主要计算内容包括：腿部角度、腿部长度、腿部角速度等
 *          该函数独立于X轴方向的运动状态，仅关注腿部几何关系
 */
void ChassisControllerEstimator::stateCaculationWithoutX(void)
{
    // 计算右腿相关的几何状态
    // phi是机身俯仰角，omega_phi是俯仰角速度（符号相反）
    context_->vmc_state.phi = -context_->imu_data.chassis_pitch_rad;
    context_->vmc_state.omega_phi = -context_->imu_data.chassis_pitch_speed_rad;

    // 计算俯仰角的三角函数值
    context_->vmc_state.cos_phi = cos(context_->vmc_state.phi);
    context_->vmc_state.sin_phi = sin(context_->vmc_state.phi);

    /* 右腿计算 */
    // 设置右腿五连杆的输入角度（后侧和前侧电机角度）
    context_->geometric_right.theta1 = context_->motorRealStatus.ckyf_pos_right_back;   // 右后电机角度
    context_->geometric_right.theta2 =  context_->motorRealStatus.ckyf_pos_right_front; // 右前电机角度
    // 五连杆解算，计算腿部几何参数
    solvePentagon(context_->geometric_right);

    // 更新右腿相关状态
    context_->vmc_state.y_right = context_->geometric_right.l0;  // 右腿长度
    // 右腿速度计算：基于链式法则对长度公式求导
    context_->vmc_state.speed_y_right = context_->geometric_right.l0_over_theta1 * context_->motorRealStatus.ckyf_velocity_right_back +
                                        context_->geometric_right.l0_over_theta2 * context_->motorRealStatus.ckyf_velocity_right_front;

    // 计算右腿与垂直方向的角度
    context_->vmc_state.angle_right = -(LQR_PI / 2 - context_->vmc_state.phi - context_->geometric_right.phi0);
    // 计算右腿角速度
    context_->vmc_state.omega_right = context_->geometric_right.phi0_over_theta1 *context_->motorRealStatus.ckyf_velocity_right_back +
                                     context_->geometric_right.phi0_over_theta2 * context_->motorRealStatus.ckyf_velocity_right_front +
                                     context_->vmc_state.omega_phi;

    // 计算右腿角度的三角函数值
    context_->vmc_state.cos_theta_right = cosf(context_->vmc_state.angle_right);
    context_->vmc_state.sin_theta_right = sinf(context_->vmc_state.angle_right);


    /* 左腿计算 */
    // 设置左腿五连杆的输入角度（后侧和前侧电机角度）
    context_->geometric_left.theta1 = context_->motorRealStatus.ckyf_pos_left_back;    // 左后电机角度
    context_->geometric_left.theta2 = context_->motorRealStatus.ckyf_pos_left_front;   // 左前电机角度
    // 五连杆解算，计算腿部几何参数
    solvePentagon(context_->geometric_left);

    // 更新左腿相关状态
    context_->vmc_state.y_left = context_->geometric_left.l0;  // 左腿长度
    // 左腿速度计算：基于链式法则对长度公式求导
    context_->vmc_state.speed_y_left = context_->geometric_left.l0_over_theta1 * context_->motorRealStatus.ckyf_velocity_left_back +
                                       context_->geometric_left.l0_over_theta2 * context_->motorRealStatus.ckyf_velocity_left_front;

    // 计算左腿与垂直方向的角度
    context_->vmc_state.angle_left = -(LQR_PI / 2 - context_->vmc_state.phi - context_->geometric_left.phi0);
    // 计算左腿角速度
    context_->vmc_state.omega_left = context_->geometric_left.phi0_over_theta1 * context_->motorRealStatus.ckyf_velocity_left_back +
                                    context_->geometric_left.phi0_over_theta2 * context_->motorRealStatus.ckyf_velocity_left_front +
                                    context_->vmc_state.omega_phi;

    // 计算左腿角度的三角函数值
    context_->vmc_state.cos_theta_left = cosf(context_->vmc_state.angle_left);
    context_->vmc_state.sin_theta_left = sinf(context_->vmc_state.angle_left);
}

/**
 * @brief 依赖X轴的运动状态计算
 * @details 计算车体加速度、速度和位移，包含IMU数据处理和卡尔曼滤波
 *          主要功能包括：重力补偿、旋转矩阵变换、速度融合估计等
 *          该函数结合X轴方向的运动状态进行综合计算
 */
void ChassisControllerEstimator::stateCaculationWithX()
{
    // IMU数据处理 - 分离重力加速度
    // 通过四元数转换，从IMU测量的加速度中减去重力分量
    chassis_acc_x =  context_->imu_data.acc_x
                   - GRAVITY_ACC * (2.0f * context_->imu_data.Quaternion1 * context_->imu_data.Quaternion3 - 2.0f * context_->imu_data.Quaternion0 * context_->imu_data.Quaternion2);
    chassis_acc_y =  context_->imu_data.acc_y
                    - GRAVITY_ACC * (2.0f * context_->imu_data.Quaternion2 * context_->imu_data.Quaternion3 + 2.0f * context_->imu_data.Quaternion0 * context_->imu_data.Quaternion1);
    chassis_acc_z =  context_->imu_data.acc_z
                    - GRAVITY_ACC * (1.0f - 2.0f * context_->imu_data.Quaternion1 * context_->imu_data.Quaternion1 - 2.0f * context_->imu_data.Quaternion2 * context_->imu_data.Quaternion2);


    // 计算地面坐标系下的偏航角速度
    // 通过旋转矩阵将机身坐标系的角速度转换到地面坐标系
    context_->vmc_state.center_omega_yaw = context_->imu_data.chassis_roll_speed_rad * body_to_ground_matrix[6] + 
                                          context_->imu_data.chassis_pitch_speed_rad * body_to_ground_matrix[7] + 
                                          context_->imu_data.chassis_yaw_speed_rad * body_to_ground_matrix[8];
    // 通过积分计算偏航角
    context_->vmc_state.center_yaw += context_->vmc_state.center_omega_yaw * time_interval;

    // 计算机体平面内的加速度
    // 将车体坐标系下的加速度转换到水平面上
    float acc_body_plane = context_->imu_data.cos_pitch_chassis * chassis_acc_x + 
                           context_->imu_data.sin_pitch_chassis * (chassis_acc_z * context_->imu_data.cos_roll_chassis - chassis_acc_y * context_->imu_data.sin_roll_chassis);
    // 补偿由于偏航运动产生的向心加速度
    acc_body_plane += context_->vmc_state.center_omega_yaw * context_->vmc_state.center_omega_yaw * robot_param.imu_to_center_x;
    // 当偏航速度过高时，认为IMU数据不可靠，清零加速度
    if (fabsf(context_->vmc_state.center_omega_yaw) > HIGH_YAW_SPEED_THRESHOLD){
            acc_body_plane = 0.0f;
    }
    // 限制加速度范围，防止撞击导致IMU数据不准确
    acc_body_plane = clip(0.0f, MAX_IMU_ACC, acc_body_plane);


    // 通过积分计算IMU处的速度
    context_->vmc_state.imu_v_x += acc_body_plane * time_interval;
    // 计算中心速度，考虑偏航运动对速度的影响
    context_->vmc_state.center_v_x = context_->vmc_state.imu_v_x - (robot_param.imu_to_right_y + robot_param.imu_to_left_y) * context_->vmc_state.center_omega_yaw / 2.0f;
    
    // 基于轮子编码器的观测速度
    // 公式：v_obs = (v_wheel_left + v_wheel_right) / 2
    // 其中每个轮子的速度包括三部分：轮子滚动速度、腿部摆动速度、腿部旋转速度
    float v_obs = (
                        (-context_->motorRealStatus.wheel_velocity_left * robot_param.R_wheel - context_->vmc_state.speed_y_left * context_->vmc_state.sin_theta_left - context_->vmc_state.y_left * context_->vmc_state.omega_left * context_->vmc_state.cos_theta_left)
                    + (-context_->motorRealStatus.wheel_velocity_right * robot_param.R_wheel - context_->vmc_state.speed_y_right * context_->vmc_state.sin_theta_right - context_->vmc_state.y_right * context_->vmc_state.omega_right * context_->vmc_state.cos_theta_right)
                    ) / 2.0f;

    float R_cov_obs;    // 观测噪声方差
    float Q_cov_process = 10.0f * time_interval * time_interval;    // 过程噪声方差
    // 根据机器人的状态调整观测方差R
    if (slip_flag || context_->controller_state == FSM_CLIMBSTAIR_LIFTLEG || context_->off_ground.off_ground_body || context_->off_ground.off_ground_l || context_->off_ground.off_ground_r){
        R_cov_obs = UNRELIABLE_OBS_VAR;  // 打滑/离地时，编码器不可靠，增大观测噪声方差
    }
    else if(context_->controller_state == FSM_SPINNING){
        R_cov_obs = RELIABLE_OBS_VAR;    // 旋转状态下使用较小的观测噪声方差
    }
    else{
        if (fabsf(context_->motorRealStatus.wheel_velocity_left * robot_param.R_wheel) < 0.1f && fabsf(context_->motorRealStatus.wheel_velocity_right * robot_param.R_wheel) < 0.1f){
            R_cov_obs = RELIABLE_OBS_VAR;  // 低速时使用较小的观测噪声方差
        }
        else{
            // 根据观测值与预测值的差异动态调整观测噪声方差
            R_cov_obs = NORMAL_OBS_VAR + 0.1f*(v_obs - context_->vmc_state.center_v_x) * (v_obs - context_->vmc_state.center_v_x);
        }
    }
    
    // 卡尔曼滤波更新步骤
    context_->vmc_state.P_center_v_x += Q_cov_process;  // 预测步骤：更新误差协方差
    float K = context_->vmc_state.P_center_v_x / (context_->vmc_state.P_center_v_x + R_cov_obs);  // 计算卡尔曼增益
    context_->vmc_state.center_v_x += K * (v_obs - context_->vmc_state.center_v_x);  // 更新速度估计
    context_->vmc_state.P_center_v_x = (1 - K) * context_->vmc_state.P_center_v_x;  // 更新误差协方差
    
    // 反向计算IMU处的速度
    context_->vmc_state.imu_v_x = context_->vmc_state.center_v_x + (robot_param.imu_to_right_y + robot_param.imu_to_left_y) * context_->vmc_state.center_omega_yaw / 2.0f;

    // 仅在机器人「静止状态」（FSM_STATIONARY）下，通过速度积分计算 x 轴位移（center_x）。
    if (context_->controller_state == FSM_STATIONARY)
    {
        context_->vmc_state.center_x += context_->vmc_state.center_v_x * time_interval;
        // 限制位移范围
        context_->vmc_state.center_x = clip(0.0f,MAX_DISPLACEMENT_X,context_->vmc_state.center_x);
    }
}

/**
 * @brief 打滑检测
 * @details 检测机器人是否发生打滑现象
 *          通过比较轮子理论速度与实际速度的差异来判断是否打滑
 */
void ChassisControllerEstimator::slipDetection(void)
{
    // 计算左腿轮子在x方向的速度分量
    float vcx_l = context_->vmc_state.speed_y_left * context_->vmc_state.sin_theta_left + context_->vmc_state.y_left * context_->vmc_state.omega_left * context_->vmc_state.cos_theta_left;
    // 计算右腿轮子在x方向的速度分量
    float vcx_r = context_->vmc_state.speed_y_right * context_->vmc_state.sin_theta_right + context_->vmc_state.y_right * context_->vmc_state.omega_right * context_->vmc_state.cos_theta_right;

    // 第一项：由偏航和俯仰运动引起的轮子速度差异
    float item_1 = context_->imu_data.chassis_yaw_speed_rad * robot_param.Wid + context_->imu_data.chassis_pitch_speed_rad * (context_->geometric_left.l0 * context_->vmc_state.cos_theta_left - context_->geometric_right.l0 * context_->vmc_state.cos_theta_right);
    // 第二项：左右腿轮子理论速度差
    float item_2 = (vcx_r-vcx_l);
        
    // 第三项：由重力分量和轮子转速差异引起的补偿项
    float item_3 = robot_param.R_wheel/sqrt(context_->imu_data.g_x_body*context_->imu_data.g_x_body+context_->imu_data.g_z_body*context_->imu_data.g_z_body)
                    *((GRAVITY_ACC * context_->imu_data.cos_pitch_chassis * context_->imu_data.cos_roll_chassis * (context_->motorRealStatus.wheel_velocity_right - context_->motorRealStatus.wheel_velocity_left)));
    // 计算总的轮子速度差，并使用低通滤波平滑结果
    delta_velocity_wheel = 0.5f * (fabsf(item_1 + item_2 + item_3)) + 0.5f * delta_velocity_wheel_last;//低通滤波

    // 根据机器人状态选择不同的打滑阈值
    float delt_v_threshold = (context_->controller_state == FSM_SPINNING) ? SLIPPAGE_THRESOLD_HIGH: SLIPPAGE_THRESOLD_LOW;
    // 判断是否异常（打滑）
    bool abnormal = delta_velocity_wheel > delt_v_threshold || (fabsf(context_->motorRealStatus.wheel_velocity_left * robot_param.R_wheel) > MAX_MOTOR_SPEED && fabsf(context_->motorRealStatus.wheel_velocity_right * robot_param.R_wheel) > MAX_MOTOR_SPEED);
    // 更新上次速度差值
    delta_velocity_wheel_last = delta_velocity_wheel;
    // 更新打滑标志
    slip_flag = abnormal;
}

/**
 * @brief 离地检测
 * @details 检测机器人腿部是否离地
 *          使用循环队列记录VMC支持力，加速度与相对速度，通过冲量计算判断离地状态
 */
void ChassisControllerEstimator::offGroundDetection(void)
{   
    // 使用循环队列记录VMC支持力，加速度与相对速度，观测冲量

    // 减去和暂存最旧数据
    offground_root_vz_diff -= offground_detect_list_body_acc[offground_detect_list_i] * time_interval;
    offground_left_vmc_impulse -= offground_detect_list_left_vmc_force[offground_detect_list_i] * time_interval;
    offground_right_vmc_impulse -= offground_detect_list_right_vmc_force[offground_detect_list_i] * time_interval;
    float left_wheel_2_root_old = offground_detect_list_left_relative_vel[offground_detect_list_i];
    float right_wheel_2_root_old = offground_detect_list_right_relative_vel[offground_detect_list_i];

    // 向队列记录最新数据
    offground_detect_list_body_acc[offground_detect_list_i] = body_to_ground_matrix[6] * chassis_acc_x + body_to_ground_matrix[7] * chassis_acc_y + body_to_ground_matrix[8] * chassis_acc_z;

    // 计算左右腿的实际支撑力
    float F_real_r = context_->geometric_right.theta1_over_l0 * context_->motorRealStatus.ckyf_troque_right_back + context_->geometric_right.theta2_over_l0 * context_->motorRealStatus.ckyf_troque_right_front;
    float F_real_l = context_->geometric_left.theta1_over_l0 * context_->motorRealStatus.ckyf_troque_left_back + context_->geometric_left.theta2_over_l0 * context_->motorRealStatus.ckyf_troque_left_front;

    // 计算左右腿的扭矩分量
    float Tp_real_r = context_->geometric_right.theta1_over_phi0 * context_->motorRealStatus.ckyf_troque_right_back + context_->geometric_right.theta2_over_phi0 * context_->motorRealStatus.ckyf_troque_right_front;
    float Tp_real_l = context_->geometric_left.theta1_over_phi0 * context_->motorRealStatus.ckyf_troque_left_back + context_->geometric_left.theta2_over_phi0 * context_->motorRealStatus.ckyf_troque_left_front;

    // 计算左右腿VMC支持力
    offground_detect_list_left_vmc_force[offground_detect_list_i] = F_real_l*context_->vmc_state.cos_theta_left - Tp_real_l / context_->geometric_left.l0 * context_->vmc_state.sin_theta_left;    // 向下为正
    offground_detect_list_right_vmc_force[offground_detect_list_i] = F_real_r*context_->vmc_state.cos_theta_right - Tp_real_r / context_->geometric_right.l0 * context_->vmc_state.sin_theta_right;   // 向下为正

    // 计算左右轮相对于髋关节的z方向速度
    float left_wheel_2_hip_z = context_->vmc_state.speed_y_left * context_->vmc_state.cos_theta_left - context_->vmc_state.y_left * context_->vmc_state.omega_left * context_->vmc_state.sin_theta_left;
    float right_wheel_2_hip_z = context_->vmc_state.speed_y_right * context_->vmc_state.cos_theta_right - context_->vmc_state.y_right * context_->vmc_state.omega_right * context_->vmc_state.sin_theta_right;

    // 计算左右轮相对于机身的z方向速度
    float left_wheel_2_root = (left_wheel_2_hip_z - context_->imu_data.chassis_roll_speed_rad * robot_param.Wid / 2.0f) * context_->imu_data.cos_roll_chassis - context_->imu_data.chassis_roll_speed_rad * context_->vmc_state.y_left * context_->imu_data.sin_roll_chassis;
    float right_wheel_2_root = (right_wheel_2_hip_z + context_->imu_data.chassis_roll_speed_rad * robot_param.Wid / 2.0f) * context_->imu_data.cos_roll_chassis - context_->imu_data.chassis_roll_speed_rad * context_->vmc_state.y_right * context_->imu_data.sin_roll_chassis;

    // 记录相对速度
    offground_detect_list_left_relative_vel[offground_detect_list_i] = left_wheel_2_root;     // 向下为正
    offground_detect_list_right_relative_vel[offground_detect_list_i] = right_wheel_2_root;   // 向下为正

    // 加上最新数据
    offground_root_vz_diff += offground_detect_list_body_acc[offground_detect_list_i] * time_interval;
    // offground_root_vz_diff = 0.0f; // 重置为0，重新计算
    // for (int i = 0; i < OFF_GROUND_DETECT_LIST_LENGTH; i++)
    // {
    //     offground_root_vz_diff += offground_detect_list_body_acc[i] * time_interval;
    // }
    offground_left_vmc_impulse += offground_detect_list_left_vmc_force[offground_detect_list_i] * time_interval;
    offground_right_vmc_impulse += offground_detect_list_right_vmc_force[offground_detect_list_i] * time_interval;

    // 计算动量变化
    float left_momentum_diff = (offground_root_vz_diff - left_wheel_2_root + left_wheel_2_root_old) * (context_->geometric_left.m_leg + robot_param.m_wheel);
    float right_momentum_diff = (offground_root_vz_diff - right_wheel_2_root + right_wheel_2_root_old) * (context_->geometric_right.m_leg + robot_param.m_wheel);

    // 计算地面支持力冲量
    // momentum_diff = impulse_gravity + impulse_vmc + impulse_ground_force
    // impulse_ground_force = momentum_diff - impulse_gravity - impulse_vmc
    float impulse_gravity = (context_->geometric_left.m_leg + robot_param.m_wheel) * -GRAVITY_ACC * time_interval * OFF_GROUND_DETECT_LIST_LENGTH;
    
    float impulse_left_vmc = - offground_left_vmc_impulse;      // 正方向定义不同
    float impulse_right_vmc = - offground_right_vmc_impulse;    // 正方向定义不同

    float impulse_left_ground = left_momentum_diff - impulse_gravity - impulse_left_vmc;
    float impulse_right_ground = right_momentum_diff - impulse_gravity - impulse_right_vmc;

    // 计算身体所受地面支持力冲量
    float body_momentum_diff = offground_root_vz_diff * robot_param.M;
    float impulse_body_gravity = robot_param.M * -GRAVITY_ACC * time_interval * OFF_GROUND_DETECT_LIST_LENGTH;

    float impulse_body_ground = body_momentum_diff - impulse_body_gravity
                              + left_momentum_diff - impulse_gravity
                              + right_momentum_diff - impulse_gravity;

    // 计算气弹簧等效支持力
    context_->off_ground.F_spring_l = spring_model_A + spring_model_B / context_->geometric_left.l_spring;
    context_->off_ground.F_spring_r = spring_model_A + spring_model_B / context_->geometric_right.l_spring;
    context_->off_ground.F_spring_vmc_l = context_->geometric_left.vmcforce_over_springforce * context_->off_ground.F_spring_l;
    context_->off_ground.F_spring_vmc_r = context_->geometric_right.vmcforce_over_springforce * context_->off_ground.F_spring_r;

    // 更新循环队列索引
    offground_detect_list_i = (offground_detect_list_i + 1) % OFF_GROUND_DETECT_LIST_LENGTH;

    // 计算平均支持力
    context_->off_ground.F_support_l = impulse_left_ground / (time_interval * OFF_GROUND_DETECT_LIST_LENGTH) + context_->off_ground.F_spring_vmc_l;
    context_->off_ground.F_support_r = impulse_right_ground / (time_interval * OFF_GROUND_DETECT_LIST_LENGTH) + context_->off_ground.F_spring_vmc_r;
    context_->off_ground.F_support_body = impulse_body_ground / (time_interval * OFF_GROUND_DETECT_LIST_LENGTH);

    // 判断离地状态（支持力小于阈值则认为离地）
    context_->off_ground.off_ground_l = (context_->off_ground.F_support_l < OFF_GROUND_FORCE_THRESHOLD);
    context_->off_ground.off_ground_r = (context_->off_ground.F_support_r < OFF_GROUND_FORCE_THRESHOLD);
    context_->off_ground.off_ground_body = (context_->off_ground.F_support_body < OFF_GROUND_FORCE_THRESHOLD);
}

/**
 * @brief 计算旋转矩阵
 * @details 计算地面系到车身坐标系的旋转矩阵以及车身坐标系到地面坐标系的旋转矩阵
 *          通过四元数转换实现坐标系之间的旋转变换
 */
void ChassisControllerEstimator::calculateRotationMatrix(void)
{
    // (1) 计算地面系到车身坐标系的旋转矩阵
    // 首先，根据四元数计算地面系到imu原始坐标系的原始旋转矩阵
    float a = context_->imu_data.Quaternion0, b = context_->imu_data.Quaternion1, c = context_->imu_data.Quaternion2, d = context_->imu_data.Quaternion3;
    float ground_to_body_raw[9] = {
        a * a + b * b - c * c - d * d,  2 * b * c + 2 * a * d,          2 * b * d - 2 * a * c,
        2 * b * c - 2 * a * d,          a * a - b * b + c * c - d * d,  2 * c * d + 2 * a * b,
        2 * b * d + 2 * a * c,          2 * c * d - 2 * a * b,          a * a - b * b - c * c + d * d};
    // 进而，可以得到地面系到车身坐标系的旋转矩阵
    ground_to_body_matrix[0] = ground_to_body_raw[0];
    ground_to_body_matrix[1] = ground_to_body_raw[1];
    ground_to_body_matrix[2] = ground_to_body_raw[2];
    ground_to_body_matrix[3] = ground_to_body_raw[3];
    ground_to_body_matrix[4] = ground_to_body_raw[4];
    ground_to_body_matrix[5] = ground_to_body_raw[5];
    ground_to_body_matrix[6] = ground_to_body_raw[6];
    ground_to_body_matrix[7] = ground_to_body_raw[7]; 
    ground_to_body_matrix[8] = ground_to_body_raw[8];


    // (2) 计算车身坐标系到地面坐标系的旋转矩阵
    // 注意到，三维旋转矩阵为正交矩阵，因此有
    body_to_ground_matrix[0] = ground_to_body_matrix[0];
    body_to_ground_matrix[1] = ground_to_body_matrix[3];
    body_to_ground_matrix[2] = ground_to_body_matrix[6];
    body_to_ground_matrix[3] = ground_to_body_matrix[1];
    body_to_ground_matrix[4] = ground_to_body_matrix[4];
    body_to_ground_matrix[5] = ground_to_body_matrix[7];
    body_to_ground_matrix[6] = ground_to_body_matrix[2];
    body_to_ground_matrix[7] = ground_to_body_matrix[5];
    body_to_ground_matrix[8] = ground_to_body_matrix[8];
    
    // 同时计算Body系下的重力加速度
    context_->imu_data.g_x_body = -GRAVITY_ACC * ground_to_body_matrix[2];
    context_->imu_data.g_y_body = -GRAVITY_ACC * ground_to_body_matrix[5];
    context_->imu_data.g_z_body = -GRAVITY_ACC * ground_to_body_matrix[8];
}

void ChassisControllerEstimator::solvePentagon(Pentagon &p)
{
    /*
    五连杆解算所借助的二维坐标系定义:
        (1) 车身系Oxyz坐标系的定义:O为两侧电机连杆中心点，x轴向前，y轴向左，z轴向上。
        (2) 借鉴车身系的定义，定义二维坐标系Oxz坐标系:O为一侧（左侧或右侧）电机连杆中心点，x轴向前，z轴向上。
        (3) 对于串联腿，节点名称定义如下
              A(E)
           J / \
          /|/   \
         / F     H
        / / \   /
       K /   \ /
       |/     G
       B       
        \      
         \     
          \    
           \   
            \  
             \ 
              C

    点定义:
        关节点:
            A: 后侧电机
            B: 后侧膝关节
            C: 轮子圆心
            D: 前侧膝关节
            E: 前侧电机

        辅助点:
            P: BD中点

    角度定义:
        所有角度以逆时针为正向
        theta_1: 后侧关节电机角度，定义为x负半轴和\vec{AB}矢量夹角，正常情况下为0左右
        theta_2: 前侧关节点击角度，定义为x负半轴和\vec{ED}矢量夹角，正常情况下为pi左右
        theta_3: 后侧膝关节角度，定义为x负半轴和\vec{BC}矢量夹角，正常情况下为钝角
        theta_4: 前侧膝关节角度，定义为x负半轴和\vec{DC}矢量夹角，正常情况下为锐角

    距离定义:
        形状参数:
            l1: AB长度
            l2: ED长度
            l3: BC长度
            l4: DC长度
            l5: AE长度
        辅助参数:
            d: BD长度
            s: PC长度
    */
    // A
    float x_A = - robot_param.l5 / 2;
    float z_A = 0;
    // E
    float x_E = robot_param.l5 / 2;
    float z_E = 0;
    // B
    float x_B = x_A - robot_param.l1 * cosf(p.theta1);
    float z_B = z_A - robot_param.l1 * sinf(p.theta1);
    // D
    float x_D = x_E - robot_param.l2 * cosf(p.theta2);
    float z_D = z_E - robot_param.l2 * sinf(p.theta2);
    // P
    float x_P = (x_B + x_D) / 2;
    float z_P = (z_B + z_D) / 2;
    // d
    float d = sqrt((x_B - x_D) * (x_B - x_D) + (z_B - z_D) * (z_B - z_D));
    // s
    float s = sqrt(robot_param.l3 * robot_param.l3 - d * d / 4);
    // C
    float x_C = x_P - s * (z_B - z_D) / d;
    float z_C = z_P + s * (x_B - x_D) / d;
    // l0
    p.l0 = sqrt(x_C * x_C + z_C * z_C);
    // phi0
    p.phi0 = LQR_PI + atan2(z_C, x_C);
    // theta3
    p.theta3 = LQR_PI + atan2(z_C - z_B, x_C - x_B);
    // theta4
    p.theta4 = LQR_PI + atan2(z_C - z_D, x_C - x_D);

    // calculate sin and cos
    p.sin_theta1 = sinf(p.theta1);
    p.cos_theta1 = cosf(p.theta1);
    
    p.sin_theta2 = sinf(p.theta2);
    p.cos_theta2 = cosf(p.theta2);
    
    p.sin_theta3 = sinf(p.theta3);
    p.cos_theta3 = cosf(p.theta3);
    
    p.sin_theta4 = sinf(p.theta4);
    p.cos_theta4 = cosf(p.theta4);
    
    p.sin_phi0 = sinf(p.phi0);
    p.cos_phi0 = cosf(p.phi0);

    // calculate the derivatives
    p.phi0_over_theta1 = (robot_param.l1 / p.l0) * (p.cos_theta4 * p.cos_phi0 + p.sin_theta4 * p.sin_phi0) * (p.sin_theta3 * p.cos_theta1 - p.cos_theta3 * p.sin_theta1) / (p.sin_theta3 * p.cos_theta4 - p.cos_theta3 * p.sin_theta4);
    p.phi0_over_theta2 = - (robot_param.l2 / p.l0) * (p.cos_theta3 * p.cos_phi0 + p.sin_theta3 * p.sin_phi0) * (p.sin_theta4 * p.cos_theta2 - p.cos_theta4 * p.sin_theta2) / (p.sin_theta3 * p.cos_theta4 - p.cos_theta3 * p.sin_theta4);

    p.l0_over_theta1 = - robot_param.l1 * (p.sin_theta4 * p.cos_phi0 - p.cos_theta4 * p.sin_phi0) * (p.sin_theta3 * p.cos_theta1 - p.cos_theta3 * p.sin_theta1) / (p.sin_theta3 * p.cos_theta4 - p.cos_theta3 * p.sin_theta4);
    p.l0_over_theta2 = robot_param.l2 * (p.sin_theta3 * p.cos_phi0 - p.cos_theta3 * p.sin_phi0) * (p.sin_theta4 * p.cos_theta2 - p.cos_theta4 * p.sin_theta2) / (p.sin_theta3 * p.cos_theta4 - p.cos_theta3 * p.sin_theta4);

    float det = p.l0_over_theta2 * p.phi0_over_theta1 - p.l0_over_theta1 * p.phi0_over_theta2;

    p.theta1_over_l0 = - p.phi0_over_theta2 / det;
    p.theta2_over_l0 = p.phi0_over_theta1 / det;

    p.theta1_over_phi0 = p.l0_over_theta2 / det;
    p.theta2_over_phi0 = - p.l0_over_theta1 / det;

    // 计算串联腿质心位置
    // 各个腿方向
    // AB
    float vec_AB_x = - p.cos_theta1;
    float vec_AB_z = - p.sin_theta1;
    // AH(AH // AD)
    float vec_AH_x = - p.cos_theta2;
    float vec_AH_z = - p.sin_theta2;
    // FJ
    float vec_FJ_x = - cosf(robot_param.ang_JFG + p.theta3);
    float vec_FJ_z = - sinf(robot_param.ang_JFG + p.theta3);
    // HG(HG // CD)
    float vec_HG_x = - p.cos_theta4;
    float vec_HG_z = - p.sin_theta4;
    // BC
    float vec_BC_x = - p.cos_theta3;
    float vec_BC_z = - p.sin_theta3;
    // 各个关节质心位置
    // AB
    float com_AB_x = robot_param.com_AB_2A * vec_AB_x;
    float com_AB_z = robot_param.com_AB_2A * vec_AB_z;
    // AH
    float com_AH_x = robot_param.com_AH_2A * vec_AH_x;
    float com_AH_z = robot_param.com_AH_2A * vec_AH_z;
    // HG
    float com_HG_x = robot_param.AH * vec_AH_x + robot_param.com_HG_2H * vec_HG_x;
    float com_HG_z = robot_param.AH * vec_AH_z + robot_param.com_HG_2H * vec_HG_z;
    // JFG
    float com_JFG_x = robot_param.AF * vec_AB_x - (robot_param.com_JFG_2J_parallel_JF - robot_param.FJ) * vec_FJ_x + robot_param.com_JFG_2J_vertical_JF * vec_FJ_z;
    float com_JFG_z = robot_param.AF * vec_AB_z - (robot_param.com_JFG_2J_parallel_JF - robot_param.FJ) * vec_FJ_z - robot_param.com_JFG_2J_vertical_JF * vec_FJ_x;
    // JK
    float com_JK_x = robot_param.AF * vec_AB_x + robot_param.FJ * vec_FJ_x + robot_param.com_JK_2J * vec_AB_x;
    float com_JK_z = robot_param.AF * vec_AB_z + robot_param.FJ * vec_FJ_z + robot_param.com_JK_2J * vec_AB_z;
    // KBC
    float com_KBC_x = (robot_param.AF + robot_param.FB) * vec_AB_x + (robot_param.BC - robot_param.com_KBC_2C) * vec_BC_x;
    float com_KBC_z = (robot_param.AF + robot_param.FB) * vec_AB_z + (robot_param.BC - robot_param.com_KBC_2C) * vec_BC_z;
    
    // 计算腿部质量
    p.m_leg = robot_param.m_AB + robot_param.m_AH + robot_param.m_HG + robot_param.m_JFG + robot_param.m_JK + robot_param.m_KBC;
    // 加权平均
    p.com_x = (robot_param.m_AB * com_AB_x + robot_param.m_AH * com_AH_x + robot_param.m_HG * com_HG_x + robot_param.m_JFG * com_JFG_x + robot_param.m_JK * com_JK_x + robot_param.m_KBC * com_KBC_x) / p.m_leg;
    p.com_y = (robot_param.m_AB * com_AB_z + robot_param.m_AH * com_AH_z + robot_param.m_HG * com_HG_z + robot_param.m_JFG * com_JFG_z + robot_param.m_JK * com_JK_z + robot_param.m_KBC * com_KBC_z) / p.m_leg;

    /*
    气弹簧解算所借助的二维坐标系定义:

        (1) 对于气弹簧解算，节点名称定义如下，其中坐标系定义与A B C点定义与上五连杆解算相同
              A
             / \
            P   \
           /     M 
          /     /
         /     /
        /     /
       B     / 
        \   /
         \ /  
          N    
           \   
            \  
             \ 
              C
    （新增）点定义:
        关节点:
            M: 气弹簧上端固定点
            B: 气弹簧下端固定点
        辅助点:
            P: M点在AB杆上的垂直投影点

    角度定义:
        ang_ABC: AB与BC的夹角, 在0到pi之间
    */
    double ang_ABC = p.theta1 + LQR_PI - p.theta3; // 注意到，ang_ABC在0到pi之间

    double cos_ang_ABC = cosf(ang_ABC);
    double sin_ang_ABC = sinf(ang_ABC);
    double PB = robot_param.l1 - robot_param.AP;
    p.l_spring = sqrtf(PB * PB + robot_param.PM * robot_param.PM + robot_param.BN * robot_param.BN - 2.0f * PB * robot_param.BN * cos_ang_ABC - 2.0f * robot_param.BN * robot_param.PM * sin_ang_ABC);
    p.l_spring -= robot_param.sprint_l_offset;
    p.vmcforce_over_springforce = (p.l0 / p.l_spring) * (PB * sin_ang_ABC - robot_param.PM * cos_ang_ABC) * robot_param.BN / (robot_param.l1 * robot_param.BC * sin_ang_ABC);
}

/**
 * @brief 计算导轮高度
 * @details 计算导轮与左右轮子之间的高度差
 *          用于判断导轮何时会接触地面，帮助机器人跨越障碍
 */
void ChassisControllerEstimator::calculateGuideWheelHeight(void)
{
    // 计算高低导轮相对于机身的高度差
    float guidewheel_high_front_2_root_heightdiff = - robot_param.guide_wheel_high_x * context_->imu_data.sin_pitch_chassis + robot_param.guide_wheel_high_z * context_->imu_data.cos_pitch_chassis;
    float guidewheel_high_back_2_root_heightdiff = robot_param.guide_wheel_high_x * context_->imu_data.sin_pitch_chassis + robot_param.guide_wheel_high_z * context_->imu_data.cos_pitch_chassis;
    float guidewheel_low_front_2_root_heightdiff = - robot_param.guide_wheel_low_x * context_->imu_data.sin_pitch_chassis + robot_param.guide_wheel_low_z * context_->imu_data.cos_pitch_chassis;
    float guidewheel_low_back_2_root_heightdiff = robot_param.guide_wheel_low_x * context_->imu_data.sin_pitch_chassis + robot_param.guide_wheel_low_z * context_->imu_data.cos_pitch_chassis;
    
    // 计算左右轮子相对于机身的高度差
    float leftwheel_2_root_heightdiff = (- context_->vmc_state.y_left * context_->vmc_state.cos_theta_left - robot_param.R_wheel);
    float rightwheel_2_root_heightdiff = (- context_->vmc_state.y_right * context_->vmc_state.cos_theta_right - robot_param.R_wheel);

    // 计算导轮与左轮的高度差
    float guidewheel_high_front_2_leftwheel_heightdiff = guidewheel_high_front_2_root_heightdiff - leftwheel_2_root_heightdiff;
    float guidewheel_high_front_2_rightwheel_heightdiff = guidewheel_high_front_2_root_heightdiff - rightwheel_2_root_heightdiff;
    float guidewheel_high_back_2_leftwheel_heightdiff = guidewheel_high_back_2_root_heightdiff - leftwheel_2_root_heightdiff;
    float guidewheel_high_back_2_rightwheel_heightdiff = guidewheel_high_back_2_root_heightdiff - rightwheel_2_root_heightdiff;
    float guidewheel_low_front_2_leftwheel_heightdiff = guidewheel_low_front_2_root_heightdiff - leftwheel_2_root_heightdiff;
    float guidewheel_low_front_2_rightwheel_heightdiff = guidewheel_low_front_2_root_heightdiff - rightwheel_2_root_heightdiff;
    float guidewheel_low_back_2_leftwheel_heightdiff = guidewheel_low_back_2_root_heightdiff - leftwheel_2_root_heightdiff;
    float guidewheel_low_back_2_rightwheel_heightdiff = guidewheel_low_back_2_root_heightdiff - rightwheel_2_root_heightdiff;

    // 计算到左轮的最小高度差（即最近距离）
    context_->guide_wheel_height_diff.to_leftwheel = min_function(guidewheel_high_front_2_leftwheel_heightdiff, min_function(guidewheel_high_back_2_leftwheel_heightdiff, min_function(guidewheel_low_front_2_leftwheel_heightdiff, guidewheel_low_back_2_leftwheel_heightdiff)));
    // 计算到右轮的最小高度差（即最近距离）
    context_->guide_wheel_height_diff.to_rightwheel = min_function(guidewheel_high_front_2_rightwheel_heightdiff, min_function(guidewheel_high_back_2_rightwheel_heightdiff, min_function(guidewheel_low_front_2_rightwheel_heightdiff, guidewheel_low_back_2_rightwheel_heightdiff)));
}
