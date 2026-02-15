#pragma once

/* Includes ------------------------------------------------------------------*/
#include <memory>
#include <cmath>
#include <cstring>

/* Types ---------------------------------------------------------------------*/
/* 机器人参数结构体 */
typedef struct 
{
    float l1, l2, l3, l4, l5; // 杆长参数
    float AF, AH, FG, HG, FB, FJ, JK, BK, BC; // 串联腿各连杆长度
    float AP, PM, BN; // 气弹簧各连杆长度
    float com_AB_2A, com_AH_2A, com_HG_2H; // AB, AH, HG杆质心位置
    float com_JFG_2J_parallel_JF, com_JFG_2J_vertical_JF; // JFG杆质心位置
    float com_JK_2J, com_KBC_2C; // JK, KBC杆质心位置
    float ang_JFG; // JFG夹角
    float m_AB, m_AH, m_HG, m_JFG, m_JK, m_KBC; // 各连杆质量
    float R_wheel; // 轮子半径
    float m_wheel; // 轮子质量
    float m_wheel_leg; // 轮子与腿的质量
    float M; // 车体质量
    float Wid; // 车体宽度
    float guide_wheel_high_x; // 防撞轮距离腿根部水平距离
    float guide_wheel_high_z; // 防撞轮距离腿根部竖直距离
    float guide_wheel_low_x; // 防撞轮距离腿根部水平距离
    float guide_wheel_low_z; // 防撞轮距离腿根部竖直距离
    float imu_to_left_y; // IMU到左侧电机连线中点的y轴位移
    float imu_to_right_y; // IMU到右侧电机连线中点的y轴位移
    float imu_to_center_x; // IMU到车体中心的x轴位移
    float l0_stable; // 等效杆长
    float sprint_l_offset, spring_l1, spring_l2, spring_F1, spring_F2; // 气弹簧参数
} BotParam;

/* 五连杆机构参数结构体 */
typedef struct
{
    float theta1, theta2, theta3, theta4; // 关节角度
    float phi0, l0; // 相位角和腿长
    float sin_theta1, sin_theta2, sin_theta3, sin_theta4, sin_phi0; // 关节角度的正弦值
    float cos_theta1, cos_theta2, cos_theta3, cos_theta4, cos_phi0; // 关节角度的余弦值
    float phi0_over_theta1, phi0_over_theta2, l0_over_theta1, l0_over_theta2; // 相位角和腿长对关节角度的偏导数
    float theta1_over_phi0, theta2_over_phi0, theta1_over_l0, theta2_over_l0; // 关节角度对相位角和腿长的偏导数
    float l_spring, vmcforce_over_springforce; // 弹簧长度和VMC力与弹簧力的比值
    float com_x, com_y; // 质心坐标
    float m_leg; // 腿部质量
} Pentagon;

/* 离地检测结构体 */
typedef struct 
{
    float F_support_l, F_support_r, F_support_body; // 左、右、整体支持力
    float F_spring_l, F_spring_r; // 气弹簧力
    float F_spring_vmc_l, F_spring_vmc_r; // 气弹簧VMC等效推力
    uint8_t off_ground_l, off_ground_r, off_ground_body; // 左、右、整体离地状态
} OffGround;

/* 虚拟模型控制状态结构体 */
typedef struct 
{
    float center_omega_yaw; // 地面系下yaw角速度
    float center_yaw; // 地面系下yaw角度
    float imu_v_x; // IMU处自然坐标系下速度
    float center_x; // IMU处自然坐标系下中心位移
    float center_v_x; // 中心速度
    float P_center_v_x; // 中心速度的方差
    
    float y_left, speed_y_left, y_right, speed_y_right; // 左右腿长度及变化率
    float angle_left, omega_left; // 左侧角度及角速度
    float angle_right, omega_right; // 右侧角度及角速度
    float sin_theta_left, cos_theta_left, sin_theta_right, cos_theta_right; // 左右侧角度的三角函数值

    float phi, omega_phi; // phi角及角速度
    float sin_phi, cos_phi; // phi角的三角函数值
} VMCState;

/* 导向轮高度差结构体 */
typedef struct
{
    float to_leftwheel;  // 到左轮的高度差
    float to_rightwheel; // 到右轮的高度差
} GuideWheelHeightDiff;


/* IMU数据结构体 */
typedef struct
{
    float chassis_roll_rad;   // 底盘横滚角（弧度）
    float chassis_pitch_rad;  // 底盘俯仰角（弧度）
    float chassis_yaw_rad;    // 底盘偏航角（弧度）

    float sin_roll_chassis;   // 底盘横滚角正弦值
    float cos_roll_chassis;   // 底盘横滚角余弦值
    float sin_pitch_chassis;  // 底盘俯仰角正弦值
    float cos_pitch_chassis;  // 底盘俯仰角余弦值
    float sin_yaw_chassis;    // 底盘偏航角正弦值
    float cos_yaw_chassis;    // 底盘偏航角余弦值

    float chassis_roll_speed_rad;   // 底盘横滚角速度（弧度/秒）
    float chassis_pitch_speed_rad;  // 底盘俯仰角速度（弧度/秒）
    float chassis_yaw_speed_rad;    // 底盘偏航角速度（弧度/秒）

    float Quaternion0; // 四元数第0个分量
    float Quaternion1; // 四元数第1个分量
    float Quaternion2; // 四元数第2个分量
    float Quaternion3; // 四元数第3个分量

    float acc_x; // X轴加速度
    float acc_y; // Y轴加速度
    float acc_z; // Z轴加速度

    float g_x_body; // 机体坐标系X轴重力分量
    float g_y_body; // 机体坐标系Y轴重力分量
    float g_z_body; // 机体坐标系Z轴重力分量
} ImuData;

/* 内部命令结构体 */
typedef struct
{
    float x_expectation;      // X方向期望位置
    float yaw_expectation;    // 偏航角期望值
    float desired_vel_x;      // 期望X方向速度
    float desired_vel_omega;  // 期望角速度
    float target_y_l;         // 左侧目标腿长
    float target_y_r;         // 右侧目标腿长
    float target_dot_y_l;     // 左侧目标腿长变化率
    float target_dot_y_r;     // 右侧目标腿长变化率
    float target_theta_l;     // 左侧目标theta角度
    float target_theta_r;     // 右侧目标theta角度
    float last_target_y_l;    // 上次左侧目标腿长
    float last_target_y_r;    // 上次右侧目标腿长
} InternalCommand;

/* 电机力矩结构体 */
typedef struct
{
    float ckyf_troque_right_front;   // 右前CKYF电动力矩
    float ckyf_troque_right_back;    // 右后CKYF电动力矩
    float ckyf_troque_left_front;    // 左前CKYF电动力矩
    float ckyf_troque_left_back;     // 左后CKYF电动力矩
    float m3508_troque_right;        // 右侧M3508电动力矩
    float m3508_troque_left;         // 左侧M3508电动力矩
} MotorTargetTorque;

/* 电机位置结构体 */
typedef struct
{
    float ckyf_pos_right_front;  // 右前CKYF电机位置
    float ckyf_pos_right_back;   // 右后CKYF电机位置
    float ckyf_pos_left_front;   // 左前CKYF电机位置
    float ckyf_pos_left_back;    // 左后CKYF电机位置
    float gimbal_pitch;          // 云台俯仰角
    float gimbal_yaw;            // 云台偏航角
} MotorTargetPos;

/* 电机速度结构体 */
typedef struct
{
    float ckyf_velocity_right_front;  // 右前CKYF电机速度
    float ckyf_velocity_right_back;   // 右后CKYF电机速度
    float ckyf_velocity_left_front;   // 左前CKYF电机速度
    float ckyf_velocity_left_back;    // 左后CKYF电机速度
    float wheel_velocity_left;        // 左轮速度
    float wheel_velocity_right;       // 右轮速度
} MotorTargetVel;

typedef struct
{
    float ckyf_pos_right_front;  // 右前CKYF电机位置
    float ckyf_pos_right_back;   // 右后CKYF电机位置
    float ckyf_pos_left_front;   // 左前CKYF电机位置
    float ckyf_pos_left_back;    // 左后CKYF电机位置
    float gimbal_pitch;          // 云台俯仰角
    float gimbal_yaw;            // 云台偏航角



    float ckyf_velocity_right_front;  // 右前CKYF电机速度
    float ckyf_velocity_right_back;   // 右后CKYF电机速度
    float ckyf_velocity_left_front;   // 左前CKYF电机速度
    float ckyf_velocity_left_back;    // 左后CKYF电机速度
    float wheel_velocity_left;        // 左轮速度
    float wheel_velocity_right;       // 右轮速度

    float ckyf_troque_right_front;   // 右前CKYF电动力矩
    float ckyf_troque_right_back;    // 右后CKYF电动力矩
    float ckyf_troque_left_front;    // 左前CKYF电动力矩
    float ckyf_troque_left_back;     // 左后CKYF电动力矩
    float m3508_troque_right;        // 右侧M3508电动力矩
    float m3508_troque_left;         // 左侧M3508电动力矩
} MotorRealStatus;
/* 电机报告模式结构体 */
typedef struct
{
    uint16_t of_motor1;  // 电机1报告模式
    uint16_t of_motor2;  // 电机2报告模式
    uint16_t of_motor3;  // 电机3报告模式
    uint16_t of_motor4;  // 电机4报告模式
} MotorReportMode;

/* 功率控制结构体 */
typedef struct
{
    float left_wheel_omega, right_wheel_omega;  // 左右轮角速度
    float power_limit;                          // 功率限制
} PowerControl;

// 虚拟模型控制参数结构体
typedef struct 
{
    float F, Digital_torque, Wheel_torque;    // Digital_torque: 虚拟关节扭矩。Wheel_torque: 轮毂电机扭矩
} VMC;

/* 关节电机转矩结构体 */
typedef struct 
{
    float T1, T2;  // 两个关节的转矩
} Real_torque;

/* 外部命令结构体 */
typedef struct
{
    float velocity_x_plan;   // X方向速度计划
    float omega_plan;        // 角速度计划
    float angle_target;      // 角度目标
    float l0_exp;           // 期望腿长
    float l0_exp_raw;       // 原始期望腿长
    uint8_t climb_stair;    // 爬楼梯标志，1为真，0为假
} ExternalCommand;

/* 调试信息结构体 */
typedef struct
{
    float test_1;
    float test_2;
    float test_3;
    float test_4;
    float test_5;
    float test_6;
    float test_7;
    float test_8;
    float test_9;
    float test_10;
} DebugInfo;


/* 控制器状态枚举 */
enum ControllerState {
    FSM_STATIONARY,          // 静止状态
    FSM_SPINNING,            // 旋转状态
    FSM_DIFF_DRIVE,          // 差动驱动状态
    FSM_CLIMBSTAIR_LIFTLEG,  // 爬楼梯抬腿状态
    FSM_CLIMBSTAIR_FOLDLEG,  // 爬楼梯收腿状态
    FSM_JUMP_SQUATING,       // 跳跃下蹲状态
    FSM_JUMP_STRETCHING,     // 跳跃伸展状态
    FSM_JUMP_FOLDING,        // 跳跃收腿状态
    FSM_JUMP_LANDING,        // 跳跃着陆状态
    FSM_LIFT_UP,             // 抬起状态
    FSM_FORCE_LIFT_UP,       // 强制抬起状态
    FSM_MANUAL_LIFT_UP,      // 手动抬起状态
    FSM_SELF_SAVE,           // 自救状态
};

/* 控制器输出模式枚举 */
enum ControllerOutputMode {
    Output_Torque,      // 力矩输出模式
    Output_Velocity,    // 速度输出模式
    Output_Position,    // 位置输出模式
};

/* Functions ----------------------------------------------------------------- */
/**
 * @brief 限制数值范围
 * @param[in] mean 中心值
 * @param[in] limit 限制范围
 * @param[in] raw 原始值
 * @return 限制后的值
 * @details 将原始值限制在mean±limit的范围内
 */
inline float clip(float mean, float limit, float raw)
{
    limit = fabsf(limit);
    if (raw > mean + limit) return mean + limit;
    else if (raw < mean - limit)    return mean - limit;
    else    return raw;
}

/**
 * @brief 功率模型函数
 * @param[in] I 电流
 * @param[in] w 角速度
 * @param[in] c_1 到 c_5 功率模型系数
 * @param[in] gearbox_ratio 减速比，默认为1.0
 * @return 计算得到的功率
 * @details 根据电流和角速度计算功率消耗
 */
inline float powerModel(float I, float w, float c_1, float c_2, float c_3, float c_4, float c_5, float gearbox_ratio = 1.0f)
{
    w *= gearbox_ratio; // 考虑减速比
    return c_1 * I * I + c_2 * I * w + c_3 * w * w + c_4 * fabsf(w) + c_5;
}

/* Classes -------------------------------------------------------------------- */
/**
 * @brief 底盘控制器上下文类
 * @details 存储整个底盘控制器的状态信息，包括VMC状态、几何参数、IMU数据等
 */
class ChassisControllerContext
{
    public:
        VMCState vmc_state;                     // 虚拟模型控制状态
        OffGround off_ground;                   // 离地检测数据
        Pentagon geometric_left;                // 左侧五连杆几何参数
        Pentagon geometric_right;               // 右侧五连杆几何参数
        ImuData imu_data;                       // IMU数据
        GuideWheelHeightDiff guide_wheel_height_diff; // 导向轮高度差
        InternalCommand internal_command;         // 内部命令
        ExternalCommand external_command;         // 外部命令
        MotorTargetTorque motor_TargetTorque;               // 电机力矩
        MotorTargetPos motor_TargetPos;                     // 电机位置
        MotorTargetVel motor_TargetVel;                     // 电机速度
        MotorRealStatus   motorRealStatus;
        MotorReportMode motor_report_mode;      // 电机报告模式
        ControllerState controller_state;        // 控制器状态
        DebugInfo debug_info;                   // 调试信息
        PowerControl power_control;             // 功率控制
        bool imu_connected;                     // IMU连接状态
        bool follow_gimbal;                     // 是否跟随云台
        float location_yaw;                     // 位置偏航角

        /**
         * @brief 构造函数
         * @details 初始化控制器状态为抬起状态，设置默认连接状态
         */
        ChassisControllerContext() :
            controller_state(FSM_LIFT_UP),
            imu_connected(false),
            follow_gimbal(false)
            {
                reset(); // 重置所有参数
            }
    
        /**
         * @brief 重置函数
         * @details 将所有状态变量重置为初始值
         */
        void reset() {
            memset(&vmc_state, 0, sizeof(VMCState));               // VMCSate清零
            memset(&off_ground, 0, sizeof(OffGround));             // 离地检测清零
            memset(&geometric_left, 0, sizeof(Pentagon));          // 左侧几何参数清零
            memset(&geometric_right, 0, sizeof(Pentagon));         // 右侧几何参数清零
            memset(&imu_data, 0, sizeof(ImuData));                // IMU数据清零
            memset(&guide_wheel_height_diff, 0, sizeof(GuideWheelHeightDiff)); // 导向轮高度差清零
            memset(&internal_command, 0, sizeof(InternalCommand));  // 内部命令清零
            memset(&external_command, 0, sizeof(ExternalCommand));  // 外部命令清零
            memset(&motor_TargetTorque, 0, sizeof(MotorTargetTorque));         // 电机力矩清零
            memset(&motor_TargetPos, 0, sizeof(MotorTargetPos));              // 电机位置清零
            memset(&motor_TargetVel, 0, sizeof(MotorTargetVel));              // 电机速度清零
            memset(&motor_report_mode, 0, sizeof(MotorReportMode)); // 电机报告模式清零
            memset(&debug_info, 0, sizeof(DebugInfo));            // 调试信息清零
            memset(&power_control, 0, sizeof(PowerControl));       // 功率控制清零
            
            vmc_state.center_omega_yaw = 0.0f;                     // 中心偏航角速度设为0
            
            controller_state = FSM_LIFT_UP;                        // 控制器状态设为抬起状态
            imu_connected = false;                                 // IMU未连接
            follow_gimbal = false;                                 // 不跟随云台

            vmc_state.sin_theta_left = 0.0f;                      // 左侧theta角正弦值
            vmc_state.sin_theta_right = 0.0f;                     // 右侧theta角正弦值
            vmc_state.sin_phi = 0.0f;                             // phi角正弦值
            vmc_state.cos_theta_left = 1.0f;                      // 左侧theta角余弦值
            vmc_state.cos_theta_right = 1.0f;                     // 右侧theta角余弦值
            vmc_state.cos_phi = 1.0f;                             // phi角余弦值
            
            imu_data.sin_roll_chassis = 0.0f;                     // 底盘横滚角正弦值
            imu_data.cos_roll_chassis = 1.0f;                     // 底盘横滚角余弦值
            imu_data.sin_pitch_chassis = 0.0f;                    // 底盘俯仰角正弦值
            imu_data.cos_pitch_chassis = 1.0f;                    // 底盘俯仰角余弦值
            imu_data.sin_yaw_chassis = 0.0f;                      // 底盘偏航角正弦值
            imu_data.cos_yaw_chassis = 1.0f;                      // 底盘偏航角余弦值
            imu_data.g_x_body = 0.0f;                             // 机体坐标系X轴重力分量
            imu_data.g_y_body = 0.0f;                             // 机体坐标系Y轴重力分量
            imu_data.g_z_body = -9.8f;                            // 机体坐标系Z轴重力分量（重力加速度）

            power_control.power_limit = 45.0f;                    // 默认功率限制45W
        }
};