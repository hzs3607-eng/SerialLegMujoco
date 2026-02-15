#include <mujoco/mujoco.h>
#include <vector>
#include <string>
#include <cmath>
#include <iostream>
#include <GLFW/glfw3.h>
// 引入实车控制器头文件
#include <chassis_controller.h>

// 引用全局控制器实例
extern ChassisSoftWareControl chassis_SoftWareController;

// ---------------------------------------------------------
// 1. 定义名称映射
// ---------------------------------------------------------
// 必须严格按照实车代码中 Motor ID 的枚举顺序排列！
const std::vector<std::string> JOINT_NAMES = {
    "right_front_j",    // Index 0: RF
    "right_behind_j",   // Index 1: RB 
    "left_front_j",     // Index 2: LF
    "left_behind_j",    // Index 3: LB
    "right_wheel_j",    // Index 4: RW
    "left_wheel_j"      // Index 5: LW
};

const std::vector<std::string> ACTUATOR_NAMES = {
    "right_front_j_ctrl",   // Index 0
    "right_behind_j_ctrl",  // Index 1
    "left_front_j_ctrl",    // Index 2
    "left_behind_j_ctrl",   // Index 3
    "right_wheel_j_ctrl",   // Index 4
    "left_wheel_j_ctrl"     // Index 5
};

// ---------------------------------------------------------
// 2. 键盘控制回调
// ---------------------------------------------------------
// 这个函数需要在 main.cpp 的 keyboardCB 中调用
void Sim_Keyboard_Callback(int key, int act) {
    if (act != GLFW_PRESS) return;

    auto& ctx = chassis_SoftWareController.context_;
    auto& cmd = ctx.external_command;

    switch (key) {
        // --- 状态切换 ---
        case GLFW_KEY_ENTER: 
            printf(">>> 触发: 抬起模式 (LIFT_UP)\n");
            chassis_SoftWareController.triggerLiftUpMode(); 
            break;
        case GLFW_KEY_SPACE: 
            printf(">>> 触发: 强制抬起 (FORCE_LIFT_UP)\n");
            chassis_SoftWareController.triggerForceLiftUpMode(); 
            break;
        case GLFW_KEY_J:
            printf(">>> 触发: 跳跃 (JUMP)\n");
            chassis_SoftWareController.triggerJumpCommand();
            break;

        // --- 运动控制 (WASD) ---
        case GLFW_KEY_W: 
            cmd.velocity_x_plan = 1.0f; 
            printf(">>> 前进: Vx = %.2f\n", cmd.velocity_x_plan);
            break;
        case GLFW_KEY_S: 
            cmd.velocity_x_plan = -1.0f; 
            printf(">>> 后退: Vx = %.2f\n", cmd.velocity_x_plan);
            break;
        case GLFW_KEY_A: 
            cmd.omega_plan = 5.0f; 
            printf(">>> 左转: Wz = %.2f\n", cmd.omega_plan);
            break;
        case GLFW_KEY_D: 
            cmd.omega_plan = -5.0f; 
            printf(">>> 右转: Wz = %.2f\n", cmd.omega_plan);
            break;
        case GLFW_KEY_Q:
            cmd.omega_plan = 0.0f;
            printf(">>> 直走：Wz = %.2f\n", cmd.omega_plan);
            break;
        case GLFW_KEY_E:
            cmd.velocity_x_plan = 2.5f;
            printf(">>> 加速：Vx = %.2f\n", cmd.velocity_x_plan);
            break;
        // default to stop
        default:
            cmd.velocity_x_plan = 0.0f;
            cmd.omega_plan = 0.0f;
            break;
    }
}

// ---------------------------------------------------------
// 3. 硬件抽象层
// ---------------------------------------------------------
void Sim_Update_IO(const mjModel* m, mjData* d) {
    
    auto& ctx = chassis_SoftWareController.context_;

    // ========================
    // A. 读取 Sensors -> Context
    // ========================
    int acc_id = mj_name2id(m, mjOBJ_SENSOR, "base_link_accel");
    int gyro_id = mj_name2id(m, mjOBJ_SENSOR, "imu_gyro");
    int quat_id = mj_name2id(m, mjOBJ_SENSOR, "base_link_site_quat");

    if (acc_id != -1 && gyro_id != -1 && quat_id != -1) {
        double* acc = d->sensordata + m->sensor_adr[acc_id];
        double* gyro = d->sensordata + m->sensor_adr[gyro_id];
        double* quat = d->sensordata + m->sensor_adr[quat_id]; 

        // 1. 加速度 (注意：实车可能需要去除重力，但Estimator里好像有处理逻辑)
        ctx.imu_data.acc_x = acc[0];
        ctx.imu_data.acc_y = acc[1];
        ctx.imu_data.acc_z = acc[2]; // MuJoCo输出包含重力(+9.8)

        // 2. 角速度
        ctx.imu_data.chassis_roll_speed_rad  = gyro[0];
        ctx.imu_data.chassis_pitch_speed_rad = gyro[1];
        ctx.imu_data.chassis_yaw_speed_rad   = gyro[2];

        // 3. 四元数
        ctx.imu_data.Quaternion0 = quat[0]; // w
        ctx.imu_data.Quaternion1 = quat[1]; // x
        ctx.imu_data.Quaternion2 = quat[2]; // y
        ctx.imu_data.Quaternion3 = quat[3]; // z

        // 4. 欧拉角转换 (Pitch/Roll/Yaw)
        double w=quat[0], x=quat[1], y=quat[2], z=quat[3];
        ctx.imu_data.chassis_pitch_rad = asin(2.0 * (w * y - z * x));
        ctx.imu_data.chassis_roll_rad  = atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y));
        ctx.imu_data.chassis_yaw_rad   = atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));

        // 5. 三角函数缓存
        ctx.imu_data.sin_pitch_chassis = std::sin(ctx.imu_data.chassis_pitch_rad);
        ctx.imu_data.cos_pitch_chassis = std::cos(ctx.imu_data.chassis_pitch_rad);
        ctx.imu_data.sin_roll_chassis  = std::sin(ctx.imu_data.chassis_roll_rad);
        ctx.imu_data.cos_roll_chassis  = std::cos(ctx.imu_data.chassis_roll_rad);
        
        ctx.imu_connected = true;
    }

    // --- 读取电机 ---   
    // 辅助lambda：获取电机数据
    auto get_motor_q = [&](const char* name) {
        int id = mj_name2id(m, mjOBJ_JOINT, name);
        if (id == -1) return 0.0;
        return d->qpos[m->jnt_qposadr[id]];
    };
    auto get_motor_v = [&](const char* name) {
        int id = mj_name2id(m, mjOBJ_JOINT, name);
        if (id == -1) return 0.0;
        return d->qvel[m->jnt_dofadr[id]];
    };

    // 填充 MotorRealStatus
    auto& ms = ctx.motorRealStatus;
    
    // 右侧
    ms.ckyf_pos_right_front      = PI-get_motor_q("right_front_j"); // Invert front and offset PI
    ms.ckyf_velocity_right_front = -get_motor_v("right_front_j"); // Invert front
    ms.ckyf_pos_right_back       = get_motor_q("right_behind_j");
    ms.ckyf_velocity_right_back  = get_motor_v("right_behind_j");
    ms.wheel_velocity_right      = -get_motor_v("right_wheel_j"); // Invert right wheel

    // 左侧
    ms.ckyf_pos_left_front       = PI-get_motor_q("left_front_j"); // Invert front and offset PI
    ms.ckyf_velocity_left_front  = -get_motor_v("left_front_j"); // Invert front
    ms.ckyf_pos_left_back        = get_motor_q("left_behind_j");
    ms.ckyf_velocity_left_back   = get_motor_v("left_behind_j");
    ms.wheel_velocity_left       = get_motor_v("left_wheel_j");


    // ========================
    // B. 运行实车算法
    // ========================
    chassis_SoftWareController.update();


    // ========================
    // C. 写入 Actuators
    // ========================
    auto& mt = ctx.motor_TargetTorque;

    // 辅助lambda：写入力矩
    auto set_torque = [&](const char* name, double val, bool invert) {
        int id = mj_name2id(m, mjOBJ_ACTUATOR, name);
        if (id != -1) d->ctrl[id] = invert ? -val : val;
    };

    // 右侧 - Front motor inverted, Back motor NOT inverted
    set_torque("right_front_j_ctrl",  mt.ckyf_troque_right_front, true);   // Invert front
    set_torque("right_behind_j_ctrl", mt.ckyf_troque_right_back,  false);
    set_torque("right_wheel_j_ctrl",  mt.m3508_troque_right,      true);   // Invert right wheel

    // 左侧 - Front motor inverted, Back motor NOT inverted
    set_torque("left_front_j_ctrl",   mt.ckyf_troque_left_front,  false);   // Invert front
    set_torque("left_behind_j_ctrl",  mt.ckyf_troque_left_back,   true);
    set_torque("left_wheel_j_ctrl",   mt.m3508_troque_left,       true);
}