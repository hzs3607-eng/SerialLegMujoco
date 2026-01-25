#include <mujoco/mujoco.h>
#include <GLFW/glfw3.h>
#include <cstdio>
#include <cmath>
#include <iostream> // 用于输出警告信息
#include <chassis_controller.h>

// =================================================================
// 1. 全局变量定义
// =================================================================
// MuJoCo 模型和数据
mjModel* m = NULL;
mjData* d = NULL;
// MuJoCo 可视化组件
mjvCamera cam;
mjvOption opt;
mjvScene scn;
mjrContext con;
// 鼠标交互状态
bool button_left = false;
bool button_middle = false;
bool button_right = false;
double lastx = 0;
double lasty = 0;
double tune_q1 = 0.0; // 控制 left_front_j
double tune_q2 = 0.0; // 控制 left_front_1_j
// 控制器目标位置 (!!关键!!)
// 我们需要一个地方来存储目标位置，这里定义为全局变量


// =================================================================
// 2. 函数原型声明
// =================================================================
// 控制器
void myController(const mjModel* m, mjData* d); // 注意：mjcb_control 的类型是 void (*)(...)
// GLFW 回调函数
void keyboardCB(GLFWwindow* window, int key, int scancode, int act, int mods);
void mouseButtonCB(GLFWwindow* window, int button, int act, int mods);
void cursorPosCB(GLFWwindow* window, double xpos, double ypos);
void scrollCB(GLFWwindow* window, double xoffset, double yoffset);
void initController(); // 声明 initController，即使它现在是空的
void Sim_Update_IO(const mjModel* m, mjData* d);
void Sim_Keyboard_Callback(int key, int act);




// =================================================================
// 3. 主函数 (程序入口)
// =================================================================
int main(void) {
    // --- 加载模型与数据 ---
    char errstr[1000] = "Could not load custom_robot.xml";
    m = mj_loadXML("custom_robot.xml", NULL, errstr, 1000);
    if (!m) {
        mju_error("Could not load custom_robot.xml: %s", errstr);
    }
    d = mj_makeData(m);
    
    // Set simulation timestep to 0.002s (2ms) for stability
    m->opt.timestep = 0.002;
    
    // --- 设置初始化条件 ---
    d->qpos[0] = 0;
    d->qpos[1] = 0;
    d->qpos[2] = 0.15;
    // --- 加载控制器 ---
    mjcb_control = myController;
    // --- 加载GLFW与可视化数据 ---
    if (!glfwInit()) {
        mju_error("Could not initialize GLFW");
    }
    GLFWwindow* window = glfwCreateWindow(1200, 900, "Demo", NULL, NULL);
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);
    // --- 初始化可视化数据结构 ---
    mjv_defaultCamera(&cam);
    mjv_defaultOption(&opt);
    //调试需要
    opt.flags[mjVIS_JOINT] = 1;      // 显示关节轴 (关键！看旋转方向)
    opt.flags[mjVIS_ACTUATOR] = 1;   // 显示执行器
    // opt.flags[mjVIS_TRANSPARENT] = 1; // 让外壳半透明，方便看内部结构
    opt.frame = mjFRAME_GEOM;        // 显示几何体的局部坐标系
    //调试需要
    mjv_defaultScene(&scn);
    mjr_defaultContext(&con);
    mjv_makeScene(m, &scn, 10000);
    mjr_makeContext(m, &con, mjFONTSCALE_150);
    // --- 加载可视化回调函数 ---
    glfwSetKeyCallback(window, keyboardCB);
    glfwSetCursorPosCallback(window, cursorPosCB);
    glfwSetMouseButtonCallback(window, mouseButtonCB);
    glfwSetScrollCallback(window, scrollCB);
    // --- 设置可视化初始化条件 ---
    opt.frame = mjFRAME_WORLD;
    // //调试需要！！！
    // m->opt.gravity[0] = 0;
    // m->opt.gravity[1] = 0;
    // m->opt.gravity[2] = 0; // 关闭重力，机器人悬浮
    // //调试需要！！！
    
    double arr_view[] = {2.5, 90.0, -45.0, 0.0, 0.0, 0.5};
    cam.distance = arr_view[0];
    cam.azimuth = arr_view[1];
    cam.elevation = arr_view[2];
    cam.lookat[0] = arr_view[3];
    cam.lookat[1] = arr_view[4];
    cam.lookat[2] = arr_view[5];
    // --- 主仿真循环 ---
    mjtNum last_print_time = -1.0;
    while (!glfwWindowShouldClose(window)) {
        double simstart = d->time;
        while (d->time - simstart < 1.0 / 60.0) {
            mj_step(m, d);
            // 调用移植接口 (每一步都运行实车算法)
            Sim_Update_IO(m, d);
            
            if (d->time - last_print_time > 0.1) {
            // --- 打印基本信息 ---
            printf("-------------------------------------------------------\n");
            printf("Sim Time: %.3f s\n", d->time);
            
            // 打印控制软件内部状态 (腿长)
            auto& ctx = chassis_SoftWareController.context_;
            printf("Leg Exp Target (raw/lpf): %.4f / %.4f m\n", ctx.external_command.l0_exp_raw, ctx.external_command.l0_exp);
            printf("Actual Leg Height (L/R): %.4f / %.4f m\n", ctx.vmc_state.y_left, ctx.vmc_state.y_right);
            printf("FSM State: %d\n", ctx.controller_state);
            // --- 获取关节数据 ---
            // 找到关节的ID
            //left_front_l
            int jnt1_id = mj_name2id(m, mjOBJ_JOINT, "left_front_j");
            int jnt2_id = mj_name2id(m, mjOBJ_JOINT, "left_front_1_j");
            int jnt3_id = mj_name2id(m, mjOBJ_JOINT, "left_front_2_j");
            int jnt4_id = mj_name2id(m, mjOBJ_JOINT, "left_front_3_j");
            //left_behind_l
            int jnt5_id = mj_name2id(m, mjOBJ_JOINT, "left_behind_j");
            int jnt6_id = mj_name2id(m, mjOBJ_JOINT, "left_behind_withwheel_j");
            int jnt7_id = mj_name2id(m, mjOBJ_JOINT, "left_wheel_j");
            //right_front_l
            int jnt8_id = mj_name2id(m, mjOBJ_JOINT, "right_front_j");
            int jnt9_id = mj_name2id(m, mjOBJ_JOINT, "right_front_1_j");
            int jnt10_id = mj_name2id(m, mjOBJ_JOINT, "right_front_2_j");
            int jnt11_id = mj_name2id(m, mjOBJ_JOINT, "right_front_3_j");
            //right_behind_l
            int jnt12_id = mj_name2id(m, mjOBJ_JOINT, "right_behind_j");
            int jnt13_id = mj_name2id(m, mjOBJ_JOINT, "right_behind_withwheel_j");
            int jnt14_id = mj_name2id(m, mjOBJ_JOINT, "right_wheel_j");
            //gimbal_yaw_l
            int jnt15_id = mj_name2id(m, mjOBJ_JOINT, "gimbal_yaw_j");
            int jnt16_id = mj_name2id(m, mjOBJ_JOINT, "pitch_j");
            int jnt17_id = mj_name2id(m, mjOBJ_JOINT, "pitch_1_j");
            int jnt18_id = mj_name2id(m, mjOBJ_JOINT, "pitch_2_j");
            // 检查ID是否有效并打印数据
            //left_front_l
            // printf("下面是left_front_l数据");
            // if (jnt1_id != -1) {
            //     // d->qpos[m->jnt_qposadr[ID]] 是获取关节位置的标准方法
            //     printf("  Joint 'left_front_j':  pos = %8.3f rad, vel = %8.3f rad/s\n", 
            //     d->qpos[m->jnt_qposadr[jnt1_id]], 
            //     d->qvel[m->jnt_dofadr[jnt1_id]]);
            // }
            // if (jnt2_id != -1) {
            //     printf("  Joint 'left_front_1_j':  pos = %8.3f rad, vel = %8.3f rad/s\n", 
            //     d->qpos[m->jnt_qposadr[jnt2_id]], 
            //     d->qvel[m->jnt_dofadr[jnt2_id]]);
            // }
            // if (jnt3_id != -1) {
            //     printf("  Joint 'left_front_2_j':  pos = %8.3f rad, vel = %8.3f rad/s\n", 
            //     d->qpos[m->jnt_qposadr[jnt3_id]], 
            //     d->qvel[m->jnt_dofadr[jnt3_id]]);
            // }
            // if (jnt4_id != -1) {
            //     printf("  Joint 'left_front_3_j':  pos = %8.3f rad, vel = %8.3f rad/s\n", 
            //     d->qpos[m->jnt_qposadr[jnt4_id]], 
            //     d->qvel[m->jnt_dofadr[jnt4_id]]);
            // }
            // //left_behind_l
            // printf("下面是left_behind_l数据");
            // if (jnt5_id != -1) {
            //     printf("  Joint 'left_behind_j':  pos = %8.3f rad, vel = %8.3f rad/s\n", 
            //     d->qpos[m->jnt_qposadr[jnt5_id]], 
            //     d->qvel[m->jnt_dofadr[jnt5_id]]);
            // }
            // if (jnt6_id != -1) {
            //     printf("  Joint 'left_behind_withwheel_j':  pos = %8.3f rad, vel = %8.3f rad/s\n", 
            //     d->qpos[m->jnt_qposadr[jnt6_id]], 
            //     d->qvel[m->jnt_dofadr[jnt6_id]]);
            // }
            // if (jnt7_id != -1) {
            //     printf("  Joint 'left_wheel_j':  pos = %8.3f rad, vel = %8.3f rad/s\n", 
            //     d->qpos[m->jnt_qposadr[jnt7_id]], 
            //     d->qvel[m->jnt_dofadr[jnt7_id]]);
            // }
            // //right_front_l
            // printf("下面是right_front_l数据");
            // if (jnt8_id != -1) {
            //     // d->qpos[m->jnt_qposadr[ID]] 是获取关节位置的标准方法
            //     printf("  Joint 'right_front_j':  pos = %8.3f rad, vel = %8.3f rad/s\n", 
            //     d->qpos[m->jnt_qposadr[jnt8_id]], 
            //     d->qvel[m->jnt_dofadr[jnt8_id]]);
            // }
            // if (jnt9_id != -1) {
            //     printf("  Joint 'right_front_1_j':  pos = %8.3f rad, vel = %8.3f rad/s\n", 
            //     d->qpos[m->jnt_qposadr[jnt9_id]], 
            //     d->qvel[m->jnt_dofadr[jnt9_id]]);
            // }
            // if (jnt10_id != -1) {
            //     printf("  Joint 'right_front_2_j':  pos = %8.3f rad, vel = %8.3f rad/s\n", 
            //     d->qpos[m->jnt_qposadr[jnt10_id]], 
            //     d->qvel[m->jnt_dofadr[jnt10_id]]);
            // }
            // if (jnt11_id != -1) {
            //     printf("  Joint 'right_front_3_j':  pos = %8.3f rad, vel = %8.3f rad/s\n", 
            //     d->qpos[m->jnt_qposadr[jnt11_id]], 
            //     d->qvel[m->jnt_dofadr[jnt11_id]]);
            // }
            // //right_behint_l
            // printf("下面是right_behind_l数据");
            // if (jnt12_id != -1) {
            //     printf("  Joint 'right_behind_j':  pos = %8.3f rad, vel = %8.3f rad/s\n", 
            //     d->qpos[m->jnt_qposadr[jnt12_id]], 
            //     d->qvel[m->jnt_dofadr[jnt12_id]]);
            // }
            // if (jnt13_id != -1) {
            //     printf("  Joint 'right_behind_withwheel_j':  pos = %8.3f rad, vel = %8.3f rad/s\n", 
            //     d->qpos[m->jnt_qposadr[jnt13_id]], 
            //     d->qvel[m->jnt_dofadr[jnt13_id]]);
            // }
            // if (jnt14_id != -1) {
            //     printf("  Joint 'right_wheel_j':  pos = %8.3f rad, vel = %8.3f rad/s\n", 
            //     d->qpos[m->jnt_qposadr[jnt14_id]], 
            //     d->qvel[m->jnt_dofadr[jnt14_id]]);
            // }
            //gimbal_yaw_l
            // printf("下面是gimbal_yaw_l数据");
            // if (jnt15_id != -1) {
            //     // d->qpos[m->jnt_qposadr[ID]] 是获取关节位置的标准方法
            //     printf("  Joint 'gimbal_yaw_j':  pos = %8.3f rad, vel = %8.3f rad/s\n", 
            //     d->qpos[m->jnt_qposadr[jnt15_id]], 
            //     d->qvel[m->jnt_dofadr[jnt15_id]]);
            // }
            // if (jnt16_id != -1) {
            //     printf("  Joint 'pitch_j':  pos = %8.3f rad, vel = %8.3f rad/s\n", 
            //     d->qpos[m->jnt_qposadr[jnt16_id]], 
            //     d->qvel[m->jnt_dofadr[jnt16_id]]);
            // }
            // if (jnt17_id != -1) {
            //     printf("  Joint 'pitch_1_j':  pos = %8.3f rad, vel = %8.3f rad/s\n", 
            //     d->qpos[m->jnt_qposadr[jnt17_id]], 
            //     d->qvel[m->jnt_dofadr[jnt17_id]]);
            // }
            // if (jnt18_id != -1) {
            //     printf("  Joint 'pitch_2_j':  pos = %8.3f rad, vel = %8.3f rad/s\n", 
            //     d->qpos[m->jnt_qposadr[jnt18_id]], 
            //     d->qvel[m->jnt_dofadr[jnt18_id]]);
            // }

            // --- 获取传感器数据 ---
            // 找到传感器的ID (使用我们新命名的名字)
            int sensorpos = mj_name2id(m, mjOBJ_SENSOR, "base_link_site_pos");
            int sensorquat = mj_name2id(m, mjOBJ_SENSOR, "base_link_site_quat");
            int sensorlinvel = mj_name2id(m, mjOBJ_SENSOR, "base_link_site_linvel");
            int sensorangvel = mj_name2id(m, mjOBJ_SENSOR, "base_link_site_angvel");
            int sensorvel = mj_name2id(m, mjOBJ_SENSOR, "base_link_site_vel");
            int sensoraccel = mj_name2id(m, mjOBJ_SENSOR, "base_link_accel");
            int sensorgyro = mj_name2id(m, mjOBJ_SENSOR, "imu_gyro");
            // 检查位置传感器ID是否有效并打印数据
            printf("下面是sensor数据");
            if (sensorpos != -1) {
                mjtNum* pos_data = d->sensordata + m->sensor_adr[sensorpos];
                printf("  Sensor 'base_link_site_pos':   (x, y, z) = (%8.3f, %8.3f, %8.3f)\n", 
                pos_data[0], pos_data[1], pos_data[2]);
            }
            // 检查速度传感器ID是否有效并打印数据
            if (sensorvel != -1) {
                mjtNum* vel_data = d->sensordata + m->sensor_adr[sensorvel];
                printf("  Sensor 'base_link_site_vel': (vx, vy, vz) = (%8.3f, %8.3f, %8.3f)\n", 
                vel_data[0], vel_data[1], vel_data[2]);
            }
            // 检验四元数角度传感器ID是否有效并打印数据
            if (sensorquat != -1) {
                mjtNum* quat_data = d->sensordata + m->sensor_adr[sensorquat];
                printf("  Sensor 'base_link_site_quat': (w, x, y, z) = (%8.3f, %8.3f, %8.3f, %8.3f)\n", 
                quat_data[0], quat_data[1], quat_data[2], quat_data[3]);
            }
            // 检验线速度传感器ID是否有效并打印数据
            if (sensorlinvel != -1) {
                mjtNum* linvel_data = d->sensordata + m->sensor_adr[sensorlinvel];
                printf("  Sensor 'base_link_site_linvel': (vx, vy, vz) = (%8.3f, %8.3f, %8.3f)\n", 
                linvel_data[0], linvel_data[1], linvel_data[2]);
            }
            // 检验角速度传感器ID是否有效并打印数据
            if (sensorangvel != -1) {
                mjtNum* angvel_data = d->sensordata + m->sensor_adr[sensorangvel];
                printf("  Sensor 'base_link_site_angvel': (w, x, y, z) = (%8.3f, %8.3f, %8.3f, %8.3f)\n", 
                angvel_data[0], angvel_data[1], angvel_data[2], angvel_data[3]);
            }
            // 检验加速度传感器ID是否有效并打印数据
            if (sensoraccel != -1) {
                mjtNum* accel_data = d->sensordata + m->sensor_adr[sensoraccel];
                printf("  Sensor 'base_link_accel': (ax, ay, az) = (%8.3f, %8.3f, %8.3f)\n", 
                accel_data[0], accel_data[1], accel_data[2]);
            }
            // 检验陀螺仪传感器ID是否有效并打印数据
            if (sensorgyro != -1) {
                mjtNum* gyro_data = d->sensordata + m->sensor_adr[sensorgyro];
                printf("  Sensor 'base_link_gyro': (roll, pitch, yaw) = (%8.3f, %8.3f, %8.3f)\n", 
                gyro_data[0], gyro_data[1], gyro_data[2]);
            }
            // --- 获取控制输入值 (你施加的力矩) ---
            // 执行器的ID
            int act1_id = mj_name2id(m, mjOBJ_ACTUATOR, "left_front_j_ctrl");
            int act2_id = mj_name2id(m, mjOBJ_ACTUATOR, "left_front_1_j_ctrl");
            int act3_id = mj_name2id(m, mjOBJ_ACTUATOR, "left_front_2_j_ctrl");
            int act4_id = mj_name2id(m, mjOBJ_ACTUATOR, "left_front_3_j_ctrl");
            int act5_id = mj_name2id(m, mjOBJ_ACTUATOR, "left_behind_j_ctrl");
            int act6_id = mj_name2id(m, mjOBJ_ACTUATOR, "left_behind_withwheel_j_ctrl");
            int act7_id = mj_name2id(m, mjOBJ_ACTUATOR, "left_wheel_j_ctrl");
            int act8_id = mj_name2id(m, mjOBJ_ACTUATOR, "right_behind_j_ctrl");
            int act9_id = mj_name2id(m, mjOBJ_ACTUATOR, "right_behind_withwheel_j_ctrl");
            int act10_id = mj_name2id(m, mjOBJ_ACTUATOR, "right_wheel_j_ctrl");
            int act11_id = mj_name2id(m, mjOBJ_ACTUATOR, "right_front_j_ctrl");
            int act12_id = mj_name2id(m, mjOBJ_ACTUATOR, "right_front_1_j_ctrl");
            int act13_id = mj_name2id(m, mjOBJ_ACTUATOR, "right_front_2_j_ctrl");
            int act14_id = mj_name2id(m, mjOBJ_ACTUATOR, "right_front_3_j_ctrl");
            int act15_id = mj_name2id(m, mjOBJ_ACTUATOR, "gimbal_yaw_j_ctrl");
            int act16_id = mj_name2id(m, mjOBJ_ACTUATOR, "pitch_j_ctrl");
            int act17_id = mj_name2id(m, mjOBJ_ACTUATOR, "pitch_1_j_ctrl");
            int act18_id = mj_name2id(m, mjOBJ_ACTUATOR, "pitch_2_j_ctrl");
            printf("下面是actuator数据");
            if (act1_id != -1 && act2_id != -1 && act3_id != -1 && act4_id != -1) {
                printf("Left_front_l Control Input: left_front_j_ctrl = %.3f, left_front_1_j_ctrl = %.3f left_front_2_j_ctrl = %.3f left_front_3_j_ctrl = %.3f\n", 
                d->ctrl[act1_id], 
                d->ctrl[act2_id],
                d->ctrl[act3_id],
                d->ctrl[act4_id]);
            }
            if (act5_id != -1 && act6_id != -1 && act7_id != -1) {
                printf("Left_behind_l Control Input: left_behind_j_ctrl = %.3f, left_behind_withwheel_j_ctrl = %.3f left_wheel_j_ctrl = %.3f\n", 
                d->ctrl[act5_id], 
                d->ctrl[act6_id],
                d->ctrl[act7_id]);
            }
            if (act8_id != -1 && act9_id != -1 && act10_id != -1) {
                printf("Right_behind_l Control Input: right_behind_j_ctrl = %.3f, right_behind_withwheel_j_ctrl = %.3f right_wheel_j_ctrl = %.3f \n", 
                d->ctrl[act8_id], 
                d->ctrl[act9_id],
                d->ctrl[act10_id]);
            }
            if (act11_id != -1 && act12_id != -1 && act13_id != -1 && act14_id != -1) {
                printf("Right_front_l Control Input: right_front_j_ctrl = %.3f, right_front_1_j_ctrl = %.3f right_front_2_j_ctrl = %.3f right_front_3_j_ctrl = %.3f\n", 
                d->ctrl[act11_id], 
                d->ctrl[act12_id],
                d->ctrl[act13_id],
                d->ctrl[act14_id]);
            }
            if (act15_id != -1 && act16_id != -1 && act17_id != -1 && act18_id != -1) {
                printf("gimbal_yaw_l Control Input: gimbal_yaw_j_ctrl = %8.3f, pitch_j_ctrl = %8.3f pitch_1_j_ctrl = %8.3f pitch_2_j_ctrl = %8.3f\n", 
                d->ctrl[act15_id], 
                d->ctrl[act16_id],
                d->ctrl[act17_id],
                d->ctrl[act18_id]);
            }
            printf("-------------------------------------------------------\n");
    
            // 更新上次打印的时间
            last_print_time = d->time;
            }
        }
        // --- 可视化更新 ---
        mjrRect viewport = {0, 0, 0, 0};
        glfwGetFramebufferSize(window, &viewport.width, &viewport.height);
        
        // 更新场景并渲染
        mjv_updateScene(m, d, &opt, NULL, &cam, mjCAT_ALL, &scn);
        mjr_render(viewport, &scn, &con);
        
        // 交换缓冲区
        glfwSwapBuffers(window);
        
        // 处理GUI事件
        glfwPollEvents();
    }
    // --- 清理 ---
    mj_deleteData(d);
    mj_deleteModel(m);
    mjr_freeContext(&con);
    mjv_freeScene(&scn);
    glfwTerminate();
    return 0;
}





// =================================================================
// 调试控制器
// =================================================================
void myController(const mjModel* m, mjData* d) {
    // //键盘测量
    // int id_L1 = mj_name2id(m, mjOBJ_ACTUATOR, "left_front_j_ctrl");
    // int id_L2 = mj_name2id(m, mjOBJ_ACTUATOR, "left_behind_j_ctrl");

    // // 降低 KP，防止爆炸
    // double kp = 5.0;  // 从 20 降到 5
    // double kd = 0.2;  // 增加阻尼

    // // --- 只控制左腿 (Left Leg Only) ---
    // if (id_L1 != -1) {
    //     int jnt = m->actuator_trnid[id_L1*2];
    //     d->ctrl[id_L1] = kp * (tune_q1 - d->qpos[m->jnt_qposadr[jnt]]) - kd * d->qvel[m->jnt_dofadr[jnt]];
    // }
    // if (id_L2 != -1) {
    //     int jnt = m->actuator_trnid[id_L2*2];
    //     d->ctrl[id_L2] = kp * (tune_q2 - d->qpos[m->jnt_qposadr[jnt]]) - kd * d->qvel[m->jnt_dofadr[jnt]];
    // }
    
    //右腿暂时不管，让它自然垂下

    // //正弦波测量
    // const char* target_joint = "right_front_j"; 
    // // -------------------------------------------

    // int jnt_id = mj_name2id(m, mjOBJ_JOINT, target_joint);
    
    // // 自动寻找对应的 Actuator (假设命名规则是 _ctrl 后缀)
    // // 如果找不到，尝试直接用 ID 匹配
    // int act_id = -1;
    // // 简单查找：假设 actuator ID 和 joint ID 顺序大致对应 (仅用于调试)
    // // 更严谨的方法是用名字查找，这里假设你之前的 names 数组逻辑是好的
    // std::string ctrl_name = std::string(target_joint) + "_ctrl";
    // act_id = mj_name2id(m, mjOBJ_ACTUATOR, ctrl_name.c_str());

    // if (jnt_id != -1 && act_id != -1) {
    //     // 生成一个正弦波目标：幅度 0.5 rad，周期约 3秒
    //     double target = 0.8 * sin(d->time * 2.0);

    //     // 简单的 P 控制
    //     double q = d->qpos[m->jnt_qposadr[jnt_id]];
    //     double v = d->qvel[m->jnt_dofadr[jnt_id]];
        
    //     // 施加力矩
    //     d->ctrl[act_id] = 5.0 * (target - q) - 0.2 * v;

    //     // 打印数据
    //     static double last_print = 0;
    //     if (d->time - last_print > 0.1) {
    //         printf("测试 [%s]: 目标=%.2f | 实际=%.2f | 电机输出=%.2f\n", 
    //                target_joint, target, q, d->ctrl[act_id]);
    //         last_print = d->time;
    //     }
    // } else {
    //     static double last_warn = 0;
    //     if (d->time - last_warn > 1.0) {
    //         printf("错误：找不到关节 %s 或其对应的控制器\n", target_joint);
    //         last_warn = d->time;
    //     }
    // }
}
  



void keyboardCB(GLFWwindow* window, int key, int scancode, int act, int mods) {
    Sim_Keyboard_Callback(key, act);


//      键盘调试的对应代码
//     if (act == GLFW_PRESS || act == GLFW_REPEAT) {
//     // 调整关节 1 (U/J)
//     if (key == GLFW_KEY_U) tune_q1 += 0.05;
//     if (key == GLFW_KEY_J) tune_q1 -= 0.05;
    
//     // 调整关节 2 (I/K)
//     if (key == GLFW_KEY_I) tune_q2 += 0.05;
//     if (key == GLFW_KEY_K) tune_q2 -= 0.05;

//     printf("当前尝试角度: Q1=%.2f, Q2=%.2f\n", tune_q1, tune_q2);
// }
    
}

void mouseButtonCB(GLFWwindow* window, int button, int act, int mods) {
    button_left = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS);
    button_middle = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE) == GLFW_PRESS);
    button_right = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS);
    glfwGetCursorPos(window, &lastx, &lasty);
}
void cursorPosCB(GLFWwindow* window, double xpos, double ypos) {
    if (!button_left && !button_middle && !button_right) return;
    double dx = xpos - lastx;
    double dy = ypos - lasty;
    lastx = xpos;
    lasty = ypos;
    int width, height;
    glfwGetWindowSize(window, &width, &height);
    bool mod_shift = (glfwGetKey(window, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS ||
                      glfwGetKey(window, GLFW_KEY_RIGHT_SHIFT) == GLFW_PRESS);
    double cart_pos[2] = {0.5, 0.5}; // 示例目标: x=0.5, z=0.5
    if (button_right) {
        if (mod_shift) {
            mjv_moveCamera(m, mjMOUSE_MOVE_H, dx/height, dy/height, &scn, &cam);
        } else {
            mjv_moveCamera(m, mjMOUSE_ZOOM, 0, dy/height, &scn, &cam);
        }
    } else if (button_left) {
        mjv_moveCamera(m, mjMOUSE_ROTATE_H, dx/height, dy/height, &scn, &cam);
    }
}
void scrollCB(GLFWwindow* window, double xoffset, double yoffset) {
    mjv_moveCamera(m, mjMOUSE_ZOOM, 0, -0.05 * yoffset, &scn, &cam);
}










