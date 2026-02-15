#include <mujoco/mujoco.h>
#include <GLFW/glfw3.h>
#include <cstdio>
#include <cmath>
#include <string>
#include <chassis_controller.h>

// =================================================================
// 1. Globals & Configuration
// =================================================================
mjModel* m = nullptr;
mjData* d = nullptr;
mjvCamera cam;
mjvOption opt;
mjvScene scn;
mjrContext con;

bool button_left = false, button_middle = false, button_right = false;
double lastx = 0, lasty = 0;
bool is_paused = true;     // Default to paused for analysis
bool step_simulation = false;
double last_control_time = -1.0; // Timer for 2ms control interval

// Extern functions from SM_io.cpp
extern void Sim_Update_IO(const mjModel* m, mjData* d);
extern void Sim_Keyboard_Callback(int key, int act);

// Cached MuJoCo IDs
struct MuJoCoIDs {
    int sensor_gyro;
    void cache(const mjModel* model) {
        sensor_gyro = mj_name2id(model, mjOBJ_SENSOR, "imu_gyro");
    }
} ids;

// =================================================================
// 2. Helper Functions
// =================================================================
const char* fsmStateToString(ControllerState state) {
    switch (state) {
        case FSM_STATIONARY:         return "STATIONARY";
        case FSM_SPINNING:           return "SPINNING";
        case FSM_DIFF_DRIVE:         return "DIFF_DRIVE";
        case FSM_CLIMBSTAIR_LIFTLEG: return "CLIMB_LIFT";
        case FSM_CLIMBSTAIR_FOLDLEG: return "CLIMB_FOLD";
        case FSM_JUMP_SQUATING:      return "JUMP_SQUAT";
        case FSM_JUMP_STRETCHING:    return "JUMP_STRETCH";
        case FSM_JUMP_FOLDING:       return "JUMP_FOLD";
        case FSM_JUMP_LANDING:       return "JUMP_LAND";
        case FSM_LIFT_UP:            return "LIFT_UP";
        case FSM_FORCE_LIFT_UP:      return "FORCE_LIFT";
        case FSM_MANUAL_LIFT_UP:     return "MANUAL_LIFT";
        case FSM_SELF_SAVE:          return "SELF_SAVE";
        default:                     return "UNKNOWN";
    }
}

void printDebugInfo() {
    auto& ctx = chassis_SoftWareController.context_;
    auto& vmc = ctx.vmc_state;
    auto& cmd = ctx.internal_command;
    auto& trq = ctx.motor_TargetTorque;

    printf("\n--- [Time: %7.3f s | State: %s] %s ---\n", 
           d->time, fsmStateToString(ctx.controller_state), is_paused ? "(PAUSED)" : "");
    
    printf("POS/VEL: X=%+6.3fm [err=%+6.4f] | VX=%+6.3fm/s [err=%+6.4f]\n", 
           vmc.center_x, vmc.center_x - cmd.x_expectation,
           vmc.center_v_x, vmc.center_v_x - cmd.desired_vel_x);

    printf("LEGS:    L_l0=%+.4f R_l0=%+.4f m | Target=%+.4f\n", 
           vmc.y_left, vmc.y_right, cmd.target_y_l);
    
    printf("FORCE:   L_f=%+7.2f R_f=%+7.2f N\n", ctx.debug_info.test_1, ctx.debug_info.test_2);
    printf("TORQUE:  L_w=%+6.2f R_w=%+6.2f | L_f=%+6.2f L_b=%+6.2f | R_f=%+6.2f R_b=%+6.2f\n",
           trq.m3508_troque_left, trq.m3508_troque_right,
           trq.ckyf_troque_left_front, trq.ckyf_troque_left_back,
           trq.ckyf_troque_right_front, trq.ckyf_troque_right_back);

    if (ids.sensor_gyro != -1) {
        mjtNum* g = d->sensordata + m->sensor_adr[ids.sensor_gyro];
        printf("IMU:     Roll=%+6.3f Pitch=%+6.3f Yaw=%+6.3f rad\n", g[0], g[1], g[2]);
    }
    printf("-------------------------------------------------------\n");
}

// =================================================================
// 3. MuJoCo Callbacks & GLFW
// =================================================================
void myController(const mjModel* model, mjData* data) {
    // Run the controller only every 2ms
    if (data->time - last_control_time >= 0.002 - 1e-6) {
        Sim_Update_IO(model, data);
        last_control_time = data->time;
    }
}

void keyboardCB(GLFWwindow* window, int key, int scancode, int act, int mods) {
    if (act == GLFW_PRESS) {
        if (key == GLFW_KEY_P) {
            is_paused = !is_paused;
            printf(">>> Simulation %s\n", is_paused ? "PAUSED" : "RESUMED");
        }
        if (key == GLFW_KEY_N) {
            is_paused = true;
            step_simulation = true;
            printf(">>> Simulation STEP (0.002s)\n");
        }
        if (key == GLFW_KEY_R) {
            mj_resetData(m, d);
            d->qpos[2] = 0.15;
            last_control_time = -1.0; // Reset control timer
            printf(">>> Simulation RESET\n");
        }
        if (key == GLFW_KEY_F) {
            if (cam.type == mjCAMERA_TRACKING) {
                // 如果当前是跟随模式，切换回自由模式
                cam.type = mjCAMERA_FREE;
                printf(">>> Camera: Free Mode (自由视角)\n");
            } else {
                // 如果当前是自由模式，切换为跟随模式
                cam.type = mjCAMERA_TRACKING;
                
                // 获取你要跟随的物体 ID (你的 XML 里基座叫 "base_link")
                int body_id = mj_name2id(m, mjOBJ_BODY, "base_link");
                
                if (body_id != -1) {
                    cam.trackbodyid = body_id; // 设置跟随目标 ID
                    printf(">>> Camera: Tracking Mode (跟随视角) -> base_link\n");
                } else {
                    printf(">>> Error: Could not find body 'base_link'\n");
                }
            }
        }
    }
    Sim_Keyboard_Callback(key, act);
    
}

void mouseButtonCB(GLFWwindow* window, int button, int act, int mods) {
    button_left   = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT)   == GLFW_PRESS);
    button_right  = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT)  == GLFW_PRESS);
    button_middle = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE) == GLFW_PRESS);
    glfwGetCursorPos(window, &lastx, &lasty);
}

void cursorPosCB(GLFWwindow* window, double xpos, double ypos) {
    if (!button_left && !button_middle && !button_right) return;
    double dx = xpos - lastx, dy = ypos - lasty;
    lastx = xpos; lasty = ypos;
    int width, height;
    glfwGetWindowSize(window, &width, &height);
    bool shift = (glfwGetKey(window, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS || glfwGetKey(window, GLFW_KEY_RIGHT_SHIFT) == GLFW_PRESS);
    if (button_right) mjv_moveCamera(m, shift ? mjMOUSE_MOVE_H : mjMOUSE_ZOOM, dx/height, dy/height, &scn, &cam);
    else if (button_left) mjv_moveCamera(m, mjMOUSE_ROTATE_H, dx/height, dy/height, &scn, &cam);
}

void scrollCB(GLFWwindow* window, double xoffset, double yoffset) {
    mjv_moveCamera(m, mjMOUSE_ZOOM, 0, -0.05 * yoffset, &scn, &cam);
}

// ==========================================
// 3. 可视化绘图模块 (Add this to taskfinal.cpp)
// ==========================================
mjvFigure figTorque;
mjvFigure figIMU;
bool plotsInitialized = false;

// 初始化图表配置
// 初始化图表配置
void InitPlots() {
    // --- 1. 力矩图表 ---
    mjv_defaultFigure(&figTorque);
    strcpy(figTorque.title, "Actuator Torques (Nm)");
    figTorque.flg_legend = 1;
    figTorque.flg_ticklabel[0] = 1; 
    figTorque.gridsize[0] = 5;      
    figTorque.gridsize[1] = 5;      
    strcpy(figTorque.xlabel, "Time");
    
    const char* tq_names[] = {"RF", "RB", "LF", "LB", "RW", "LW"};
    const float tq_colors[][3] = {
        {1, 0, 0}, {0.5f, 0, 0},   // Right: Red
        {0, 0, 1}, {0, 0, 0.5f},   // Left: Blue
        {1, 0.5f, 0}, {0, 0.5f, 1} // Wheels: Orange/Cyan
    };
    
    for (int i=0; i<6; i++) {
        strcpy(figTorque.linename[i], tq_names[i]);
        
        // ★★★ 修正点：使用 linergb ★★★
        figTorque.linergb[i][0] = tq_colors[i][0];
        figTorque.linergb[i][1] = tq_colors[i][1];
        figTorque.linergb[i][2] = tq_colors[i][2];
    }
    figTorque.range[1][0] = -40; 
    figTorque.range[1][1] =  40; 

    // --- 2. IMU图表 ---
    mjv_defaultFigure(&figIMU);
    strcpy(figIMU.title, "IMU RPY (rad)");
    figIMU.flg_legend = 1;
    figIMU.flg_ticklabel[0] = 1;
    strcpy(figIMU.linename[0], "Roll");
    strcpy(figIMU.linename[1], "Pitch");
    strcpy(figIMU.linename[2], "Yaw");
    
    // ★★★ 修正点：使用 linergb ★★★
    figIMU.linergb[0][0]=1; figIMU.linergb[0][1]=0; figIMU.linergb[0][2]=0; // Roll: Red
    figIMU.linergb[1][0]=0; figIMU.linergb[1][1]=1; figIMU.linergb[1][2]=0; // Pitch: Green
    figIMU.linergb[2][0]=0; figIMU.linergb[2][1]=0; figIMU.linergb[2][2]=1; // Yaw: Blue
    
    figIMU.range[1][0] = -1.0;
    figIMU.range[1][1] =  1.0;

    plotsInitialized = true;
}

// 更新图表数据 (在主循环中调用)
void UpdatePlots(const mjModel* m, const mjData* d) {
    if (!plotsInitialized) return;

    // --- 更新力矩数据 ---
    // d->ctrl 对应 actuator 输出
    // 假设顺序: 0:RF, 1:RB, 2:LF, 3:LB, 4:RW, 5:LW
    float torques[6];
    torques[0] = (float)d->ctrl[10]; //LF
    torques[1] = (float)d->ctrl[7]; //LB
    torques[2] = (float)d->ctrl[0]; //LW
    torques[3] = (float)d->ctrl[4]; //RB
    torques[4] = (float)d->ctrl[9]; //RW
    torques[5] = (float)d->ctrl[6];//RF
    // 将数据推入滚动缓冲区
    int pnt = mjMIN(200, figTorque.linepnt[0] + 1); // 限制点数
    for (int i=0; i<6; i++) {
        // 数据左移 (滚动效果)
        for (int j=0; j<pnt-1; j++) {
            figTorque.linedata[i][2*j+1] = figTorque.linedata[i][2*j+3];
        }
        // 填入新数据
        figTorque.linedata[i][2*(pnt-1)] = (float)d->time;      // X: Time
        figTorque.linedata[i][2*(pnt-1)+1] = torques[i];        // Y: Torque
        figTorque.linepnt[i] = pnt;
    }

    // --- 更新 IMU 数据 ---
    // 计算 RPY (假设基座是 FreeJoint，qpos[3-6] 是四元数)
    double q0 = d->qpos[3], q1 = d->qpos[4], q2 = d->qpos[5], q3 = d->qpos[6];
    float rpy[3];
    // MuJoCo quaternion is [w, x, y, z]
    rpy[0] = atan2(2*(q0*q1 + q2*q3), 1 - 2*(q1*q1 + q2*q2)); // Roll
    rpy[1] = asin(2*(q0*q2 - q3*q1));                         // Pitch
    rpy[2] = atan2(2*(q0*q3 + q1*q2), 1 - 2*(q2*q2 + q3*q3)); // Yaw

    for (int i=0; i<3; i++) {
        for (int j=0; j<pnt-1; j++) {
            figIMU.linedata[i][2*j+1] = figIMU.linedata[i][2*j+3];
        }
        figIMU.linedata[i][2*(pnt-1)] = (float)d->time;
        figIMU.linedata[i][2*(pnt-1)+1] = rpy[i];
        figIMU.linepnt[i] = pnt;
    }
}

// 绘制图表 (在 mjr_render 之前调用)
void RenderPlots(mjrRect viewport, mjrContext* ctx) { 
    if (!plotsInitialized) InitPlots();

// ★★★ 修改建议：把宽度从 300 改为 400 或 500 ★★★
    // ★★★ 把高度从 200 改为 250 (让垂直方向也没那么挤) ★★★
    int w = 500; // 宽度
    int h = 250; // 高度

    // 力矩图放在右下角
    mjrRect rectTorque = {viewport.width - w, 0, w, h}; 
    
    // IMU图放在力矩图的上方
    mjrRect rectIMU    = {viewport.width - w, h + 10, w, h};

    // 使用传入的 ctx 上下文进行绘制
    mjr_figure(rectTorque, &figTorque, ctx);
    mjr_figure(rectIMU,    &figIMU,    ctx);
}

// =================================================================
// 4. Main Entry
// =================================================================
int main(void) {
    char errstr[1000];
    m = mj_loadXML("hero.xml", nullptr, errstr, 1000);
    if (!m) mju_error("Load error: %s", errstr);
    d = mj_makeData(m);
    d->qpos[2] = 0.3; // Initial height
    
    ids.cache(m);
    mjcb_control = myController;

    if (!glfwInit()) mju_error("GLFW error");
    GLFWwindow* window = glfwCreateWindow(1200, 900, "MuJoCo Wheel-Leg Sim", nullptr, nullptr);
    if (!window) mju_error("Window creation failed");

    printf("\n=======================================================\n");
    printf("   MuJoCo Wheel-Leg Simulation Started (PAUSED)\n");
    printf("-------------------------------------------------------\n");
    printf("   [P] toggle Pause/Resume\n");
    printf("   [N] Step one physics step (0.002s)\n");
    printf("   [R] Reset simulation\n");
    printf("   [W/S/A/D] Change velocity/turn targets\n");
    printf("   [Q] Go straight\n");
    printf("   [E] Faster\n");
    printf("=======================================================\n\n");

    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);

    mjv_defaultCamera(&cam);
    mjv_defaultOption(&opt);
    mjv_defaultScene(&scn);
    mjr_defaultContext(&con);
    mjv_makeScene(m, &scn, 10000);
    mjr_makeContext(m, &con, mjFONTSCALE_100);

    glfwSetKeyCallback(window, keyboardCB);
    glfwSetCursorPosCallback(window, cursorPosCB);
    glfwSetMouseButtonCallback(window, mouseButtonCB);
    glfwSetScrollCallback(window, scrollCB);

    cam.distance = 2.5; cam.azimuth = 90; cam.elevation = -45;
    cam.lookat[0] = 0; cam.lookat[1] = 0; cam.lookat[2] = 0.5;

    InitPlots();
    mjtNum last_print_time = -1.0;
    while (!glfwWindowShouldClose(window)) {
        if (!is_paused) {
            double simstart = d->time;
            while (d->time - simstart < 1.0 / 60.0) {
                mj_step(m, d);
            }
            if (!is_paused || step_simulation) {
             UpdatePlots(m, d);
            }
        } else if (step_simulation) {
            mj_step(m, d);
            step_simulation = false; // Reset after one physics step
            printDebugInfo();        // Force log on explicit step
        } else {
            mj_forward(m, d);
        }
        
        if (d->time - last_print_time > 0.1) {
            last_print_time = d->time;
            printDebugInfo();
        }

        mjrRect viewport = {0, 0, 0, 0};
        glfwGetFramebufferSize(window, &viewport.width, &viewport.height);
        mjv_updateScene(m, d, &opt, nullptr, &cam, mjCAT_ALL, &scn);
        mjr_render(viewport, &scn, &con);
        RenderPlots(viewport,&con);
        glfwSwapBuffers(window);
        glfwPollEvents();
    }

    mj_deleteData(d); mj_deleteModel(m);
    mjr_freeContext(&con); mjv_freeScene(&scn);
    glfwTerminate();
    return 0;
}










