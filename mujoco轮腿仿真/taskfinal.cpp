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
    // This callback is called during mj_step
    Sim_Update_IO(model, data);
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
            printf(">>> Simulation RESET\n");
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

// =================================================================
// 4. Main Entry
// =================================================================
int main(void) {
    char errstr[1000];
    m = mj_loadXML("custom_robot.xml", nullptr, errstr, 1000);
    if (!m) mju_error("Load error: %s", errstr);
    d = mj_makeData(m);
    m->opt.timestep = 0.002;
    d->qpos[2] = 0.15; // Initial height
    
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
    printf("=======================================================\n\n");

    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);

    mjv_defaultCamera(&cam);
    mjv_defaultOption(&opt);
    mjv_defaultScene(&scn);
    mjr_defaultContext(&con);
    mjv_makeScene(m, &scn, 10000);
    mjr_makeContext(m, &con, mjFONTSCALE_150);

    glfwSetKeyCallback(window, keyboardCB);
    glfwSetCursorPosCallback(window, cursorPosCB);
    glfwSetMouseButtonCallback(window, mouseButtonCB);
    glfwSetScrollCallback(window, scrollCB);

    cam.distance = 2.5; cam.azimuth = 90; cam.elevation = -45;
    cam.lookat[0] = 0; cam.lookat[1] = 0; cam.lookat[2] = 0.5;

    mjtNum last_print_time = -1.0;
    while (!glfwWindowShouldClose(window)) {
        if (!is_paused) {
            double simstart = d->time;
            while (d->time - simstart < 1.0 / 60.0) {
                mj_step(m, d);
            }
        } else if (step_simulation) {
            mj_step(m, d);
            step_simulation = false; // Reset after one physics step
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
        glfwSwapBuffers(window);
        glfwPollEvents();
    }

    mj_deleteData(d); mj_deleteModel(m);
    mjr_freeContext(&con); mjv_freeScene(&scn);
    glfwTerminate();
    return 0;
}










