# RM2026-轮腿机器人代码架构

---
## **Controller**
    -封装Utils、FSM、Estimator、Actuator，作为对外接口
    -将Chassis的目标搬给ChassisControllerContext

---
## **Utils**
**核心类：`ChassisControllerContext`**

    - 存储机器人当前状态的物理量与几何量（位置、质心、速度、姿态等）
    - 作为FSM、Estimator、Actuator模块间的数据交换中心
    - 提供外部控制指令（速度、角速度等）的接入接口

---

## **Params**
- **功能**：集中存储机器人固定参数（机械参数、控制参数等） ，便于参数调试与维护

---


## **FSM**

---

## **Estimator**


---

## **Actuator**
