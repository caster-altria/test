# LinkageController Demo

C 板姿态与两台 GM6020 的联动控制中间件。

A 电机与 C 板固定 1:1 联动；B 电机与 C 板的联动比由左拨杆决定：

| 左拨杆 | 联动比（B : C 板） |
|---|---|
| 下档 | 1 : 0.5 |
| 中档 | 1 : −1 |
| 上档 | 1 : 3 |

---

# 使用方法

### 编辑 `CMakeLists.txt`

```cmake
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    sp_middleware/tools/linkage_controller/linkage_controller.cpp
    sp_middleware/tools/pid/pid.cpp
    sp_middleware/tools/math_tools/math_tools.cpp
)
```

### 创建电机对象

```cpp
// applications/can_task.cpp
sp::RM_Motor motor_a(1, sp::RM_Motors::GM6020);  // 一个电机ID为1, 电流控制模式的6020
sp::RM_Motor motor_b(2, sp::RM_Motors::GM6020);  // 一个电机ID为2, 电流控制模式的6020
```

- `multi_circle` 取 **`true`（默认，多圈）或 `false`（单圈）都可以**，本模块都能正常工作。
  原理见下面「关于 `multi_circle`」。本工程用默认的 `true`。
- 必须用 `GM6020`（电流控制帧）：`cmd()` 输出单位是 **N·m**。
- ⚠️ 使用前需用 RoboMaster Assistant（≥2.7，固件 ≥1.0.11.2）**打开电流环开关**，
  否则驱动器会按电压解释指令。

### 关于 `multi_circle`

本模块的角度处理**只依赖差值**，所以两种模式等价：

```cpp
const float error_a_rad = limit_angle(angle_a_rad - this->target_a_rad_);
```

反馈 `angle_a_rad` 里多出来的 $2k\pi$ 到了差值里仍然是 $2k\pi$，
又被 `limit_angle` 折掉：

$$limit\_angle\big(T - (\theta + 2k\pi)\big) = limit\_angle(T - \theta)$$

因此 `fdb` 是单圈还是多圈，**算出的误差完全相同**（已逐位验证）：

| 电机转过的圈数 | 多圈 `fdb` | 单圈 `fdb` | 多圈误差 | 单圈误差 |
|---|---|---|---|---|
| 0 | 1.000 | 1.000 | 0.00061 | 0.00061 |
| 1 | 7.283 | 1.000 | 0.00061 | 0.00061 |
| 5 | 32.416 | 1.000 | 0.00061 | 0.00061 |

> `multi_circle = false` 时 `motor.angle` 的范围是 `[-179.96°, +180.00°]`，
> 比严格的 2π 少一个编码器计数（0.044°）。这点偏差对应 P 项约 0.009 N·m，可忽略。
>
> ⚠️ 若换成 `ratio != 1` 的电机（如减速比 19.2），`multi_circle = false` 的单圈范围
> 会变成 ±9.4°，而 `limit_angle` 仍按 2π 折算 —— **周期错配**。此时必须用 `multi_circle = true`。

### 新建 `applications/linkage_task.cpp`

新建这个任务文件（本工程的实现在 `applications/linkage_task.cpp`）。它负责：

- 从 `sp::DBus` 读左拨杆映射成 `sp::LinkageGear`
- 从 `sp::DBus` 读右拨杆决定 联动 / 复位 / 失能
- 把 C 板 yaw、yaw 角速度与档位送进控制器

其中电机对象是从 `can_task.cpp` 复用的：

```cpp
extern sp::RM_Motor motor_a;   // 定义于 can_task.cpp
extern sp::RM_Motor motor_b;   // 定义于 can_task.cpp
```

### 在 `Src/freertos.c` 中创建任务

```c
osThreadId LINKAGETaskHandle;

extern void linkage_task(void const * argument);

osThreadDef(LINKAGETask, linkage_task, osPriorityHigh, 0, 256);
LINKAGETaskHandle = osThreadCreate(osThread(LINKAGETask), NULL);
```

任务周期 1ms，必须与 `CONTROL_PERIOD_S` 一致。

---

# 初始化

```cpp
#include "tools/linkage_controller/linkage_controller.hpp"

sp::LinkageController linkage(motor_a, motor_b);

linkage.init(board_yaw_rad);   // 建立零参考
```

| 参数 | 含义 |
|---|---|
| `board_yaw_rad` | 当前 C 板 yaw，单位 rad，范围 `(-π, π]` |

**这是"以当前姿态为零参考"，不是把电机归到绝对零位。**
`init()` 把主轴清零、两个偏移取当前电机角，于是目标角等于实际角，
电机保持当前位置不动。上电或重新进入联动档位时调用一次即可。

> `init()` 与 `reset()` 的区别：`init()` **电机不动**（建立参考），
> `reset()` **电机转向箭头同向**（完成对齐）。详见下面「复位」。

---

# 调用

```cpp
linkage.update(board_yaw_rad, board_rate_rad_s, gear, now_ms);
```

| 参数 | 含义 |
|---|---|
| `board_yaw_rad` | 当前 C 板 yaw，单位 rad，范围 `(-π, π]` |
| `board_rate_rad_s` | 当前 C 板 yaw 角速度，单位 rad/s，传 `imu.vyaw` |
| `gear` | 当前左拨杆档位 |
| `now_ms` | 当前时间戳，单位 ms，用于 `is_alive()` 掉线检测 |

`board_rate_rad_s` **必须传**：控制器靠它区分"C 板主动转动"和"手拧电机"。

### 失能

```cpp
linkage.disable();
```

两台电机输出置 0，并清空 PID 与手动标记。右拨杆下档时调用。
重新使能前需要再调一次 `init()`。

### 复位

```cpp
linkage.reset(board_yaw_rad);
```

右拨杆切到上档时调用**一次**（不要每拍都调，否则会反复清 PID 状态）。

动作是：主轴清零，两个偏移取 **C 板 yaw 按联动比折算**的值，于是

```
target_a = board_yaw_rad
target_b = board_yaw_rad × 联动比
```

两个目标角就是"两个箭头与 C 板箭头同向"的位置，**电机会主动转过去完成对齐**。

对齐之后关系被冻结：主轴随 C 板推进、偏移不变，所以
`(电机角 − C 板角)` 保持为 0，之后转板子两者同向跟随，不会回正。

> 复位期间仍要跑 `update()`，否则电机没有力矩输出会松开。
> 复位需要电机出力，**电流环开关必须先打开**。

---

# 查询结果

```cpp
float master   = linkage.master_angle();  // 只读! 虚拟主轴角, 单位: rad
float target_a = linkage.target_a();      // 只读! A 目标角, 单位: rad, (-pi, pi]
float target_b = linkage.target_b();      // 只读! B 目标角, 单位: rad, (-pi, pi]
```

`master_angle()` 是内部中间量，连续多圈。调车时看它的变化能判断当前驱动源
（C 板 / 手拧 A / 手拧 B）。

`target_a()` / `target_b()` 由主轴按联动比投影得到，稳态时电机实际角应收敛到它们。

---

# 参数

## 位置环 PID

| 参数 | 默认值 | 说明 |
|---|---|---|
| `LOOP_KP` | 12.0 | 比例增益（无量纲） |
| `LOOP_KI` | 0.0 | 积分增益（无量纲）。默认关闭 |
| `LOOP_KD` | 0.3 | 微分增益（无量纲），提供阻尼 |
| `LOOP_MAX_TORQUE_NM` | 1.5 | 输出限幅，单位 N·m |
| `LOOP_MAX_INTEGRAL_NM` | 0.0 | 积分限幅，单位 N·m |
| `LOOP_D_FILTER_ALPHA` | 0.8 | D 项滤波系数，越小滤波越强 |
| `LOOP_INTEGRAL_PAUSE_NM` | 0.5 | 积分分离阈值（N·m），P 项超过它暂停积分 |

### 关于 `LOOP_KI`

默认 0：位置环输出是力矩，空载/轻载时 P 项不需要持续输出就能维持位置，
板子停下后误差自己收敛到 0。

**如果执行机构带恒定负载力矩**（明显摩擦、重力矩），P 项必须持续输出 `load`
才能顶住，于是留下稳态误差 `load / KP`。这时要把 **`LOOP_KI` 和
`LOOP_MAX_INTEGRAL_NM` 一起**改成非 0（后者至少 `KP × 允许偏差`，约 1.2 N·m）。

> 只改 `LOOP_KI` 无效：`sp::PID` 内部是
> `iout = limit_max(iout + ki*trapezoid, max_iout)`，
> `max_iout` 为 0 时 `iout` 被钳在 0。

## 检测阈值

| 参数 | 默认值 | 说明 |
|---|---|---|
| `BOARD_STILL_RATE_RAD_S` | 0.15 | C 板"静止"判据（≈8.6 °/s），单位 rad/s |
| `MANUAL_MOVE_THRESHOLD_RAD` | 5e-4 | 手转判据的角度增量（≈30 °/s），单位 rad |
| `MANUAL_ERROR_THRESHOLD_RAD` | 0.05 | 手转判据的角度偏差（≈2.9°），单位 rad |
| `MANUAL_STOP_THRESHOLD_RAD` | 1e-4 | 停止判据的增量，单位 rad |
| `MANUAL_STOP_COUNT_MAX` | 100 | 连续静止 100 拍（0.1s）认为手已松开 |

**`MANUAL_STOP_COUNT_MAX` 最需要实测**：0.1s 对人拧过程中的自然停顿可能偏短，
会提前退出手动模式。手感不对就调到 300~500。

## 方向符号

```cpp
static constexpr float MOTOR_A_DIRECTION_SIGN = 1.0f;
static constexpr float MOTOR_B_DIRECTION_SIGN = 1.0f;
```

用于把电机安装方向统一到"C 板逆时针为正"的坐标系。某台电机装反就改成 `-1.0f`。

---

# 测试心得

1. **先看 `master_angle()` 有没有在动**。转板子时它不动 → `board_rate_rad_s`
   传错（应为 `imu.vyaw`），或 `BOARD_STILL_RATE_RAD_S` 设得太大。
2. **确认电流环开关已打开**，否则 `cmd()` 的 N·m 会被当成 V 解释。
3. **`multi_circle` 两种模式都可以**，但若电机 `ratio != 1`，必须用 `true`
   （见「关于 `multi_circle`」）。
4. **复位只在进入上档那一拍调用**，每拍都调会反复清 PID 状态，表现为持续抖动。
5. **`LOOP_KI` 保持 0**，除非确认机构有恒定负载力矩。
