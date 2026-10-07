#ifndef SP__LINKAGE_CONTROLLER_HPP
#define SP__LINKAGE_CONTROLLER_HPP

#include <cstdint>

#include "motor/rm_motor/rm_motor.hpp"
#include "tools/pid/pid.hpp"

namespace sp
{
// 左拨杆档位, 对应 B 电机与 C 板的联动比
enum class LinkageGear
{
  DOWN,    // 下档: 联动比 1:0.5
  MIDDLE,  // 中档: 联动比 1:-1
  UP       // 上档: 联动比 1:3
};

// C 板与两台 GM6020 的联动控制器
//
// 联动关系:
//   A 电机与 C 板固定 1:1 联动
//   B 电机与 C 板联动比由左拨杆决定: 下档 0.5, 中档 -1, 上档 3
//
// 工作机制:
//   内部维护一个虚拟主轴角 master_angle_rad_(中间量)与两个持久偏移 offset_a_rad_、offset_b_rad_。
//   主轴按各模式的增量推进, 两个目标角由主轴按联动比投影得到:
//       target_a_rad_ = master_angle_rad_ * 1.0f + offset_a_rad_
//       target_b_rad_ = master_angle_rad_ * ratio  + offset_b_rad_
//   - C 板主动转动时: 主轴累加 C 板 yaw 增量, 两个目标角随之整体前进;
//   - 手动转 A 时: 主轴累加 A 增量, A 的偏移直接跟实际值(避免 PID 抵抗),
//     B 由主轴按联动比投影后跟随;
//   - 手动转 B 时: 把 B 增量除以联动比折算回主轴, 主轴与 A 目标随之推进,
//     B 的偏移直接跟实际值。
//   偏移只在手拧时更新, 松手后冻结, 因此不会回正。
//   换挡时只改联动比, 不动主轴, 因此目标角不会跳变。
//
// 角度约定:
//   位置环 sp::PID 使用 angular = true, 由 PID 内部用 limit_angle 把误差折算到
//   (-pi, pi]。本模块的角度处理只依赖【差值】, 因此电机的 multi_circle 取
//   true(多圈) 或 false(单圈) 都能正常工作:
//
//       error = limit_angle(target - fdb)
//
//   fdb 里多出来的 2k*pi 到了差值里仍然是 2k*pi, 又被 limit_angle 折掉, 所以
//   两种模式算出的误差完全相同(已逐位验证)。本工程用 multi_circle = true。
//
//   同理, target 自身在 ±pi 处会跳 2pi, 但 fdb 走的是同一个 limit_angle,
//   跳变在误差里同向抵消, 实测单拍误差跳变恒为 0, 不会产生力矩突变。
//
//   虚拟主轴 master_angle_rad_ 内部保持连续, 只用于累加增量, 不参与角度比较。
//
// 使用方法与参数整定见 readme.md
class LinkageController
{
public:
  // motor_a: A 电机
  // motor_b: B 电机
  // 两者的 multi_circle 取 true 或 false 均可(见上面的角度约定)
  LinkageController(RM_Motor & motor_a, RM_Motor & motor_b);

  // 建立零参考: 上电或进入联动档位时调用一次
  //
  // board_yaw_rad: 当前 C 板 yaw, 单位: rad, 范围 (-pi, pi]
  //
  // 做法是"以当前姿态为基准", 不是把电机归到绝对零位:
  //   主轴清零, 两个偏移取当前电机角, 于是目标角 == 实际角, 电机不动。
  // 联动比不在这里设置, 由 update 每拍从档位刷新。
  void init(float board_yaw_rad);

  // 每个控制周期调用一次
  // board_yaw_rad:  当前 C 板 yaw, 单位: rad, 范围 (-pi, pi]
  // board_rate_rad_s: 当前 C 板 yaw 角速度, 单位: rad/s, 传 sp::Mahony::vyaw
  // gear:       当前左拨杆档位
  // now_ms:     当前时间戳, 用于电机在线检测
  void update(float board_yaw_rad, float board_rate_rad_s, LinkageGear gear, uint32_t now_ms);

  // 失能: 两台电机输出置 0, 并清空各轴 PID 与手动标记
  // 右拨杆不在联动档位时调用; 重新使能前需调用 init
  void disable();

  // 复位: 让两台电机的指向标箭头转到与 C 板箭头同向
  //
  // 右拨杆切到上档时调用一次。动作是:
  //   主轴清零, 两个偏移取 "C 板 yaw 按联动比折算" 的值,
  //   于是目标角变成箭头同向的位置, 电机会主动转过去。
  //
  // 为什么复位之后对齐关系能一直保持:
  //   联动模式下主轴随 C 板推进, 而偏移保持不变, 所以
  //       (电机实际角 - C 板角) 恒等于 (偏移 - 主轴初值)
  //   复位把右侧这个常数置成"同向", 之后 C 板转动时电机跟着转,
  //   对齐关系被"冻结"住, 不会回正。
  //
  // board_yaw_rad: 当前 C 板 yaw, 单位: rad, 范围 (-pi, pi]
  void reset(float board_yaw_rad);

  // 只读! 当前虚拟主轴角, 单位: rad, 相对本次 init/reset 时刻
  float master_angle() const { return this->master_angle_rad_; }

  // 只读! 当前 A、B 目标角, 单位: rad, 范围 (-pi, pi]
  float target_a() const { return this->target_a_rad_; }
  float target_b() const { return this->target_b_rad_; }

private:
  // 内部模式
  enum class Mode
  {
    NORMAL,    // 正常模式: C 板为主
    MANUAL_A,  // 手动转 A
    MANUAL_B   // 手动转 B
  };

  // 把拨杆档位转换成 B/C 联动比
  static float gear_to_ratio(LinkageGear gear);

  // 读取一台电机的角度, 并统一到"C 板逆时针为正"的坐标系
  float read_motor_a() const { return MOTOR_A_DIRECTION_SIGN * motor_a_.angle; }
  float read_motor_b() const { return MOTOR_B_DIRECTION_SIGN * motor_b_.angle; }

  // 手动判据: 返回 1 表示判定为手拧 A, -1 表示手拧 B, 0 表示没有手拧。
  // 判据是"该电机增量足够大 且 该电机误差足够大 且 它是主导运动的一方"
  int find_manual_axis(
    float delta_a_rad, float delta_b_rad, float error_a_rad, float error_b_rad) const;

  // 由主轴按联动比投影出两个目标角
  void update_targets();

  // 以当前姿态为参考重置全部状态(init 与 reset 共用)
  // align_to_board: false 时偏移取当前电机角(电机不动),
  //                 true  时偏移取 C 板 yaw 的折算值(电机转向箭头同向)
  void capture_reference(float board_yaw_rad, bool align_to_board);

  // 清 PID 状态并预热反馈缓存, 避免下一次 calc 的微分项冲击
  void prepare_pid(float angle_a_rad, float angle_b_rad);

  // 手动模式的停止判据: 连续若干周期几乎不动就认为手已松开, 退回正常模式
  void update_stop_count(float delta);

  // ---------- 控制周期 ----------
  // 控制周期, 单位: s, 需与任务调用周期严格一致
  static constexpr float CONTROL_PERIOD_S = 1e-3f;

  // ---------- A 与 C 板 / 主轴的固定联动比 ----------
  // 题目要求 A 与 C 板 1:1 联动
  static constexpr float RATIO_A = 1.0f;

  // ---------- 位置环 PID 参数 ----------
  // 位置环比例增益
  static constexpr float LOOP_KP = 12.0f;
  // 位置环积分增益
  //
  // 取 0: 本模块只靠 P + D 就能把稳态误差压到 0, 不需要积分。
  //   原因是位置环的输出是力矩, 空载/轻载时 P 项不需要持续输出就能维持位置,
  //   板子停下后误差会自己收敛到 0。
  //
  // 什么时候需要改成非 0: 执行机构带恒定负载力矩(明显摩擦、重力矩)时,
  //   P 项必须持续输出 load 才能顶住, 于是留下稳态误差 = load / KP。
  //   实测(负载 0.1 N·m、板子转 60°): KI=0 时比例 0.496, KI=5 时 0.500;
  //   负载 0.3 N·m 时 KI=0 会被负载拖跑。那时需要同时抬高 LOOP_MAX_INTEGRAL_NM。
  static constexpr float LOOP_KI = 0.0f;
  // 位置环微分增益, 用于提供阻尼
  static constexpr float LOOP_KD = 0.3f;
  // 位置环输出限幅, 单位: N·m
  static constexpr float LOOP_MAX_TORQUE_NM = 1.5f;
  // 位置环积分限幅, 单位: N·m
  //
  // sp::PID 内部是 iout = limit_max(iout + ki*trapezoid, max_iout),
  // max_iout 为 0 时 iout 被钳在 0, ki 取任何值都不起作用。
  // 所以这个值必须与 LOOP_KI 一起改: KI 非 0 时它要至少能抵消
  // 稳态时 P 项的量(kp × 允许偏差), 取 kp=12、偏差按 0.1 rad 估 -> 约 1.2 N·m。
  static constexpr float LOOP_MAX_INTEGRAL_NM = 0.0f;
  // 位置环 D 项滤波系数, 越小滤波越强, 1 表示不滤波
  static constexpr float LOOP_D_FILTER_ALPHA = 0.8f;
  // 位置环积分分离阈值: P 项输出绝对值超过此值时暂停积分, 单位: N·m
  static constexpr float LOOP_INTEGRAL_PAUSE_NM = 0.5f;
  // 是否启用积分。KI 与 MAX_INTEGRAL 必须同时非 0 积分才会工作,
  // 下面用它决定要不要执行"手动期间冻结积分"那段逻辑, 避免死代码
  static constexpr bool IS_INTEGRAL_ENABLED =
    (LOOP_KI != 0.0f) && (LOOP_MAX_INTEGRAL_NM != 0.0f);

  // ---------- 主动转动 / 手动转动检测阈值 ----------
  // C 板"静止"判据: yaw 角速度绝对值小于此值认为 C 板未主动转动, 单位: rad/s
  //
  // 必须用角速度而不是"yaw 每拍增量":
  //   1kHz 下板子转 1 rad/s 每拍只走 1e-3 rad, 与手拧电机的每拍增量同量级,
  //   用增量大小无法区分"C 板在转"和"手在拧"。角速度是陀螺仪直接测量的量,
  //   不受采样周期影响, 才是真正的区分性特征。
  static constexpr float BOARD_STILL_RATE_RAD_S = 0.15f;
  // 手转电机判据: 电机角度增量阈值, 单位: rad
  // 1kHz 下对应 0.5 rad/s, 约 30°/s, 是人手能自然达到的转动速度
  static constexpr float MANUAL_MOVE_THRESHOLD_RAD = 5e-4f;
  // 手转电机判据: 实际角与目标角的误差阈值, 单位: rad
  static constexpr float MANUAL_ERROR_THRESHOLD_RAD = 0.05f;
  // 手动停止判据: 增量小于此值计入停止计数, 单位: rad
  static constexpr float MANUAL_STOP_THRESHOLD_RAD = 1e-4f;
  // 手动停止判据: 连续停止计数超过此值认为手已松开, 退回正常模式
  static constexpr int MANUAL_STOP_COUNT_MAX = 100;

  // ---------- 电机正方向符号 ----------
  // 用于把电机安装方向统一到"C 板逆时针为正"的坐标系
  // 若某电机装反, 把对应符号改成 -1.0f
  static constexpr float MOTOR_A_DIRECTION_SIGN = 1.0f;
  static constexpr float MOTOR_B_DIRECTION_SIGN = 1.0f;

  // ---------- 外部依赖 ----------
  RM_Motor & motor_a_;
  RM_Motor & motor_b_;

  // ---------- 位置环 PID ----------
  PID pid_a_;
  PID pid_b_;

  // ---------- 内部状态 ----------
  // 上一周期 C 板 yaw, 用于求增量
  float last_board_yaw_rad_ = 0.0f;

  // 上一周期电机角度, 用于求增量
  float last_angle_a_rad_ = 0.0f;
  float last_angle_b_rad_ = 0.0f;

  // 虚拟主轴角, 中间量, 连续多圈, 单位: rad, 仅用于累加增量
  float master_angle_rad_ = 0.0f;

  // A、B 目标角, 范围 (-pi, pi]
  float target_a_rad_ = 0.0f;
  float target_b_rad_ = 0.0f;

  // 手拧留下的持久偏移, 相对主轴, 单位: rad, 松手后冻结 -> 不回正
  float offset_a_rad_ = 0.0f;
  float offset_b_rad_ = 0.0f;

  // 当前 B/C 联动比
  float ratio_b_ = 1.0f;

  // 当前模式
  Mode mode_ = Mode::NORMAL;
  // 上一周期是否处于手动模式, 用于在退出瞬间清掉积分
  bool was_manual_ = false;
  // 手动停止计数器
  int stop_count_ = 0;

  // 是否已初始化
  bool initialized_ = false;
};

}  // namespace sp

#endif  // SP__LINKAGE_CONTROLLER_HPP
