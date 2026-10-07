#include "linkage_controller.hpp"

#include <cmath>

#include "tools/math_tools/math_tools.hpp"

namespace sp
{
float LinkageController::gear_to_ratio(LinkageGear gear)
{
  switch (gear) {
    case LinkageGear::DOWN:
      return 0.5f;
    case LinkageGear::MIDDLE:
      return -1.0f;
    case LinkageGear::UP:
      return 3.0f;
  }
  return 1.0f;  // 不会到达
}

int LinkageController::find_manual_axis(
  float delta_a_rad, float delta_b_rad, float error_a_rad, float error_b_rad) const
{
  // 判据: 该电机增量足够大 且 该电机偏差足够大 且 它是主导运动的一方
  const bool a_is_moving = std::abs(delta_a_rad) > MANUAL_MOVE_THRESHOLD_RAD;
  const bool b_is_moving = std::abs(delta_b_rad) > MANUAL_MOVE_THRESHOLD_RAD;
  const bool a_is_off_target = std::abs(error_a_rad) > MANUAL_ERROR_THRESHOLD_RAD;
  const bool b_is_off_target = std::abs(error_b_rad) > MANUAL_ERROR_THRESHOLD_RAD;

  const bool a_is_dominant = std::abs(delta_a_rad) >= std::abs(delta_b_rad);
  const bool b_is_dominant = std::abs(delta_b_rad) > std::abs(delta_a_rad);

  if (a_is_moving && a_is_off_target && a_is_dominant) return 1;
  if (b_is_moving && b_is_off_target && b_is_dominant) return -1;
  return 0;
}

void LinkageController::update_targets()
{
  this->target_a_rad_ = limit_angle(this->master_angle_rad_ * RATIO_A + this->offset_a_rad_);
  this->target_b_rad_ = limit_angle(this->master_angle_rad_ * this->ratio_b_ + this->offset_b_rad_);
}

LinkageController::LinkageController(RM_Motor & motor_a, RM_Motor & motor_b)
: motor_a_(motor_a),
  motor_b_(motor_b),
  pid_a_(
    CONTROL_PERIOD_S, LOOP_KP, LOOP_KI, LOOP_KD, LOOP_MAX_TORQUE_NM, LOOP_MAX_INTEGRAL_NM,
    LOOP_D_FILTER_ALPHA,
    true,   // angular = true, 由 sp::PID 内部用 limit_angle 处理 ±pi 跳变
    false   // dynamic = false
    ),
  pid_b_(
    CONTROL_PERIOD_S, LOOP_KP, LOOP_KI, LOOP_KD, LOOP_MAX_TORQUE_NM, LOOP_MAX_INTEGRAL_NM,
    LOOP_D_FILTER_ALPHA,
    true,   // angular = true
    false   // dynamic = false
    )
{
}

void LinkageController::capture_reference(float board_yaw_rad, bool align_to_board)
{
  const float angle_a_rad = this->read_motor_a();
  const float angle_b_rad = this->read_motor_b();

  // 主轴清零, 目标角只由偏移决定
  this->last_board_yaw_rad_ = board_yaw_rad;
  this->master_angle_rad_ = 0.0f;

  if (align_to_board) {
    // 复位: 让两个箭头与 C 板箭头同向。
    // 同向意味着目标角就是 C 板 yaw 按联动比折算的结果, 因此偏移取这两个值;
    // 主轴为 0, 所以偏移 == 目标角。
    this->offset_a_rad_ = board_yaw_rad * RATIO_A;
    this->offset_b_rad_ = board_yaw_rad * this->ratio_b_;
  }
  else {
    // 初始化: 以当前位置为参考, 偏移取当前电机角,
    // 于是目标角等于实际角, PID 误差为零, 不会产生冲击
    this->offset_a_rad_ = angle_a_rad;
    this->offset_b_rad_ = angle_b_rad;
  }

  this->last_angle_a_rad_ = angle_a_rad;
  this->last_angle_b_rad_ = angle_b_rad;

  this->mode_ = Mode::NORMAL;
  this->was_manual_ = false;
  this->stop_count_ = 0;

  this->prepare_pid(angle_a_rad, angle_b_rad);
  this->initialized_ = true;
}

void LinkageController::init(float board_yaw_rad)
{
  // 不需要档位: init 只建立零参考, 联动比由 update 每拍从拨杆刷新
  this->capture_reference(board_yaw_rad, false);
}

void LinkageController::reset(float board_yaw_rad)
{
  // 与 init 的唯一区别: 偏移取"C 板 yaw 按联动比折算"的值而不是当前电机角,
  // 于是两个目标角变成箭头同向的位置, 电机会主动转过去完成对齐。
  this->capture_reference(board_yaw_rad, true);
}

void LinkageController::prepare_pid(float angle_a_rad, float angle_b_rad)
{
  // 清掉积分与微分缓存, 再用两次"误差为零"的计算把内部反馈状态填好。
  //
  // 预热是必须的: sp::PID 的 D 项是"上一拍反馈"的差分, 内部 fdb 初始为 0,
  // 第一次 calc 会算出 kd * (0 - angle) / dt, 会把力矩直接踢到限幅。
  // 需要两次的原因: 第一次把 fdb 填上, 但此时 dbuf[0] 仍是那个错误的差分,
  // 第二次才算出一个正常的(接近 0 的)微分项。
  // 因为误差为零, P 项与 I 项本来就是 0。
  this->update_targets();
  this->pid_a_.clear();
  this->pid_b_.clear();
  this->pid_a_.calc(this->target_a_rad_, angle_a_rad);
  this->pid_b_.calc(this->target_b_rad_, angle_b_rad);
  this->pid_a_.calc(this->target_a_rad_, angle_a_rad);
  this->pid_b_.calc(this->target_b_rad_, angle_b_rad);
}

void LinkageController::disable()
{
  this->motor_a_.cmd(0.0f);
  this->motor_b_.cmd(0.0f);

  this->pid_a_.clear();
  this->pid_b_.clear();

  this->mode_ = Mode::NORMAL;
  this->was_manual_ = false;
  this->stop_count_ = 0;
  this->initialized_ = false;
}

void LinkageController::update(
  float board_yaw_rad, float board_rate_rad_s, LinkageGear gear, uint32_t now_ms)
{
  if (!this->initialized_) return;

  // ---------- 1. 求 C 板 yaw 增量 ----------
  // 用 limit_angle 把差值折算到 (-pi, pi], 避免跨 ±pi 时跳变
  const float board_delta_rad = limit_angle(board_yaw_rad - this->last_board_yaw_rad_);
  this->last_board_yaw_rad_ = board_yaw_rad;

  // ---------- 2. 读取电机角度与实际增量 ----------
  const float angle_a_rad = this->read_motor_a();
  const float angle_b_rad = this->read_motor_b();

  const float delta_a_rad = limit_angle(angle_a_rad - this->last_angle_a_rad_);
  const float delta_b_rad = limit_angle(angle_b_rad - this->last_angle_b_rad_);

  // ---------- 3. 换挡 ----------
  // 只更新联动比, 不动主轴与偏移, 因此换挡瞬间目标角不会跳变
  this->ratio_b_ = gear_to_ratio(gear);

  // ---------- 4. 当前误差, 用于手动判据 ----------
  const float error_a_rad = limit_angle(angle_a_rad - this->target_a_rad_);
  const float error_b_rad = limit_angle(angle_b_rad - this->target_b_rad_);

  // ---------- 5. 判断驱动源并推进主轴 ----------
  // 用陀螺仪角速度而不是 yaw 每拍增量判断"C 板是否主动转动":
  // 1kHz 下板子转 1 rad/s 每拍只走 1e-3 rad, 与手拧电机的每拍增量同量级,
  // 用增量判别无法区分两者。
  const bool is_board_driving = std::abs(board_rate_rad_s) > BOARD_STILL_RATE_RAD_S;

  if (is_board_driving) {
    // C 板主动转动: 主轴随 C 板 1:1 推进, 两个偏移保持不变
    this->mode_ = Mode::NORMAL;
    this->stop_count_ = 0;
    this->master_angle_rad_ += RATIO_A * board_delta_rad;
  }
  else {
    // C 板静止: 才可能有人手转电机
    if (this->mode_ == Mode::NORMAL) {
      const int manual_axis = this->find_manual_axis(
        delta_a_rad, delta_b_rad, error_a_rad, error_b_rad);
      if (manual_axis != 0) {
        this->mode_ = (manual_axis > 0) ? Mode::MANUAL_A : Mode::MANUAL_B;
        this->stop_count_ = 0;
      }
    }

    // 手拧时把主轴向"手搬动的方向"推进, 并让被拧轴的偏移跟住实际角,
    // 于是另一轴由主轴投影出来、自动按联动比跟随。
    // 手停下后偏移不再更新, 位移被保留 -> 松手不回正。
    if (this->mode_ == Mode::MANUAL_A) {
      this->master_angle_rad_ += RATIO_A * delta_a_rad;
      this->offset_a_rad_ = angle_a_rad - this->master_angle_rad_ * RATIO_A;
      this->update_stop_count(delta_a_rad);
    }
    else if (this->mode_ == Mode::MANUAL_B) {
      this->master_angle_rad_ += delta_b_rad / this->ratio_b_;
      this->offset_b_rad_ = angle_b_rad - this->master_angle_rad_ * this->ratio_b_;
      this->update_stop_count(delta_b_rad);
    }
  }

  // ---------- 6. 由主轴按联动比投影出目标角 ----------
  // 位置式投影(而不是累加实测增量): 只要 PID 把误差收敛到 0,
  // 就恒有 angle_b / angle_a = ratio_b_
  this->update_targets();

  // ---------- 7. 位置环 PID ----------
  // 手动期间冻结积分(仅在积分启用时有意义): 误差被手带着变化,
  // 放任积分累积会让松手瞬间弹一下。
  // sp::PID 内部是 iout = limit_max(iout + ki*trapezoid, max_iout),
  // 把 iout 临时置 0 即可让本次增量为 0, 算完再恢复原值(冻结而不是清零)。
  const bool manual_active = (this->mode_ != Mode::NORMAL);

  float integral_torque_a = 0.0f;
  float integral_torque_b = 0.0f;
  if (IS_INTEGRAL_ENABLED && manual_active) {
    integral_torque_a = this->pid_a_.data.iout;
    integral_torque_b = this->pid_b_.data.iout;
    this->pid_a_.data.iout = 0.0f;
    this->pid_b_.data.iout = 0.0f;
  }

  this->pid_a_.calc(this->target_a_rad_, angle_a_rad, LOOP_INTEGRAL_PAUSE_NM);
  this->pid_b_.calc(this->target_b_rad_, angle_b_rad, LOOP_INTEGRAL_PAUSE_NM);

  if (IS_INTEGRAL_ENABLED && manual_active) {
    this->pid_a_.data.iout = integral_torque_a;
    this->pid_b_.data.iout = integral_torque_b;
  }
  else if (this->was_manual_) {
    // 刚退出手动模式: 清掉 PID 状态并重新预热, 避免退出瞬间微分项冲击
    this->prepare_pid(angle_a_rad, angle_b_rad);
  }
  this->was_manual_ = manual_active;

  // ---------- 8. 输出力矩 ----------
  // PID 输出是统一坐标系下的力矩, 乘回方向符号才对应实际电机;
  // 电机掉线则输出 0
  const float torque_a = this->motor_a_.is_alive(now_ms)
                           ? MOTOR_A_DIRECTION_SIGN * this->pid_a_.out
                           : 0.0f;
  const float torque_b = this->motor_b_.is_alive(now_ms)
                           ? MOTOR_B_DIRECTION_SIGN * this->pid_b_.out
                           : 0.0f;
  this->motor_a_.cmd(torque_a);
  this->motor_b_.cmd(torque_b);

  // ---------- 9. 保存本拍电机角度 ----------
  this->last_angle_a_rad_ = angle_a_rad;
  this->last_angle_b_rad_ = angle_b_rad;
}

void LinkageController::update_stop_count(float delta_rad)
{
  const bool is_stopped = std::abs(delta_rad) < MANUAL_STOP_THRESHOLD_RAD;
  this->stop_count_ = is_stopped ? this->stop_count_ + 1 : 0;
  if (this->stop_count_ > MANUAL_STOP_COUNT_MAX) {
    this->mode_ = Mode::NORMAL;
    this->stop_count_ = 0;
  }
}

}  // namespace sp
