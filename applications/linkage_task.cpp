#include "cmsis_os.h"
#include "io/dbus/dbus.hpp"
#include "motor/rm_motor/rm_motor.hpp"
#include "tools/linkage_controller/linkage_controller.hpp"
#include "tools/mahony/mahony.hpp"

extern sp::RM_Motor motor_a;
extern sp::RM_Motor motor_b;

extern sp::Mahony imu;
extern sp::DBus remote;

sp::LinkageController linkage(motor_a, motor_b);

namespace
{
// 左拨杆 -> 联动比档位
sp::LinkageGear map_remote_to_gear(sp::DBusSwitchMode switch_of_left)
{
  switch (switch_of_left) {
    case sp::DBusSwitchMode::DOWN:
      return sp::LinkageGear::DOWN;
    case sp::DBusSwitchMode::MID:
      return sp::LinkageGear::MIDDLE;
    case sp::DBusSwitchMode::UP:
    default:
      return sp::LinkageGear::UP;
  }
}

}  // namespace

extern "C" void linkage_task()
{
  // 上一拍的右拨杆档位。用它检测"档位切换"这个事件:
  // 切进联动/复位档位时才需要重建参考, 稳态下每拍只调用 update。
  sp::DBusSwitchMode last_switch = sp::DBusSwitchMode::DOWN;

  while (true) {
    const uint32_t now_ms = osKernelSysTick();
    const sp::LinkageGear gear = map_remote_to_gear(remote.sw_l);
    const sp::DBusSwitchMode current_switch = remote.sw_r;
    const bool switch_changed = (current_switch != last_switch);
    last_switch = current_switch;

    // 带单位的别名: sp::Mahony 的成员用注释标单位(工程既有风格),
    // 这里换个名字让传参处的单位一目了然
    const float board_yaw_rad = imu.yaw;      // C 板 yaw,       单位: rad
    const float board_rate_rad_s = imu.vyaw;  // C 板 yaw 角速度, 单位: rad/s

    // 电机掉线时无论如何都失能
    if (!motor_a.is_alive(now_ms) || !motor_b.is_alive(now_ms)) {
      linkage.disable();
      motor_a.cmd(0.0f);
      motor_b.cmd(0.0f);
    }
    else if (current_switch == sp::DBusSwitchMode::DOWN) {
      // 下档: 失能, 两台电机无力
      linkage.disable();
      motor_a.cmd(0.0f);
      motor_b.cmd(0.0f);
    }
    else if (current_switch == sp::DBusSwitchMode::MID) {
      // 中档: 姿态联动。刚切进来时以当前姿态建立零参考, 避免冲击
      if (switch_changed) linkage.init(board_yaw_rad);
      // board_rate_rad_s 用来判断"C 板是否主动转动", 与手拧电机区分开
      linkage.update(board_yaw_rad, board_rate_rad_s, gear, now_ms);
    }
    else {
      // 上档: 复位。刚切进来时重建对齐参考, 之后照常出力保持位置
      if (switch_changed) linkage.reset(board_yaw_rad);
      linkage.update(board_yaw_rad, board_rate_rad_s, gear, now_ms);
    }

    osDelay(1);
  }
}
