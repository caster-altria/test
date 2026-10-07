#include "cmsis_os.h"
#include "io/can/can.hpp"
#include "motor/rm_motor/rm_motor.hpp"

sp::CAN can1(&hcan1);
sp::RM_Motor motor_a(1, sp::RM_Motors::GM6020);  // 一个电机ID为1, 电流控制模式的6020
sp::RM_Motor motor_b(2, sp::RM_Motors::GM6020);  // 一个电机ID为2, 电流控制模式的6020

extern "C" void can_task()
{
  can1.config();
  can1.start();

  while (true) {
    // motor_a.cmd(0.0f);
    // motor_b.cmd(0.0f);

    motor_a.write(can1.tx_data);
    motor_b.write(can1.tx_data);

    can1.send(motor_b.tx_id);

    osDelay(1);
  }
}

extern "C" void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef * hcan)
{
  auto stamp_ms = osKernelSysTick();

  while (HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO0) > 0) {
    if (hcan == &hcan1) {
      can1.recv();

      if (can1.rx_id == motor_a.rx_id) motor_a.read(can1.rx_data, stamp_ms);
      if (can1.rx_id == motor_b.rx_id) motor_b.read(can1.rx_data, stamp_ms);
    }
  }
}