buzzer_task 烧录之后蜂鸣器响三声 完成要求4.4.1.1.1
led_task 正常运行时三色更替的呼吸灯，以检查程序是否阻塞 完成要求4.4.1.1.2
imu_task 创建imu实例，完成imu的启动，其中imu方向矩阵根据实际情况调整
plotter_task 创建plotter实例，完成数据打印 完成要求4.4.1.1.3
uart_task 创建遥控器，使c板与dt7遥控器能够正常通讯。 完成要求4.4.1.1.4
can_task 创建电机和can实例，完成数据传输功能以及接收 CAN 反馈帧并解析回电机对象。cmd由linkage_task完成。
linkage_task 利用ai在sp_middle中间件中添加linkage中间件，
             右拨杆三个挡位由类sp::LinkageGear表示，表示为模式DBusSwitchMode
             判断挡位是否有变化之后进行以下行动，
             上档调用linkage.reset进行目标复位，
             下档通过 linkage.disable使目标失能，
             切换为中档以及在中档切换联动比时调用linkage.init，检测并记录初始角度
             在中档和上档调用linkage.update使用pid向电机输出力矩，其中当c版不转动时转动，由电机偏差及角度增量是否超过阈值以及两电机角度增量大小判断是否为手拧模式以及主动运动方，将变量按比例同步到中间量master_angle，同时改变角度初始偏差量。
             最后设置目标角度target，并且通过pid计算输出力矩。


任务完成说明：在验收时，右拨杆上档功能有明显偏差，且此时拨动左拨杆，会产生角度突变。录制视频时，可能提到的左拨杆关联联动比有误，实际功能符合要求。