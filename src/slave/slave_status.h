#pragma once

// slave_status
// 职责：从机低频串口状态输出。
// 运行约束：只允许在 Core 0 状态任务中调用，不进入 10 kHz 控制路径。

// 打印一行从机状态：接收序号、命令 X、实际 X、目标/实际角、跟踪误差、
// UV 输出、UV 联锁、ESP-NOW 计数、包龄和故障位。
void printSlaveStatusLine();
