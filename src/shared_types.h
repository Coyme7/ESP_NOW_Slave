#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>

//通信数据结构
// 模式
enum SystemMode {
    MODE_COLLAB_DRAW = 0, // 双端协同绘图 (主端控制，从端跟随)
    MODE_AUTO_DRAW   = 1, // 进阶：自动绘图 (从端画圆，主端旋钮自动转)
    MODE_BLE_MEDIA   = 2  // 蓝牙控制器 (主端模拟齿轮，从机待机)
};

// ESP-NOW 数据包结构体 
typedef struct {
    float x;
    float y;
    bool button;
    uint8_t mode; // 让主从机的模式同步
    uint32_t packet_id;
} ControlPacket;

//  双核共享内存 ***加 volatile 放入 DRAM ***
struct SharedData {
    volatile float master_x_pos;   // 主端旋钮位置
    volatile float master_y_pos;
    volatile float slave_x_pos;    // 从端云台位置
    volatile float slave_y_pos;
    volatile bool  is_boundary_hit;// 从端是否撞墙 (触发死区)
    volatile uint8_t current_mode; // 当前运行模式
};

DRAM_ATTR SharedData sysData = {0, 0, 0, 0, false, MODE_COLLAB_DRAW};
