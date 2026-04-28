#include <shared_types.h>


uint8_t masterAddress[] = {0x24, 0x58, 0x7c, 0xd0, 0xb3, 0x64};
ControlPacket txPacket, rxPacket;

void onDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
    memcpy(&rxPacket, incomingData, sizeof(rxPacket));
    // 从机的一切指令听主机的
    sysData.master_x_pos = rxPacket.x;
    sysData.master_y_pos = rxPacket.y;
    sysData.current_mode = rxPacket.mode; 
}

// ==========================================
// 核心 1 (Core 1): 电压位置环执行核
// ==========================================
void IRAM_ATTR task_foc_loop(void *pvParameters) {
    float simulated_gimbal_x = 0; // 模拟云台真实物理位置
    float auto_draw_angle = 0;
    
    while(true) {
        uint8_t mode = sysData.current_mode;
        
        if (mode == MODE_COLLAB_DRAW) {
            // 协同模式：死死咬住主端传来的位置 (高刚性 PD 控制模拟)
            // float error = sysData.master_x_pos - simulated_gimbal_x;
            // motor.target_voltage = error * Kp - motor.velocity * Kd;
            simulated_gimbal_x = sysData.master_x_pos * 0.98f; // 模拟跟随滞后
            
            // 模拟死区检测：画笔撞到画框物理边界
            if (simulated_gimbal_x > 100.0f) {
                sysData.is_boundary_hit = true;
            } else {
                sysData.is_boundary_hit = false;
            }
            
        } else if (mode == MODE_AUTO_DRAW) {
            // 进阶模式：从端自己画圆，不理会主端输入
            auto_draw_angle += 0.05f;
            simulated_gimbal_x = sin(auto_draw_angle) * 50.0f; // 产生正弦轨迹
            sysData.is_boundary_hit = false;
            
        } else if (mode == MODE_BLE_MEDIA) {
            // 蓝牙模式：从端彻底卸载力矩，电机 Disable
            // motor.disable();
            simulated_gimbal_x = 0;
        }

        sysData.slave_x_pos = simulated_gimbal_x; // 更新自身位置，准备发回主机
        vTaskDelay(pdMS_TO_TICKS(1)); 
    }
}

// ==========================================
// 核心 0 (Core 0): 状态回传调度核
// ==========================================
void task_comm_loop(void *pvParameters) {
    uint32_t packet_id = 0;
    while(true) {
        // 将从端的实际执行位置发回给主机 (闭环逆向反馈)
        txPacket.x = sysData.slave_x_pos;
        txPacket.y = sysData.slave_y_pos;
        txPacket.button = sysData.is_boundary_hit; // 借用 button 位传撞墙信号
        txPacket.packet_id = packet_id++;
        
        esp_now_send(masterAddress, (uint8_t *) &txPacket, sizeof(txPacket));
        
        Serial.printf("[Slave] Mode: %d | Master Cmd: %.2f | Gimbal Pos: %.2f | Hit: %d\n", 
                      sysData.current_mode, sysData.master_x_pos, sysData.slave_x_pos, sysData.is_boundary_hit);

        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

extern "C" void app_main() {
    initArduino();
    Serial.begin(115200);
    WiFi.mode(WIFI_STA);
    esp_now_init();
    esp_now_register_recv_cb(onDataRecv);
    
    esp_now_peer_info_t peerInfo = {};
    memcpy(peerInfo.peer_addr, masterAddress, 6);
    esp_now_add_peer(&peerInfo);

    xTaskCreatePinnedToCore(task_comm_loop, "CommTask", 4096, NULL, 1, NULL, 0); // Core 0
    xTaskCreatePinnedToCore(task_foc_loop,  "FOCTask",  8192, NULL, configMAX_PRIORITIES - 1, NULL, 1); // Core 1
}