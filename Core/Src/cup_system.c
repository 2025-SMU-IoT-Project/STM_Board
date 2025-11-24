#include "cup_system.h"
#include "main.h"
#include <stdio.h>
#include <stdint.h>

#define IR1_PIN   GPIO_PIN_6
#define IR1_PORT  GPIOC

#define IR2_PIN   GPIO_PIN_9
#define IR2_PORT  GPIOC

// --- 상태값 ---
static uint8_t prev_ir1 = 1;
static uint8_t waiting_ir2 = 0;     // 1이면 IR2 판정을 기다리는 중
static uint32_t ir2_wait_start = 0; // IR2 대기 시작 시간

uint8_t ir2_now = 0;

// main.c 변수/함수
extern char current_event_id[16];
extern void generate_event_id(char *id_str);
extern void LoRa_Send(const char *msg);

void CupSystem_Init(void)
{
    prev_ir1 = HAL_GPIO_ReadPin(IR1_PORT, IR1_PIN);
    waiting_ir2 = 0;
}

void CupSystem_Update(void)
{
    uint8_t cur_ir1 = HAL_GPIO_ReadPin(IR1_PORT, IR1_PIN);
    static char ir_buffer[128];

    // 1) IR1 감지 (컵 투입 이벤트)
    if (prev_ir1 == 1 && cur_ir1 == 0)
    {
        isIRTriggered = 1;
        generate_event_id(current_event_id);

        printf("[IR1] Cup detected!\r\n");

        // LoRa로 IR1 데이터 전송
        snprintf(ir_buffer, sizeof(ir_buffer),
            "{\"id\":\"%s\",\"sensorId\":\"IR1\",\"binId\":\"1\",\"beamBlocked\":true}",
            current_event_id);
        LoRa_Send(ir_buffer);

        //IR2 판정을 기다림
        waiting_ir2 = 1;
        ir2_wait_start = HAL_GetTick();
    }

    prev_ir1 = cur_ir1;

//    2) if IR1 이후 IR2 판정을 기다리는 중
    if (waiting_ir2)
    {
    	HAL_Delay(500);
        uint8_t ir2_now = HAL_GPIO_ReadPin(IR2_PORT, IR2_PIN);

        // ✦ 종이컵 감지 (LOW)
        if (ir2_now == 0)
        {
            waiting_ir2 = 0;

            printf("[IR2] PAPER detected\r\n");

            // LoRa로 IR2 데이터 전송 (종이컵)
            snprintf(ir_buffer, sizeof(ir_buffer),
                "{\"id\":\"%s\",\"sensorId\":\"IR2\",\"binId\":\"1\",\"beamBlocked\":true}",
                current_event_id);
            LoRa_Send(ir_buffer);
        }
        // ✦ 최대 대기시간(300ms) 동안 아무 변화가 없으면 = 플라스틱
        else if (HAL_GetTick() - ir2_wait_start > 300)
        {
            waiting_ir2 = 0;

            printf("[IR2] PLASTIC detected (timeout)\r\n");

            // LoRa로 IR2 데이터 전송 (플라스틱)
            snprintf(ir_buffer, sizeof(ir_buffer),
                "{\"id\":\"%s\",\"sensorId\":\"IR2\",\"binId\":\"1\",\"beamBlocked\":false}",
                current_event_id);
            LoRa_Send(ir_buffer);
        }
    }
//    */
}
