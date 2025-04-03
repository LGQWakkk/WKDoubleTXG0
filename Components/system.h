#pragma once
// 20250320 Wakkk

#include "main.h"
#include "tim.h"
#include "gpio.h"

#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>

#define SYSTEM_TICK_HANDLE htim3

#define LED1_ON()       HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET)
#define LED1_OFF()      HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET)
#define LED2_ON()       HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_RESET)
#define LED2_OFF()      HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, GPIO_PIN_SET)

#define IR_PWM_TIMER_HANDLE htim1
#define IR1_PWM_CHANNEL TIM_CHANNEL_1
#define IR2_PWM_CHANNEL TIM_CHANNEL_4

// LED指示灯控制逻辑
#define RF_STATE_TX_RUNNING 0  // 处于发射模式
#define RF_STATE_RX_WAITING 1  // 处于接收模式(没有收到信号)
#define RF_STATE_RX_RUNNING 2  // 处于接收模式(正在接收信号)
#define RF_STATE_FAILED     3  // 模块初始化失败(存在硬件问题)
#define RF_STATE_INVALID    4  // 无效状态

void set_rf_state(uint8_t index, uint8_t state);
uint8_t get_rf_state(uint8_t index);
uint8_t write_tx_buffer(uint8_t index, uint8_t *data);

void handle_led_state(void);

void system_init(void);
void system_loop(void);

void ir_marker_init(void);
void ir_marker_set_intensity(uint8_t index, int16_t intensity);
