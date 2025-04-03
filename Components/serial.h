#pragma once
// 20250320 Wakkk

#include <stdint.h>
#include <stdbool.h>

// 接收状态机状态
#define RX_WAITING_HEAD 0
#define RX_WAITING_TAIL 1
#define RX_DONE 2

#define M1_CONFIG_FRAME_HEAD 0xC1
#define M1_CONFIG_FRAME_TAIL 0x41
#define M2_CONFIG_FRAME_HEAD 0xC2
#define M2_CONFIG_FRAME_TAIL 0x42
#define ALL_CONFIG_FRAME_HEAD 0xC3
#define ALL_CONFIG_FRAME_TAIL 0x43

#define M1_DATA_FRAME_HEAD 0xD1
#define M1_DATA_FRAME_TAIL 0x51
#define M2_DATA_FRAME_HEAD 0xD2
#define M2_DATA_FRAME_TAIL 0x52
#define ALL_DATA_FRAME_HEAD 0xD3
#define ALL_DATA_FRAME_TAIL 0x53

// 配置数据包协议
#define CONFIG_MODE_INDEX       1
#define CONFIG_POWER_INDEX      2
#define CONFIG_CHANNEL_INDEX    3
#define CONFIG_SPEED_INDEX      4    
#define CONFIG_TX_ADDR_INDEX    5
#define CONFIG_RX0_ADDR_INDEX   10
#define CONFIG_RX1_ADDR_INDEX   15
#define CONFIG_RX2_ADDR_INDEX   16
#define CONFIG_RX3_ADDR_INDEX   17
#define CONFIG_RX4_ADDR_INDEX   18
#define CONFIG_RX5_ADDR_INDEX   19

// Functions
void serial_init(void);
void clear_uart_rx_buffer(void);
uint8_t handle_serial_rx(uint8_t data);
uint8_t handle_rf_config(uint8_t index, uint8_t *buf);
uint8_t uart_tx_dma(uint8_t *buffer);
