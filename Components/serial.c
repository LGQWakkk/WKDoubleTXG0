// 20250320 Wakkk
#include "serial.h"
#include "si24r1.h"
#include "system.h"

uint8_t uart_dma_buf[1] = {0};//DMA单字节接收缓冲区
// RX BUFFER
#define UART_RX_BUFFER_SIZE 32   // 所有数据包长度均为32字节
static uint8_t uart_rx_buffer[UART_RX_BUFFER_SIZE];
static uint8_t uart_rx_buffer_ptr = 0;
// TX BUFFER
#define UART_TX_BUFFER_SIZE 32   // 所有数据包长度均为32字节
static uint8_t uart_tx_buffer[UART_TX_BUFFER_SIZE];
static uint8_t is_uart_tx_dma_sending = 0;//UART DMA是否正在发送数据
// RX STATE
static uint8_t uart_rx_state = RX_WAITING_HEAD;

// 串口发送接收初始化
void serial_init(void)
{
    // 开始DMA接收
    HAL_UART_Receive_DMA(&huart1, uart_dma_buf, 1);
}

// UART串口DMA接收回调函数
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if(huart == &huart1)
    {
        // 调用单字节接收处理回调函数
        handle_serial_rx(uart_dma_buf[0]);
        uart_dma_buf[0] = 0;
        HAL_UART_Receive_DMA(&huart1, uart_dma_buf, 1);//重新准备接收
    }
}

// UART串口DMA发送完成回调函数
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
    if(huart == &huart1)
    {
        is_uart_tx_dma_sending = 0;//reset
    }
}

// 串口DMA发送数据
// RETURN SUCCESS: DMA空闲 开始发送
// RETURN FAILURE: DMA忙 等待发送完成 本次发送无效
uint8_t uart_tx_dma(uint8_t *buffer)
{
    if(is_uart_tx_dma_sending == 1){
        return RETURN_FAILURE;//DMA忙 等待发送完成 本次发送无效
    }else{
        for(uint8_t i=0; i<32; i++){
            uart_tx_buffer[i] = buffer[i];//Copy
        }
        HAL_UART_Transmit_DMA(&huart1, uart_tx_buffer, 32);
        is_uart_tx_dma_sending = 1;
        return RETURN_SUCCESS;//DMA空闲 开始发送
    }
}

// 清空串口接收缓冲区
void clear_uart_rx_buffer(void)
{
    for(uint8_t i=0; i<UART_RX_BUFFER_SIZE; i++){
        uart_rx_buffer[i] = 0;
    }
    uart_rx_buffer_ptr = 0;
}

// 单字节串口接收回调函数
uint8_t handle_serial_rx(uint8_t data)
{
    switch (uart_rx_state)
    {
    case RX_WAITING_HEAD://等待数据包帧头
        if(data == M1_CONFIG_FRAME_HEAD || data == M2_CONFIG_FRAME_HEAD || data == ALL_CONFIG_FRAME_HEAD || data == M1_DATA_FRAME_HEAD || data == M2_DATA_FRAME_HEAD || data == ALL_DATA_FRAME_HEAD){
            clear_uart_rx_buffer();
            uart_rx_buffer[uart_rx_buffer_ptr] = data;
            uart_rx_buffer_ptr++;
            uart_rx_state = RX_WAITING_TAIL;
        }
        break;
    case RX_WAITING_TAIL://等待数据包帧尾
        if(data == M1_CONFIG_FRAME_TAIL || data == M2_CONFIG_FRAME_TAIL || data == ALL_CONFIG_FRAME_TAIL || data == M1_DATA_FRAME_TAIL || data == M2_DATA_FRAME_TAIL || data == ALL_DATA_FRAME_TAIL){
            uart_rx_buffer[uart_rx_buffer_ptr] = data;//帧尾
            uart_rx_buffer_ptr ++;
            // 接收完毕 处理数据包
            if(uart_rx_buffer[0] == M1_CONFIG_FRAME_HEAD || uart_rx_buffer[0] == M2_CONFIG_FRAME_HEAD || uart_rx_buffer[0] == ALL_CONFIG_FRAME_HEAD){
                // Config
                if(uart_rx_buffer[0] == M1_CONFIG_FRAME_HEAD && uart_rx_buffer[31] == M1_CONFIG_FRAME_TAIL){
                    handle_rf_config(M1, uart_rx_buffer);
                }else if(uart_rx_buffer[0] == M2_CONFIG_FRAME_HEAD && uart_rx_buffer[31] == M2_CONFIG_FRAME_TAIL){
                    handle_rf_config(M2, uart_rx_buffer);
                }else if(uart_rx_buffer[0] == ALL_CONFIG_FRAME_HEAD && uart_rx_buffer[31] == ALL_CONFIG_FRAME_TAIL){
                    handle_rf_config(M1, uart_rx_buffer);
                    handle_rf_config(M2, uart_rx_buffer);
                }
            }else if(uart_rx_buffer[0] == M1_DATA_FRAME_HEAD || uart_rx_buffer[0] == M2_DATA_FRAME_HEAD || uart_rx_buffer[0] == ALL_DATA_FRAME_HEAD){
                // Data
                // 确保模块处于发射模式才可以发送
                if(uart_rx_buffer[0] == M1_DATA_FRAME_HEAD && uart_rx_buffer[31] == M1_DATA_FRAME_TAIL){
                    if(get_rf_state(M1) == RF_STATE_TX_RUNNING){
                        // M1发送数据包
                        write_tx_buffer(M1, uart_rx_buffer);
                    }
                }else if(uart_rx_buffer[0] == M2_DATA_FRAME_HEAD && uart_rx_buffer[31] == M2_DATA_FRAME_TAIL){
                    if(get_rf_state(M2) == RF_STATE_TX_RUNNING){
                        // M2发送数据包
                        write_tx_buffer(M2, uart_rx_buffer);
                    }
                }else if(uart_rx_buffer[0] == ALL_DATA_FRAME_HEAD && uart_rx_buffer[31] == ALL_DATA_FRAME_TAIL){
                    if(get_rf_state(M1) == RF_STATE_TX_RUNNING && get_rf_state(M2) == RF_STATE_TX_RUNNING){
                        // M1 M2发送数据包
                        write_tx_buffer(M1, uart_rx_buffer);
                        write_tx_buffer(M2, uart_rx_buffer);
                    }
                }
            }else{
                // 错误数据包
            }
            clear_uart_rx_buffer();
            uart_rx_state = RX_WAITING_HEAD;//复位
        }else{//还没有收到帧尾
            if(uart_rx_buffer_ptr >= (UART_RX_BUFFER_SIZE-1)){//本应该接收到帧尾 但是没有 接收错误
                clear_uart_rx_buffer();
                uart_rx_state = RX_WAITING_HEAD;//重新准备接收
            }else{//接收过程中 保存数据到缓冲区
                uart_rx_buffer[uart_rx_buffer_ptr] = data;
                uart_rx_buffer_ptr ++;
            }
        }
    default:
        break;
    }
    return RETURN_SUCCESS;
}

// 处理配置数据包
uint8_t handle_rf_config(uint8_t index, uint8_t *buf)
{
    if(index != M1 && index != M2){
        return RETURN_FAILURE;//invalid index
    }
    // First Set CE LOW
    CE_LOW

    // set mode
    if(buf[CONFIG_MODE_INDEX]==0){//TX MODE
        si24r1_set_mode(index, MODE_TX);
        set_rf_state(index, RF_STATE_TX_RUNNING);//LED
    }else if(buf[CONFIG_MODE_INDEX]==1){//RX MODE
        si24r1_set_mode(index, MODE_RX);
        set_rf_state(index, RF_STATE_RX_WAITING);//LED
    }else{
        return RETURN_FAILURE;
    }

    // set power
    if(buf[CONFIG_POWER_INDEX] > 7){
        return RETURN_FAILURE;
    }
    si24r1_set_power(index, buf[CONFIG_POWER_INDEX]);

    // set channel
    if(buf[CONFIG_CHANNEL_INDEX] > 125){//最大支持126个信道 2400MHz ~ 2525MHz
        return RETURN_FAILURE;
    }
    si24r1_set_channel(index, buf[CONFIG_CHANNEL_INDEX]);

    // set speed
    if(buf[CONFIG_SPEED_INDEX]==0){//250kbps
        si24r1_set_speed(index, SPEED_250K);
    }else if(buf[CONFIG_SPEED_INDEX]==1){//1Mbps
        si24r1_set_speed(index, SPEED_1M);
    }else if(buf[CONFIG_SPEED_INDEX]==2){//2Mbps
        si24r1_set_speed(index, SPEED_2M);
    }else{
        return RETURN_FAILURE;
    }

    // set tx address
    si24r1_set_tx_addr(index, (uint8_t*)(buf[CONFIG_TX_ADDR_INDEX]), 5);

    // set rx address
    uint8_t rx_addr_tmp[5];//接收通道0-通道5共用前四个字节地址
    for(uint8_t i=0;i<5;i++){
        rx_addr_tmp[i] = buf[CONFIG_RX0_ADDR_INDEX+i];
    }
    si24r1_set_rx_addr(index, 0, rx_addr_tmp, 5);
    for(uint8_t i=0;i<5;i++){
        rx_addr_tmp[4] = buf[CONFIG_RX1_ADDR_INDEX+i];//更新最后一个字节
        si24r1_set_rx_addr(index, 1+i, rx_addr_tmp, 5);
    }

    // Set CE HIGH
    CE_HIGH
    return RETURN_SUCCESS;
}
