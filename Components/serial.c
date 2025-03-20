// 20250320 Wakkk
#include "serial.h"
#include "si24r1.h"
#include "system.h"

uint8_t uart_dma_buf[1] = {0};//DMA���ֽڽ��ջ�����
// RX BUFFER
#define UART_RX_BUFFER_SIZE 32   // �������ݰ����Ⱦ�Ϊ32�ֽ�
static uint8_t uart_rx_buffer[UART_RX_BUFFER_SIZE];
static uint8_t uart_rx_buffer_ptr = 0;
// TX BUFFER
#define UART_TX_BUFFER_SIZE 32   // �������ݰ����Ⱦ�Ϊ32�ֽ�
static uint8_t uart_tx_buffer[UART_TX_BUFFER_SIZE];
static uint8_t is_uart_tx_dma_sending = 0;//UART DMA�Ƿ����ڷ�������
// RX STATE
static uint8_t uart_rx_state = RX_WAITING_HEAD;

// ���ڷ��ͽ��ճ�ʼ��
void serial_init(void)
{
    // ��ʼDMA����
    HAL_UART_Receive_DMA(&huart1, uart_dma_buf, 1);
}

// UART����DMA���ջص�����
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if(huart == &huart1)
    {
        // ���õ��ֽڽ��մ���ص�����
        handle_serial_rx(uart_dma_buf[0]);
        uart_dma_buf[0] = 0;
        HAL_UART_Receive_DMA(&huart1, uart_dma_buf, 1);//����׼������
    }
}

// UART����DMA������ɻص�����
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
    if(huart == &huart1)
    {
        is_uart_tx_dma_sending = 0;//reset
    }
}

// ����DMA��������
// RETURN SUCCESS: DMA���� ��ʼ����
// RETURN FAILURE: DMAæ �ȴ�������� ���η�����Ч
uint8_t uart_tx_dma(uint8_t *buffer)
{
    if(is_uart_tx_dma_sending == 1){
        return RETURN_FAILURE;//DMAæ �ȴ�������� ���η�����Ч
    }else{
        for(uint8_t i=0; i<32; i++){
            uart_tx_buffer[i] = buffer[i];//Copy
        }
        HAL_UART_Transmit_DMA(&huart1, uart_tx_buffer, 32);
        is_uart_tx_dma_sending = 1;
        return RETURN_SUCCESS;//DMA���� ��ʼ����
    }
}

// ��մ��ڽ��ջ�����
void clear_uart_rx_buffer(void)
{
    for(uint8_t i=0; i<UART_RX_BUFFER_SIZE; i++){
        uart_rx_buffer[i] = 0;
    }
    uart_rx_buffer_ptr = 0;
}

// ���ֽڴ��ڽ��ջص�����
uint8_t handle_serial_rx(uint8_t data)
{
    switch (uart_rx_state)
    {
    case RX_WAITING_HEAD://�ȴ����ݰ�֡ͷ
        if(data == M1_CONFIG_FRAME_HEAD || data == M2_CONFIG_FRAME_HEAD || data == ALL_CONFIG_FRAME_HEAD || data == M1_DATA_FRAME_HEAD || data == M2_DATA_FRAME_HEAD || data == ALL_DATA_FRAME_HEAD){
            clear_uart_rx_buffer();
            uart_rx_buffer[uart_rx_buffer_ptr] = data;
            uart_rx_buffer_ptr++;
            uart_rx_state = RX_WAITING_TAIL;
        }
        break;
    case RX_WAITING_TAIL://�ȴ����ݰ�֡β
        if(data == M1_CONFIG_FRAME_TAIL || data == M2_CONFIG_FRAME_TAIL || data == ALL_CONFIG_FRAME_TAIL || data == M1_DATA_FRAME_TAIL || data == M2_DATA_FRAME_TAIL || data == ALL_DATA_FRAME_TAIL){
            uart_rx_buffer[uart_rx_buffer_ptr] = data;//֡β
            uart_rx_buffer_ptr ++;
            // ������� �������ݰ�
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
                // ȷ��ģ�鴦�ڷ���ģʽ�ſ��Է���
                if(uart_rx_buffer[0] == M1_DATA_FRAME_HEAD && uart_rx_buffer[31] == M1_DATA_FRAME_TAIL){
                    if(get_rf_state(M1) == RF_STATE_TX_RUNNING){
                        // M1�������ݰ�
                        write_tx_buffer(M1, uart_rx_buffer);
                    }
                }else if(uart_rx_buffer[0] == M2_DATA_FRAME_HEAD && uart_rx_buffer[31] == M2_DATA_FRAME_TAIL){
                    if(get_rf_state(M2) == RF_STATE_TX_RUNNING){
                        // M2�������ݰ�
                        write_tx_buffer(M2, uart_rx_buffer);
                    }
                }else if(uart_rx_buffer[0] == ALL_DATA_FRAME_HEAD && uart_rx_buffer[31] == ALL_DATA_FRAME_TAIL){
                    if(get_rf_state(M1) == RF_STATE_TX_RUNNING && get_rf_state(M2) == RF_STATE_TX_RUNNING){
                        // M1 M2�������ݰ�
                        write_tx_buffer(M1, uart_rx_buffer);
                        write_tx_buffer(M2, uart_rx_buffer);
                    }
                }
            }else{
                // �������ݰ�
            }
            clear_uart_rx_buffer();
            uart_rx_state = RX_WAITING_HEAD;//��λ
        }else{//��û���յ�֡β
            if(uart_rx_buffer_ptr >= (UART_RX_BUFFER_SIZE-1)){//��Ӧ�ý��յ�֡β ����û�� ���մ���
                clear_uart_rx_buffer();
                uart_rx_state = RX_WAITING_HEAD;//����׼������
            }else{//���չ����� �������ݵ�������
                uart_rx_buffer[uart_rx_buffer_ptr] = data;
                uart_rx_buffer_ptr ++;
            }
        }
    default:
        break;
    }
    return RETURN_SUCCESS;
}

// �����������ݰ�
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
    if(buf[CONFIG_CHANNEL_INDEX] > 125){//���֧��126���ŵ� 2400MHz ~ 2525MHz
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
    uint8_t rx_addr_tmp[5];//����ͨ��0-ͨ��5����ǰ�ĸ��ֽڵ�ַ
    for(uint8_t i=0;i<5;i++){
        rx_addr_tmp[i] = buf[CONFIG_RX0_ADDR_INDEX+i];
    }
    si24r1_set_rx_addr(index, 0, rx_addr_tmp, 5);
    for(uint8_t i=0;i<5;i++){
        rx_addr_tmp[4] = buf[CONFIG_RX1_ADDR_INDEX+i];//�������һ���ֽ�
        si24r1_set_rx_addr(index, 1+i, rx_addr_tmp, 5);
    }

    // Set CE HIGH
    CE_HIGH
    return RETURN_SUCCESS;
}
