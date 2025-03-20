// 20250320 Wakkk
#include "system.h"
#include "si24r1.h"
#include "serial.h"

static uint8_t rf1_state = RF_STATE_FAILED;//SI24R1ģ��1����״̬
static uint8_t rf2_state = RF_STATE_FAILED;//SI24R1ģ��2����״̬

// RADIO BUFFER
#define SI24R1_BUFFER_SIZE 32
static uint8_t m1_tx_buffer[SI24R1_BUFFER_SIZE];//ģ��1���ͻ�����
static uint8_t m2_tx_buffer[SI24R1_BUFFER_SIZE];//ģ��2���ͻ�����
static uint8_t m1_rx_buffer[SI24R1_BUFFER_SIZE];//ģ��1���ջ�����
static uint8_t m2_rx_buffer[SI24R1_BUFFER_SIZE];//ģ��2���ջ�����

static uint8_t rf1_has_new_data_to_send = 0;   //��TXģʽ�� �Ƿ��������ݴ�����
static uint8_t rf1_is_sending_data = 0;        //��TXģʽ�� �Ƿ����ڷ�������
static uint16_t rf1_wait_time_ms = 500;        //������ʱ��û���յ�������Ϊ�ǳ�ʱ
static uint16_t rf1_timeout_count = 0;         //��ʱ������

static uint8_t rf2_has_new_data_to_send = 0;   //��TXģʽ�� �Ƿ��������ݴ�����
static uint8_t rf2_is_sending_data = 0;        //��TXģʽ�� �Ƿ����ڷ�������
static uint16_t rf2_wait_time_ms = 500;        //������ʱ��û���յ�������Ϊ�ǳ�ʱ
static uint16_t rf2_timeout_count = 0;         //��ʱ������

static uint8_t tick_update = 0;        //ϵͳ�жϱ�־λ
static uint32_t system_time = 0;       //ϵͳʱ���(ms)
static uint8_t overloop_flag = 0;      //��ֵΪ1��ʾϵͳѭ����ʱ �޷�����1000Hz����Ƶ��
static uint8_t system_init_flag = 0;   //ϵͳ�Ƿ��Ѿ���ʼ�����
static uint16_t tick_count = 0;        //ϵͳʱ�Ӽ�����(0-100)

// ��������ģ��ģʽ(������Ϊ�ڲ�flagʹ��)
void set_rf_state(uint8_t index, uint8_t state)
{
    if(index == M1){
        rf1_state = state;
    }else if(index == M2){
        rf2_state = state;
    }
}
// ��ȡ����ģ��ģʽ
uint8_t get_rf_state(uint8_t index)
{
    if(index == M1){
        return rf1_state;
    }else if(index == M2){
        return rf2_state;
    }else{
        return RF_STATE_INVALID;
    }
}

// ��SI24R1���ͻ�����д��Ҫ���͵����ݰ�
// ����SUCCESS��ʾ�ɹ�д�뻺����
// ����FAIL��ʾ���������� �������ݰ���д֮ǰ������������
uint8_t write_tx_buffer(uint8_t index, uint8_t *data)
{
    if(index == M1){
        for(uint8_t i=0; i<SI24R1_BUFFER_SIZE; i++){
            m1_tx_buffer[i] = data[i];
        }
        if(rf1_has_new_data_to_send){//��������δ��ȡ ��д
            return RETURN_FAILURE;
        }else{
            rf1_has_new_data_to_send = 1;//׼������
            return RETURN_SUCCESS;
        }
    }else if(index == M2){
        for(uint8_t i=0; i<SI24R1_BUFFER_SIZE; i++){
            m2_tx_buffer[i] = data[i];
        }
        if(rf2_has_new_data_to_send){//��������δ��ȡ ��д
            return RETURN_FAILURE;
        }else{
            rf2_has_new_data_to_send = 1;//׼������
            return RETURN_SUCCESS;
        }
    }else{//��Ч����
        return RETURN_FAILURE;
    }
}

// 1000Hz ϵͳʱ���жϻص�����
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim == (&SYSTEM_TICK_HANDLE))
    {
        if(tick_update){
            overloop_flag = 1;//loopû�����ü�������Ͼ��ٴν����ж� ������overloop
        }
        tick_update = 1;
        system_time ++;
        // tick for led blink
        tick_count ++;
        if(tick_count>=1000){
            tick_count = 0;
        }
        handle_led_state();//Handle LED state
    }
}

// ����ģ��״̬����LED״̬
void handle_led_state(void)
{
    switch (rf1_state)
    {
    case RF_STATE_TX_RUNNING://������ʾ���ڷ���ģʽ
        LED1_ON();
        break;
    case RF_STATE_RX_WAITING://1s������˸��ʾ���ڽ���ģʽ ����û���յ��ź�
        if(tick_count <= 500){
            LED1_ON();
        }else{
            LED1_OFF();
        }
        break;
    case RF_STATE_RX_RUNNING://������˸��ʾ���ڽ���ģʽ ���ڽ����ź�
        if(tick_count==0 || tick_count==130){
            LED1_ON();
        }
        if(tick_count==30 || tick_count==160){
            LED1_OFF();
        }
        break;
    case RF_STATE_FAILED://Ϩ���ʾģ���ʼ��ʧ��
        LED1_OFF();
        break;
    default:
        break;
    }
    switch (rf2_state)
    {
        case RF_STATE_TX_RUNNING://������ʾ���ڷ���ģʽ
            LED2_ON();
            break;
        case RF_STATE_RX_WAITING://1s������˸��ʾ���ڽ���ģʽ ����û���յ��ź�
            if(tick_count <= 500){
                LED2_ON();
            }else{
                LED2_OFF();
            }
            break;
        case RF_STATE_RX_RUNNING://������˸��ʾ���ڽ���ģʽ ���ڽ����ź�
            if(tick_count==0 || tick_count==130){
                LED2_ON();
            }
            if(tick_count==30 || tick_count==160){
                LED2_OFF();
            }
            break;
        case RF_STATE_FAILED://Ϩ���ʾģ���ʼ��ʧ��
            LED2_OFF();
            break;
        default:
            break;
    }
}

// �����ű��ʼ��
// PWMƵ��1000Hz
// �Ƚ�ֵ0-999
void ir_marker_init(void)
{
    HAL_TIM_Base_Start(&IR_PWM_TIMER_HANDLE);
    HAL_TIM_PWM_Start(&IR_PWM_TIMER_HANDLE, IR1_PWM_CHANNEL);
    HAL_TIM_PWM_Start(&IR_PWM_TIMER_HANDLE, IR2_PWM_CHANNEL);
    ir_marker_set_intensity(0, 0);
    ir_marker_set_intensity(1, 0);
}

// ���ú����ű�����
// index: 0 or 1
// intensity: PWMռ�ձ� 0-999
void ir_marker_set_intensity(uint8_t index, int16_t intensity)
{
    if(intensity > 999){
        intensity = 999;
    }
    if(index == 0){//MARKER1
        __HAL_TIM_SetCompare(&IR_PWM_TIMER_HANDLE, IR1_PWM_CHANNEL, intensity);
    }else if(index == 1){//MARKER2
        __HAL_TIM_SetCompare(&IR_PWM_TIMER_HANDLE, IR2_PWM_CHANNEL, intensity);
    }
}

// ��ʼ��SI24R1
void radio_init(void)
{
    // Check Connection
    if(si24r1_check(M1) == RETURN_SUCCESS){
        LED1_ON();
        si24r1_init(M1);
        si24r1_set_power(M1, 5);
        // si24r1_set_mode(M1, MODE_RX);
        si24r1_set_mode(M1, MODE_TX);
        // rf1_state = RF_STATE_RX_WAITING;
        rf1_state = RF_STATE_TX_RUNNING;
    }else{
        LED1_OFF();
    }
    HAL_Delay(100);
    if(si24r1_check(M2) == RETURN_SUCCESS){
        LED2_ON();
        si24r1_init(M2);
        si24r1_set_power(M2, 5);
        // si24r1_set_mode(M2, MODE_TX);
        si24r1_set_mode(M2, MODE_RX);
        rf2_state = RF_STATE_RX_WAITING;
        // rf2_state = RF_STATE_TX_RUNNING;
    }else{
        LED2_OFF();
    }
    HAL_Delay(100);
}

void system_init(void)
{
    LED1_OFF();
    LED2_OFF();
    ir_marker_init();
    radio_init();
    serial_init();
    
    HAL_TIM_Base_Start_IT(&SYSTEM_TICK_HANDLE);// ����ϵͳʱ��
    system_init_flag = 1;

    // ir_marker_set_intensity(0, 200);
    // ir_marker_set_intensity(1, 200);
}

void system_loop(void)//1000Hz
{
    if(tick_update && system_init_flag){
////////////////////////////////////////////////////////////////////////////////////////////
        // M1
        if(rf1_state == RF_STATE_TX_RUNNING){//���ڷ���ģʽ
            if(rf1_has_new_data_to_send && !rf1_is_sending_data){//�������ݴ������ҿ���
                rf1_has_new_data_to_send = 0;
                rf1_is_sending_data = 1;
                si24r1_fast_tx(M1, m1_tx_buffer);//��������
            }else if(rf1_is_sending_data){//���ڵȴ��������
                if(si24r1_fast_check(M1) == RETURN_SUCCESS){ //������� ��⵽�ж�
                    rf1_is_sending_data = 0;
                }
            }
        }else if(rf1_state == RF_STATE_RX_WAITING || rf1_state == RF_STATE_RX_RUNNING){//����ģʽ
            if(si24r1_fast_check(M1) == RETURN_SUCCESS){//���յ�����
                uint8_t len = si24r1_fast_rx(M1, m1_rx_buffer);
                if(len == SI24R1_BUFFER_SIZE){//32Bytes OK
                    rf1_state = RF_STATE_RX_RUNNING;
                    //������յ�������
                    uart_tx_dma(m1_rx_buffer);//ת����UART
                }
                rf1_timeout_count = 0;
            }else{//��ʱû���յ�����
                rf1_timeout_count ++;
                if(rf1_timeout_count >= rf1_wait_time_ms){//��ʱ
                    si24r1_fast_clear(M1);// ����ж�
                    rf1_state = RF_STATE_RX_WAITING;
                    rf1_timeout_count = 0;
                }
            }
        }

        //M2
        if(rf2_state == RF_STATE_TX_RUNNING){//���ڷ���ģʽ
            if(rf2_has_new_data_to_send && !rf2_is_sending_data){//�������ݴ������ҿ���
                rf2_has_new_data_to_send = 0;
                rf2_is_sending_data = 1;
                si24r1_fast_tx(M2, m2_tx_buffer);//��������
            }else if(rf2_is_sending_data){//���ڵȴ��������
                if(si24r1_fast_check(M2) == RETURN_SUCCESS){ //������� ��⵽�ж�
                    rf2_is_sending_data = 0;
                }
            }
        }else if(rf2_state == RF_STATE_RX_WAITING || rf2_state == RF_STATE_RX_RUNNING){//����ģʽ
            if(si24r1_fast_check(M2) == RETURN_SUCCESS){//���յ�����
                uint8_t len = si24r1_fast_rx(M2, m2_rx_buffer);
                if(len == SI24R1_BUFFER_SIZE){//32Bytes OK
                    rf2_state = RF_STATE_RX_RUNNING;
                    //������յ�������
                    uart_tx_dma(m2_rx_buffer);//ת����UART
                }
                rf2_timeout_count = 0;
            }else{//��ʱû���յ�����
                rf2_timeout_count ++;
                if(rf2_timeout_count >= rf2_wait_time_ms){//��ʱ
                    si24r1_fast_clear(M2);// ����ж�
                    rf2_state = RF_STATE_RX_WAITING;
                    rf2_timeout_count = 0;
                }
            }
        }
////////////////////////////////////////////////////////////////////////////////////////////
    tick_update = 0;//�����ڴ���ĩβ�Լ��overloop
    }
}
