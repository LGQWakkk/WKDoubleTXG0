// 20250320 Wakkk
#include "system.h"
#include "si24r1.h"
#include "serial.h"

static uint8_t rf1_state = RF_STATE_FAILED;//SI24R1模块1工作状态
static uint8_t rf2_state = RF_STATE_FAILED;//SI24R1模块2工作状态

// RADIO BUFFER
#define SI24R1_BUFFER_SIZE 32
static uint8_t m1_tx_buffer[SI24R1_BUFFER_SIZE];//模块1发送缓冲区
static uint8_t m2_tx_buffer[SI24R1_BUFFER_SIZE];//模块2发送缓冲区
static uint8_t m1_rx_buffer[SI24R1_BUFFER_SIZE];//模块1接收缓冲区
static uint8_t m2_rx_buffer[SI24R1_BUFFER_SIZE];//模块2接收缓冲区

static uint8_t rf1_has_new_data_to_send = 0;   //在TX模式下 是否有新数据待发送
static uint8_t rf1_is_sending_data = 0;        //在TX模式下 是否正在发送数据
static uint16_t rf1_wait_time_ms = 500;        //超出此时间没有收到数据认为是超时
static uint16_t rf1_timeout_count = 0;         //超时计数器

static uint8_t rf2_has_new_data_to_send = 0;   //在TX模式下 是否有新数据待发送
static uint8_t rf2_is_sending_data = 0;        //在TX模式下 是否正在发送数据
static uint16_t rf2_wait_time_ms = 500;        //超出此时间没有收到数据认为是超时
static uint16_t rf2_timeout_count = 0;         //超时计数器

static uint8_t tick_update = 0;        //系统中断标志位
static uint32_t system_time = 0;       //系统时间戳(ms)
static uint8_t overloop_flag = 0;      //此值为1表示系统循环超时 无法满足1000Hz处理频率
static uint8_t system_init_flag = 0;   //系统是否已经初始化完毕
static uint16_t tick_count = 0;        //系统时钟计数器(0-100)

// 设置无线模块模式(仅仅作为内部flag使用)
void set_rf_state(uint8_t index, uint8_t state)
{
    if(index == M1){
        rf1_state = state;
    }else if(index == M2){
        rf2_state = state;
    }
}
// 获取无线模块模式
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

// 向SI24R1发送缓冲区写入要发送的数据包
// 返回SUCCESS表示成功写入缓冲区
// 返回FAIL表示缓冲区已满 本次数据包覆写之前缓冲区中数据
uint8_t write_tx_buffer(uint8_t index, uint8_t *data)
{
    if(index == M1){
        for(uint8_t i=0; i<SI24R1_BUFFER_SIZE; i++){
            m1_tx_buffer[i] = data[i];
        }
        if(rf1_has_new_data_to_send){//还有数据未读取 覆写
            return RETURN_FAILURE;
        }else{
            rf1_has_new_data_to_send = 1;//准备发送
            return RETURN_SUCCESS;
        }
    }else if(index == M2){
        for(uint8_t i=0; i<SI24R1_BUFFER_SIZE; i++){
            m2_tx_buffer[i] = data[i];
        }
        if(rf2_has_new_data_to_send){//还有数据未读取 覆写
            return RETURN_FAILURE;
        }else{
            rf2_has_new_data_to_send = 1;//准备发送
            return RETURN_SUCCESS;
        }
    }else{//无效索引
        return RETURN_FAILURE;
    }
}

// 1000Hz 系统时基中断回调函数
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim == (&SYSTEM_TICK_HANDLE))
    {
        if(tick_update){
            overloop_flag = 1;//loop没有来得及处理完毕就再次进入中断 发生了overloop
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

// 根据模块状态更新LED状态
void handle_led_state(void)
{
    switch (rf1_state)
    {
    case RF_STATE_TX_RUNNING://常亮表示处于发射模式
        LED1_ON();
        break;
    case RF_STATE_RX_WAITING://1s周期闪烁表示处于接收模式 但是没有收到信号
        if(tick_count <= 500){
            LED1_ON();
        }else{
            LED1_OFF();
        }
        break;
    case RF_STATE_RX_RUNNING://短暂闪烁表示处于接收模式 正在接收信号
        if(tick_count==0 || tick_count==130){
            LED1_ON();
        }
        if(tick_count==30 || tick_count==160){
            LED1_OFF();
        }
        break;
    case RF_STATE_FAILED://熄灭表示模块初始化失败
        LED1_OFF();
        break;
    default:
        break;
    }
    switch (rf2_state)
    {
        case RF_STATE_TX_RUNNING://常亮表示处于发射模式
            LED2_ON();
            break;
        case RF_STATE_RX_WAITING://1s周期闪烁表示处于接收模式 但是没有收到信号
            if(tick_count <= 500){
                LED2_ON();
            }else{
                LED2_OFF();
            }
            break;
        case RF_STATE_RX_RUNNING://短暂闪烁表示处于接收模式 正在接收信号
            if(tick_count==0 || tick_count==130){
                LED2_ON();
            }
            if(tick_count==30 || tick_count==160){
                LED2_OFF();
            }
            break;
        case RF_STATE_FAILED://熄灭表示模块初始化失败
            LED2_OFF();
            break;
        default:
            break;
    }
}

// 红外信标初始化
// PWM频率1000Hz
// 比较值0-999
void ir_marker_init(void)
{
    HAL_TIM_Base_Start(&IR_PWM_TIMER_HANDLE);
    HAL_TIM_PWM_Start(&IR_PWM_TIMER_HANDLE, IR1_PWM_CHANNEL);
    HAL_TIM_PWM_Start(&IR_PWM_TIMER_HANDLE, IR2_PWM_CHANNEL);
    ir_marker_set_intensity(0, 0);
    ir_marker_set_intensity(1, 0);
}

// 设置红外信标亮度
// index: 0 or 1
// intensity: PWM占空比 0-999
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

// 初始化SI24R1
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
    
    HAL_TIM_Base_Start_IT(&SYSTEM_TICK_HANDLE);// 启动系统时基
    system_init_flag = 1;

    // ir_marker_set_intensity(0, 200);
    // ir_marker_set_intensity(1, 200);
}

void system_loop(void)//1000Hz
{
    if(tick_update && system_init_flag){
////////////////////////////////////////////////////////////////////////////////////////////
        // M1
        if(rf1_state == RF_STATE_TX_RUNNING){//处于发射模式
            if(rf1_has_new_data_to_send && !rf1_is_sending_data){//有新数据待发送且空闲
                rf1_has_new_data_to_send = 0;
                rf1_is_sending_data = 1;
                si24r1_fast_tx(M1, m1_tx_buffer);//开启发送
            }else if(rf1_is_sending_data){//处于等待发送完毕
                if(si24r1_fast_check(M1) == RETURN_SUCCESS){ //发送完毕 检测到中断
                    rf1_is_sending_data = 0;
                }
            }
        }else if(rf1_state == RF_STATE_RX_WAITING || rf1_state == RF_STATE_RX_RUNNING){//接收模式
            if(si24r1_fast_check(M1) == RETURN_SUCCESS){//接收到数据
                uint8_t len = si24r1_fast_rx(M1, m1_rx_buffer);
                if(len == SI24R1_BUFFER_SIZE){//32Bytes OK
                    rf1_state = RF_STATE_RX_RUNNING;
                    //处理接收到的数据
                    uart_tx_dma(m1_rx_buffer);//转发到UART
                }
                rf1_timeout_count = 0;
            }else{//此时没有收到数据
                rf1_timeout_count ++;
                if(rf1_timeout_count >= rf1_wait_time_ms){//超时
                    si24r1_fast_clear(M1);// 清空中断
                    rf1_state = RF_STATE_RX_WAITING;
                    rf1_timeout_count = 0;
                }
            }
        }

        //M2
        if(rf2_state == RF_STATE_TX_RUNNING){//处于发射模式
            if(rf2_has_new_data_to_send && !rf2_is_sending_data){//有新数据待发送且空闲
                rf2_has_new_data_to_send = 0;
                rf2_is_sending_data = 1;
                si24r1_fast_tx(M2, m2_tx_buffer);//开启发送
            }else if(rf2_is_sending_data){//处于等待发送完毕
                if(si24r1_fast_check(M2) == RETURN_SUCCESS){ //发送完毕 检测到中断
                    rf2_is_sending_data = 0;
                }
            }
        }else if(rf2_state == RF_STATE_RX_WAITING || rf2_state == RF_STATE_RX_RUNNING){//接收模式
            if(si24r1_fast_check(M2) == RETURN_SUCCESS){//接收到数据
                uint8_t len = si24r1_fast_rx(M2, m2_rx_buffer);
                if(len == SI24R1_BUFFER_SIZE){//32Bytes OK
                    rf2_state = RF_STATE_RX_RUNNING;
                    //处理接收到的数据
                    uart_tx_dma(m2_rx_buffer);//转发到UART
                }
                rf2_timeout_count = 0;
            }else{//此时没有收到数据
                rf2_timeout_count ++;
                if(rf2_timeout_count >= rf2_wait_time_ms){//超时
                    si24r1_fast_clear(M2);// 清空中断
                    rf2_state = RF_STATE_RX_WAITING;
                    rf2_timeout_count = 0;
                }
            }
        }
////////////////////////////////////////////////////////////////////////////////////////////
    tick_update = 0;//放置在处理末尾以检测overloop
    }
}
