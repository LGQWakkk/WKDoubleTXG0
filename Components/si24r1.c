// 20250310 Wakkk
// WKDoubleTXG0 Project 
#include "si24r1.h"
#include "system.h"

uint8_t enable_ack_payload;//是否允许数据回传
uint8_t si24r1_ack_payload_buffer[32];
uint16_t ack_payload_count;//已经发送的ACK PAYLOAD数据包个数
uint8_t irq_state=0;

//发送并接收一个字节
uint8_t spi_read_write_byte(uint8_t tx_data)//OK
{
	uint8_t rx_data = 0x00;
	HAL_SPI_TransmitReceive(&SI24R1_SPI_HANDLE, &tx_data, &rx_data, 1, HAL_MAX_DELAY);
	return rx_data;
}

// 读取寄存器一个字节
// index: 1 or 2 选择要通信的模块
uint8_t spi_read_reg(uint8_t index, uint8_t reg_addr)//OK
{
    uint8_t btmp;
    CS_LOW
    spi_read_write_byte(NRF_READ_REG | reg_addr);
    btmp = spi_read_write_byte(0xFF);
    CS_HIGH
    return btmp;
}

//读取多个字节
//index: 1 or 2 选择要通信的模块
//reg_addr:寄存器地址
//pBuf:读取到的数据存放地址
//len:要读取的数据长度
uint8_t spi_read_buf(uint8_t index, uint8_t reg_addr, uint8_t *pBuf, uint8_t len)//OK
{
    uint8_t btmp;
    CS_LOW
    spi_read_write_byte(NRF_READ_REG | reg_addr);
    for( btmp = 0; btmp < len; btmp ++ ){
        *( pBuf + btmp ) = spi_read_write_byte(0xFF);
    }
    CS_HIGH
    return RETURN_SUCCESS;
}

//写寄存器
uint8_t spi_write_reg(uint8_t index, uint8_t reg_addr, uint8_t Value)//OK
{
    CS_LOW
    spi_read_write_byte(NRF_WRITE_REG | reg_addr);
    spi_read_write_byte(Value);
    CS_HIGH
    return RETURN_SUCCESS;
}

//写入寄存器多个字节
//reg_addr:寄存器地址
//pBuf:要写入的数据地址
//len:要写入的数据长度
uint8_t spi_write_buf(uint8_t index, uint8_t reg_addr, uint8_t *pBuf, uint8_t len)//OK
{
    uint8_t i;
    CS_LOW
    spi_read_write_byte(NRF_WRITE_REG | reg_addr);
    for(i = 0; i < len; i ++)
    {
        spi_read_write_byte( *( pBuf + i ) );
    }
    CS_HIGH
    return RETURN_SUCCESS;
}

//清空TX缓冲区
uint8_t si24r1_flush_tx_fifo(uint8_t index)//OK
{
    CS_LOW
    spi_read_write_byte(FLUSH_TX);	//清TX FIFO命令
    CS_HIGH
    return RETURN_SUCCESS;
}

//清空RX缓冲区
uint8_t si24r1_flush_rx_fifo(uint8_t index)//OK
{
    CS_LOW
    spi_read_write_byte(FLUSH_RX);	//清RX FIFO命令
    CS_HIGH
    return RETURN_SUCCESS;
}

//重新使用上一包数据
uint8_t si24r1_reuse_tx_payload(uint8_t index)//OK
{
    CS_LOW
    spi_read_write_byte(REUSE_TX_PL);		//重新使用上一包命令
    CS_HIGH
    return RETURN_SUCCESS;
}

//NOP
uint8_t si24r1_nop(uint8_t index)//OK
{
    CS_LOW
    spi_read_write_byte(NOP);		//空操作命令
    CS_HIGH
    return RETURN_SUCCESS;
}

//读取状态寄存器
uint8_t si24r1_read_status_reg(uint8_t index)//OK
{
    uint8_t Status;
    CS_LOW
    Status = spi_read_write_byte( NRF_READ_REG + STATUS );	//读状态寄存器
    CS_HIGH
    return Status;
}

//清除中断状态
//IRQ_Source:要清除的中断源
//返回值为清除后的状态寄存器值
uint8_t si24r1_clear_irq_flag(uint8_t index, uint8_t IRQ_Source)//OK
{
    uint8_t btmp = 0;
    IRQ_Source &= ( 1 << RX_DR ) | ( 1 << TX_DS ) | ( 1 << MAX_RT );	//中断标志处理
    btmp = si24r1_read_status_reg(index);			      //读状态寄存器
    CS_LOW	
    spi_read_write_byte( NRF_WRITE_REG + STATUS );	//写状态寄存器命令
    spi_read_write_byte( IRQ_Source | btmp );		//清相应中断标志
    CS_HIGH	
    return ( si24r1_read_status_reg(index));			    //返回状态寄存器状态
}

//读取中断状态
uint8_t si24r1_read_irq_status(uint8_t index)//OK
{
    return ( si24r1_read_status_reg(index) & (( 1 << RX_DR ) | ( 1 << TX_DS ) | ( 1 << MAX_RT )));	//返回中断状态
}
 
//读FIFO中数据宽度
//返回值为数据宽度
uint8_t si24r1_read_fifo_width(uint8_t index)//OK
{
    uint8_t btmp;
    CS_LOW
    spi_read_write_byte(R_RX_PL_WID);	//读FIFO中数据宽度命令
    btmp = spi_read_write_byte(0xFF);	//读数据
    CS_HIGH
    return btmp;
}

//读取接收到的数据
//pRxBuf:接收到的数据存放地址
uint8_t si24r1_read_rx_payload(uint8_t index, uint8_t *pRxBuf)//OK
{
    uint8_t Width, PipeNum;
    PipeNum = ( spi_read_reg(index,  STATUS ) >> 1 ) & 0x07;	//读接收状态
    Width = si24r1_read_fifo_width(index);		//读接收数据个数
    CS_LOW
    spi_read_write_byte( RD_RX_PLOAD );			//读有效数据命令
    for( PipeNum = 0; PipeNum < Width; PipeNum ++ )
    {
        *( pRxBuf + PipeNum ) = spi_read_write_byte( 0xFF );		//读数据
    }
    CS_HIGH
    si24r1_flush_rx_fifo(index);	//清空RX FIFO
    return Width;
}

//发送数据(带应答) 单次数据不超过32字节
uint8_t si24r1_tx_payload_ack(uint8_t index, uint8_t *pTxBuf, uint8_t len)//OK
{
    uint8_t btmp;
    uint8_t length = (len > 32) ? 32 : len;		//数据长达大约32 则只发送32个
    si24r1_flush_tx_fifo(index);		//清TX FIFO
    CS_LOW	
    spi_read_write_byte(WR_TX_PLOAD);	//发送命令
    for(btmp = 0; btmp < length; btmp++)
    {
        spi_read_write_byte( *( pTxBuf + btmp ) );	//发送数据
    }
    CS_HIGH
    return RETURN_SUCCESS;
}

//发送数据（不带应答） 单次数据不超过32字节
//返回数值为发送完毕之后读取到的状态寄存器值  
//此函数可以达到1.548kHz 32Bytes数据包发送频率 但是接收不确保有此频率(STM32F103:72MHZ SPI:9MHZ)
uint8_t si24r1_tx_payload_nack(uint8_t index, uint8_t *pTxBuf, uint8_t len)//OK
{
    if(len > 32 || len == 0){
        return 0;//数据长度大于32 或者等于0 不执行
    }
    CS_LOW
    spi_read_write_byte(WR_TX_PLOAD_NACK);	//发送命令
    while(len--){
        spi_read_write_byte(*pTxBuf);		//发送数据
        pTxBuf++;
    }
    CS_HIGH
    // 等待TX_DS中断(应立即发生)
    while(GET_IRQ_STATUS(index));//此处约等待0.6-1.4ms左右
    uint8_t _status = spi_read_reg(index, STATUS);
    spi_write_reg(index, STATUS, _status);// 清除中断
    return _status;
}

//在接收模式下写入发送数据 单次数据不超过32字节
uint8_t si24r1_rx_payload_ack(uint8_t index, uint8_t *pData, uint8_t len)//OK
{
    uint8_t btmp;
    len = ( len > 32 ) ? 32 : len;	//数据长度大于32个则只写32个字节
    CS_LOW	
    spi_read_write_byte( W_ACK_PLOAD );
    for(btmp = 0; btmp < len; btmp ++){
        spi_read_write_byte( *( pData + btmp ) );
    }
    CS_HIGH
    return RETURN_SUCCESS;
}

//设置发送地址 不大于5字节
uint8_t si24r1_set_tx_addr(uint8_t index, uint8_t *pAddr, uint8_t len)//OK
{
    len = ( len > 5 ) ? 5 : len;		        //地址不能大于5个字节
    spi_write_buf(index, TX_ADDR, pAddr, len);	//写地址
    return RETURN_SUCCESS;
}

//设置接收通道地址
//PipeNum:通道 0-5
//pAddr:地址存放地址
//Len:地址长度 不大于5字节
// 注意 只有接收通道0有自由的5字节地址 对于剩下5个通道和通道0共用4字节高位地址
uint8_t si24r1_set_rx_addr(uint8_t index, uint8_t PipeNum, uint8_t *pAddr, uint8_t Len)//OK
{
    Len = (Len > 5) ? 5 : Len;
    PipeNum = (PipeNum > 5) ? 5 : PipeNum;		            //通道不大于5 地址长度不大于5个字节
    spi_write_buf(index, RX_ADDR_P0 + PipeNum, pAddr, Len);	//写入地址
    return RETURN_SUCCESS;
}

//////////////////////////////////////////RF_SETUP//////////////////////////////////////////
//设置通信速率
uint8_t si24r1_set_speed(uint8_t index, si24r1_speed_type Speed)//OK
{
	uint8_t btmp = 0;
	btmp = spi_read_reg(index,  RF_SETUP);
	btmp &= ~( ( 1<<5 ) | ( 1<<3 ) );
	if( Speed == SPEED_250K ){
		btmp |= ( 1<<5 );
	}
	else if( Speed == SPEED_1M ){
   	    btmp &= ~( ( 1<<5 ) | ( 1<<3 ) );
	}
	else if( Speed == SPEED_2M ){
		btmp |= ( 1<<3 );
	}
	spi_write_reg(index,  RF_SETUP, btmp);
    return RETURN_SUCCESS;
}

// 使能恒载波发射模式 用来测试发射功率
// bit7
uint8_t si24r1_enable_cont_wave(uint8_t index)//OK
{
    uint8_t btmp;
    btmp = spi_read_reg(index, RF_SETUP);
    btmp |= 0x80;//1<<7
    spi_write_reg(index, RF_SETUP, btmp);
    return RETURN_SUCCESS;
}

// 关闭恒载波发射模式
uint8_t si24r1_disable_cont_wave(uint8_t index)//OK
{
    uint8_t btmp;
    btmp = spi_read_reg(index, RF_SETUP);
    btmp &= (~0x80);
    spi_write_reg(index, RF_SETUP, btmp);
    return RETURN_SUCCESS;
}

// 设置功率
// 因为不同芯片具体数值对应功率不同 这里仅仅是对功率等级进行设置
// 功率等级范围0-7 占寄存器低三位
// 注意不同芯片可能有些值不支持设置 请查阅数据手册
uint8_t si24r1_set_power(uint8_t index, uint8_t power_level)//OK
{
    if(power_level > 7) power_level = 7;
    uint8_t btmp;
    btmp = spi_read_reg(index,  RF_SETUP) & ~0x07;
    btmp |= power_level;  
    spi_write_reg(index, RF_SETUP, btmp);
    return RETURN_SUCCESS;
}
//////////////////////////////////////////RF_SETUP//////////////////////////////////////////

//////////////////////////////////////////RF_CH/////////////////////////////////////////////
//设置频率/设置信道
uint8_t si24r1_set_channel(uint8_t index, uint8_t FreqPoint)//OK
{
    spi_write_reg(index, RF_CH, FreqPoint & 0x7F);
    return RETURN_SUCCESS;
}
//////////////////////////////////////////RF_CH/////////////////////////////////////////////

// 检测和SI24R1的硬件通信连接是否正常
// 通过向地址寄存器写入和读取 检测SPI通信是否正常
// 注意此函数会改变地址!!!
uint8_t si24r1_check(uint8_t index)//OK
{
    uint8_t i;
    uint8_t check_count = 0;
    // uint8_t buf[5]={0XA5, 0XA5, 0XA5, 0XA5, 0XA5};
    uint8_t buf[5]={index, 0XA5, 0XA5, 0XA5, index};
    uint8_t read_buf[5] = {0};
    while(1)
    {
        if(check_count > 10) return RETURN_FAILURE;
        spi_write_buf(index, TX_ADDR, buf, 5);	        //写入5个字节的地址
        spi_read_buf(index, TX_ADDR, read_buf, 5);		//读出写入的地址  
        for(i = 0; i < 5; i++){
            if(buf[i] != read_buf[i]){//failed
                break;
            }
        }
        if(5 == i){//pass
            break;
        }
        HAL_Delay(200);
        check_count ++;
    }
    return RETURN_SUCCESS;
}

//设置发送/接收模式
uint8_t si24r1_set_mode(uint8_t index, si24r1_mode_t Mode)//OK
{
    // CE_LOW
    uint8_t controlreg = 0;
    controlreg = spi_read_reg(index, CONFIG);
    if(Mode == MODE_TX){
        controlreg &= ~(1<< PRIM_RX);
    }
    else if(Mode == MODE_RX){
        controlreg |= (1<< PRIM_RX); 
    }else{//Error Mode
        return RETURN_FAILURE;
    }
    spi_write_reg(index, CONFIG, controlreg);
    // CE_HIGH
    return RETURN_SUCCESS;
}

//SI24R1初始化
uint8_t si24r1_init(uint8_t index)//OK
{
    CS_HIGH
    // CE_LOW
    CE_HIGH
    uint8_t addr[5] = {DEFAULT_TX_ADDR};
    si24r1_clear_irq_flag(index, IRQ_ALL);
    spi_write_reg(index,  DYNPD, ( 1 << 0 ) );//使能通道1动态数据长度
    spi_write_reg(index,  FEATRUE, 0x07 );
    spi_read_reg(index,  DYNPD );
    spi_read_reg(index,  FEATRUE );
    spi_write_reg(index,  CONFIG, /*( 1<<MASK_RX_DR ) |*/    //接收中断
                                    ( 1 << EN_CRC ) |      //使能CRC 1个字节
                                    ( 1 << PWR_UP ) );     //开启设备
    spi_write_reg(index,  EN_AA, ( 1 << ENAA_P0 ) );       //通道0自动应答
    spi_write_reg(index,  EN_RXADDR, ( 1 << ERX_P0 ) );    //通道0接收
    spi_write_reg(index,  SETUP_AW, AW_5BYTES );           //地址宽度 5个字节
    spi_write_reg(index,  SETUP_RETR, ARD_4000US |
                        ( REPEAT_CNT & 0x0F ) );            //重复等待时间 250us
    spi_write_reg(index,  RF_CH, 60 );                      //初始化通道
    spi_write_reg(index,  RF_SETUP, 0x26 );
    si24r1_set_tx_addr(index,  &addr[0], 5 );               //设置TX地址
    si24r1_set_rx_addr(index,  0, &addr[0], 5 );            //设置RX地址
    return RETURN_SUCCESS;
}

//发送数据包 阻塞等待发送完毕
//txbuf: 待发送数据缓冲区
//length: 待发送数据长度
//timeout_ms: 最大等待时间
//返回值:  MAX_TX：达到最大重发次数  TX_OK：发送完成  0xFF:其他原因
uint8_t si24r1_tx_packet(uint8_t index, uint8_t *txbuf, uint8_t length, uint16_t timeout_ms)
{
	uint8_t _status = 0;
	uint16_t _1ms_count = 0;
	CS_LOW
	spi_read_write_byte( FLUSH_TX );//清空之前要发送的数据
	CS_HIGH
	CE_LOW		
	spi_write_buf(index,  WR_TX_PLOAD, txbuf, length );	//写数据到TX BUF 32字节  TX_PLOAD_WIDTH
	CE_HIGH //启动发送

	while(GET_IRQ_STATUS(index)){               //阻塞等待接收中断 每1ms检测一次
        HAL_Delay(1);
        _1ms_count ++;
        if(_1ms_count >= timeout_ms)break;
	}
	if(_1ms_count >= timeout_ms){
        si24r1_debug("si24r1: tx packet timeout!\r\n");
    }else{
        si24r1_debug("si24r1: tx packet done! wait time: %dms\r\n", _1ms_count);
    }

	_status = spi_read_reg(index, STATUS);		//读状态寄存器
	spi_write_reg(index,  STATUS, _status ); //清除TX_DS或MAX_RT中断标志

	if(_status & MAX_TX){
		spi_write_reg(index,  FLUSH_TX,0xff );	//清除TX FIFO寄存器
    si24r1_debug("si24r1: tx failed: MAX TX\r\n");
		return MAX_TX; 
	}
	if(_status & TX_OK){
    si24r1_debug("si24r1: tx success\r\n");
		return TX_OK;
	}
	si24r1_debug("si24r1: tx failed: other error\r\n");
	return 0xFF;	//其他原因发送失败
}

//非阻塞接收数据 同时处理ACK PAYLOAD数据包
uint8_t si24r1_rx_packet_nowait(uint8_t index, uint8_t* rxbuf)
{
    uint8_t _status = 0, _rxlength = 0;
    _status = spi_read_reg(index, STATUS);
    spi_write_reg(index, STATUS, _status);//清除中断
    if(_status & RX_OK)	//接收到数据
    {
        _rxlength = spi_read_reg(index,  R_RX_PL_WID );
        spi_read_buf(index,  RD_RX_PLOAD,rxbuf,_rxlength );
        spi_write_reg(index,  FLUSH_RX,0xff );
        return _rxlength; //返回接收到的数据字节数
    }
    if((_status & TX_OK) && (enable_ack_payload)){ //作为接收端收到TXDS表示ACK PAYLOAD发送完成
        // TX FIFO中ACK PAYLOAD已经自动清除
        // 这里再次进行手动清除
        CS_LOW	
        spi_read_write_byte(FLUSH_TX);
        CS_HIGH	
        // 再次填充TX FIFO 准备下一次返回数据
        si24r1_rx_payload_ack(index, si24r1_ack_payload_buffer, 32);
        ack_payload_count ++;//返回的数据包计数
    }
    return 0;//没有收到数据	
}

// 此函数支持双向通信 可以接收ACK PAYLOAD数据包
// 返回值: 0:发送成功 1:接收到ACK PAYLOAD 2:发送超时 3:其他原因
uint8_t si24r1_tx_packet_nowait(uint8_t index, uint8_t *txbuf, uint8_t* rxbuf)
{
    uint8_t rx_length = 0;//接收到的数据长度
    uint8_t ret = 3;
    uint8_t _status = 0;
    static uint16_t timeout_count = 0;//超时计数器
    _status = spi_read_reg(index, STATUS);		//读状态寄存器
    if(_status & (TX_DS | RX_DR | MAX_RT)){//存在中断
        spi_write_reg(index,  STATUS, _status );//清除对应中断
    }
    if(_status & TX_OK){//发送成功 重新装填
        ret=0;
        CS_LOW
        spi_read_write_byte(FLUSH_TX);//清空之前要发送的数据
        CS_HIGH
        CE_LOW		
        spi_write_buf(index, WR_TX_PLOAD, txbuf,32);//WR_TX_PLOAD  WR_TX_PLOAD_NACK
        CE_HIGH	//启动发送
    }
    if(_status & RX_OK){//接收到ACK PAYLOAD
        // 读取ACK PAYLOAD数据
        rx_length = spi_read_reg(index, R_RX_PL_WID);
        spi_read_buf(index, RD_RX_PLOAD,rxbuf,rx_length);
        spi_write_reg(index, FLUSH_RX,0xff);
        ret=1;
    }
    if(_status & MAX_TX){//发送超时 重新装填
        spi_write_reg(index,  FLUSH_TX,0xff );	//清除TX FIFO寄存器
        ret=2;
        CS_LOW
        spi_read_write_byte(FLUSH_TX);//清空之前要发送的数据
        CS_HIGH
        CE_LOW		
        spi_write_buf(index, WR_TX_PLOAD, txbuf,32);//WR_TX_PLOAD  WR_TX_PLOAD_NACK
        CE_HIGH	//启动发送
    }
    if(ret==3){//无事情发生 或许应该强制发送以启动循环
        timeout_count ++;
        if(timeout_count >= 100){
            timeout_count = 0;
            // 强制发送
            spi_write_reg(index,  FLUSH_TX,0xff );	//清除TX FIFO寄存器
            CS_LOW
            spi_read_write_byte(FLUSH_TX);//清空之前要发送的数据
            CS_HIGH
            CE_LOW		
            spi_write_buf(index, WR_TX_PLOAD, txbuf,32);//WR_TX_PLOAD  WR_TX_PLOAD_NACK
            CE_HIGH	//启动发送
        }
    }else{
        timeout_count = 0;
    }
    return ret;
}

// SI24R1以最高效率发送数据包 无应答
uint8_t si24r1_fast_tx(uint8_t index, uint8_t *tx_buffer)
{
    CS_LOW
    spi_read_write_byte(WR_TX_PLOAD_NACK);	//发送命令
    // for(uint8_t i=0; i<32; i++){
    //     spi_read_write_byte(tx_buffer[i]);		//发送数据
    // }
    
    HAL_SPI_Transmit(&SI24R1_SPI_HANDLE, tx_buffer, 32, HAL_MAX_DELAY);//20250321 Faster
    CS_HIGH
    return RETURN_SUCCESS;
}

uint8_t si24r1_fast_rx(uint8_t index, uint8_t *rx_buffer)
{
    uint8_t _status = 0, _rxlength = 0;
    _status = spi_read_reg(index, STATUS);
    spi_write_reg(index, STATUS, _status);//清除中断
    if(_status & RX_OK)	//接收到数据
    {
        _rxlength = spi_read_reg(index, R_RX_PL_WID );
        spi_read_buf(index, RD_RX_PLOAD, rx_buffer, _rxlength);
        spi_write_reg(index, FLUSH_RX, 0xff);
        return _rxlength; //返回接收到的数据字节数
    }
}

// 判断快速发送模式下是否出现中断(表示发送完毕)
uint8_t si24r1_fast_check(uint8_t index)
{
    if(!GET_IRQ_STATUS(index)){//出现中断
        // uint8_t _status = spi_read_reg(index, STATUS);
        // spi_write_reg(index, STATUS, _status);// 清除中断
        // return _status;
        return RETURN_SUCCESS;//发送完毕
    }else{
        return RETURN_FAILURE;//还未检测到中断
    }
}

// 快速清除中断
uint8_t si24r1_fast_clear(uint8_t index)
{
    uint8_t _status = spi_read_reg(index, STATUS);
    spi_write_reg(index, STATUS, _status);// 清除中断
    // return RETURN_SUCCESS;
    return _status;
}
