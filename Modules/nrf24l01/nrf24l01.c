#include "nrf24l01.h"

/* 地址定义 */
const uint8_t TX_ADDRESS[]= {0xAA,0xBB,0xCC,0x00,0x01}; // 发送地址 
const uint8_t RX_ADDRESS[]= {0xAA,0xBB,0xCC,0x00,0x01};	// 接收地址 

/**
 * @brief SPI读写一个字节
 * @param dat: 要发送的数据
 * @return: 接收到的数据
 */
static uint8_t SPI_Transmit_One_Byte(uint8_t date)
{
    uint8_t rxdata = 0;
    // 使用 HAL 库全双工函数，设置 10ms 超时防止总线死锁
    if(HAL_SPI_TransmitReceive(&hspi2, &date, &rxdata, 1, 10) != HAL_OK)
    {
        return 0xFF; 
    }
    return rxdata;
}

//通过SPI写寄存器
uint8_t NRF24L01_Write_Reg(uint8_t regaddr,uint8_t data)
{
	uint8_t status;	
    Clr_NRF24L01_CSN;                    //使能SPI传输
  	status = SPI_Transmit_One_Byte(regaddr); //发送寄存器号 
  	SPI_Transmit_One_Byte(data);            //写入寄存器的值
  	Set_NRF24L01_CSN;                    //禁止SPI传输	   
  	return(status);       		         //返回状态值
}
//读取SPI寄存器值 ，regaddr:要读的寄存器
uint8_t NRF24L01_Read_Reg(uint8_t regaddr)
{
	uint8_t reg_val;	    
	Clr_NRF24L01_CSN;                //使能SPI传输		
  	SPI_Transmit_One_Byte(regaddr);     //发送寄存器号
  	reg_val = SPI_Transmit_One_Byte(0XFF);//读取寄存器内容
  	Set_NRF24L01_CSN;                //禁止SPI传输		    
  	return(reg_val);                 //返回状态值
}	


//在指定位置读出指定长度的数据
//*pBuf:数据指针
//返回值,此次读到的状态寄存器值 
uint8_t NRF24L01_Read_Buf(uint8_t regaddr,uint8_t *pBuf,uint8_t datalen)
{
	uint8_t status,uint8_t_ctr;	
  	Clr_NRF24L01_CSN;                     //使能SPI传输
  	status = SPI_Transmit_One_Byte(regaddr);   //发送寄存器值(位置),并读取状态值  
	for(uint8_t_ctr=0;uint8_t_ctr<datalen;uint8_t_ctr++) 
        pBuf[uint8_t_ctr]=SPI_Transmit_One_Byte(0XFF);//读出数据
  	Set_NRF24L01_CSN;                     //关闭SPI传输
  	return status;                        //返回读到的状态值
}


//在指定位置写指定长度的数据
//*pBuf:数据指针
//返回值,此次读到的状态寄存器值
uint8_t NRF24L01_Write_Buf(uint8_t regaddr, uint8_t *pBuf, uint8_t datalen)
{
	uint8_t status,uint8_t_ctr;	    
	Clr_NRF24L01_CSN;                                    //使能SPI传输
  	status = SPI_Transmit_One_Byte(regaddr);                //发送寄存器值(位置),并读取状态值
  	for(uint8_t_ctr=0; uint8_t_ctr<datalen; uint8_t_ctr++)SPI_Transmit_One_Byte(*pBuf++); //写入数据	 
  	Set_NRF24L01_CSN;                                    //关闭SPI传输
  	return status;                                       //返回读到的状态值
}		

/**
 * @brief 发送数据包
 */
uint8_t NRF24L01_TxPacket(uint8_t *txbuf) {
    uint8_t state;
    Clr_NRF24L01_CE; 
    NRF24L01_Write_Buf(WR_TX_PLOAD, txbuf, TX_PLOAD_WIDTH);
    Set_NRF24L01_CE; 

    // 必须等待发送完成或达到最大重发次数
    // 如果不使用中断引脚，需要轮询 STATUS 寄存器
    do {
        state = NRF24L01_Read_Reg(STATUS);
    } while(!(state & (TX_OK | MAX_TX))); 

    Clr_NRF24L01_CE; // 关闭发射
    NRF24L01_Write_Reg(SPI_WRITE_REG + STATUS, state); // 清除标志位

    if(state & MAX_TX) {
        NRF24L01_Write_Reg(FLUSH_TX, 0xff);
        return 0x10;
    }
    return 0; // 发送成功
}
/**
 * @brief NRF24L01读取接收到的数据
 * @param rxbuf: 接收数据存放的缓冲区首地址
 * @return 0: 成功接收; 1: 尚未接收到数据
 */
uint8_t NRF24L01_RxPacket(uint8_t *rxbuf)
{
    uint8_t state;
    state = NRF24L01_Read_Reg(STATUS); 
    
    // 判断是否有数据到达 (RX_DR 位)
    if (state & RX_OK) 
    {
        // 从 FIFO 读取数据
        NRF24L01_Read_Buf(RD_RX_PLOAD, rxbuf, RX_PLOAD_WIDTH);
        
        // 读取完后立即清除标志位 (重要：写1清除)
        NRF24L01_Write_Reg(SPI_WRITE_REG + STATUS, state); 
        
        // 注意：不建议在此处直接 FLUSH_RX，除非你想丢弃 FIFO 中剩下的包
        // NRF24L01 的 FIFO 深度为 3 级，直接 FLUSH 会清空所有包
        return 0; 
    }
    
    return 1; // 没有收到数据
}

void NRF24L01_TX_Mode(void)
{
    Clr_NRF24L01_CE;
    
    // 配置地址和参数
    NRF24L01_Write_Buf(SPI_WRITE_REG + TX_ADDR, (uint8_t*)TX_ADDRESS, TX_ADR_WIDTH);
    NRF24L01_Write_Buf(SPI_WRITE_REG + RX_ADDR_P0, (uint8_t*)RX_ADDRESS, RX_ADR_WIDTH); // 用于接收ACK
    
    NRF24L01_Write_Reg(SPI_WRITE_REG + EN_AA, 0x01);     
    NRF24L01_Write_Reg(SPI_WRITE_REG + EN_RXADDR, 0x01); 
    NRF24L01_Write_Reg(SPI_WRITE_REG + SETUP_RETR, 0x1a); 
    NRF24L01_Write_Reg(SPI_WRITE_REG + RF_CH, CONNECT_CHANNAL); 
    NRF24L01_Write_Reg(SPI_WRITE_REG + RF_SETUP, 0x0f);   
    NRF24L01_Write_Reg(SPI_WRITE_REG + NCONFIG, 0x0e);    // 发送模式
    
    // 初始化完成后保持 CE 为低，等待 TxPacket 触发
    Clr_NRF24L01_CE; 
}

void NRF24L01_RX_Mode(void)
{
    Clr_NRF24L01_CE;
    
    NRF24L01_Write_Buf(SPI_WRITE_REG + RX_ADDR_P0, (uint8_t*)RX_ADDRESS, RX_ADR_WIDTH);
    NRF24L01_Write_Reg(SPI_WRITE_REG + EN_AA, 0x01);
    NRF24L01_Write_Reg(SPI_WRITE_REG + EN_RXADDR, 0x01);
    NRF24L01_Write_Reg(SPI_WRITE_REG + RF_CH, CONNECT_CHANNAL);
    NRF24L01_Write_Reg(SPI_WRITE_REG + RX_PW_P0, RX_PLOAD_WIDTH);
    NRF24L01_Write_Reg(SPI_WRITE_REG + RF_SETUP, 0x0f);
    NRF24L01_Write_Reg(SPI_WRITE_REG + NCONFIG, 0x0f);    // 接收模式
    
    Set_NRF24L01_CE; // 接收模式必须保持 CE 为高，才能持续监听
}

/**
 * @brief 向特定缓冲区装载数据并触发发送
 * @param tx_buf: 待发送数据
 * @param len: 数据长度
 */
void NRF_TxPacket_AP(uint8_t *tx_buf, uint8_t len)
{
    Clr_NRF24L01_CE; // 进入待机模式
    
    // 使用0xA8指令写入数据 (W_ACK_PAYLOAD)
    // 注意：如果单纯发送数据包，通常使用 WR_TX_PLOAD (0xA0)
    NRF24L01_Write_Buf(0xa8, tx_buf, len); 
    
    Set_NRF24L01_CE; // 触发无线传输
}

/**
 * @brief 模块在线检测
 * @return 0:成功, 1:失败
 */
uint8_t NRF24L01_Check(void)
{
    uint8_t buf[5] = {0XA5,0XA5,0XA5,0XA5,0XA5};
    uint8_t buf1[5];
    uint8_t i;

    // 写入5个字节测试地址
    NRF24L01_Write_Buf(SPI_WRITE_REG+TX_ADDR, buf, 5);
    NRF24L01_Read_Buf(TX_ADDR, buf1, 5);

    for(i=0; i<5; i++) 
    {
        if(buf1[i] != 0XA5) 
        {
            return 1; // 失败  
        }
    }
    return 0; // 成功
}

void NRF_Init(uint8_t model, uint8_t ch)
{
	Clr_NRF24L01_CE;
	
	NRF24L01_Write_Buf(SPI_WRITE_REG+RX_ADDR_P0,(uint8_t*)RX_ADDRESS,RX_ADR_WIDTH);	//写RX节点地址
	NRF24L01_Write_Buf(SPI_WRITE_REG+TX_ADDR,(uint8_t*)TX_ADDRESS,TX_ADR_WIDTH); 		//写TX节点地址 
	NRF24L01_Write_Reg(SPI_WRITE_REG+EN_AA,0x01); 															//使能通道0的自动应答
	NRF24L01_Write_Reg(SPI_WRITE_REG+EN_RXADDR,0x01);														//使能通道0的接收地址
	NRF24L01_Write_Reg(SPI_WRITE_REG+SETUP_RETR,0x1a);													//设置自动重发间隔时间:500us;最大自动重发次数:10次 2M波特率下
	NRF24L01_Write_Reg(SPI_WRITE_REG+RF_CH,ch);																	//设置RF通道为CHANAL
	NRF24L01_Write_Reg(SPI_WRITE_REG+RF_SETUP,0x0f); 														//设置TX发射参数,0db增益,2Mbps,低噪声增益开启
	//Write_Reg(NRF_WRITE_REG+RF_SETUP,0x07); 												    			//设置TX发射参数,0db增益,1Mbps,低噪声增益开启
	//Write_Reg(NRF_WRITE_REG+RF_SETUP,0x27); 												   				//设置TX发射参数,0db增益,250Kbps,低噪声增益开启
/////////////////////////////////////////////////////////
	if(model==1)				//RX
	{
		NRF24L01_Write_Reg(SPI_WRITE_REG+RX_PW_P0,RX_PLOAD_WIDTH);								//选择通道0的有效数据宽度 
		NRF24L01_Write_Reg(SPI_WRITE_REG + NCONFIG, 0x0f);   		        					// IRQ收发完成中断开启,16位CRC,主接收
	}
	else if(model==2)		//TX
	{
		NRF24L01_Write_Reg(SPI_WRITE_REG+RX_PW_P0,RX_PLOAD_WIDTH);								//选择通道0的有效数据宽度 
		NRF24L01_Write_Reg(SPI_WRITE_REG + NCONFIG, 0x0e);   		 									// IRQ收发完成中断开启,16位CRC,主发送
	}
	else if(model==3)		//RX2
	{
		NRF24L01_Write_Reg(FLUSH_TX,0xff);
		NRF24L01_Write_Reg(FLUSH_RX,0xff);
		NRF24L01_Write_Reg(SPI_WRITE_REG + NCONFIG, 0x0f);   		  								// IRQ收发完成中断开启,16位CRC,主接收
		
		SPI_Transmit_One_Byte(0x50);
		SPI_Transmit_One_Byte(0x73);
		NRF24L01_Write_Reg(SPI_WRITE_REG+0x1c,0x01);
		NRF24L01_Write_Reg(SPI_WRITE_REG+0x1d,0x06);
	}
	else								//TX2
	{
		NRF24L01_Write_Reg(SPI_WRITE_REG + NCONFIG, 0x0e);   											// IRQ收发完成中断开启,16位CRC,主发送
		NRF24L01_Write_Reg(FLUSH_TX,0xff);
		NRF24L01_Write_Reg(FLUSH_RX,0xff);
		
		SPI_Transmit_One_Byte(0x50);
		SPI_Transmit_One_Byte(0x73);
		NRF24L01_Write_Reg(SPI_WRITE_REG+0x1c,0x01);
		NRF24L01_Write_Reg(SPI_WRITE_REG+0x1d,0x06);
	}
	Set_NRF24L01_CE;  
}

void NRF24L01_Init(void)
{
    // 1. 检查硬件是否在线
    while(NRF24L01_Check());
    
    // 2. 配置为双向接收模式
    NRF_Init(MODEL_RX2, CONNECT_CHANNAL);
    // NRF24L01_TX_Mode();
    // SetLedMode(rLED_UP, LED_ON);
    // SetLedMode(rLED_DOWN, LED_OFF);
}



