#include "nrf24l01.h"
#include "led.h"
#include "bsp_delay.h"

/* 信号强度与错误统计变量 */
uint16_t Nrf_Erro = 0;
uint8_t NRF24L01_2_RXDATA[RX_PLOAD_WIDTH];

/* 地址定义 */
const uint8_t TX_ADDRESS[]= {0xAA,0xBB,0xCC,0x00,0x01};
const uint8_t RX_ADDRESS[]= {0xAA,0xBB,0xCC,0x00,0x01};	//½ÓÊÕµØÖ· 

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

//Í¨¹ýSPIÐ´¼Ä´æÆ÷
static uint8_t NRF24L01_Write_Reg(uint8_t regaddr,uint8_t data)
{
	uint8_t status;	
    Clr_NRF24L01_CSN;                    //Ê¹ÄÜSPI´«Êä
  	status = SPI_Transmit_One_Byte(regaddr); //·¢ËÍ¼Ä´æÆ÷ºÅ 
  	SPI_Transmit_One_Byte(data);            //Ð´Èë¼Ä´æÆ÷µÄÖµ
  	Set_NRF24L01_CSN;                    //½ûÖ¹SPI´«Êä	   
  	return(status);       		         //·µ»Ø×´Ì¬Öµ
}
//¶ÁÈ¡SPI¼Ä´æÆ÷Öµ £¬regaddr:Òª¶ÁµÄ¼Ä´æÆ÷
static uint8_t NRF24L01_Read_Reg(uint8_t regaddr)
{
	uint8_t reg_val;	    
	Clr_NRF24L01_CSN;                //Ê¹ÄÜSPI´«Êä		
  	SPI_Transmit_One_Byte(regaddr);     //·¢ËÍ¼Ä´æÆ÷ºÅ
  	reg_val = SPI_Transmit_One_Byte(0XFF);//¶ÁÈ¡¼Ä´æÆ÷ÄÚÈÝ
  	Set_NRF24L01_CSN;                //½ûÖ¹SPI´«Êä		    
  	return(reg_val);                 //·µ»Ø×´Ì¬Öµ
}	


//ÔÚÖ¸¶¨Î»ÖÃ¶Á³öÖ¸¶¨³¤¶ÈµÄÊý¾Ý
//*pBuf:Êý¾ÝÖ¸Õë
//·µ»ØÖµ,´Ë´Î¶Áµ½µÄ×´Ì¬¼Ä´æÆ÷Öµ 
static uint8_t NRF24L01_Read_Buf(uint8_t regaddr,uint8_t *pBuf,uint8_t datalen)
{
	uint8_t status,uint8_t_ctr;	
  	Clr_NRF24L01_CSN;                     //Ê¹ÄÜSPI´«Êä
  	status = SPI_Transmit_One_Byte(regaddr);   //·¢ËÍ¼Ä´æÆ÷Öµ(Î»ÖÃ),²¢¶ÁÈ¡×´Ì¬Öµ  
	for(uint8_t_ctr=0;uint8_t_ctr<datalen;uint8_t_ctr++) 
        pBuf[uint8_t_ctr]=SPI_Transmit_One_Byte(0XFF);//¶Á³öÊý¾Ý
  	Set_NRF24L01_CSN;                     //¹Ø±ÕSPI´«Êä
  	return status;                        //·µ»Ø¶Áµ½µÄ×´Ì¬Öµ
}


//ÔÚÖ¸¶¨Î»ÖÃÐ´Ö¸¶¨³¤¶ÈµÄÊý¾Ý
//*pBuf:Êý¾ÝÖ¸Õë
//·µ»ØÖµ,´Ë´Î¶Áµ½µÄ×´Ì¬¼Ä´æÆ÷Öµ
static uint8_t NRF24L01_Write_Buf(uint8_t regaddr, uint8_t *pBuf, uint8_t datalen)
{
	uint8_t status,uint8_t_ctr;	    
	Clr_NRF24L01_CSN;                                    //Ê¹ÄÜSPI´«Êä
  	status = SPI_Transmit_One_Byte(regaddr);                //·¢ËÍ¼Ä´æÆ÷Öµ(Î»ÖÃ),²¢¶ÁÈ¡×´Ì¬Öµ
  	for(uint8_t_ctr=0; uint8_t_ctr<datalen; uint8_t_ctr++)SPI_Transmit_One_Byte(*pBuf++); //Ð´ÈëÊý¾Ý	 
  	Set_NRF24L01_CSN;                                    //¹Ø±ÕSPI´«Êä
  	return status;                                       //·µ»Ø¶Áµ½µÄ×´Ì¬Öµ
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
            SetLedMode(rLEDR, LED_ON);
            return 1; // 失败  
        }
    }
    return 0; // 成功
}

void ANO_NRF_Init(uint8_t model, uint8_t ch)
{
	Clr_NRF24L01_CE;
	
	NRF24L01_Write_Buf(SPI_WRITE_REG+RX_ADDR_P0,(uint8_t*)RX_ADDRESS,RX_ADR_WIDTH);	//Ð´RX½ÚµãµØÖ·
	NRF24L01_Write_Buf(SPI_WRITE_REG+TX_ADDR,(uint8_t*)TX_ADDRESS,TX_ADR_WIDTH); 		//Ð´TX½ÚµãµØÖ· 
	NRF24L01_Write_Reg(SPI_WRITE_REG+EN_AA,0x01); 															//Ê¹ÄÜÍ¨µÀ0µÄ×Ô¶¯Ó¦´ð
	NRF24L01_Write_Reg(SPI_WRITE_REG+EN_RXADDR,0x01);														//Ê¹ÄÜÍ¨µÀ0µÄ½ÓÊÕµØÖ·
	NRF24L01_Write_Reg(SPI_WRITE_REG+SETUP_RETR,0x1a);													//ÉèÖÃ×Ô¶¯ÖØ·¢¼ä¸ôÊ±¼ä:500us;×î´ó×Ô¶¯ÖØ·¢´ÎÊý:10´Î 2M²¨ÌØÂÊÏÂ
	NRF24L01_Write_Reg(SPI_WRITE_REG+RF_CH,ch);																	//ÉèÖÃRFÍ¨µÀÎªCHANAL
	NRF24L01_Write_Reg(SPI_WRITE_REG+RF_SETUP,0x0f); 														//ÉèÖÃTX·¢Éä²ÎÊý,0dbÔöÒæ,2Mbps,µÍÔëÉùÔöÒæ¿ªÆô
	//Write_Reg(NRF_WRITE_REG+RF_SETUP,0x07); 												    			//ÉèÖÃTX·¢Éä²ÎÊý,0dbÔöÒæ,1Mbps,µÍÔëÉùÔöÒæ¿ªÆô
	//Write_Reg(NRF_WRITE_REG+RF_SETUP,0x27); 												   				//ÉèÖÃTX·¢Éä²ÎÊý,0dbÔöÒæ,250Kbps,µÍÔëÉùÔöÒæ¿ªÆô
/////////////////////////////////////////////////////////
	if(model==1)				//RX
	{
		NRF24L01_Write_Reg(SPI_WRITE_REG+RX_PW_P0,RX_PLOAD_WIDTH);								//Ñ¡ÔñÍ¨µÀ0µÄÓÐÐ§Êý¾Ý¿í¶È 
		NRF24L01_Write_Reg(SPI_WRITE_REG + NCONFIG, 0x0f);   		        					// IRQÊÕ·¢Íê³ÉÖÐ¶Ï¿ªÆô,16Î»CRC,Ö÷½ÓÊÕ
	}
	else if(model==2)		//TX
	{
		NRF24L01_Write_Reg(SPI_WRITE_REG+RX_PW_P0,RX_PLOAD_WIDTH);								//Ñ¡ÔñÍ¨µÀ0µÄÓÐÐ§Êý¾Ý¿í¶È 
		NRF24L01_Write_Reg(SPI_WRITE_REG + NCONFIG, 0x0e);   		 									// IRQÊÕ·¢Íê³ÉÖÐ¶Ï¿ªÆô,16Î»CRC,Ö÷·¢ËÍ
	}
	else if(model==3)		//RX2
	{
		NRF24L01_Write_Reg(FLUSH_TX,0xff);
		NRF24L01_Write_Reg(FLUSH_RX,0xff);
		NRF24L01_Write_Reg(SPI_WRITE_REG + NCONFIG, 0x0f);   		  								// IRQÊÕ·¢Íê³ÉÖÐ¶Ï¿ªÆô,16Î»CRC,Ö÷½ÓÊÕ
		
		SPI_Transmit_One_Byte(0x50);
		SPI_Transmit_One_Byte(0x73);
		NRF24L01_Write_Reg(SPI_WRITE_REG+0x1c,0x01);
		NRF24L01_Write_Reg(SPI_WRITE_REG+0x1d,0x06);
        SetLedMode(bLEDL, LED_ON);
	}
	else								//TX2
	{
		NRF24L01_Write_Reg(SPI_WRITE_REG + NCONFIG, 0x0e);   											// IRQÊÕ·¢Íê³ÉÖÐ¶Ï¿ªÆô,16Î»CRC,Ö÷·¢ËÍ
		NRF24L01_Write_Reg(FLUSH_TX,0xff);
		NRF24L01_Write_Reg(FLUSH_RX,0xff);
		
		SPI_Transmit_One_Byte(0x50);
		SPI_Transmit_One_Byte(0x73);
		NRF24L01_Write_Reg(SPI_WRITE_REG+0x1c,0x01);
		NRF24L01_Write_Reg(SPI_WRITE_REG+0x1d,0x06);
	}
		Set_NRF24L01_CE;  
}


/**
 * @brief 发送数据包
 */
uint8_t NRF24L01_TxPacket(uint8_t *txbuf)
{
    uint8_t state;
    Clr_NRF24L01_CE; // 进入Standby模式
    NRF24L01_Write_Buf(WR_TX_PLOAD, txbuf, TX_PLOAD_WIDTH); 
    Set_NRF24L01_CE; // 触发发送
    
    // 原源码为while等待，若需优化可在此处记录时间戳，改为异步检查
    while(READ_NRF24L01_IRQ != 0); 
    
    state = NRF24L01_Read_Reg(STATUS); 
    NRF24L01_Write_Reg(SPI_WRITE_REG+STATUS, state); // 清除标志位
    
    if(state & MAX_TX) {
        NRF24L01_Write_Reg(FLUSH_TX, 0xff);
        return 0x10;
    }
    if(state & TX_OK) return 0;
    return 1;
}

/**
 * @brief 4ms周期调用函数（核心事件处理）
 */
void ANO_NRF_Check_Event(void)
{
    uint8_t sta = NRF24L01_Read_Reg(SPI_READ_REG + STATUS);
    
    if(sta & (1<<6)) // RX_DR: 接收到数据
    {
        uint8_t rx_len = NRF24L01_Read_Reg(R_RX_PL_WID);
        if(rx_len <= 32)
        {
            NRF24L01_Read_Buf(RD_RX_PLOAD, NRF24L01_2_RXDATA, rx_len);
            Nrf_Erro = 0; // 成功接收，清除错误计数
        }
        else 
        {
            NRF24L01_Write_Reg(FLUSH_RX, 0xff); // 异常长度清空缓存
        }
    }
    // 清除状态标志
    NRF24L01_Write_Reg(SPI_WRITE_REG + STATUS, sta);
}

void NRF24L01_init(void)
{
    uint8_t i = 0;
    uint8_t status = 1; // 默认失败

    do {
        ANO_NRF_Init(MODEL_RX2, 40);
        delay_ms(5);          // 给硬件一点喘息时间，增加成功率
        status = NRF24L01_Check();
        i++;
        
        if(i > 100) break;     // 尝试100次后放弃
    } while(status == 1);

    if(status == 0) {
        SetLedMode(bLEDR, LED_ON); // 初始化成功，亮蓝灯
    } else {
        SetLedMode(rLEDR, LED_ON); // 初始化彻底失败，亮红灯
    }
}