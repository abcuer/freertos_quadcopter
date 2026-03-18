#include "bsp_iic.h"
#include "bsp_dwt.h"

/**
  * @brief SDA线输入模式配置
  * @param None
  * @retval None
  */
void SDA_Input_Mode(iic_bus_t *bus)
{
    GPIO_InitTypeDef GPIO_InitStructure = {0};

    GPIO_InitStructure.Pin = bus->IIC_SDA_PIN;
    GPIO_InitStructure.Mode = GPIO_MODE_INPUT;
    GPIO_InitStructure.Pull = GPIO_PULLUP;
    GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(bus->IIC_SDA_PORT, &GPIO_InitStructure);
}

/**
  * @brief SDA线输出模式配置
  * @param None
  * @retval None
  */
void SDA_Output_Mode(iic_bus_t *bus)
{
    GPIO_InitTypeDef GPIO_InitStructure = {0};

    GPIO_InitStructure.Pin = bus->IIC_SDA_PIN;
    GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_OD;
    GPIO_InitStructure.Pull = GPIO_NOPULL;
    GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(bus->IIC_SDA_PORT, &GPIO_InitStructure);
}

/**
  * @brief SDA线输出一个位
  * @param val 输出的数据
  * @retval None
  */void SDA_Output(iic_bus_t *bus, uint16_t val)
{
    if ( val )
    {
        bus->IIC_SDA_PORT->BSRR |= bus->IIC_SDA_PIN;
    }
    else
    {
        bus->IIC_SDA_PORT->BSRR = (uint32_t)bus->IIC_SDA_PIN << 16U;
    }
}

/**
  * @brief SCL线输出一个位
  * @param val 输出的数据
  * @retval None
  */
void SCL_Output(iic_bus_t *bus, uint16_t val)
{
    if ( val )
    {
        bus->IIC_SCL_PORT->BSRR |= bus->IIC_SCL_PIN;
    }
    else
    {
         bus->IIC_SCL_PORT->BSRR = (uint32_t)bus->IIC_SCL_PIN << 16U;
    }
}

/**
  * @brief SDA输入一位
  * @param None
  * @retval GPIO读入一位
  */
uint8_t SDA_Input(iic_bus_t *bus)
{
	if(HAL_GPIO_ReadPin(bus->IIC_SDA_PORT, bus->IIC_SDA_PIN) == GPIO_PIN_SET){
		return 1;
	}else{
		return 0;
	}
}

/**
  * @brief IIC起始信号
  * @param None
  * @retval None
  */
void IICStart(iic_bus_t *bus)
{
    IIC_SDA_H(bus);
    IIC_SCL_H(bus);
    IIC_DELAY();
    IIC_SDA_L(bus); // SCL高时，SDA由高变低
    IIC_DELAY();
    IIC_SCL_L(bus);
}

/**
  * @brief IIC结束信号
  * @param None
  * @retval None
  */
void IICStop(iic_bus_t *bus)
{
    IIC_SCL_L(bus);
    IIC_SDA_L(bus);
    IIC_DELAY();
    IIC_SCL_H(bus); // SCL高时，SDA由低变高
    IIC_DELAY();
    IIC_SDA_H(bus);
}

/**
  * @brief IIC等待确认信号
  * @param None
  * @retval None
  */
uint8_t IICWaitAck(iic_bus_t *bus)
{
    uint8_t retry = 0;
    IIC_SDA_H(bus); // 释放SDA线（开漏模式写1即释放）
    IIC_DELAY();
    IIC_SCL_H(bus);
    IIC_DELAY();
    
    while (IIC_SDA_READ(bus))
    {
        retry++;
        if (retry > 200) // 超时判断
        {
            IICStop(bus);
            return 1;
        }
    }
    IIC_SCL_L(bus);
    return 0;
}

/**
  * @brief IIC发送确认信号
  * @param None
  * @retval None
  */
void IICSendAck(iic_bus_t *bus)
{
    IIC_SCL_L(bus);
    IIC_SDA_L(bus);
    IIC_DELAY();
    IIC_SCL_H(bus);
    IIC_DELAY();
    IIC_SCL_L(bus);
}

/**
  * @brief IIC发送非确认信号
  * @param None
  * @retval None
  */
void IICSendNotAck(iic_bus_t *bus)
{
    IIC_SCL_L(bus);
    IIC_SDA_H(bus);
    IIC_DELAY();
    IIC_SCL_H(bus);
    IIC_DELAY();
    IIC_SCL_L(bus);
}

/**
  * @brief IIC发送一个字节
  * @param cSendByte 需要发送的字节
  * @retval None
  */
void IICSendByte(iic_bus_t *bus, uint8_t data)
{
    for (uint8_t i = 0; i < 8; i++)
    {
        IIC_SCL_L(bus);
        if (data & 0x80) IIC_SDA_H(bus);
        else IIC_SDA_L(bus);
        data <<= 1;
        IIC_DELAY();
        IIC_SCL_H(bus);
        IIC_DELAY();
    }
    IIC_SCL_L(bus);
}
/**
  * @brief IIC接收一个字节
  * @param None
  * @retval 接收到的字节
  */
uint8_t IICReceiveByte(iic_bus_t *bus)
{
    uint8_t receive = 0;
    IIC_SDA_H(bus); // 开漏模式下，写1即可直接读取
    for (uint8_t i = 0; i < 8; i++)
    {
        IIC_SCL_L(bus);
        IIC_DELAY();
        IIC_SCL_H(bus);
        receive <<= 1;
        if (IIC_SDA_READ(bus)) receive++;
        IIC_DELAY();
    }
    IIC_SCL_L(bus);
    return receive;
}

uint8_t IIC_Write_One_Byte(iic_bus_t *bus, uint8_t daddr,uint8_t reg,uint8_t data)
{				   	  	    																 
  IICStart(bus);  
	
	IICSendByte(bus,daddr<<1);	    
	if(IICWaitAck(bus))	
	{
		IICStop(bus);		 
		return 1;		
	}
	IICSendByte(bus,reg);
	IICWaitAck(bus);	   	 										  		   
	IICSendByte(bus,data);     						   
	IICWaitAck(bus);  		    	   
  IICStop(bus);
	delay_us(1);
	return 0;
}

uint8_t IIC_Write_Multi_Byte(iic_bus_t *bus, uint8_t daddr,uint8_t reg,uint8_t length,uint8_t buff[])
{			
	unsigned char i;	
  IICStart(bus);  
	
	IICSendByte(bus,daddr<<1);	    
	if(IICWaitAck(bus))
	{
		IICStop(bus);
		return 1;
	}
	IICSendByte(bus,reg);
	IICWaitAck(bus);	
	for(i=0;i<length;i++)
	{
		IICSendByte(bus,buff[i]);     						   
		IICWaitAck(bus); 
	}		    	   
  IICStop(bus);
	delay_us(1);
	return 0;
} 

unsigned char IIC_Read_One_Byte(iic_bus_t *bus, uint8_t daddr,uint8_t reg)
{
	unsigned char dat;
	IICStart(bus);
	IICSendByte(bus,daddr<<1);
	IICWaitAck(bus);
	IICSendByte(bus,reg);
	IICWaitAck(bus);
	
	IICStart(bus);
	IICSendByte(bus,(daddr<<1)+1);
	IICWaitAck(bus);
	dat = IICReceiveByte(bus);
	IICSendNotAck(bus);
	IICStop(bus);
	return dat;
}

uint8_t IIC_Read_Multi_Byte(iic_bus_t *bus, uint8_t daddr, uint8_t reg, uint8_t length, uint8_t buff[])
{
    unsigned char i;
    
    // 1. 发送写地址和目标寄存器
    IICStart(bus);
    IICSendByte(bus, daddr << 1);
    if(IICWaitAck(bus))
    {
        IICStop(bus);        
        return 1;        
    }
    IICSendByte(bus, reg);
    IICWaitAck(bus);
    
    // --- 关键修改 1: 在重复起始信号前增加极微小延迟 ---
    delay_us(2); 

    // 2. 发送重复起始信号和读地址
    IICStart(bus); // Repeated Start
    IICSendByte(bus, (daddr << 1) + 1);
    if(IICWaitAck(bus))
    {
        IICStop(bus);        
        return 1;        
    }

    // 3. 循环读取数据
    for(i = 0; i < length; i++)
    {
        buff[i] = IICReceiveByte(bus);
        
        // --- 关键修改 2: 严格控制 ACK 逻辑 ---
        if(i < (length - 1))
        {
            IICSendAck(bus); // 还没读完，发应答
        }
    }
    
    // 4. 最后一个字节读完，发送 NACK 并停止
    IICSendNotAck(bus);
    IICStop(bus);
    
    return 0; // 返回 0 表示通信成功
}

// uint8_t IIC_Read_Multi_Byte(iic_bus_t *bus, uint8_t daddr, uint8_t reg, uint8_t length, uint8_t buff[])
// {
// 	unsigned char i;
// 	IICStart(bus);
// 	IICSendByte(bus,daddr<<1);
// 	if(IICWaitAck(bus))
// 	{
// 		IICStop(bus);		 
// 		return 1;		
// 	}
// 	IICSendByte(bus,reg);
// 	IICWaitAck(bus);
	
// 	IICStart(bus);
// 	IICSendByte(bus,(daddr<<1)+1);
// 	IICWaitAck(bus);
// 	for(i=0;i<length;i++)
// 	{
// 		buff[i] = IICReceiveByte(bus);
// 		if(i<length-1)
// 		{IICSendAck(bus);}
// 	}
// 	IICSendNotAck(bus);
// 	IICStop(bus);
// 	return 0;
// }


void IICInit(iic_bus_t *bus)
{
    GPIO_InitTypeDef GPIO_InitStructure = {0};

    // SCL 可以是推挽输出
    GPIO_InitStructure.Pin = bus->IIC_SCL_PIN;
    GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStructure.Pull = GPIO_PULLUP;
    GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(bus->IIC_SCL_PORT, &GPIO_InitStructure);

    // SDA 关键：配置为开漏输出 (Output Open-Drain)
    GPIO_InitStructure.Pin = bus->IIC_SDA_PIN;
    GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_OD; 
    GPIO_InitStructure.Pull = GPIO_PULLUP; // 依靠内部或外部上拉
    GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(bus->IIC_SDA_PORT, &GPIO_InitStructure);

    IIC_SCL_H(bus);
    IIC_SDA_H(bus);
}
