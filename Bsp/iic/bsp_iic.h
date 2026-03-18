#ifndef __BSP_IIC_H
#define __BSP_IIC_H

#include "stm32f1xx_hal.h"
#include "gpio.h"

#define IIC_SCL_H(bus)  (bus->IIC_SCL_PORT->BSRR = bus->IIC_SCL_PIN)
#define IIC_SCL_L(bus)  (bus->IIC_SCL_PORT->BRR  = bus->IIC_SCL_PIN)
#define IIC_SDA_H(bus)  (bus->IIC_SDA_PORT->BSRR = bus->IIC_SDA_PIN)
#define IIC_SDA_L(bus)  (bus->IIC_SDA_PORT->BRR  = bus->IIC_SDA_PIN)
#define IIC_SDA_READ(bus) (bus->IIC_SDA_PORT->IDR & bus->IIC_SDA_PIN)

// 若要达到400KHz，此处填1即可 (DWT_Delay保证了最小耗时)
#define IIC_DELAY()     delay_us(1)

typedef struct
{
	GPIO_TypeDef * IIC_SDA_PORT;
	GPIO_TypeDef * IIC_SCL_PORT;
	uint16_t IIC_SDA_PIN;
	uint16_t IIC_SCL_PIN;
	//void (*CLK_ENABLE)(void);
}iic_bus_t;

void IICStart(iic_bus_t *bus);
void IICStop(iic_bus_t *bus);
unsigned char IICWaitAck(iic_bus_t *bus);
void IICSendAck(iic_bus_t *bus);
void IICSendNotAck(iic_bus_t *bus);
void IICSendByte(iic_bus_t *bus, unsigned char cSendByte);
unsigned char IICReceiveByte(iic_bus_t *bus);
void IICInit(iic_bus_t *bus);

uint8_t IIC_Write_One_Byte(iic_bus_t *bus, uint8_t daddr,uint8_t reg,uint8_t data);
uint8_t IIC_Write_Multi_Byte(iic_bus_t *bus, uint8_t daddr,uint8_t reg,uint8_t length,uint8_t buff[]);
unsigned char IIC_Read_One_Byte(iic_bus_t *bus, uint8_t daddr,uint8_t reg);
uint8_t IIC_Read_Multi_Byte(iic_bus_t *bus, uint8_t daddr, uint8_t reg, uint8_t length, uint8_t buff[]);
#endif
