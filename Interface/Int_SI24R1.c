#include "Int_SI24R1.h"
#include <stdint.h>

uint8_t TX_ADDRESS[TX_ADR_WIDTH] = {0x0A,0x01,0x06,0x0E,0x01};

//SPI读写一个字节
static uint8_t SPI_RW(uint8_t byte)
{
	uint8_t rx_data = 0;
	HAL_SPI_TransmitReceive(&hspi1, &byte, &rx_data, 1, 1000);                              
	return rx_data;
}

//写寄存器
uint8_t Int_SI24R1_Write_Reg(uint8_t reg, uint8_t value)
{
	uint8_t status;

	CS_LOW;               
	status = SPI_RW(reg);				
	SPI_RW(value);
	CS_HIGH;  
	
	return (status);
}

//写缓冲区
uint8_t Int_SI24R1_Write_Buf(uint8_t reg, const uint8_t *pBuf, uint8_t size)
{
	uint8_t status,byte_ctr;

	CS_LOW;                                  			
	status = SPI_RW(reg);                          
	for(byte_ctr = 0; byte_ctr < size; byte_ctr++)     
	{
		SPI_RW(*pBuf++);
	}
	CS_HIGH;                                     	

	return (status);       
}							  					   



uint8_t Int_SI24R1_Read_Reg(uint8_t reg)
{
 	uint8_t value;

	CS_LOW;    
	SPI_RW(reg);			
	value = SPI_RW(0);
	CS_HIGH;

	return (value);
}



uint8_t Int_SI24R1_Read_Buf(uint8_t reg, uint8_t *pBuf, uint8_t size)
{
	uint8_t status,byte_ctr;

	CS_LOW;                                        
	status = SPI_RW(reg);                           
	for(byte_ctr=0;byte_ctr<size;byte_ctr++)
	{
		pBuf[byte_ctr] = SPI_RW(0);
	}
	CS_HIGH;                                        

  return (status);    
}



void Int_SI24R1_RX_Mode(void)
{
	CE_LOW;
	Int_SI24R1_Write_Buf(SI24R1_WRITE_REG + RX_ADDR_P0, TX_ADDRESS, TX_ADR_WIDTH);//接收地址
	Int_SI24R1_Write_Reg(SI24R1_WRITE_REG + EN_AA, 0x01);//使能自动应答
	Int_SI24R1_Write_Reg(SI24R1_WRITE_REG + EN_RXADDR, 0x01);//使能通道0的接收地址
	Int_SI24R1_Write_Reg(SI24R1_WRITE_REG + RF_CH, 40);
	Int_SI24R1_Write_Reg(SI24R1_WRITE_REG + RX_PW_P0, TX_PLOAD_WIDTH);
	Int_SI24R1_Write_Reg(SI24R1_WRITE_REG + RF_SETUP, 0x06);
	Int_SI24R1_Write_Reg(SI24R1_WRITE_REG + CONFIG, 0x0f);
	Int_SI24R1_Write_Reg(SI24R1_WRITE_REG + STATUS, 0xff);
	CE_HIGH;
}						



void Int_SI24R1_TX_Mode(void)
{
	CE_LOW;
	Int_SI24R1_Write_Buf(SI24R1_WRITE_REG + TX_ADDR, TX_ADDRESS, TX_ADR_WIDTH);//发送地址
	Int_SI24R1_Write_Buf(SI24R1_WRITE_REG + RX_ADDR_P0, TX_ADDRESS, TX_ADR_WIDTH);//为了使得发送成功，必须要设置RX_ADDR_P0,因为ACK是发给这个地址的

	Int_SI24R1_Write_Reg(SI24R1_WRITE_REG + EN_AA, 0x01);       										
	Int_SI24R1_Write_Reg(SI24R1_WRITE_REG + EN_RXADDR, 0x01);   										
	Int_SI24R1_Write_Reg(SI24R1_WRITE_REG + SETUP_RETR, 0x0a);  							
	Int_SI24R1_Write_Reg(SI24R1_WRITE_REG + RF_CH, 40);         										
	Int_SI24R1_Write_Reg(SI24R1_WRITE_REG + RF_SETUP, 0x06);    										
	Int_SI24R1_Write_Reg(SI24R1_WRITE_REG + CONFIG, 0x0e);      										
	CE_HIGH;
}


uint8_t Int_SI24R1_RxPacket(uint8_t *rxbuf)
{
	uint8_t state;
	state = Int_SI24R1_Read_Reg(STATUS);  			                  
	Int_SI24R1_Write_Reg(SI24R1_WRITE_REG+STATUS,state);              

	if(state & RX_DR)
	{
		Int_SI24R1_Read_Buf(RD_RX_PLOAD,rxbuf,TX_PLOAD_WIDTH);   
		Int_SI24R1_Write_Reg(FLUSH_RX,0xff);					          
		return 0; 
	}
	return 1; 
}



uint8_t Int_SI24R1_TxPacket(uint8_t *txbuf)
{
	uint8_t state;
	CE_LOW;																									
	Int_SI24R1_Write_Buf(WR_TX_PLOAD, txbuf, TX_PLOAD_WIDTH);	  
 	CE_HIGH;																									  
	 
	//while(IRQ == 1);																			
	state = Int_SI24R1_Read_Reg(STATUS);
	while((state & (TX_DS | MAX_RT)) == 0)																
	{
		state = Int_SI24R1_Read_Reg(STATUS);
		vTaskDelay(1);
	}

	Int_SI24R1_Write_Reg(SI24R1_WRITE_REG + STATUS, state); 		

	if(state & MAX_RT)																		
	{
		Int_SI24R1_Write_Reg(FLUSH_TX,0xff);						
		return 1;
	}
	if(state & TX_DS)																			 
	{
		return 0;
	}
	return 1;					
}


uint8_t rx_buf[5] = {0};

//初始化检测
uint8_t Int_SI24R1_Check()
{
	uint8_t i;
	Int_SI24R1_Read_Buf(SI24R1_READ_REG + TX_ADDR, rx_buf, TX_ADR_WIDTH);

	Int_SI24R1_Write_Buf(SI24R1_WRITE_REG + TX_ADDR, TX_ADDRESS, TX_ADR_WIDTH);//发送地址
	Int_SI24R1_Read_Buf(SI24R1_READ_REG + TX_ADDR, rx_buf, TX_ADR_WIDTH);
	for(i = 0; i < TX_ADR_WIDTH; i++)
	{
		if(rx_buf[i] != TX_ADDRESS[i])
		{
			return 1;
		}
	}
	return 0;
}


void Int_SI24R1_Init(void)
{
	HAL_Delay(200);
	while(Int_SI24R1_Check() == 1)
	{
		HAL_Delay(10);
	}
	//默认状态为接收状态
	Int_SI24R1_RX_Mode();
	Debug_Printf("successful!\r\n");
}
