#include "pigpio.h"
#include <stdio.h>
#include <string.h>
#include "SPI_Messages.h"
#include <stdlib.h>
uint8_t RXbuf[4]={0,0,0,0};
uint8_t Err[]={0xFF,0xFF,0xFF,0xFF};

int main (void)
{
//responses to register access
gpioInitialise();	
struct Register_Access_Command Register_Read={REGISTER_READ_COMMAND,REGISTER_READ_ADDRESS,REGISTER_READ_DATA,0b00000000};
struct Register_Access_Command Register_Write={REGISTER_WRITE_COMMAND,REGISTER_READ_ADDRESS,REGISTER_READ_DATA,0b00000000};
struct Register_Response Register_Read_Response={(((REGISTER_READ_RESPONSE)<<4)|(ST<<2)|UD),REGISTER_DATA_0,REGISTER_DATA_0,CRC_0};
struct Register_Response Register_Write_Response={(((REGISTER_WRITE_RESPONSE)<<4)|(ST<<2)|UD),REGISTER_DATA_0,REGISTER_DATA_0,CRC_0};
unsigned uart1=serOpen("/dev/ttyAMA0",115200,0);
uint32_t RegRead=((Register_Read.Command__Fixed_Bits)<<24)|((Register_Read.Register_Address)<<16)|((Register_Read.Register_Data)<<8);
uint32_t RegWrite=((Register_Read.Command__Fixed_Bits)<<24)|((Register_Read.Register_Address)<<16)|((Register_Read.Register_Data)<<8);
uint32_t ReadResponse=((Register_Read_Response.Command_BS_UD)<<24)|((Register_Read_Response.Register_Data_H)<<16)|((Register_Read_Response.Register_Data_L)<<8);
uint32_t WriteResponse=((Register_Write_Response.Command_BS_UD)<<24)|((Register_Write_Response.Register_Data_H)<<16)|((Register_Write_Response.Register_Data_L)<<8);

Register_Read.CRC=CRC8(RegRead);
Register_Write.CRC=CRC8(RegWrite);
Register_Read_Response.CRC=CRC8(ReadResponse);
Register_Write_Response.CRC=CRC8(WriteResponse);
printf("%x",Register_Write.CRC);
uint8_t Readcmd[]={Register_Read.Command__Fixed_Bits,Register_Read.Register_Address,Register_Read.Register_Data,Register_Read.CRC};
uint8_t Writecmd[]={Register_Write.Command__Fixed_Bits,Register_Write.Register_Address,Register_Write.Register_Data,Register_Write.CRC};
uint8_t ReadcmdResponse[]={Register_Read_Response.Command_BS_UD,Register_Read_Response.Register_Data_H,Register_Read_Response.Register_Data_L,Register_Read_Response.CRC};
uint8_t WritecmdResponse[]={Register_Write_Response.Command_BS_UD,Register_Write_Response.Register_Data_H,Register_Write_Response.Register_Data_L,Register_Write_Response.CRC};

while(1)
{
  if(serRead(uart1,(char*)RXbuf,sizeof(Readcmd))>0)
   {	
    if(memcmp(RXbuf,Readcmd,sizeof(Readcmd))==0)
     {
      serWrite(uart1,(char*)ReadcmdResponse,sizeof(ReadcmdResponse));
      serClose(uart1);
      gpioTerminate();
      exit(0);
     }
     else if(memcmp(RXbuf,Writecmd,sizeof(Writecmd))==0)
     {
	  serWrite(uart1,(char*)WritecmdResponse,sizeof(WritecmdResponse));
      serClose(uart1);
      gpioTerminate();
      exit(0); 
	 }
     else
     {
	  serWrite(uart1,(char*)Err,sizeof(Err));
      serClose(uart1);
      gpioTerminate();
      exit(0);
     }
   }
  else continue;
}
}
static uint8_t CRC8(uint32_t SPI_data)
{
	uint64_t mask = MAX_UINT64_T - MAX_CRC;
	uint64_t rem = (uint64_t)((((uint64_t)SPI_data) | (((uint64_t)CRC_SEED) << MSG_LEN)) & mask);
	uint64_t divider = ((uint64_t)CRC_POLY) << (MSG_LEN - 1);

	for (uint16_t i = MSG_LEN + CRC_LEN; i > 0; i--) //32 = 4*8
	{
		if (((rem >> (i - 1)) & 0x01) != 0)
		{
			rem ^= divider;
		}
		divider >>= 1;
		if ((rem & mask) == 0) //end of calc
		{
			break;
		}
	}
	// Return 8-bit CRC calculated 
	return (uint8_t)(rem & MAX_CRC);
}

