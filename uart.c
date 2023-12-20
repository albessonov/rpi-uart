#include "pigpio.h"
#include <stdio.h>
#include <string.h>
#include "SPI_Messages.h"
#include <stdlib.h>
#include <unistd.h>
#include <pthread.h>
uint8_t RXbuf[4]={0,0,0,0};
uint8_t Err[]={0xFF,0xFF,0xFF,0xFF};
char OSD[]={0,0,0,0};
FILE *fp;
char *line = NULL;
size_t len = 0;
ssize_t readstat;
const char path1[]="/home/albessonov/accelerometer/acceleration_x.txt";
const char path2[]="/home/albessonov/accelerometer/acceleration_y.txt";
uint8_t RCOMMAND0x00=0b10000100;
uint8_t RCOMMAND0x01=0b11000100;
uint8_t RCOMMAND0x02=0b10100100;
uint8_t RCOMMAND0x03=0b11100100;

int main (void)
{
double Num;
if (gpioInitialise() < 0)
{
    printf("pigpio initialisation failed");
    exit(1);
    }
else
{
   // pigpio initialised okay.
}
unsigned uart1=serOpen("/dev/ttyAMA0",115200,0);

struct Register_Access_Command Register_Read={REGISTER_READ_COMMAND,REGISTER_READ_ADDRESS,REGISTER_READ_DATA,0b00000000};
struct Register_Access_Command Register_Write={REGISTER_WRITE_COMMAND,REGISTER_READ_ADDRESS,REGISTER_READ_DATA,0b00000000};
struct Register_Response Register_Read_Response={(((REGISTER_READ_RESPONSE)<<4)|(ST<<2)|UD),REGISTER_DATA_0,REGISTER_DATA_0,CRC_0};
struct Register_Response Register_Write_Response={(((REGISTER_WRITE_RESPONSE)<<4)|(ST<<2)|UD),REGISTER_DATA_0,REGISTER_DATA_0,CRC_0};

struct Sensor_Data_Request Request0x0={SOURCEID0x00,0,0,0};
struct Sensor_Data_Request Request0x1={SOURCEID0x01,0,0,0};
struct Sensor_Data_Request Request0x2={SOURCEID0x02,0,0,0};
struct Sensor_Data_Request Request0x3={SOURCEID0x03,0,0,0};
	//variables to calc CRC
uint32_t Register_Read_32=((Register_Read.Command__Fixed_Bits)<<24)|((Register_Read.Register_Address)<<16)|((Register_Read.Register_Data)<<8);
uint32_t Register_Write_32=((Register_Read.Command__Fixed_Bits)<<24)|((Register_Read.Register_Address)<<16)|((Register_Read.Register_Data)<<8);
uint32_t Register_Read_Response_32=((Register_Read_Response.Command_BS_UD)<<24)|((Register_Read_Response.Register_Data_H)<<16)|((Register_Read_Response.Register_Data_L)<<8);
uint32_t Register_Write_Response_32=((Register_Write_Response.Command_BS_UD)<<24)|((Register_Write_Response.Register_Data_H)<<16)|((Register_Write_Response.Register_Data_L)<<8);

uint32_t Request0x0_32=0;
uint32_t Request0x1_32=0x10000000;
uint32_t Request0x2_32=0x20000000;
uint32_t Request0x3_32=0x30000000;

Request0x0.CRC=CRC8(Request0x0_32);
Request0x1.CRC=CRC8(Request0x1_32);
Request0x2.CRC=CRC8(Request0x2_32);
Request0x3.CRC=CRC8(Request0x3_32);

Register_Read.CRC=CRC8(Register_Read_32);
Register_Write.CRC=CRC8(Register_Write_32);
Register_Read_Response.CRC=CRC8(Register_Read_Response_32);
Register_Write_Response.CRC=CRC8(Register_Write_Response_32);

uint8_t Register_Read_cmd[]={Register_Read.Command__Fixed_Bits,Register_Read.Register_Address,Register_Read.Register_Data,Register_Read.CRC};
uint8_t Register_Write_cmd[]={Register_Write.Command__Fixed_Bits,Register_Write.Register_Address,Register_Write.Register_Data,Register_Write.CRC};
uint8_t Register_Read_Response_cmd[]={Register_Read_Response.Command_BS_UD,Register_Read_Response.Register_Data_H,Register_Read_Response.Register_Data_L,Register_Read_Response.CRC};
uint8_t Register_Write_Response_cmd[]={Register_Write_Response.Command_BS_UD,Register_Write_Response.Register_Data_H,Register_Write_Response.Register_Data_L,Register_Write_Response.CRC};

uint8_t Request0x0cmd[]={Request0x0.Command__Fixed_Bits_0,Request0x0.Fixed_Bits_1,Request0x0.Fixed_Bits_2,Request0x0.CRC};
uint8_t Request0x1cmd[]={Request0x1.Command__Fixed_Bits_0,Request0x0.Fixed_Bits_1,Request0x0.Fixed_Bits_2,Request0x1.CRC};
uint8_t Request0x2cmd[]={Request0x2.Command__Fixed_Bits_0,Request0x0.Fixed_Bits_1,Request0x0.Fixed_Bits_2,Request0x2.CRC};
uint8_t Request0x3cmd[]={Request0x3.Command__Fixed_Bits_0,Request0x0.Fixed_Bits_1,Request0x0.Fixed_Bits_2,Request0x3.CRC};
while(1)
{
  if(serRead(uart1,(char*)RXbuf,sizeof(Register_Read_cmd))>0)
   {	
    if(memcmp(RXbuf,Register_Read_cmd,sizeof(Register_Read_cmd))==0)
     {
      serWrite(uart1,(char*)Register_Read_Response_cmd,sizeof(Register_Read_Response_cmd));
      serClose(uart1);
      gpioTerminate();
      exit(0);
     }
     else if(memcmp(RXbuf,Register_Write_cmd,sizeof(Register_Write_cmd))==0)
     {
	  serWrite(uart1,(char*)Register_Write_Response_cmd,sizeof(Register_Write_Response_cmd));
      serClose(uart1);
      gpioTerminate();
      exit(0); 
	 }
	 else if(memcmp(RXbuf,Request0x0cmd,sizeof(Request0x0cmd))==0)
     {
		 fp = fopen(path1, "r");
	     while ((readstat = getline(&line, &len, fp)) != -1)
	     {
			 Num = atof (line);
			 uint16_t val=20.47*Num+2047;
             uint8_t valL = (uint8_t) ((val<<4) & 0x00ff);
             uint8_t valH = (uint8_t) (((val<<4) & 0xff00) >> 8);
             char resp[4];
             resp[0]=(RCOMMAND0x00|(valH>>6));
             resp[1]=((valH<<2)|(valL>>6));
             resp[2]=(valL<<2);
             resp[3]=CRC8((((uint32_t)resp[0])<<24)|(((uint32_t)resp[1])<<16)|(((uint32_t)resp[2])<<8));
             printf("%x\n",(((uint32_t)resp[0])<<24)|(((uint32_t)resp[1])<<16)|(((uint32_t)resp[2])<<8));      
             serWrite(uart1, resp ,sizeof(resp));
             usleep(50); //will be replaced later
         }
      serClose(uart1);
      gpioTerminate();
      exit(0); 
	 }
	 else if(memcmp(RXbuf,Request0x1cmd,sizeof(Request0x1cmd))==0)
     {
	  fp = fopen(path1, "r");
	     while ((readstat = getline(&line, &len, fp)) != -1)
	     {
        Num = atof (line);
			 uint16_t val=20.47*Num+2047;
             uint8_t valL = (uint8_t) ((val<<4) & 0x00ff);
             uint8_t valH = (uint8_t) (((val<<4) & 0xff00) >> 8);
             char resp[4];
             resp[0]=(RCOMMAND0x01|(valH>>6));
             resp[1]=((valH<<2)|(valL>>6));
             resp[2]=(valL<<2);
             resp[3]=CRC8((((uint32_t)resp[0])<<24)|(((uint32_t)resp[1])<<16)|(((uint32_t)resp[2])<<8));
             printf("%x\n",(((uint32_t)resp[0])<<24)|(((uint32_t)resp[1])<<16)|(((uint32_t)resp[2])<<8));      
             serWrite(uart1, resp ,sizeof(resp));
             usleep(50); //will be replaced later
         }
      serClose(uart1);
      gpioTerminate();
      exit(0); 
	 }
	 else if(memcmp(RXbuf,Request0x2cmd,sizeof(Request0x2cmd))==0)
     {
	  fp = fopen(path2, "r");
	     while ((readstat = getline(&line, &len, fp)) != -1)
	     {
             Num = atof (line);
			 uint16_t val=20.47*Num+2047;
             uint8_t valL = (uint8_t) ((val<<4) & 0x00ff);
             uint8_t valH = (uint8_t) (((val<<4) & 0xff00) >> 8);
             char resp[4];
             resp[0]=(RCOMMAND0x02|(valH>>6));
             resp[1]=((valH<<2)|(valL>>6));
             resp[2]=(valL<<2);
             resp[3]=CRC8((((uint32_t)resp[0])<<24)|(((uint32_t)resp[1])<<16)|(((uint32_t)resp[2])<<8));
             printf("%x\n",(((uint32_t)resp[0])<<24)|(((uint32_t)resp[1])<<16)|(((uint32_t)resp[2])<<8));      
             serWrite(uart1, resp ,sizeof(resp));
             usleep(50); //will be replaced later
         }
      serClose(uart1);
      gpioTerminate();
      exit(0); 
	 }
	 else if(memcmp(RXbuf,Request0x3cmd,sizeof(Request0x3cmd))==0)
     {
	  fp = fopen(path2, "r");
	     while ((readstat = getline(&line, &len, fp)) != -1)
	     {
             Num = atof (line);
			 uint16_t val=20.47*Num+2047;
             uint8_t valL = (uint8_t) ((val<<4) & 0x00ff);
             uint8_t valH = (uint8_t) (((val<<4) & 0xff00) >> 8);
             char resp[4];
             resp[0]=(RCOMMAND0x03|(valH>>6));
             resp[1]=((valH<<2)|(valL>>6));
             resp[2]=(valL<<2);
             resp[3]=CRC8((((uint32_t)resp[0])<<24)|(((uint32_t)resp[1])<<16)|(((uint32_t)resp[2])<<8));
             printf("%x\n",(((uint32_t)resp[0])<<24)|(((uint32_t)resp[1])<<16)|(((uint32_t)resp[2])<<8));      
             serWrite(uart1, resp ,sizeof(resp));
             usleep(50); //will be replaced later
         }
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

