
#include <stdio.h>
#include <string.h>
#include "SPI_Messages.h"
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <fcntl.h>
#include <sys/stat.h>
#define UART_INPUT_MAX_SIZE  4
  uint8_t RXbuf[UART_INPUT_MAX_SIZE];
uint8_t Err[]={0xFF,0xFF,0xFF,0xFF};
char OSD[]={0,0,0,0};
FILE *fp;
char *line = NULL;
size_t len = 0;
ssize_t readstat;
const char path1[]="/home/albessonov/accelerometer/ACC_X_05.txt";
const char path2[]="/home/albessonov/accelerometer/ACC_Y_05.txt";
uint8_t RCOMMAND0x00=0b10000100;
uint8_t RCOMMAND0x01=0b11000100;
uint8_t RCOMMAND0x02=0b10100100;
uint8_t RCOMMAND0x03=0b11100100;
ssize_t ctr0=0,ctr1=0,ctr2=0,ctr3=0;
char *ach0,*ach1,*ach2,*ach3,*end0,*end1,*end2,*end3;
long int mov0=0,mov1=0,mov2=0,mov3=0,numend0=0,numend1=0,numend2=0,numend3=0;
int serialDataAvail(const int fd);
int main (void)
{
double Num;

unsigned uart1=open("/dev/ttyAMA0",921600);

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

uint32_t Request0x0_32=0x10000000;
uint32_t Request0x1_32=0x10000000;
uint32_t Request0x2_32=0x50000000;
uint32_t Request0x3_32=0x50000000;

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
        memset(RXbuf, 0, UART_INPUT_MAX_SIZE);// clean the buf for next reception
        if(serialDataAvail (uart1)!=-1)
        read(uart1,RXbuf,4);
        //printf("%d\n%d\n%d\n%d\n", RXbuf[0],RXbuf[1],RXbuf[2],RXbuf[3]);


    if(memcmp(RXbuf,Register_Read_cmd,sizeof(Register_Read_cmd))==0)
     {
      write(uart1, Register_Read_Response_cmd, sizeof(Register_Read_Response_cmd));
      close(uart1);

      exit(0);
     }
     else if(memcmp(RXbuf,Register_Write_cmd,sizeof(Register_Write_cmd))==0)
     {
      write(uart1, Register_Write_Response_cmd, sizeof(Register_Write_Response_cmd));
      close(uart1);
      exit(0);
	 }
	 else if(memcmp(RXbuf,Request0x0cmd,sizeof(Request0x0cmd))==0)
     {
		 uint8_t valH,valL;
		 fp = fopen(path1, "r");
		 fseek(fp,ctr0,0);
	     readstat = getline(&line, &len, fp);
	     ach0=strchr (line,' ');
	     //circular reading
	     end0=strchr (line,'/')+1;
	     numend0=end0-line+1;
	     if(numend0<=10)
	     {
			 fseek(fp,0,0);
			 ctr0=0;
			 mov0=0;
		 }//returns the pointer to start of the reading to the beginning of the file
	     //printf("%ld\n",numend0);
	     mov0=ach0-line+1;
		 Num = atof (line);
	     int16_t val=20.47*Num;
	     if(val>=0)
	     {
          valL = (uint8_t) ((val<<4) & 0x00ff);
          valH = (uint8_t) (((val<<4) & 0xff00) >> 8);
	     }
	     else
	     {
          valL = (uint8_t) (((-val)<<4) & 0x00ff);
          valH = (uint8_t) (((((-val)<<4) & 0xff00) >> 8)|0x80);
	     }
         char resp[4];
         resp[0]=(RCOMMAND0x00|(valH>>6));
         resp[1]=((valH<<2)|(valL>>6));
         resp[2]=(valL<<2);
         resp[3]=CRC8((((uint32_t)resp[0])<<24)|(((uint32_t)resp[1])<<16)|(((uint32_t)resp[2])<<8));
	 write(uart1, resp, sizeof(resp));
         ctr0+=mov0;
         //printf("%ld\n%ld\n",ctr0,strlen(line));

	 }
	 else if(memcmp(RXbuf,Request0x1cmd,sizeof(Request0x1cmd))==0)
     {
	  uint8_t valH,valL;
	  fp = fopen(path1, "r");
	  fseek(fp,ctr1,0);
	  readstat = getline(&line, &len, fp);
	  ach1=strchr(line,' ');
	  end1=strchr (line,'/')+1;
      numend1=end1-line+1;
	     if(numend1<=10)
	     {
			 fseek(fp,0,0);
			 ctr1=0;
			 mov1=0;
		 }
	  mov1=ach1-line+1;
      Num = atof (line);
      int16_t val=20.47*Num;
   	   if(val>=0)
	     {
          valL = (uint8_t) ((val<<4) & 0x00ff);
          valH = (uint8_t) (((val<<4) & 0xff00) >> 8);
	     }
	     else
	     {
          valL = (uint8_t) (((-val)<<4) & 0x00ff);
          valH = (uint8_t) (((((-val)<<4) & 0xff00) >> 8)|0x80);
	     }
      char resp[4];
      resp[0]=(RCOMMAND0x01|(valH>>6));
      resp[1]=((valH<<2)|(valL>>6));
      resp[2]=(valL<<2);
      resp[3]=CRC8((((uint32_t)resp[0])<<24)|(((uint32_t)resp[1])<<16)|(((uint32_t)resp[2])<<8));
      write(uart1, resp, sizeof(resp));
      ctr1+=mov1;
	 }
	 else if(memcmp(RXbuf,Request0x2cmd,sizeof(Request0x2cmd))==0)
     {
	  uint8_t valH,valL;
	  fp = fopen(path2, "r");
	  fseek(fp,ctr2,0);
	  readstat = getline(&line, &len, fp);
	  ach2=strchr (line,' ');
	  end2=strchr (line,'/')+1;
	     numend2=end2-line+1;
	     if(numend2<=10)
	     {
			 fseek(fp,0,0);
			 ctr2=0;
			 mov2=0;
		 }
	  mov2=ach2-line+1;
      Num = atof (line);
	  int16_t val=20.47*Num;
       if(val>=0)
	     {
          valL = (uint8_t) ((val<<4) & 0x00ff);
          valH = (uint8_t) (((val<<4) & 0xff00) >> 8);
	     }
	     else
	     {
          valL = (uint8_t) (((-val)<<4) & 0x00ff);
          valH = (uint8_t) (((((-val)<<4) & 0xff00) >> 8)|0x80);
	     }
      char resp[4];
      resp[0]=(RCOMMAND0x02|(valH>>6));
      resp[1]=((valH<<2)|(valL>>6));
      resp[2]=(valL<<2);
      resp[3]=CRC8((((uint32_t)resp[0])<<24)|(((uint32_t)resp[1])<<16)|(((uint32_t)resp[2])<<8));
       write(uart1, resp, sizeof(resp));
       ctr2+=mov2;
     }
	 else if(memcmp(RXbuf,Request0x3cmd,sizeof(Request0x3cmd))==0)
     {
	  uint8_t valH,valL;
	  fp = fopen(path2, "r");
	  fseek(fp,ctr3,0);
	  readstat = getline(&line, &len, fp);
	  if(numend3<=10)
	     {
			 fseek(fp,0,0);
			 ctr3=0;
			 mov3=0;
		 }
	  ach3=strchr (line,' ');
	  end0=strchr (line,'/')+1;
	     numend3=end3-line+1;
	     if(numend3<=10)
	     {
			 fseek(fp,0,0);
			 ctr3=0;
			 mov3=0;
		 }
	  mov3=ach3-line+1;
      Num = atof (line);
	  int16_t val=20.47*Num;
      if(val>=0)
	     {
          valL = (uint8_t) ((val<<4) & 0x00ff);
          valH = (uint8_t) (((val<<4) & 0xff00) >> 8);
	     }
	     else
	     {
          valL = (uint8_t) (((-val)<<4) & 0x00ff);
          valH = (uint8_t) (((((-val)<<4) & 0xff00) >> 8)|0x80);
	     }
      char resp[4];
      resp[0]=(RCOMMAND0x03|(valH>>6));
      resp[1]=((valH<<2)|(valL>>6));
      resp[2]=(valL<<2);
      resp[3]=CRC8((((uint32_t)resp[0])<<24)|(((uint32_t)resp[1])<<16)|(((uint32_t)resp[2])<<8));
      write(uart1, resp, sizeof(resp));

	 }
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
int serialDataAvail(const int fd)
{
int result;
if(ioctl(fd,FIONREAD, &result)==-1)
   return -1;

return result;
}


