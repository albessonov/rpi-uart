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
#include <math.h>

int main (void)
{
double Num;

unsigned uart1=serialOpen("/dev/ttyAMA0",921600);

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
fp0x0 = fopen(path1, "r");
if(fp0x0==NULL){printf("Error x");}

fp0x2 = fopen(path2, "r");
	  if(fp0x2==NULL)
	  printf("Error Y");
while(1)
{
    memset(RXbuf, 0, UART_INPUT_MAX_SIZE);// clean the buf for next reception
    if(serialDataAvail (uart1)!=-1)
    read(uart1,RXbuf,4);
if(memcmp(RXbuf,Register_Read_cmd,sizeof(Register_Read_cmd))==0)
     {
      write(uart1, Register_Read_Response_cmd, sizeof(Register_Read_Response_cmd));
      //close(uart1);
      //exit(0);
     }
else if(memcmp(RXbuf,Register_Write_cmd,sizeof(Register_Write_cmd))==0)
     {
      write(uart1, Register_Write_Response_cmd, sizeof(Register_Write_Response_cmd));
      //close(uart1);
      //exit(0);
	 }
else if(memcmp(RXbuf,Request0x0cmd,sizeof(Request0x0cmd))==0)
     {
		 uint8_t valH,valL;
		 if(numend0<=10)
	     {
			 fseek(fp0x0,0,0);
			 ctr0=0;
			 mov0=0;
		 }//returns the pointer to start of the reading to the beginning of the file in case of EOF is reached
	     printf("X:%d\n",fseek(fp0x0,mov0,1));
	     readstat = getline(&line, &len, fp0x0);
	     ach0=strchr (line,' ');
	     //circular reading
	     end0=strchr (line,'/')+1;
	     numend0=end0-line+1;
	     mov0=(ach0-line+1);
		 Num = atof (line);
         printf("%f\n",Num);
	     int16_t val=round(20.47*Num);
             //Num=Num*20.47;
             //int16_t val2=Num;
	     //printf("%d  %d\r\n", val, val2);
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
         //printf("%ld\n%ld\n",ctr0,numend0);

	 }
else if(memcmp(RXbuf,Request0x1cmd,sizeof(Request0x1cmd))==0)
     {
	  uint8_t valH,valL;
	  if(numend1<=10)
	     {
			 /*fseek(fp0x0,0,0);
			 ctr1=0;
			 mov1=0;*/
			 printf("EOF reached");
			 exit(0);
		 }
	  fseek(fp0x0,mov1,1);
	  readstat = getline(&line, &len, fp0x0);
	  ach1=strchr(line,' ');
	  end1=strchr (line,'/')+1;
      numend1=end1-line+1;
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
      //ctr1+=mov1;
	 }
else if(memcmp(RXbuf,Request0x2cmd,sizeof(Request0x2cmd))==0)
     {
	  uint8_t valH,valL;
	  if(numend2<=10)
	     {
			 fseek(fp0x2,0,0);
			 ctr2=0;
			 mov2=0;
		 }
	  fseek(fp0x2,mov2,1);
	  readstat = getline(&line, &len, fp0x2);
	  ach2=strchr (line,' ');
	  end2=strchr (line,'/')+1;
      numend2=end2-line+1;
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
      //ctr2+=mov2;
      //printf("%ld\n%ld\n",ctr2,numend2);

     }
else if(memcmp(RXbuf,Request0x3cmd,sizeof(Request0x3cmd))==0)
     {
	  uint8_t valH,valL;
	  if(numend3<=10)
	     {
			 fseek(fp0x1,0,0);
			 ctr3=0;
			 mov3=0;
		 }
	  fseek(fp0x1,mov3,1);
	  readstat = getline(&line, &len, fp0x1);
	  ach3=strchr (line,' ');
	  end0=strchr (line,'/')+1;
      numend3=end3-line+1;
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
      //ctr3+=mov3;
	 }
}
}




