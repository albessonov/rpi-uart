#include <stdint.h>
#define REGISTER_READ_COMMAND 0b11000000
#define REGISTER_WRITE_COMMAND 0b10000000
#define REGISTER_READ_ADDRESS 0b01010101 //изменить!!
#define REGISTER_READ_DATA 0b11111111
#define ST 0b01
#define REGISTER_DATA_0 0b00000000
#define CRC_0 0b00000000
#define REGISTER_READ_RESPONSE 0b0110
#define REGISTER_WRITE_RESPONSE 0b0100
#define UD 0b00
#define SOURCEID0x00 0b00000000
#define SOURCEID0x01 0b00010000
#define SOURCEID0x02 0b00100000
#define SOURCEID0x03 0b00110000

#define CRC_POLY  0x12F
#define SPI_DATA_LEN_BITS 32
#define CRC_SPI_SEED 0xFF
#define CRC_LEN 8
#define MSG_LEN 32
#define CRC_SEED 0xFF
#define MAX_CRC 0xFF
#define MAX_UINT64_T 0xFFFFFFFFFFFFFFFF
int list2int(int *list, int n) ;    
static uint8_t CRC8(uint32_t SPI_data);
struct Register_Access_Command
  {
	  uint8_t Command__Fixed_Bits;
	  uint8_t Register_Address;
	  uint8_t Register_Data;
	  uint8_t CRC;
  };
struct Register_Response
  {
	  uint8_t Command_BS_UD;
	  uint8_t Register_Data_H;
	  uint8_t Register_Data_L;
	  uint8_t CRC;
  };  
struct Sensor_Data_Request
 {
          uint8_t Command__Fixed_Bits_0;
          uint8_t Fixed_Bits_1;
	  uint8_t Fixed_Bits_2;
	  uint8_t CRC;
     
 };
