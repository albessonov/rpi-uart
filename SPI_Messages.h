#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>

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
#define SOURCEID0x00 0b00010000
#define SOURCEID0x01 0b00110000
#define SOURCEID0x02 0b01010000
#define SOURCEID0x03 0b10010000

#define CRC_POLY  0x12F
#define SPI_DATA_LEN_BITS 32
#define CRC_SPI_SEED 0xFF
#define CRC_LEN 8
#define MSG_LEN 32
#define CRC_SEED 0xFF
#define MAX_CRC 0xFF
#define MAX_UINT64_T 0xFFFFFFFFFFFFFFFF
#define UART_INPUT_MAX_SIZE  4
uint8_t RXbuf[UART_INPUT_MAX_SIZE];
FILE *fp0x0,*fp0x1,*fp0x2,*fp0x3;
char *line = NULL;
size_t len = 0;
ssize_t readstat;
const char path1[]="/home/albessonov/accelerometer/ACC_X_05.txt";
const char path2[]="/home/albessonov/accelerometer/ACC_Y_05.txt";
uint8_t RCOMMAND0x00=0b10000100;
uint8_t RCOMMAND0x01=0b11000100;
uint8_t RCOMMAND0x02=0b10100100;
uint8_t RCOMMAND0x03=0b11100100;
long int ctr0=0,ctr1=0,ctr2=0,ctr3=0;
char *ach0,*ach1,*ach2,*ach3,*end0,*end1,*end2,*end3;
long int mov0=0,mov1=0,mov2=0,mov3=0,numend0=0,numend1=0,numend2=0,numend3=0;
int serialDataAvail(const int fd);
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
 int serialOpen (const char *device, const int baud)
{
  struct termios options ;
  speed_t myBaud ;
  int     status, fd ;

  switch (baud)
  {
    case      50:	myBaud =      B50 ; break ;
    case      75:	myBaud =      B75 ; break ;
    case     110:	myBaud =     B110 ; break ;
    case     134:	myBaud =     B134 ; break ;
    case     150:	myBaud =     B150 ; break ;
    case     200:	myBaud =     B200 ; break ;
    case     300:	myBaud =     B300 ; break ;
    case     600:	myBaud =     B600 ; break ;
    case    1200:	myBaud =    B1200 ; break ;
    case    1800:	myBaud =    B1800 ; break ;
    case    2400:	myBaud =    B2400 ; break ;
    case    4800:	myBaud =    B4800 ; break ;
    case    9600:	myBaud =    B9600 ; break ;
    case   19200:	myBaud =   B19200 ; break ;
    case   38400:	myBaud =   B38400 ; break ;
    case   57600:	myBaud =   B57600 ; break ;
    case  115200:	myBaud =  B115200 ; break ;
    case  230400:	myBaud =  B230400 ; break ;
    case  460800:	myBaud =  B460800 ; break ;
    case  500000:	myBaud =  B500000 ; break ;
    case  576000:	myBaud =  B576000 ; break ;
    case  921600:	myBaud =  B921600 ; break ;
    case 1000000:	myBaud = B1000000 ; break ;
    case 1152000:	myBaud = B1152000 ; break ;
    case 1500000:	myBaud = B1500000 ; break ;
    case 2000000:	myBaud = B2000000 ; break ;
    case 2500000:	myBaud = B2500000 ; break ;
    case 3000000:	myBaud = B3000000 ; break ;
    case 3500000:	myBaud = B3500000 ; break ;
    case 4000000:	myBaud = B4000000 ; break ;

    default:
      return -2 ;
  }

  if ((fd = open (device, O_RDWR | O_NOCTTY | O_NDELAY | O_NONBLOCK)) == -1)
    return -1 ;

  fcntl (fd, F_SETFL, O_RDWR) ;

// Get and modify current options:

  tcgetattr (fd, &options) ;

    cfmakeraw   (&options) ;
    cfsetispeed (&options, myBaud) ;
    cfsetospeed (&options, myBaud) ;

    options.c_cflag |= (CLOCAL | CREAD) ;
    options.c_cflag &= ~PARENB ;
    options.c_cflag &= ~CSTOPB ;
    options.c_cflag &= ~CSIZE ;
    options.c_cflag |= CS8 ;
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG) ;
    options.c_oflag &= ~OPOST ;

    options.c_cc [VMIN]  =   0 ;
    options.c_cc [VTIME] = 100 ;	// Ten seconds (100 deciseconds)

  tcsetattr (fd, TCSANOW, &options) ;

  ioctl (fd, TIOCMGET, &status);

  status |= TIOCM_DTR ;
  status |= TIOCM_RTS ;

  ioctl (fd, TIOCMSET, &status);

  usleep (10000) ;	// 10mS

  return fd ;
}
int serialDataAvail(const int fd)
{
int result;
if(ioctl(fd,FIONREAD, &result)==-1)
   return -1;

return result;
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
