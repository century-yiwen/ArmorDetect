#include "UART.h"

int get_fd() {
	int fd;
    fd = open("/dev/ttyUSB0", O_RDWR);
	//int flags=fcntl(fd,F_GETFL,0);
	//fcntl(fd,F_GETFL,flags|O_NONBLOCK);
	if (-1 == fd) {
        perror("open ttyUSB0");
		return 0;
	}
	else return fd;
}
int Init_Uart()
{
	int fd;
	fd = get_fd();
	printf("open success...\n");

	struct termios opt;
	//get set
	tcgetattr(fd, &opt);
	tcflush(fd, TCIOFLUSH);
	//set baud rate
	cfsetispeed(&opt, B115200);
	cfsetospeed(&opt, B115200);
	//set data bit
	opt.c_cflag &= ~CSIZE;
	opt.c_cflag |= CS8;
	//set stop bit : 1 bit
	opt.c_cflag &= ~CSTOPB;
	//set device
	if (0 != tcsetattr(fd, TCSANOW, &opt)) {
		perror("tcsetattr");
		return -1;
	}
	return fd;
}
void UART_SendData(int fd, unsigned int X, unsigned int Y, int Yes)
{

	char order[8];
	order[0] = 0xEF;
	order[1] = 0xED;
    order[2] = (unsigned char)(X >> 8);
    order[3] = (unsigned char)(X);
    order[4] = (unsigned char)(Y >> 8);
    order[5] = (unsigned char)(Y);
    order[6] = (unsigned char)(X | Y) & 0xFF;
    order[7] = '>';
	write(fd, order, 8);
}
char UART_GetData(int fd)
{
	char data;
	read(fd, &data, 1);
	//  printf("%c\n",data);
	return data;
}

void Sendata(int x,int y)
{
	int fd;
	fd = Init_Uart();
    UART_SendData(fd, x, y, 1);
	close(fd);

}
