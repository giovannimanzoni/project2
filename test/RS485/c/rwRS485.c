/*

setup OS serial connection with 9600 8N1, Flow control Xon/Xoff

*/

#include "stdio.h"
#include "string.h"
#include "unistd.h"
#include "fcntl.h"
#include "errno.h"
#include "sys/types.h"
#include "sys/stat.h"
#include "stdlib.h"
#include "stdarg.h"
#include "termios.h"
#include "linux/serial.h"

/* RS485 ioctls: */
#define TIOCGRS485      0x542E
#define TIOCSRS485      0x542F


// http://stackoverflow.com/questions/12531289/serial-programming-rs485

int main(void) {
	char txBuffer[50];
	char rxBuffer[50];
	int fd;
	struct termios tty_attributes;
	struct serial_rs485 rs485conf;
	int len=0;
	ssize_t n = 0, spot = 0;
	char buf = '\0';

	if ((fd = open("/dev/ttyS1",O_RDWR|O_NOCTTY | O_NDELAY ))<0) {
        	fprintf (stderr,"Open error on %s\n", strerror(errno));
        	exit(EXIT_FAILURE);
	}

	fcntl(fd, F_SETFL, 0);

	if(tcgetattr(fd, &tty_attributes) != 0) return -1;//"Getting the parameters failed.";

        // Set the baud rate
    	if(cfsetispeed(&tty_attributes, B9600) != 0 || cfsetospeed(&tty_attributes, B9600) != 0) return -2; //"Setting the baud rate failed.";

        // c_cflag
        // Enable receiver
        tty_attributes.c_cflag |= CREAD;
	tty_attributes.c_cflag  = (tty_attributes.c_cflag & ~CSIZE) | CS8;    //8-bit characters
	tty_attributes.c_cflag |= (CLOCAL | CREAD);// und erlaubt 'Lesen'.
	tty_attributes.c_cflag &= ~(PARENB | PARODD);
	tty_attributes.c_cflag &= ~CSTOPB;
	tty_attributes.c_cflag &= ~CRTSCTS;

        // c_iflag
        // Ignore framing errors and parity errors.
	tty_attributes.c_iflag     &= ~IGNBRK;
	tty_attributes.c_iflag &= ~(IXON | IXOFF | IXANY);

	//Local Flags
	tty_attributes.c_lflag  = 0;

	//Output Flags
	tty_attributes.c_oflag  = 0;

        // Minimum number of characters for non-canonical read.
        tty_attributes.c_cc[VMIN]=1;

        // Timeout in deciseconds for non-canonical read.
        tty_attributes.c_cc[VTIME]=0;

	/* Flush Port, then applies attributes */
	tcflush( fd, TCIFLUSH );

//        tcsetattr(fd, TCSANOW, &tty_attributes);
	if(tcsetattr(fd, TCSAFLUSH, &tty_attributes) != 0) return -3; //"Setting the new parameters failed";


	/* Don't forget to read first the current state of the RS-485 options with ioctl.
            If You don't do this, You will destroy the rs485conf.delay_rts_last_char_tx
            parameter which is automatically calculated by the driver when You opens the
            port device. */
         if (ioctl (fd, TIOCGRS485, &rs485conf) < 0) {
                 printf("Error: TIOCGRS485 ioctl not supported.\n");
         }

        // Set RS485 mode:
        rs485conf.flags |= SER_RS485_ENABLED;

	/* Set logical level for RTS pin equal to 1 when sending: */
	rs485conf.flags |= SER_RS485_RTS_ON_SEND;

	/* Set logical level for RTS pin equal to 0 after sending: */
	rs485conf.flags &= ~(SER_RS485_RTS_AFTER_SEND);

	/* Set rts/txen delay before send, if needed: (in microseconds) */
        rs485conf.delay_rts_before_send = 0;

        /* Set rts/txen delay after send, if needed: (in microseconds) */
        rs485conf.delay_rts_after_send = 0;

        if (ioctl (fd, TIOCSRS485, &rs485conf) < 0) {
           printf("ioctl error\n");
        }

        //***************************//
	//   TEST WRITING TO RS485
        //***************************//
	// ENABLE FOR TEST
/*
	printf("TEST WRITING TO RS485\n\r");
        txBuffer[0]='A';
        for(;;){
		if(write(fd,txBuffer,1)!=1) printf("error write\n\r");
	}
*/
        //***************************//
	//   TEST READING FROM RS485
        //***************************//
	// ENABLE FOR TEST


	printf("TEST READING FROM RS485\n\r");
	for(;;){
		n = 0; spot = 0;
		buf = '\0';
		memset(rxBuffer, 0, sizeof(rxBuffer));

		do {
			n = read( fd, &buf, 1 );
			sprintf( &rxBuffer[spot], "%c", buf );
			spot += n;
			printf("chr: %c\n\r",buf);
		} while( (buf != '\r')  && ( n > 0));
		printf("Mex: %s\n\r",rxBuffer);
	}

	close(fd);
	return EXIT_SUCCESS;
}
