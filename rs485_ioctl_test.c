/*
 * Test program using ioctls to enable RS485-mode and termio to change baudrate
 *
 * Author : FDD - Allen
 *
 */

#include <stdlib.h>
#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <errno.h>
#include <string.h>
#include <linux/serial.h>
#include <termios.h>

/* RS485 ioctls: */
#define TIOCGRS485      0x542E
#define TIOCSRS485      0x542F
#define CVPLOUTINV   000100000000       /* VPL: for inverse of RTS */
const char *port = "/dev/ttyS1";
const unsigned int TEST_BAUDRATE = B115200;

int set_baudrate_through_termio(int fd);

int main( int argc, char **argv )
{
	struct serial_rs485 rs485conf;
	unsigned int i, count;
	char buf[80];

	int fd = open(port, O_RDWR);
	if (fd < 0) {
		/* Error handling. See errno. */
		fprintf( stderr, "Error opening port \"%s\" (%d): %s\n", port, errno, strerror( errno ));
		exit(-1);
	}

	/* enable RS485 though ioctl */
	rs485conf.flags |= SER_RS485_ENABLED;

	if (ioctl (fd, TIOCSRS485, &rs485conf) < 0) {
		fprintf( stderr, "Error sending ioctl port (%d): %s\n",  errno, strerror( errno ));
	}

	/* change baudrate though ioctl */
	set_baudrate_through_termio(fd);

	/* Use read() and write() syscalls here... */

	/* Read data from RS485, and write the same data to RS485 */
	/* press quit + ENTER to exit the infinite */

	while(1)
	{
		memset(buf, 0, sizeof(buf));
		count = read(fd, buf, sizeof(buf)); /* read data to RS485 */

		if(!strcmp(buf, "quit\n")) /* exit the infinite */
			break;

		snprintf(buf, sizeof(buf), "%s - %d\r\n", buf, count);
		write( fd, buf, strlen(buf)); /* send data to RS485 */
	}

	memset(buf, 0, sizeof(buf));
	strcpy(buf, "bye !");
	write( fd, buf, strlen(buf));
	
	/* Close the device when finished: */
	if (close (fd) < 0) {
		fprintf( stderr, "Error closing port (%d): %s\n", errno, strerror( errno ));
	}

	exit(0);
}

int set_baudrate_through_termio(int fd)
{
	struct termios new_termios;
	if (tcgetattr(fd, &new_termios) != 0)
	{
		fprintf(stderr, "tcgetattr(fd, &old_termios) failed: %s\n", strerror(errno));
		return 1;
    }

	new_termios.c_cflag |= CVPLOUTINV;

	/* set baudrate for input  */
	if (cfsetispeed(&new_termios, TEST_BAUDRATE) != 0) {
        fprintf(stderr, "cfsetispeed(&new_termios, B1152000) failed: %s\n", strerror(errno));
        return 1;
    }

	/* set baudrate for output  */
    if (cfsetospeed(&new_termios, TEST_BAUDRATE) != 0) {
        fprintf(stderr, "cfsetospeed(&new_termios, B1152000) failed: %s\n", strerror(errno));
        return 1;
    }

	if (tcsetattr(fd, TCSANOW, &new_termios) != 0) {
        fprintf(stderr, "tcsetattr(fd, TCSANOW, &new_termios) failed: %s\n", strerror(errno));
        return 1;
    }
}
