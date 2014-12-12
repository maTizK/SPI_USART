#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <stdio.h>

int main(int argc, char **argv) {

    int fd;
    int i;
    ssize_t len;
    unsigned char buf[8192];
    struct termios tio;

    fd = open( argv[1], O_RDONLY | O_NOCTTY | O_NONBLOCK );
    cfmakeraw(&tio);
    cfsetispeed(&tio,B230400);
    cfsetospeed(&tio,B230400);
    tio.c_cflag = (tio.c_cflag & ~CSIZE) | CS8; 
    tio.c_iflag &= ~IGNBRK; 
    tio.c_lflag = 0; 
    tio.c_oflag = 0;
    tio.c_iflag &= ~(IXON | IXOFF | IXANY);
    tio.c_cflag &= ~(PARENB | PARODD); 
    tio.c_cflag &= ~CSTOPB;
    tio.c_cflag &= ~CRTSCTS;
    tcsetattr(fd,TCSANOW,&tio);

    for(;;) {
        len = read( fd, &buf[0], 8192 );
        //if( len > 0 ) write(1,buf,len);
        //printf("len=%d\n", len);
        for(i=0; i<len; i++)
        {
          if (buf[i]==0x01) continue;
          printf("%c", buf[i]);
        }
        if (len > 0) printf("\n");
        sleep(1);
    }

}
