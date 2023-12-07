#include <stdio.h>
#include <fcntl.h>
#include <string.h>
#include <unistd.h>
#include <termios.h>
#include <stdlib.h>

__uint16_t recv(int controller){
  // Button press code ideas- 
  // 0x1 - Left Button
  // 0x2 - Center Button
  // 0x3 - Right Button

  // Also put the busy waiting here! 

  // if there's no response, just return FN_RET_NULL
    char *port = malloc(13);
    char *port_use = port;
    __uint8_t msb;
    __uint8_t lsb;
    __uint16_t output = -1;
    switch(controller)
    {
        case 0:
        case 1: 
        port_use = "/dev/ttyACM0";
        break;
        case 2: 
        port_use = "/dev/ttyACM1";
        break;
        case 3:
        port_use = "/dev/ttyACM2";
        break;
        case 4:
        port_use = "/dev/ttyACM3";
        break;
        default: port_use = "err";
    }
    if(port_use == "err")
    {
        printf("[RECV] Wrong Controller Input\n");
        free(port);
        return 0x4;
    }

    int serial_port = open(port_use, O_RDWR);
    // printf("%d\n", serial_port);
    if(serial_port < 0)
    {
        printf("[RECV] Error opening serial port. \n");
    }


    struct termios options;
    tcgetattr(serial_port, &options);
    options.c_iflag &= ~(INLCR | IGNCR | ICRNL | IXON | IXOFF);
    options.c_oflag &= ~(ONLCR | OCRNL);
    options.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
    tcsetattr(serial_port, TCSANOW, &options);
    struct termios tty;

    memset(&tty, 0, sizeof(tty));

    if(tcgetattr(serial_port,&tty) != 0)
    {
        printf("[RECV] Error getting serial port attributes 3\n");
    }
    cfsetospeed(&tty,B1152000);
    cfsetispeed(&tty, B1152000);

    tty.c_cflag != (CLOCAL | CREAD);
    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag != CS8;

    if (tcsetattr(serial_port,TCSANOW,&tty) != 0)
    {
        printf("[RECV] Error setting serial port attributes \n");
        free(port);
        return 0x4; 
    }
    char readbuff[256];
    memset(&readbuff, '\0',sizeof(readbuff));
    printf("[RECV] waiting... \n");

    int num_bytes = 0;
    while(num_bytes < 2)
    {
        num_bytes = read(serial_port, &readbuff, sizeof(readbuff));
        // printf("numbytes = %d\n", num_bytes);

        if(num_bytes > 0)
        {
            printf("Some of the readbuf: %x %x %x %x %x %x (0-5)\n", readbuff[0], readbuff[1], readbuff[2], readbuff[3], readbuff[4], readbuff[5]);
            // printf("Recieved [%s]\n", readbuff);
            msb = (__uint16_t)readbuff[0]; // set MSB
            printf("msb: %x\n", msb);
            lsb = (__uint16_t)readbuff[1]; // set LSB
            printf("lsb: %x\n", lsb);
            memset(&readbuff, '\0', sizeof(readbuff)); // DELETE BUFFER
        }
    }
    __uint8_t buffer1[1] = {0xF0};
    __uint8_t buffer2[1] = {0x01};
    int bytes_written1 = write(serial_port, buffer1, 1);
    int bytes_written2 = write(serial_port, buffer2, 1);
    if(bytes_written1 + bytes_written2 != 2)
    {
        printf("Error sending ACK");
    }
    
    free(port);
    output = (msb << 8) + lsb; // set output
    return output;

}

void send(int controller, __uint16_t data){
    __uint8_t msb = data & 0xFF;
    __uint8_t lsb = data >> 8;
    char *port = malloc(13);
    char *port_use = port;
    switch(controller)
    {
        case 0:
        case 1: 
        port_use = "/dev/ttyACM0";
        break;
        case 2: 
        port_use = "/dev/ttyACM1";
        break;
        case 3:
        port_use = "/dev/ttyACM2";
        break;
        case 4:
        port_use = "/dev/ttyACM3";
        break;
        default: "err";
    }    
    if(port_use == "err")
    {
        printf("Wrong Controller Input");
        
    }
    int serial_port = open(port_use, O_RDWR);
    if(serial_port < 0)
    {
        printf("Error opening serial port. \n");
        
    }

    struct termios tty;
    memset(&tty, 0, sizeof(tty));

    if(tcgetattr(serial_port,&tty) != 0)
    {
        printf("Error getting serial port attributes 2\n");
        

    }
    cfsetospeed(&tty,B9600);
    cfsetispeed(&tty, B9600);

    tty.c_cflag != (CLOCAL | CREAD);
    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag != CS8;
    if (tcsetattr(serial_port,TCSANOW,&tty) != 0)
    {
        printf("Error setting serial port attributes \n");
        
    }
    __uint8_t buffer1[1] = {msb};
    __uint8_t buffer2[1] = {lsb};
    int bytes_written1 = write(serial_port, buffer2, 1);
    int bytes_written2 = write(serial_port, buffer1, 1);
    if(bytes_written1 + bytes_written2 != 2)
    {
        printf("Error writing to serial port");
    }
    free(port);
    close(serial_port);
}

int main()
{

    send(1, 0xFF00);
    send(1, 0x0405);
    send(1, 0xFF01);     

    return 0;
}