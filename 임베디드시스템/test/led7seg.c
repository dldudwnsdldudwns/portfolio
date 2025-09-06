#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <stdint.h>

int main(int argc,char **argv){
    int ret;
    int fd;
    int8_t bval;

    bval = (int8_t) atoi(argv[1]);

    fd= open("/dev/led7seg",O_WRONLY);
    ret = write(fd,&bval,sizeof(bval));

    close(fd);

    return 0;
    }