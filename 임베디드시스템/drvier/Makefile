cc = $(CROSS_COMPILE)gcc
LD = $(CROSS_COMPILE)ld
KDIR=~/linux
PWD = $(CURDIR)

obj-m += led7seg.o

all:
	make -C $(KDIR) M=$(PWD) modules

clean:
	make -C $(KDIR) M=$(PWD) clean