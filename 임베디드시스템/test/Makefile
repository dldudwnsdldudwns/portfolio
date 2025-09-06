CC := $(CROSS_COMPILE)gcc 
CFLAGS = 
LDFLAGS = -static 

all: led7seg

midterm_test: led7seg.o
	$(CC) $(LDFLAGS) -o led7seg led7seg.o

midterm_test.o: led7seg.c
	$(CC) $(CFLAGS) -o led7seg.o -c led7seg.c 

clean:
	rm led7seg.o led7seg
