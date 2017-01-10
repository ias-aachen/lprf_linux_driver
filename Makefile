obj-m += lprf.o

KERNELDIR ?= /home/pi/kernel/linux

all:
	make -C $(KERNELDIR) M=$(PWD) LDDINC=$(PWD)/../include modules

clean:
	make -C $(KERNELDIR) M=$(PWD) clean


