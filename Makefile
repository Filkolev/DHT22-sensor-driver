obj-m+=dht22_driver.o
dht22_driver-objs+=dht22.o dht22_sm.o

all:
	make C=2 -C /lib/modules/$(shell uname -r)/build/ M=$(PWD) modules

clean:
	make -C /lib/modules/$(shell uname -r)/build/ M=$(PWD) clean
