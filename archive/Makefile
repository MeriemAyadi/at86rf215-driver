obj-m +=at86rf215.o

KDIR =/usr/src/linux-headers-4.14.98-v7+/

all:
	$(MAKE) -C $(KDIR) SUBDIRS=$(PWD) modules
clean:
	rm -rf *.o *.ko *.mod.* *.symvers *.order
