KERNEL?=$(shell uname -r)
MODNAME?=imx585
obj-m := $(MODNAME).o
all:
	make -C /lib/modules/$(KERNEL)/build M=$(PWD) modules
install:
	make -C /lib/modules/$(KERNEL)/build M=$(PWD) modules_install
clean:
	rm -fR .*.cmd *.o *.ko .tmp_versions Module.symvers modules.order *.mod.c