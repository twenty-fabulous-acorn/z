ifneq ($(KERNELRELEASE),)
	obj-m	+= bmc.o
else
	KERNELDIR ?= /lib/modules/$(shell uname -r)/build/
	PWD := $(shell pwd)

all:
	@$(MAKE) -C $(KERNELDIR) M=$(PWD) modules

install:
	@$(MAKE) -C $(KERNELDIR) M=$(PWD) modules_install

clean:
	@$(MAKE) -C $(KERNELDIR) M=$(PWD) clean
endif
