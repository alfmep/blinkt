
LINUX_SRC ?= /lib/modules/$(shell uname -r)/build

PWD := $(shell pwd)

# Devicetree overlay to be built
dtb-y += blinkt.dtbo

# Kernel module to be built
obj-m := blinkt.o


modules:
	$(MAKE) -C $(LINUX_SRC) M=$(PWD) modules

modules_install:
	$(MAKE) -C $(LINUX_SRC) M=$(PWD) modules_install

clean:
	$(MAKE) -C $(LINUX_SRC) M=$(PWD) clean

help:
	$(MAKE) -C $(LINUX_SRC) M=$(PWD) help
