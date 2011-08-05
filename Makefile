#
ARCH=arm
#DIR=/home/jakub/Software/OpenWrt-SDK-at91-for-Linux-i686/staging_dir/toolchain-arm_gcc4.1.2/bin
# CC =  $(DIR)/bin/arm-linux-gcc
#CROSS_COMPILE=/home/mumin/Software/OpenWrt-SDK-at91-for-Linux-i686/staging_dir/toolchain-arm_gcc4.1.2/bin/arm-linux-
# obj-m := igloo.kmod.o
obj-m := sld.o
KDIR := /home/mumin/Software/linux-2.6.29.3-MMnet1000/
PWD := $(shell pwd)

# export PATH=$PATH:/home/mumin/Software/OpenWrt-SDK-at91-for-Linux-i686/staging_dir/toolchain-arm_gcc4.1.2/bin

all : jhd162a

jhd162a :
	make -C $(KDIR) M=$(PWD) modules

clean :
	$(MAKE) -C $(KDIR) M=$(PWD) clean
