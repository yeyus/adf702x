# my optional module name
MODULE=m_test

# this two variables, depends where you have you raspberry kernel source and tools installed

CCPREFIX=/home/yeyus/tools/arm-bcm2708/gcc-linaro-arm-linux-gnueabihf-raspbian-x64/bin/arm-linux-gnueabihf-
KERNEL_SRC=/home/yeyus/linux


obj-m += ${MODULE}.o

module_upload=${MODULE}.ko

all: clean compile

compile:
	make ARCH=arm CROSS_COMPILE=${CCPREFIX} -C ${KERNEL_SRC} M=$(PWD) modules

clean:
	make -C ${KERNEL_SRC} M=$(PWD) clean


# this just copies a file to raspberry
install:
	scp ${module_upload} pi@192.168.1.128:/home/pi/modules

info:
	modinfo  ${module_upload}
