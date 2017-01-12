#!/bin/bash

LPRF_DIR=$(dirname "$0")

pushd ${LPRF_DIR} > /dev/null

if ifconfig | grep "wpan0" &> /dev/null
then
	echo "Deactivate WPAN interface..."
	ip link set wpan0 down
fi

if dtoverlay -l | grep "spi" &> /dev/null
then
	echo "Remove spi device tree overlay from kernel..."
	dtoverlay -r spi
fi

if ls | grep "lprf.dtbo" &> /dev/null && [ lprf-overlay.dts -ot lprf.dtbo ]
then
	echo "Device tree file binary already up to date."
else
	echo "Compile LPRF device tree file..."
	dtc -@ -I dts -O dtb -o lprf.dtbo lprf-overlay.dts
	
	if dtoverlay -l | grep "lprf" &> /dev/null
	then
		echo "remove LPRF device tree file from kernel..."
		dtoverlay -r lprf
	fi
fi

if dtoverlay -l | grep "lprf" &> /dev/null
then
	echo "lprf chip is already in device tree."
else
	echo "Load LPRF device tree file into kernel..."
	dtoverlay lprf.dtbo
fi

if ls | grep "lprf.ko" &> /dev/null && 
		[ lprf.c -ot lprf.ko ] &&
		[ lprf.h -ot lprf.ko ] &&
		[ lprf_registers.h -ot lprf.ko ]
then
	echo "Lprf kernel module already up to date."
else
	echo "Compile LPRF kernel module..."
	make
fi
	

if lsmod | grep "regmap_spi" &> /dev/null
then
	echo "regmap_spi module is already loaded."
else
	echo "Load spi_regmap module into kernel..."
	insmod /home/pi/kernel/linux/drivers/base/regmap/regmap-spi.ko
fi

if ls /dev/ | grep "lprf" &> /dev/null
then
	echo "Remove LPRF device file /dev/lprf..."
	rm -f /dev/lprf
fi

if lsmod | grep "lprf" &> /dev/null
then
	echo "Remove LPRF module from kernel..."
	rmmod lprf
fi

echo "Insert LPRF module into kernel..."
insmod lprf.ko

echo "Create device file /dev/lprf..."
major=$(awk "\$2==\"lprf\" {print \$1}" /proc/devices)
mknod /dev/lprf c $major 0

echo "Activate WPAN interface..."
sudo iwpan dev wpan0 set pan_id 0xdead
sudo iwpan dev wpan0 set short_addr 0xbeef
ip link set wpan0 up

popd > /dev/null
