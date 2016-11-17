#!/bin/bash

LPRF_DIR="/home/pi/workspace/lprf_driver_rx"

if dtoverlay -l | grep "lprf" &> /dev/null
then
	echo "lprf chip is already in device tree. Nothing to do."
else
	echo "lprf chip device tree file will be loaded..."
	dtoverlay $LPRF_DIR/lprf.dtbo
fi


if lsmod | grep "regmap_spi" &> /dev/null
then
	echo "regmap_spi module is already loaded. Nothing to do."
else
	echo "regmap_spi module will be loaded..."
	insmod /home/pi/kernel/linux/drivers/base/regmap/regmap-spi.ko
fi

if ls /dev/ | grep "lprf" &> /dev/null
then
	echo "Device file /dev/lprf exists. Device file will be removed ..."
	rm -f /dev/lprf
fi

if lsmod | grep "lprf" &> /dev/null
then
	echo "lprf module is loaded. Module will be removed..."
	rmmod lprf_rx
fi

echo "lprf module will be inserted..."
insmod $LPRF_DIR/lprf_rx.ko

echo "device file /dev/lprf will be created..."
major=$(awk "\$2==\"lprf_rx\" {print \$1}" /proc/devices)
mknod /dev/lprf c $major 0
