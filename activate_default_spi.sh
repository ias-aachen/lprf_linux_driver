#!/bin/bash

LPRF_DIR="/home/pi/workspace/lprf_driver_rx/"

if dtoverlay -l | grep "lprf" &> /dev/null
then
	echo "LPRF overlay loaded. LPRF overlay will be removed..."
	dtoverlay -r lprf
else
    echo "LPRF device tree overlay not loaded."
fi

if ls ${LPRF_DIR} | grep "spi.dtbo" &> /dev/null
then
	echo "Device tree file binary already exists. Nothing to do."
else
	echo "Spi device tree file will be compiled..."
	dtc -@ -I dts -O dtb -o ${LPRF_DIR}spi.dtbo ${LPRF_DIR}spi-overlay.dts
fi

if dtoverlay -l | grep "spi" &> /dev/null
then
	echo "spi is already in device tree. Nothing to do."
else
	echo "spi device tree file will be loaded..."
	dtoverlay ${LPRF_DIR}spi.dtbo
fi

