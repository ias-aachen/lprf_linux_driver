# Device Tree Overlay
dtc -@ -I dts -O dtb -o lprf.dtbo workspace/lprf_driver_rx/lprf-overlay.dts
dtc -I fs /sys/firmware/devicetree/base > device_tree.txt
sudo dtoverlay ./lprf.dtbo
sudo dtoverlay -r lprf
dtoverlay -l

# Kernel Module
sudo insmod /home/pi/kernel/linux/drivers/base/regmap/regmap-spi.ko
sudo insmod /home/pi/workspace/lprf_driver_rx/lprf_rx.ko
sudo rmmod lprf_rx
lsmod

# Char Driver
major=$(awk "\$2==\"lprf_rx\" {print \$1}" /proc/devices)
sudo rm -f /dev/lprf
sudo mknod /dev/lprf c $major 0
