# Device Tree Overlay
dtc -@ -I dts -O dtb -o lprf.dtbo workspace/lprf_driver_rx/lprf-overlay.dts
dtc -I fs /sys/firmware/devicetree/base > device_tree.txt
sudo dtoverlay ./lprf.dtbo
sudo dtoverlay -r lprf
dtoverlay -l

# Kernel Module
sudo insmod /home/pi/kernel/linux/drivers/base/regmap/regmap-spi.ko
sudo insmod /home/pi/workspace/lprf_driver_rx/lprf.ko
sudo rmmod lprf
lsmod

# Char Driver
major=$(awk "\$2==\"lprf\" {print \$1}" /proc/devices)
sudo rm -f /dev/lprf
sudo mknod /dev/lprf c $major 0
xxd /dev/lprf
xxd -b /dev/lprf

# WPAN
sudo ip link set wpan0 down
sudo iwpan dev wpan0 set pan_id 0xdead
sudo iwpan dev wpan0 set short_addr 0xbeef
sudo ip link set wpan0 up
