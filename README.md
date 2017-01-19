# The LPRF Chip
The LPRF Chip is a low power transceiver chip designed at the institute for Integrated Analog Circuits and RF Systems (IAS) at RWTH Aachen university. To make this chip compatible with the IEEE 802.15.4 standard the MAC layer code in the linux network stack is used. This repository implements a linux device driver as an interface between the LPRF chip and the IEEE 802.15.4 network stack of the linux kernel.

# System Setup and Installation Guide
This driver was tested on a Raspberry Pi model B+ with a self compiled kernel version 4.4.30+.

## Prepare the SD Card for Raspberry Pi
### Download and unzip latest raspbian image
Make sure the link and the file name are correct. You can do this also manually with the browser.
```
wget https://downloads.raspberrypi.org/raspbian_latest
unzip raspbian_latest
```

### Copy image to SD-card
Replace sdf with the actual device name found by `df -h`. Make sure to unmount all partitions found by `df -h`. To identify the SD-card you can run this command twice, with and without the SD-card. Also replace the filename of the image file. The unzip-command will show you the correct file name.
```
df -h
umount /dev/sdf1
umount /dev/sdf2
sudo dd bs=4M if=2017-01-11-raspbian-jessie.img of=/dev/sdf
sync
```
Copying the image file to the SD card may take a while. To see the progress you can type `sudo pkill -USR1 -n -x dd` in another terminal window. The progress will be shown in the original window.

## Configure Raspberry Pi
Insert the SD-card and start the Raspberry Pi. Log in as the default user "pi" with password "raspberry". 
### Configuration Tool
Run `sudo raspi-config` in a terminal to start the configuration tool. Then change the following settings:
- Expand File System
- Boot Options -> B1 Desktop / CLI -> B1 Console
- Boot Options -> B3 Splash Screen -> Deactivate Splash Screen
- Optionally adjust keyboard layout, time zone in Localisation Options or Internationalization
- Interfacing Options or Advanced Options -> SSH -> Yes

Then reboot the Raspberrry Pi 
```
sudo reboot
```

### Change network settings
Type `sudo nano /etc/network/interfaces` and change the file to:
```
auto lo
iface lo inet loopback

auto eth0
iface eth0 inet dhcp
hwaddress ether 02:E4:73:03:BE:83
```
As long as you are able to connect to the Raspberry Pi via SSH you migth use the default network setting as well. After changing the settings reboot the Raspberry Pi.
```
sudo reboot
```

### Remote access
You should now be able to connect to the Raspberry Pi via ssh:
```
ssh pi@137.226.200.211
```

If you want to access the file system of the Raspberry Pi from the host computer you can do this with sshfs:
```
mkdir pi
sshfs pi@137.226.200.211: pi
```

### Update the Raspberry Pi
```
sudo apt-get update
sudo apt-get upgrade
```

## Compiling the linux kernel
### Download the linux source tree
```
mkdir kernel
cd kernel
git clone --depth=1 https://github.com/raspberrypi/linux.git
cd linux
```

### Prepare kernel compiling 
```
sudo apt-get install bc libncurses5-dev libncursesw5-dev
```
For Raspberry 1:
```
KERNEL=kernel
make bcmrpi_defconfig
```
For Raspberry 2/3
```
KERNEL=kernel7
make bcm2709_defconfig
```
Open the kernel configuration Tool.
```
make menuconfig
```
In the configuration tool adjust the following settings:

- Device Drivers -> Network device support -> USB Network Adapters -> Multi-
  purpose USB Networking Framework -> SMSC LAN95XX based USB 2.0 10/100
  ethernet devices <*>
- Device Drivers -> SPI support -> BCM2835 SPI controller <*>
- Device Drivers -> USB support -> USB Mass Storage support <*>
- Device Drivers -> USB support -> DesignWare USB2 DRD Core Support <*>
- Networking support -> RF switch subsystem support <*>
- Networking support -> Wireless -> cfg80211 – wireless configuration API <*>
- Networking support -> Wireless -> Generic IEEE 802.11 Networking Stack
  (mac80211) <*>
- Networking support -> Networking Options -> TCP/IP networking -> The IPv6
  protocol <*>
- Networking support -> Networking Options -> 6LoWPAN Support <*>
- Networking support -> Networking Options -> IEEE Std 802.15.4 Low-Rate Wireless
  Personal Area Networks support <*>
- Networking support -> Networking Options -> IEEE Std 802.15.4 Low-Rate Wireless
  Personal Area Networks support -> IEEE 802.15.4 socket interface <*>
- Networking support -> Networking Options -> IEEE Std 802.15.4 Low-Rate Wireless
  Personal Area Networks support -> 6lowpan support over IEEE 802.15.4 <*>
- Networking support -> Networking Options -> IEEE Std 802.15.4 Low-Rate Wireless
  Personal Area Networks support -> Generic IEEE 802.15.4 Soft Networking Stack
  (mac802154) <*>
- Networking support -> Networking Options -> NETLINK: mmaped IO <*>
- Networking support -> Networking Options -> NETLINK: socket monitoring interface
  <*>
- Device Drivers -> Network device support -> Wireless LAN -> Realtek 8192C USB WiFi <*>
- Device Drivers -> Network Device Support -> IEEE 802.15.4 drivers ->
  AT86RF230/231/233/212 transceiver driver \<M>

### Compiling the kernel
```
make -j4 zImage modules dtbs
```
Takes almost 12 hours

### Install Modules and copy files in the boot folder
```
sudo make modules_install
sudo cp arch/arm/boot/dts/*.dtb /boot/
sudo cp arch/arm/boot/dts/overlays/*.dtb* /boot/overlays/
sudo cp arch/arm/boot/dts/overlays/README /boot/overlays/
sudo scripts/mkknlimg arch/arm/boot/zImage /boot/$KERNEL.img
sudo reboot
```

The system should now be set up correctly and the driver can be downloaded and used.

# Use the LPRF driver

For the driver to work the chip must be connected to the Raspberry Pi correctly via SPI.
```
Pin 19 -- MOSI
Pin 21 -- MISO
Pin 23 -- SCK
Pin 24 -- SELN
```

## Build and install the WPAN tools
```
cd
sudo apt-get install libnl-3-dev libnl-genl-3-dev
sudo apt-get install dh-autoreconf
git clone https://github.com/linux-wpan/wpan-tools
cd wpan-tools
./autogen.sh
./configure CFLAGS='-g -O0' --prefix=/usr --sysconfdir=/etc --libdir=/usr/lib
make
sudo make install
```

## Download the source code
```
cd
git clone https://github.com/namlit/lprf_linux_driver.git
cd lprf_linux_driver
```

## Compile and load driver
The source code contains a bash script to handle all the compilation and configuration automatically. Note that you need to modifiy the makefile first if you downloaded the linux source to a different directory than described above. Execute the following script to set everything up:
```
sudo ./load_lprf_rx_driver.sh
```

The Driver should now be compiled and loaded into the kernel. The WPAN interface should be up and ready to be used. If you want to use the normal SPI interface that can be accessed from user space without using the driver you can execute the following script:
```
sudo ./activate_default_spi.sh
```

## Use the WPAN interface
To show the capabilities of the chip and the connected interfaces use the following commands:
```
iwpan list
iwpan dev
```

To set the channel of the chip use
```
sudo iwpan phy phy0 set channel <channel_page> <channel_number>
```
Make sure to replace "phy0" with the correct device name determined by the above commands.

To test the chip you can use the wpan-ping command
```
wpan-ping -a 0xbeef -s 100 -c 5
```

## Sniffing with Wireshark
You can use the chip in monitor mode and watch the received packets via wireshark. Therefore you need to install wireshark on your host PC:
```
sudo apt-get install wireshark
```

On the Raspberry Pi you need to install tshark, which is a text based version of wireshark:
```
sudo apt-get install tshark
```

To set up the monitor interface after the driver has been successfully loaded you can use the following script:
```
sudo ./monitor_interface.sh
```

Now start wireshark on your host computer with the following command:
```
ssh pi@137.226.200.211 'tshark -i monitor0 -F pcap -w -' | wireshark -k -i -
```

This will start tshark on the Raspberry Pi via SSH. The output of tshark is redirected to the host computer using a pipe and serves as input for wireshark.

## Debugging via char driver interface
Kernel Modules can implement a char driver interface that enables user space programs to write data to and read from a driver by accessing a device file. For proper function of the LPRF chip with the IEEE 802.15.4 stack a char driver interface is not needed. However, this driver implements this interface to get the possibility of reading and writing raw data without using the IEEE 802.15.4 stack for debugging purposes.

### Reading raw data
To read raw data as it comes from the chip you can just read from /dev/lprf. One way to do this is to use the hexdump tool xxd:
```
xxd -b -c 8 /dev/lprf
```
For more info type `xxd -h`.

### Writing raw data
You can send raw data by writing data to /dev/lprf. Note that the driver will still append the IEEE 802.15.4 synchronization header and physical header. For writing to a device file you usually need administrator writes. To write some data N times to the chip you can use the following script.
```
sudo python3 write_to_char_driver.py -n <N>
```
For more information about this script you can type `python3 write_to_char_driver.py -h`.

## List of manual commands
In the following there are some commands listed, that can be used instead of the automatic configuration script.

### Device Tree Overlay
As SPI is not a hotplugging interface the chip must be listed in the linux device tree before it can be used. This can be done with using device tree overlays.
```
dtc -@ -I dts -O dtb -o lprf.dtbo workspace/lprf_driver/lprf-overlay.dts
dtc -I fs /sys/firmware/devicetree/base > device_tree.txt
sudo dtoverlay ./lprf.dtbo
sudo dtoverlay -r lprf
dtoverlay -l
```

### Kernel Module
```
sudo insmod /home/pi/kernel/linux/drivers/base/regmap/regmap-spi.ko
sudo insmod /home/pi/workspace/lprf_driver_rx/lprf.ko
sudo rmmod lprf
lsmod
```

### Char Driver

```
major=$(awk "\$2==\"lprf\" {print \$1}" /proc/devices)
sudo rm -f /dev/lprf
sudo mknod /dev/lprf c $major 0
xxd -b -c 8 /dev/lprf
```

### WPAN
```
sudo ip link set wpan0 down
sudo iwpan dev wpan0 set pan_id 0xdead
sudo iwpan dev wpan0 set short_addr 0xbeef
sudo ip link set wpan0 up
```

# Setup Eclipse

When working with the source code it is worth it to setup an integrated development environment like eclipse. This enables a lot of features like jumping to function declarations and implementations somethere in the linux kernel code. This Guide assumes that the file system of the Raspberry Pi is mounted to a Linux host computer via SSH as described above. Eclipse will run on the host PC and access the files on the Raspberry Pi. Note that the actual compiling still needs to be done on the Raspberry Pi.


## Install Eclipse
```
sudo apt-get install eclipse eclipse-cdt g++
```
## Add new Project
- Select a new workspace on startup. (e.g. ~/eclipse_workspaces/lprf_linux_driver)
- Window -> Open Perspective -> C/C++
- File -> New -> Makefile Project with existing code
  - Project Name: LPRF_Linux_Driver
  - Existing Code Location: ~/pi/lprf_linux_driver
  - Toolchains for indexer settings: none
- Window -> Show View -> Project Explorer (if not already there)
- Rigth click on Project in Project Explorer -> Properties
  - C/C++ General -> Paths and Symbols -> Source Location -> Link Folder...
    - Link to folder in the file system
    - Browse... -> Folder to Linux Kernel source (e.g. ~/pi/kernel/linux)

After applying the changes Eclipse will start the indexer to build the indices. The progress should be displayed in the status bar on the bottom rigth. For the hole linux kernel source code this migth take a while.
