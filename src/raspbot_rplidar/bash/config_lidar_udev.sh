#!/bin/bash

echo "----------serial port(ttyUSBX) list-----------------"
lsusb

echo "----------serial port(ttyUSBX) authority-----------------"
ls -l /dev/ttyUSB*
# udevadm info --attribute-walk --name=/dev/ttyUSB0

echo 'KERNEL=="ttyUSB*", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", MODE:="0777", GROUP:="dialout",  SYMLINK+="raspbot_rplidar_port"' \
>/etc/udev/rules.d/rplidar_udev_port.rules

echo "udev restart ..."
service udev reload
sleep 2
service udev restart
echo "udev restart sucessed"

sleep 2
echo "----------serial port(ttyUSBX) authority-----------------"
ls -l /dev |grep ttyUSB