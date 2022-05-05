#!/bin/bash

echo "----------serial port(ttyUSBX) list-----------------"
lsusb

echo "\n"
# echo "----------serial port(ttyUSBX) detail-----------------"
# udevadm info --attribute-walk --name=/dev/ttyUSB0

echo  'KERNEL=="ttyUSB*", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="7523", MODE:="0777", GROUP:="dialout",  SYMLINK+="raspbot_com_port"' \
>/etc/udev/rules.d/raspbot_com_port.rules

echo "\n"
echo "--------------------service reload -----------------"
echo "udev restart ..."
service udev reload
sleep 2
service udev restart
echo "udev restart sucessed"

echo "\n"
sleep 2
echo "----------serial port(ttyUSBX) authority-----------------"
ls -l /dev |grep ttyUSB

echo "\n"
echo "--------------------- notice -------------------------------"
echo "if the authority is not the same as below"
echo "******************************************************************************************"
echo "****  lrwxrwxrwx   1 root  root          ---------    raspbot_com_port -> ttyUSB*    *****"
echo "****  crwxrwxrwx   1 root  dialout       ---------    ttyUSB*                        *****"
echo "******************************************************************************************"
echo "please replug the usb port,then input 'ls -l /dev |grep ttyUSB' in the terminator"