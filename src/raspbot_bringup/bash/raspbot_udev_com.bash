#!/bin/bash

lsusb
ls -l /dev/ttyUSB*
# udevadm info --attribute-walk --name=/dev/ttyUSB0

echo  'KERNEL=="ttyUSB*", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="7523", MODE:="0777", GROUP:="dialout",  SYMLINK+="raspbot_com_port"' >/etc/udev/rules.d/raspbot_com_port.rules

echo "udev restart ..."
service udev reload
sleep 2
service udev restart
echo "udev restart sucessed"

ls -l /dev |grep ttyUSB