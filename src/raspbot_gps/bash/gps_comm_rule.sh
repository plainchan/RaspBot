
echo  'KERNEL=="ttyACM*", ATTRS{idVendor}=="1546", ATTRS{idProduct}=="01a9", MODE:="0777", GROUP:="dialout",  SYMLINK+="raspbot_gps_com_port"' \
>/etc/udev/rules.d/raspbot_gps_com_port.rules

service udev reload
sleep 2
service udev restart

echo "\nls -l /dev |grep ttyACM*"
ls -l /dev |grep ttyACM*
echo "please replug"