# echo  'KERNELS=="1-2", KERNEL=="ttyUSB*", MODE:="0777", SYMLINK+="imu"' >/etc/udev/rules.d/tty_imu.rules
echo  'KERNEL=="ttyUSB*", ATTRS{idVendor}=="0403", ATTRS{idProduct}=="6001", MODE:="0777", GROUP:="dialout",  SYMLINK+="tty_mecbot"' >/etc/udev/rules.d/tty_mecbot.rules
    
service udev reload
    
sleep 2
    
service udev restart


