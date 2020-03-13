rule_file=/etc/udev/rules.d/50-walker_usb_port.rules
if [ ! -f "$rule_file" ]; then
    echo "$rule_file does not exist, create a new file automatically."
    sudo bash -c 'echo "SUBSYSTEMS==\"usb\", ATTRS{idProduct}==\"2303\", ATTRS{manufacturer}==\"Prolific Technology Inc.\", ATTRS{version}==\" 2.00\", MODE=\"0777\", SYMLINK+=\"walker_motor_left\"" >> /etc/udev/rules.d/50-walker_usb_port.rules'
    sudo bash -c 'echo "SUBSYSTEMS==\"usb\", ATTRS{idProduct}==\"2303\", ATTRS{manufacturer}==\"Prolific Technology Inc.\", ATTRS{version}==\" 1.10\", MODE=\"0777\", SYMLINK+=\"walker_motor_right\"" >> /etc/udev/rules.d/50-walker_usb_port.rules'
    sudo bash -c 'echo "SUBSYSTEMS==\"usb\", ATTRS{idProduct}==\"6001\", ATTRS{manufacturer}==\"FTDI\", ATTRS{idVendor}==\"0403\", MODE=\"0666\", SYMLINK+=\"walker_force_sensor\"" >> /etc/udev/rules.d/50-walker_usb_port.rules'
fi

sudo udevadm trigger
#sudo service udev restart
