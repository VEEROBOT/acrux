#!/bin/bash

# Usage : source install.sh

# # Install udev rules
RULE_CONTENT=$(cat <<EOL
SUBSYSTEM=="tty", KERNELS=="1-2", SYMLINK+="esp"
SUBSYSTEM=="tty", KERNELS=="1-3", SYMLINK+="lidar"
KERNEL=="event*", SUBSYSTEM=="input", ATTRS{idVendor}=="045e", ATTRS{idProduct}=="028e", SYMLINK+="/input/haptics"
KERNEL=="js*", SUBSYSTEM=="input", ATTRS{idVendor}=="045e", ATTRS{idProduct}=="028e", SYMLINK+="input/joy"
EOL
)

RULE_FILE="/etc/udev/rules.d/rbl.rules"

echo "$RULE_CONTENT" | sudo tee "$RULE_FILE" > /dev/null

sudo udevadm control --reload-rules
sudo usermod -a -G dialout $USER
sudo usermod -a -G input $USER

# No error checking
echo "USB Ports configured!.."

# Package installation
echo -e "\nChecking for dependencies, installing if necessary..."

cat requirements.txt | xargs sudo apt-get install -y 

# If required add a new on boot service
