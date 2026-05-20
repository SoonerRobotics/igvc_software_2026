#!/bin/bash
set -e

echo "Installing IGVC CAN udev rules and services..."

# Install udev rules
sudo cp misc/igvc.rules /etc/udev/rules.d/99-igvc.rules

# Services
sudo cp misc/igvc-can-slcan@.service /etc/systemd/system/igvc-can-slcan@.service

# Load slcan module and ensure it persists on boot
# On Jetson, this module is often missing from the default kernel.
if sudo modprobe slcan 2>/dev/null; then
    echo "SLCAN module loaded successfully."
    if ! grep -q "slcan" /etc/modules; then
        echo "slcan" | sudo tee -a /etc/modules
    fi
else
    echo "Warning: slcan kernel module not found. USB-to-CAN adapters using SLCAN will not work."
    echo "You may need to recompile your kernel or use an out-of-tree driver if you require SLCAN support."
fi

# Reload everything
sudo systemctl daemon-reload
sudo systemctl restart systemd-networkd
sudo udevadm control --reload-rules