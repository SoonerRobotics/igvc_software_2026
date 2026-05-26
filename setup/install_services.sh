#!/bin/bash
set -e

echo "Installing IGVC CAN udev rules and services..."

# Install udev rules
sudo cp misc/igvc.rules /etc/udev/rules.d/99-igvc.rules

# Services
sudo cp misc/igvc-can-slcan@.service /etc/systemd/system/igvc-can-slcan@.service
sudo cp misc/igvc-csharp.service /etc/systemd/system/igvc-csharp.service

# Reload everything
sudo systemctl daemon-reload
sudo systemctl restart systemd-networkd
sudo udevadm control --reload-rules

# Print how to enable igvc-csharp
echo "To enable the igvc-csharp service, run:"
echo "sudo systemctl enable --now igvc-csharp.service"