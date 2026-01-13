#!/bin/sh

# Ensure ip command exists
if ! command -v ip >/dev/null 2>&1; then
    echo "Error: 'ip' command not found."
    exit 1
fi

echo "Detecting CAN interfaces..."
echo

# Get CAN interfaces and their states
CAN_IFACES=$(ip -o link show | awk -F': ' '/can[0-9]+/ {print $2}')

if [ -z "$CAN_IFACES" ]; then
    echo "No CAN interfaces found."
    exit 1
fi

# Display interfaces with status
i=1
for iface in $CAN_IFACES; do
    STATE=$(ip link show "$iface" | grep -o "state [A-Z]*" | awk '{print $2}')
    echo "[$i] $iface - $STATE"
    i=$((i + 1))
done

echo
printf "Select an interface (number): "
read SELECTION

# Validate selection
SELECTED_IFACE=$(echo "$CAN_IFACES" | awk "NR==$SELECTION")
if [ -z "$SELECTED_IFACE" ]; then
    echo "Invalid selection."
    exit 1
fi

# Ask for bitrate
printf "Enter CAN bitrate [125000]: "
read BITRATE
BITRATE=${BITRATE:-125000}

echo
echo "Configuring $SELECTED_IFACE with bitrate $BITRATE..."

# Bring interface down before reconfiguring (safe practice)
sudo ip link set "$SELECTED_IFACE" down 2>/dev/null

# Configure CAN interface
if ! sudo ip link set "$SELECTED_IFACE" type can bitrate "$BITRATE"; then
    echo "Failed to set bitrate."
    exit 1
fi

# Bring interface up
if ! sudo ip link set "$SELECTED_IFACE" up; then
    echo "Failed to bring interface up."
    exit 1
fi

echo
echo "OK - $SELECTED_IFACE is now UP with bitrate $BITRATE"
