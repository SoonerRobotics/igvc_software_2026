#!/bin/bash

SCRIPTS=(
    "./install_dependencies.sh"
    "./install_dotnet.sh"
    "./install_vn.sh"
    "./install_gui.sh"
    # "./install_controller.sh"
)

for script in "${SCRIPTS[@]}"; do
    if [ -f "$script" ]; then
        echo "Running $script..."
        bash "$script"
        if [ $? -ne 0 ]; then
            echo "Error: $script failed"
            exit 1
        fi
    else
        echo "Error: $script not found"
        exit 1
    fi
done

echo "All scripts completed successfully"
