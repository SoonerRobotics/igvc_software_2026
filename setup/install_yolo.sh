#!/bin/bash
set -e
ONNX_VERSION="1.23.2"

# Download the ONNX Runtime GPU package
wget https://github.com/microsoft/onnxruntime/releases/download/v${ONNX_VERSION}/onnxruntime-linux-x64-gpu-${ONNX_VERSION}.tgz

# Extract and move to /opt/onnxruntime
tar -xf onnxruntime-linux-x64-gpu-${ONNX_VERSION}.tgz
sudo mv onnxruntime-linux-x64-gpu-${ONNX_VERSION} /opt/onnxruntime

# Clean up
rm -rf onnxruntime-linux-x64-gpu-${ONNX_VERSION}.tgz

# Add to ldconfig
echo 'export LD_LIBRARY_PATH=/opt/onnxruntime/lib:$LD_LIBRARY_PATH' >> ~/.bashrc
source ~/.bashrc