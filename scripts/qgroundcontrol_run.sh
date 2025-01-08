#!/bin/bash

# Define the QGroundControl filename
QGC_FILE="QGroundControl.AppImage"

# Check if QGroundControl already exists
if [ -f "$QGC_FILE" ]; then
    echo "QGroundControl already exists. Skipping download."
else
    echo "QGroundControl not found. Downloading..."
    wget -O "$QGC_FILE" https://d176tv9ibo4jno.cloudfront.net/latest/QGroundControl.AppImage
    chmod +x "$QGC_FILE"
    echo "QGroundControl downloaded and made executable."
fi

# Extracting Appimage
echo "Extracting QGroundControl"
./QGroundControl.AppImage --appimage-extract
cd squashfs-root
./AppRun
cd ~/
