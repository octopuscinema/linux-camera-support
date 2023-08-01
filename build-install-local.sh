#!/bin/bash

echo ---------------------------------------------------------------
echo Build and install OCTOPUSCAMERA16 camera support kernel modules
echo ---------------------------------------------------------------

# Show all info
set -x

# Non interactive mode
export DEBIAN_FRONTEND=noninteractive

# Install kernel building dependancies
sudo apt-get update
sudo apt-get install -qq raspberrypi-kernel-headers

# Build imx585 driver
cd ./drivers/media/i2c
make
xz -f imx585.ko
sudo cp imx585.ko.xz /usr/lib/modules/$(uname -r)/kernel/drivers/media/i2c/imx585.ko.xz

# Build Raspberry Pi 4 CSI-2 (bcm2835-unicam) driver
cd ../../../drivers/media/platform/bcm2835
make
xz -f bcm2835-unicam.ko
sudo cp bcm2835-unicam.ko.xz /usr/lib/modules/$(uname -r)/kernel/drivers/media/platform/bcm2835/bcm2835-unicam.ko.xz
