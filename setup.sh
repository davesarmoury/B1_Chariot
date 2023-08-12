#!/usr/bin/env bash

echo "RUN sudo /opt/nvidia/jetson-io/jetson-io.py"
echo "Select Configure 40-pin expansion header at the bottom. Then select Configure header pins manually."
echo "Select spi1 (19, 21, 23, 24, 26)"
echo "Reboot"

sudo groupadd -f -r gpio
sudo usermod -a -G gpio $USER

cd ~
git clone https://github.com/NVIDIA/jetson-gpio.git
sudo cp ~/jetson-gpio/lib/python/Jetson/GPIO/99-gpio.rules /etc/udev/rules.d
rm -rf jetson-gpio

pip3 install adafruit-circuitpython-ads1x15
