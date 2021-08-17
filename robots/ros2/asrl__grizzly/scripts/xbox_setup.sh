#!/bin/bash

# Set up the gamepad
sudo rmmod xpad
sudo modprobe uinput
sudo modprobe joydev
sudo xboxdrv
