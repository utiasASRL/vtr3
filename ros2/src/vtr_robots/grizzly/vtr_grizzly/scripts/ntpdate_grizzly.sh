#!/bin/bash

# run chrony on the grizzly and synch to this computer
# Make sure this ip address matches the one on your machine
ssh administrator@192.168.1.11 'sudo /usr/sbin/ntpdate 192.168.1.22'

