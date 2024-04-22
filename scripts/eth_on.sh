#!/bin/bash

sudo ifconfig enp4s0 down
sudo ifconfig enp4s0 up
sudo ifconfig enp4s0 20.0.0.1

echo "Reseto Ethernet hecho! Espera un poco a que se asigne la IP"

exit
