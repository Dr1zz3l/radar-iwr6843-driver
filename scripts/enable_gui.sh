#!/bin/bash

# Enable X11 forwarding for Docker containers
# Run this on your host machine after reboot if GUI apps don't work

echo "Enabling X11 forwarding for Docker..."
xhost +local:docker

echo "✓ GUI access enabled for Docker containers"
echo "You can now run RViz and other GUI applications in the container"
