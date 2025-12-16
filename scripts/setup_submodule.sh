#!/bin/bash

# Setup script for TI IWR6843 driver submodule
set -e

cd "$(dirname "$0")/.."

echo "Setting up TI mmWave radar driver submodule..."

# Check if submodule already exists
if [ -d "mmwave_ti_ros/.git" ]; then
    echo "Submodule already exists. Updating..."
    git submodule update --init --recursive
else
    echo "Adding submodule..."
    git submodule add https://git.ti.com/git/mmwave_radar/mmwave_ti_ros.git mmwave_ti_ros || true
    git submodule update --init --recursive
fi

echo "Submodule setup complete!"