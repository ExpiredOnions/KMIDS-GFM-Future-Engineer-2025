#!/bin/bash

# Absolute path to this script's directory
DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Run the UPS shutdown Python script with sudo
sudo python3 "$DIR/ups_shutdown.py" 10

# Wait a bit and then immediately shutdown (optional)
sudo shutdown now
