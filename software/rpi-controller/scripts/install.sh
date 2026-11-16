#!/bin/sh
set -eu

cd /home/pi/hexapod
sudo install -m 0644 systemd/hexapod.service /etc/systemd/system/hexapod.service
sudo systemctl daemon-reload

echo "Installed. Validate before enabling:"
echo "  PYTHONPATH=/home/pi/hexapod/src python3 -m hexapod.cli --config /home/pi/hexapod/config/hexapod.toml validate"
echo "  sudo systemctl start hexapod.service"
