#!/usr/bin/env bash
set -eo pipefail

LAPTOP_IP="${1:-10.99.40.20}"
PORT="${2:-11811}"

source /opt/ros/humble/setup.bash

echo "Starting FastDDS Discovery Server"
echo "  address: ${LAPTOP_IP}:${PORT}"
echo "Keep this terminal open while Jetson and RViz are running."

exec fast-discovery-server -i 0 -l "${LAPTOP_IP}" -p "${PORT}"
