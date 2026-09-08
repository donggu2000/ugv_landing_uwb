#!/usr/bin/env bash
set -euo pipefail

cd "$(dirname "$0")/.."
docker build -t ugv_landing_sim:humble -f docker/Dockerfile .
