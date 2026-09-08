#!/usr/bin/env bash
set -euo pipefail

CONTAINER_NAME="${CONTAINER_NAME:-ugv_landing_sim}"
docker exec -it "$CONTAINER_NAME" /bin/bash
