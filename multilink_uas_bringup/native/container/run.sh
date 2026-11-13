#!/usr/bin/env bash
# GPU-isolated Jammy environment; source is shared, dependencies stay outside Git.
set -euo pipefail
repo=$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)
runtime=${MULTILINK_RUNTIME_ROOT:-"$(dirname "$repo")/multilink_autonomy_runtime"}
name=${MULTILINK_CONTAINER:-multilink-comparison-20261111}
if ! docker container inspect "$name" >/dev/null 2>&1; then
  mkdir -p "$runtime/root"
  docker run -d --name "$name" --gpus all --shm-size 4g \
    -e NVIDIA_DRIVER_CAPABILITIES=all \
    -v "$runtime/root:/root" \
    -v "$repo:/root/multilink_autonomy_ws/src/multilink_autonomy" \
    -v /tmp/.X11-unix:/tmp/.X11-unix:ro -e DISPLAY="${DISPLAY:-:0}" \
    --entrypoint /bin/bash \
    ros@sha256:d9bcf2e6f72df127fdee07048c2e87b1b4b9a59a74af0592f4489fc2844aa855 \
    -c 'exec sleep infinity'
fi
docker start "$name" >/dev/null
exec docker exec -e DISPLAY="${DISPLAY:-:0}" -e PYTHONDONTWRITEBYTECODE=1 "$name" "$@"
