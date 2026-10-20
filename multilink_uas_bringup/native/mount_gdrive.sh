#!/usr/bin/env bash
# Foreground mount: Ctrl+C unmounts. Credentials stay outside the repository.
set -euo pipefail
mount_dir="${1:-/mnt/gdrive}"
mkdir -p "$mount_dir"
if mountpoint -q "$mount_dir"; then
  echo "Already mounted: $mount_dir" >&2
  exit 1
fi
exec rclone mount uas_gdrive_private: "$mount_dir" \
  --read-only --vfs-cache-mode full \
  --vfs-cache-max-size 4G --vfs-cache-min-free-space 20G \
  --vfs-cache-max-age 10m --cache-dir /root/.cache/rclone-uas \
  --buffer-size 16M --vfs-read-ahead 128M --tpslimit 2 \
  --log-level INFO
