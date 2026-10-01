#!/bin/bash
set -eo pipefail
cd "$(dirname "$0")"
case "${1:-stack}" in
  stack|sim|stop|status) exec /usr/bin/python3 manager.py "${1:-stack}" ;;
  check)
    source ./env.sh
    exec python3 check_topics.py --duration 30 --best-effort ;;
  check-stack)
    bash ./run.sh check
    env -i HOME="$HOME" USER="${USER:-root}" PATH=/usr/bin:/bin bash --noprofile --norc -c 'source ./env_ros1.sh && exec python3 check_planner.py' ;;
  check-inference)
    source ./env.sh
    python3 check_inference.py
    echo 'Policy target remains in the observation-only process. Stop/restart the stack to reset.' >&2 ;;
  rviz)
    source ./env.sh
    exec rviz2 -d "$PWD/unipilot.rviz" ;;
  *) echo 'Usage: ./run.sh {stack|sim|status|check|check-stack|check-inference|rviz|stop}'; exit 2 ;;
esac
