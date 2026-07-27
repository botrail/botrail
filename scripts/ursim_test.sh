#!/usr/bin/env bash
# Runs the URSim integration tests (python/tests/test_ursim.py) against a
# dockerized Universal Robots simulator, then tears it down.
#
#   scripts/ursim_test.sh            # full run
#   scripts/ursim_test.sh -k joint   # extra args are passed to pytest
#
# Env overrides: URSIM_IMAGE, URSIM_CONTAINER, PYTEST.
set -euo pipefail
cd "$(dirname "$0")/.."

IMAGE="${URSIM_IMAGE:-universalrobots/ursim_e-series}"
NAME="${URSIM_CONTAINER:-botrail-ursim}"
PYTEST="${PYTEST:-.venv/bin/pytest}"

docker rm -f "$NAME" >/dev/null 2>&1 || true
docker run --rm -d --name "$NAME" \
  -e ROBOT_MODEL=UR5 \
  -p 29999:29999 -p 30001-30004:30001-30004 \
  "$IMAGE" >/dev/null
trap 'docker rm -f "$NAME" >/dev/null 2>&1 || true' EXIT

echo "waiting for the URSim dashboard (up to 180s)..."
python3 - <<'EOF'
import socket, sys, time
deadline = time.time() + 180
while time.time() < deadline:
    try:
        with socket.create_connection(("127.0.0.1", 29999), timeout=2) as s:
            if s.recv(128):
                sys.exit(0)
    except OSError:
        time.sleep(2)
sys.exit("URSim dashboard did not come up within 180s")
EOF

BOTRAIL_URSIM_HOST=127.0.0.1 "$PYTEST" python/tests/test_ursim.py -v "$@"
