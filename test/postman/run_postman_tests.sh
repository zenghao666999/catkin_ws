#!/usr/bin/env bash
# ===========================================================================
# run_postman_tests.sh -- rosbridge API test runner (Python WebSocket)
#
# Prerequisites:
#   - rosbridge_websocket running on port 9090
#   - pip install websocket-client   (one-time)
#
# Usage:
#   ./run_postman_tests.sh                  # default port 9090
#   ./run_postman_tests.sh --port 9091
#
# Output: plain-text results + exit code
# ===========================================================================

set -euo pipefail

ROSBIRDGE_PORT="9090"
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
RUNNER="$SCRIPT_DIR/rosbridge_test_runner.py"

while [[ $# -gt 0 ]]; do
    case "$1" in
        --port) ROSBIRDGE_PORT="$2"; shift 2 ;;
        --help|-h)
            echo "Usage: $0 [--port N]"
            echo "  --port N   WebSocket port (default 9090)"
            exit 0 ;;
        *) echo "Unknown: $1"; exit 1 ;;
    esac
done

echo "========================================="
echo "  rosbridge API Test Runner (Python)"
echo "========================================="
echo "Runner: $RUNNER"
echo "Target: ws://localhost:$ROSBIRDGE_PORT"
echo ""

# Check Python
if ! command -v python3 &>/dev/null; then
    echo "ERROR: python3 not found."
    exit 1
fi

# Check websocket-client
python3 -c "import websocket" 2>/dev/null || {
    echo "Installing websocket-client ..."
    pip install websocket-client
}

# Wait for rosbridge
echo "Waiting for rosbridge on port $ROSBIRDGE_PORT ..."
MAX_RETRIES=10
RETRY=0
while [ $RETRY -lt $MAX_RETRIES ]; do
    if timeout 2 bash -c "echo >/dev/tcp/localhost/$ROSBIRDGE_PORT" 2>/dev/null; then
        echo "rosbridge is ready."
        break
    fi
    RETRY=$((RETRY + 1))
    echo "  retry $RETRY/$MAX_RETRIES ..."
    sleep 2
done

if [ $RETRY -ge $MAX_RETRIES ]; then
    echo "ERROR: Nothing listening on port $ROSBIRDGE_PORT."
    echo "Start rosbridge: roslaunch filter rosbridge_test.launch"
    exit 1
fi

# Run tests
echo ""
python3 "$RUNNER" --port "$ROSBIRDGE_PORT" --verbose
EXIT_CODE=$?

echo ""
echo "=== Done (exit code $EXIT_CODE) ==="
exit $EXIT_CODE
