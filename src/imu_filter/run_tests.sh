#!/usr/bin/env bash
# ===========================================================================
# imu_filter test launcher ? multi-mode test runner
#
# Usage:
#   ./run_tests.sh                  # rostest mode (needs roscore)
#   ./run_tests.sh --direct         # pytest directly (no roscore needed)
#   ./run_tests.sh --rebuild        # catkin_make first, then test
#   ./run_tests.sh --api            # Postman/Newman API tests (needs rosbridge)
#
# Copy to other packages:
#   cp run_tests.sh ../cam_track/run_tests.sh
#   then edit PKG= inside the script
# ===========================================================================

set -euo pipefail

PKG="filter"
MODE="rostest"
REBUILD=false
MODE_API=false

for arg in "$@"; do
    case "$arg" in
        --direct)   MODE="direct" ;;
        --api)      MODE_API=true ;;
        --rebuild)  REBUILD=true ;;
        --help|-h)
            echo "Usage: $0 [--direct] [--rebuild] [--api]"
            echo ""
            echo "  --direct     Run pytest directly (no roscore)"
            echo "  --rebuild    Run catkin_make before testing"
            echo "  --api        Run Postman/Newman API tests (needs rosbridge)"
            exit 0
            ;;
        *) echo "Unknown: $arg (use --help)"; exit 1 ;;
    esac
done

# Locate workspace root
WS_DIR="$(cd "$(dirname "$0")" && pwd)"
while [ "$WS_DIR" != "/" ]; do
    if [ -f "$WS_DIR/src/CMakeLists.txt" ]; then break; fi
    WS_DIR="$(dirname "$WS_DIR")"
done

if [ "$WS_DIR" = "/" ]; then
    echo "ERROR: catkin workspace not found (src/CMakeLists.txt)"
    exit 1
fi

echo "Workspace: $WS_DIR"
echo "Package:   $PKG"
echo "Mode:      $MODE"

# Source ROS
if [ -f "/opt/ros/noetic/setup.bash" ]; then
    source /opt/ros/noetic/setup.bash
elif [ -f "/opt/ros/melodic/setup.bash" ]; then
    source /opt/ros/melodic/setup.bash
fi

if [ -f "$WS_DIR/devel/setup.bash" ]; then
    source "$WS_DIR/devel/setup.bash"
fi

# Optional rebuild
if [ "$REBUILD" = true ]; then
    echo ""
    echo "=== catkin_make ==="
    cd "$WS_DIR"
    catkin_make --pkg "$PKG"
fi

# ===================================================================
# API Test Mode (rosbridge + Python WebSocket runner)
# ===================================================================
if [ "$MODE_API" = true ]; then
    echo ""
    echo "=== rosbridge API Tests ==="
    RUNNER="$WS_DIR/test/postman/run_postman_tests.sh"
    if [ ! -f "$RUNNER" ]; then
        PARENT="$(dirname "$WS_DIR")"
        RUNNER="$PARENT/test/postman/run_postman_tests.sh"
    fi
    if [ -f "$RUNNER" ]; then
        bash "$RUNNER"
        echo ""
        echo "=== Done (API) ==="
    else
        echo "ERROR: run_postman_tests.sh not found."
        echo "  Searched: $RUNNER"
        exit 1
    fi
    exit 0
fi

# Run tests
if [ "$MODE" = "direct" ]; then
    echo ""
    echo "=== pytest (direct) ==="
    cd "$WS_DIR/src/$PKG"
    python3 -m pytest test/ -v --tb=short --color=yes
else
    echo ""
    echo "=== rostest ==="
    rostest "$PKG" imu_filter_pytest.test
fi

echo ""
echo "=== Done ==="
