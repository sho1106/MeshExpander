#!/usr/bin/env bash
# =============================================================================
# build.sh -- MeshExpander: configure, build, (optionally) fetch bunny, test
#
# Works on: Linux, macOS, WSL, Git Bash on Windows
#
# Usage:
#   bash scripts/build.sh [options]
#
# Options:
#   --debug       Build in Debug mode (default: Release)
#   --no-bunny    Skip Stanford Bunny download/conversion
#   --no-test     Skip running tests
#   -j N          Parallel build jobs (default: number of CPU cores)
# =============================================================================

set -euo pipefail

# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------
RED='\033[0;31m'; GREEN='\033[0;32m'; CYAN='\033[0;36m'; NC='\033[0m'
info()  { echo -e "${CYAN}[INFO]${NC}  $*"; }
ok()    { echo -e "${GREEN}[OK]${NC}    $*"; }
error() { echo -e "${RED}[ERROR]${NC} $*" >&2; }
die()   { error "$*"; exit 1; }

# ---------------------------------------------------------------------------
# Parse arguments
# ---------------------------------------------------------------------------
CONFIG="Release"
SKIP_BUNNY=false
SKIP_TEST=false
JOBS=$(nproc 2>/dev/null || sysctl -n hw.logicalcpu 2>/dev/null || echo 4)

while [[ $# -gt 0 ]]; do
    case "$1" in
        --debug)    CONFIG="Debug" ;;
        --no-bunny) SKIP_BUNNY=true ;;
        --no-test)  SKIP_TEST=true ;;
        -j)         shift; JOBS="$1" ;;
        -j*)        JOBS="${1#-j}" ;;
        -h|--help)
            head -20 "$0" | grep "^#" | sed 's/^# \{0,1\}//'
            exit 0 ;;
        *) die "Unknown option: $1" ;;
    esac
    shift
done

# ---------------------------------------------------------------------------
# Paths
# ---------------------------------------------------------------------------
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
BUILD="$ROOT/build"
DATA_DIR="$BUILD/tests/data"

info "Root:   $ROOT"
info "Build:  $BUILD"
info "Config: $CONFIG  Jobs: $JOBS"

# ---------------------------------------------------------------------------
# Check prerequisites
# ---------------------------------------------------------------------------
command -v cmake >/dev/null 2>&1 || die "cmake not found. Install cmake >= 3.16."

CMAKE_VER=$(cmake --version | head -1 | grep -oE '[0-9]+\.[0-9]+' | head -1)
CMAKE_MAJOR=${CMAKE_VER%%.*}
CMAKE_MINOR=${CMAKE_VER#*.}
if (( CMAKE_MAJOR < 3 || (CMAKE_MAJOR == 3 && CMAKE_MINOR < 16) )); then
    die "cmake >= 3.16 required (found $CMAKE_VER)"
fi
ok "cmake $CMAKE_VER"

# ---------------------------------------------------------------------------
# Configure
# ---------------------------------------------------------------------------
info "Configuring..."
cmake -S "$ROOT" -B "$BUILD" \
    -DCMAKE_BUILD_TYPE="$CONFIG" \
    2>&1 | grep -v "^--" || true
ok "Configure done"

# ---------------------------------------------------------------------------
# Build
# ---------------------------------------------------------------------------
info "Building ($JOBS jobs)..."
cmake --build "$BUILD" --config "$CONFIG" --parallel "$JOBS"
ok "Build done"

# ---------------------------------------------------------------------------
# Stanford Bunny download + conversion (optional)
# ---------------------------------------------------------------------------
if [[ "$SKIP_BUNNY" == false ]]; then
    BUNNY_ARCHIVE="$ROOT/bunny.tar.gz"
    BUNNY_DIR="$ROOT/bunny/reconstruction"

    mkdir -p "$DATA_DIR"

    # Check if STL files already exist
    if [[ -f "$DATA_DIR/bunny.stl" && -f "$DATA_DIR/bunny_res4.stl" ]]; then
        ok "Stanford Bunny STL files already present — skipping download"
    else
        info "Downloading Stanford Bunny..."

        if [[ ! -f "$BUNNY_ARCHIVE" ]]; then
            if command -v curl >/dev/null 2>&1; then
                curl -L --retry 3 --progress-bar \
                    "http://graphics.stanford.edu/pub/3Dscanrep/bunny.tar.gz" \
                    -o "$BUNNY_ARCHIVE"
            elif command -v wget >/dev/null 2>&1; then
                wget -q --show-progress \
                    "http://graphics.stanford.edu/pub/3Dscanrep/bunny.tar.gz" \
                    -O "$BUNNY_ARCHIVE"
            else
                error "Neither curl nor wget found. Cannot download Stanford Bunny."
                error "Download manually from:"
                error "  http://graphics.stanford.edu/pub/3Dscanrep/bunny.tar.gz"
                error "Then run: python3 scripts/ply_to_stl.py --all bunny/reconstruction build/tests/data"
                SKIP_BUNNY=true
            fi
        else
            ok "Archive already present: $BUNNY_ARCHIVE"
        fi

        if [[ "$SKIP_BUNNY" == false ]]; then
            info "Extracting..."
            tar -xzf "$BUNNY_ARCHIVE" -C "$ROOT"

            info "Converting PLY -> binary STL..."
            python3 "$SCRIPT_DIR/ply_to_stl.py" --all "$BUNNY_DIR" "$DATA_DIR"
            ok "Bunny STL files ready in $DATA_DIR"
        fi
    fi
else
    info "Skipping Stanford Bunny (--no-bunny)"
fi

# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------
if [[ "$SKIP_TEST" == false ]]; then
    info "Running all tests..."
    cmake --build "$BUILD" --config "$CONFIG" --target check -- -j "$JOBS"
    ok "All tests passed"
else
    info "Skipping tests (--no-test)"
    info "To run tests manually:"
    info "  cmake --build $BUILD --config $CONFIG --target check"
fi

echo ""
ok "Done. Build artifacts in: $BUILD"
