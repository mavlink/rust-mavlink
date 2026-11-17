#!/usr/bin/env bash

set -euo pipefail

TARGET_DIR="${1:-target/performance}"

rm -rf "$TARGET_DIR"

start=$(python3 - <<'PY'
import time
print(time.time_ns())
PY
)

CARGO_TARGET_DIR="$TARGET_DIR" \
cargo build \
  --release \
  --package mavlink \
  --features dialect-common

end=$(python3 - <<'PY'
import time
print(time.time_ns())
PY
)

elapsed_ms=$(( (end - start) / 1000000 ))

# Cargo also copies the rlib to release/libmavlink.rlib. That path exists in
# the old layout and in the build-dir layout from Rust 1.100, where
# release/deps is gone.
artifact="$TARGET_DIR/release/libmavlink.rlib"

if [ ! -f "$artifact" ]; then
    echo "Unable to locate libmavlink rlib" >&2
    exit 1
fi

case "$(uname -s)" in
    Darwin)
        size_bytes=$(stat -f '%z' "$artifact")
        ;;
    Linux)
        size_bytes=$(stat -c '%s' "$artifact")
        ;;
    *)
        echo "Unsupported platform" >&2
        exit 1
        ;;
esac

printf 'compile_time_ms=%s\n' "$elapsed_ms"
printf 'artifact_size_bytes=%s\n' "$size_bytes"
printf 'artifact=%s\n' "$artifact"
