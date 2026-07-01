#!/usr/bin/env python3
import argparse
import sys


def read_frames(path):
    with open(path, encoding="utf-8") as file:
        return [line.strip() for line in file if line.strip()]


def main():
    parser = argparse.ArgumentParser(
        description="Assert every frame parsed by c_library_v2 is also parsed by rust-mavlink."
    )
    parser.add_argument("--c", required=True, help="accepted frames from c_library_v2")
    parser.add_argument("--rust", required=True, help="accepted frames from rust-mavlink")
    args = parser.parse_args()

    c_frames = read_frames(args.c)
    rust_frames = read_frames(args.rust)

    rust_index = 0
    missing = []
    for c_index, c_frame in enumerate(c_frames):
        search_index = rust_index
        while search_index < len(rust_frames) and rust_frames[search_index] != c_frame:
            search_index += 1

        if search_index == len(rust_frames):
            missing.append((c_index, c_frame))
        else:
            rust_index = search_index + 1

    print(f"c_library_v2 frames: {len(c_frames)}")
    print(f"rust-mavlink frames: {len(rust_frames)}")

    if missing:
        print(
            "rust-mavlink missed frames accepted by c_library_v2:",
            file=sys.stderr,
        )
        for index, frame in missing[:20]:
            print(f"  c frame #{index}: {frame}", file=sys.stderr)
        if len(missing) > 20:
            print(f"  ... and {len(missing) - 20} more", file=sys.stderr)
        return 1

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
