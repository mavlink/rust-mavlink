#!/usr/bin/env python3

import argparse


def percent_change(base: int, current: int) -> str:
    if base == 0:
        return "n/a"

    value = ((current - base) / base) * 100
    return f"{value:+.2f}%"


def seconds(milliseconds: int) -> str:
    return f"{milliseconds / 1000:.2f} s"


def mib(value: int) -> str:
    return f"{value / (1024 * 1024):.2f} MiB"


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--base-time", type=int, required=True)
    parser.add_argument("--pr-time", type=int, required=True)
    parser.add_argument("--base-size", type=int, required=True)
    parser.add_argument("--pr-size", type=int, required=True)

    args = parser.parse_args()

    print("# rust-mavlink performance")
    print()
    print("| Metric | Base | PR | Change |")
    print("| --- | ---: | ---: | ---: |")
    print(
        f"| Compilation time | {seconds(args.base_time)} | "
        f"{seconds(args.pr_time)} | "
        f"{percent_change(args.base_time, args.pr_time)} |"
    )
    print(
        f"| libmavlink size | {mib(args.base_size)} | "
        f"{mib(args.pr_size)} | "
        f"{percent_change(args.base_size, args.pr_size)} |"
    )


if __name__ == "__main__":
    main()
