#!/usr/bin/env python3

import json
import sys
from pathlib import Path

def main():
    definitions_dir = (
        Path(__file__).parent.parent
        / "mavlink"
        / "mavlink"
        / "message_definitions"
        / "v1.0"
    )

    definitions = sorted(
        definitions_dir.glob("*.xml"), key=lambda path: path.stem.lower()
    )

    if not definitions:
        print(f"No dialect definitions found in {definitions_dir}", file=sys.stderr)
        return 1

    features = [f"dialect-{definition.stem.lower()}" for definition in definitions]
    print(json.dumps(features))

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
