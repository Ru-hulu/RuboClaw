"""Standalone integer addition program."""

from __future__ import annotations

import argparse
import json


def parse_arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Add two integers.")
    parser.add_argument("--a", type=int, required=True, help="The first integer.")
    parser.add_argument("--b", type=int, required=True, help="The second integer.")
    return parser.parse_args()


def main() -> None:
    arguments = parse_arguments()
    result = {
        "a": arguments.a,
        "b": arguments.b,
        "result": arguments.a + arguments.b,
    }
    print(json.dumps(result, ensure_ascii=False))


if __name__ == "__main__":
    main()
