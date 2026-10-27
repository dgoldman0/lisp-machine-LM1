"""LM-1 entry point.

    python -m lm1 run ...     Run a binary (CLI)
"""

import sys


def main() -> int:
    from .cli import main as cli_main
    return cli_main()


if __name__ == "__main__":
    sys.exit(main())
