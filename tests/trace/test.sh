#!/bin/sh
# ctest entry point for the trace reference check. See check.py for what it does.
set -e
exec python3 "$(dirname "$0")/check.py" "$@"
