#!/bin/bash
# Backward-compatible wrapper around modular orchestration infrastructure.
# Invokes orchestrate.sh for full pipeline execution.
#
# This script is maintained for compatibility. For more control,
# use orchestrate.sh directly with --stages, --bagfile, etc.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Forward to modular orchestration with full pipeline
"${SCRIPT_DIR}/orchestrate.sh" --stages full "$@"