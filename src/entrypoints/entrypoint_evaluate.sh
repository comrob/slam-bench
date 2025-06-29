#!/bin/bash
echo "Starting trajectory analysis..."

# The shell finds 'scripts/trajectory_analysis.py' because we are in '/app'.
COMMAND="python3 scripts/trajectory_analysis.py"

if [ "$TEST_MODE" == "1" ]; then
    COMMAND="$COMMAND --test"
fi

echo "Running command: $COMMAND"
$COMMAND