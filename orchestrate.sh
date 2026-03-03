#!/bin/bash
# Modular orchestration CLI for SLAM benchmark pipeline.
# Provides flexible stage selection, configuration overrides, and framework integration.
#
# Usage:
#   ./orchestrate.sh --stages slam,bag,evaluate
#   ./orchestrate.sh --stages slam,bag --bagfile /path/to/bag.mcap --output-dir /tmp/results
#   ./orchestrate.sh --help

set -euo pipefail

# --- Constants ---
readonly SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
readonly LIB_DIR="${SCRIPT_DIR}/lib"
readonly ENV_FILE="${SCRIPT_DIR}/.env"

# --- Defaults ---
STAGES=""
BAGFILE=""
CONFIG_OVERRIDE=""
OUTPUT_DIR=""
ENV_PATH="$ENV_FILE"
SKIP_CLEANUP=false
CLEANUP_POLICY="auto"
SKIP_SETUP=false
GPU_MODE=false
NO_EXIT_TRAP=false
VERBOSE=false
FOLLOW_SLAM_LOGS=false
FOLLOW_LOGS_PID=""

# --- Functions ---

usage() {
    cat << EOF
Usage: $(basename "$0") [OPTIONS]

OPTIONS:
  --stages STAGES              Comma-separated list of stages to run.
                               Available: setup, slam, slam-gpu, bag, record, evaluate, report
                               Special: full, slam-bag (shortcuts)
                               Default: full

  --bagfile PATH              Path to MCAP bagfile (overrides BAGFILE_NAME in .env)
  --config-override PATH      Path to config overlay file to source after .env
  --output-dir PATH           Override OUTPUT_PATH_HOST for this run
  --env-file PATH             Path to .env file (default: ./.env)
  
    --skip-cleanup              Do not cleanup containers on exit
    --cleanup-on-exit           Force cleanup on exit (overrides auto behavior)
  --skip-setup                Skip setup stage (assume output dir exists)
  --gpu                       Use GPU variant of SLAM (run_slam_nvidia instead of run_slam)
  --no-exit-trap              Do not register cleanup on EXIT (manual cleanup required)
    --verbose                   Print debug information
    --follow-slam-logs          Follow run_slam and record_odometry logs after SLAM stage starts

  --help                       Show this help message

EXAMPLES:
  # Full pipeline with default settings
  $(basename "$0")

  # Only SLAM and bagfile playback
  $(basename "$0") --stages slam,bag

    # IMPORTANT: stage order matters (bag requires slam already running)
    $(basename "$0") --stages slam,bag --follow-slam-logs

  # SLAM only with custom bagfile
  $(basename "$0") --stages slam --bagfile /data/myrun.mcap

  # Evaluate only (trajectory already recorded)
  $(basename "$0") --stages evaluate

  # Full pipeline with config override and custom output
  $(basename "$0") \\
    --stages full \\
    --config-override ./custom.env \\
    --output-dir /results/run_001

EOF
}

parse_args() {
    while [ $# -gt 0 ]; do
        case "$1" in
            --stages)
                STAGES="$2"
                shift 2
                ;;
            --bagfile)
                BAGFILE="$2"
                shift 2
                ;;
            --config-override)
                CONFIG_OVERRIDE="$2"
                shift 2
                ;;
            --output-dir)
                OUTPUT_DIR="$2"
                shift 2
                ;;
            --env-file)
                ENV_PATH="$2"
                shift 2
                ;;
            --skip-cleanup)
                SKIP_CLEANUP=true
                CLEANUP_POLICY="manual"
                shift
                ;;
            --cleanup-on-exit)
                SKIP_CLEANUP=false
                CLEANUP_POLICY="manual"
                shift
                ;;
            --skip-setup)
                SKIP_SETUP=true
                shift
                ;;
            --gpu)
                GPU_MODE=true
                shift
                ;;
            --no-exit-trap)
                NO_EXIT_TRAP=true
                shift
                ;;
            --verbose)
                VERBOSE=true
                shift
                ;;
            --follow-slam-logs)
                FOLLOW_SLAM_LOGS=true
                shift
                ;;
            --help)
                usage
                exit 0
                ;;
            *)
                echo "Error: Unknown option '$1'"
                usage
                exit 1
                ;;
        esac
    done
}

debug() {
    if [[ "$VERBOSE" == "true" ]]; then
        echo "[DEBUG] $*" >&2
    fi
}

load_library() {
    if [ ! -f "${LIB_DIR}/orchestration.sh" ]; then
        echo "ERROR: Orchestration library not found at ${LIB_DIR}/orchestration.sh" >&2
        exit 1
    fi
    source "${LIB_DIR}/orchestration.sh"
    debug "Loaded orchestration library from ${LIB_DIR}/orchestration.sh"
}

configure_environment() {
    debug "Configuring environment..."

    setup_env "$ENV_PATH" || {
        error "Failed to load environment."
        exit 1
    }

    if [[ "$NO_EXIT_TRAP" != "true" ]]; then
        if [[ "$SKIP_CLEANUP" == "true" ]]; then
            export CLEANUP_ON_EXIT=false
        else
            export CLEANUP_ON_EXIT=true
        fi
        register_cleanup
    fi

    if [ -n "$BAGFILE" ]; then
        debug "Overriding BAGFILE_NAME with: $BAGFILE"
        export BAGFILE_NAME="$BAGFILE"
    fi

    if [ -n "$OUTPUT_DIR" ]; then
        debug "Overriding OUTPUT_PATH_HOST with: $OUTPUT_DIR"
        export OUTPUT_PATH_HOST="$OUTPUT_DIR"
        OUTPUT_PATH="$OUTPUT_DIR"
    fi

    if [ -n "$CONFIG_OVERRIDE" ]; then
        load_config_overlay "$CONFIG_OVERRIDE" || {
            error "Failed to load config overlay: $CONFIG_OVERRIDE"
            exit 1
        }
    fi

    debug "Environment configuration complete."
}

expand_stages() {
    local stages="$1"

    case "$stages" in
        "full")
            echo "setup,slam,bag,evaluate,report"
            ;;
        "slam-bag")
            echo "setup,slam,bag"
            ;;
        *)
            echo "$stages"
            ;;
    esac
}

apply_auto_cleanup_policy() {
    local expanded_stages
    expanded_stages=$(expand_stages "$STAGES")

    # For SLAM-only runs, keep services alive by default for interactive debugging.
    if [[ "$CLEANUP_POLICY" == "auto" ]]; then
        case "$expanded_stages" in
            slam|slam-gpu)
                SKIP_CLEANUP=true
                info "Auto cleanup disabled for SLAM-only run. Use --cleanup-on-exit to force cleanup."
                ;;
        esac
    fi
}

execute_stages() {
    local stages="$1"
    local stage_count
    local idx

    # Expand shortcuts
    stages=$(expand_stages "$stages")
    debug "Expanded stages: $stages"

    # Validate setup
    if [[ ! "$stages" =~ "skip-setup" ]]; then
        validate_setup || {
            error "Setup validation failed."
            exit 1
        }
    fi

    # Split and execute each stage
    IFS=',' read -ra STAGE_ARRAY <<< "$stages"
    stage_count=${#STAGE_ARRAY[@]}
    idx=0

    for stage in "${STAGE_ARRAY[@]}"; do
        stage=$(echo "$stage" | xargs) # Trim whitespace

        case "$stage" in
            setup)
                info "--- Stage: setup ---"
                run_stage_setup || exit 1
                ;;
            slam)
                info "--- Stage: slam ---"
                if [[ "$GPU_MODE" == "true" ]]; then
                    run_stage_slam_nvidia || exit 1
                else
                    run_stage_slam || exit 1
                fi
                if [[ "$FOLLOW_SLAM_LOGS" == "true" ]]; then
                    if (( idx < stage_count - 1 )); then
                        info "Following SLAM logs in background while continuing to next stage(s)..."
                        $COMPOSE_CMD logs -f run_slam record_odometry &
                        FOLLOW_LOGS_PID=$!
                    else
                        info "Following SLAM logs (Ctrl+C to stop log streaming)..."
                        $COMPOSE_CMD logs -f run_slam record_odometry
                    fi
                fi
                ;;
            slam-gpu)
                info "--- Stage: slam-gpu ---"
                run_stage_slam_nvidia || exit 1
                if [[ "$FOLLOW_SLAM_LOGS" == "true" ]]; then
                    if (( idx < stage_count - 1 )); then
                        info "Following SLAM logs in background while continuing to next stage(s)..."
                        $COMPOSE_CMD logs -f run_slam_nvidia record_odometry &
                        FOLLOW_LOGS_PID=$!
                    else
                        info "Following SLAM logs (Ctrl+C to stop log streaming)..."
                        $COMPOSE_CMD logs -f run_slam_nvidia record_odometry
                    fi
                fi
                ;;
            bag)
                info "--- Stage: bag ---"
                run_stage_bag || exit 1
                ;;
            record)
                info "--- Stage: record (blocking, Ctrl+C to stop) ---"
                run_stage_record_only || exit 1
                ;;
            evaluate)
                info "--- Stage: evaluate ---"
                run_stage_evaluate || exit 1
                ;;
            report)
                info "--- Stage: report ---"
                run_stage_report || true  # Report generation is best-effort
                ;;
            *)
                error "Unknown stage: '$stage'"
                exit 1
                ;;
        esac

        idx=$((idx + 1))
    done

    if [[ -n "$FOLLOW_LOGS_PID" ]]; then
        kill "$FOLLOW_LOGS_PID" >/dev/null 2>&1 || true
        wait "$FOLLOW_LOGS_PID" 2>/dev/null || true
        FOLLOW_LOGS_PID=""
    fi

    success "All requested stages completed successfully."
}

# --- Main ---

main() {
    parse_args "$@"

    # Default to full pipeline
    STAGES="${STAGES:-full}"

    debug "Script directory: $SCRIPT_DIR"
    debug "Environment file: $ENV_PATH"
    debug "Stages to run: $STAGES"
    debug "Skip cleanup: $SKIP_CLEANUP"
    debug "GPU mode: $GPU_MODE"

    load_library
    apply_auto_cleanup_policy
    configure_environment
    execute_stages "$STAGES"
}

main "$@"
