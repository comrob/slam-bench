#!/bin/bash
# Reusable orchestration library for SLAM benchmark pipeline.
# Provides modular stage functions for independent composition.
#
# Usage:
#   source lib/orchestration.sh
#   setup_env "/path/to/.env"
#   run_stage_slam
#   run_stage_bag
#   cleanup

set -euo pipefail

# --- Global Configuration ---
# SCRIPT_DIR should be set by caller (orchestrate.sh) before sourcing this library.
# If not set, compute it here (for standalone usage).
if [ -z "${SCRIPT_DIR:-}" ]; then
    SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
    readonly SCRIPT_DIR
fi

readonly DEFAULT_TIMEOUT_SLAM=30
readonly DEFAULT_TIMEOUT_BAG=600
readonly DEFAULT_TIMEOUT_EVALUATE=120

# State tracking (will be set by setup functions, but only if not already set readonly)
if ! (readonly | grep -q "COMPOSE_CMD"); then
    COMPOSE_CMD=""
fi
if ! (readonly | grep -q "ENV_FILE"); then
    ENV_FILE=""
fi
if ! (readonly | grep -q "OUTPUT_PATH"); then
    OUTPUT_PATH=""
fi
CLEANUP_ON_EXIT=${CLEANUP_ON_EXIT:-true}

# --- Logging ---
readonly C_RESET='\033[0m'
readonly C_RED='\033[0;31m'
readonly C_GREEN='\033[0;32m'
readonly C_YELLOW='\033[0;33m'
readonly C_BLUE='\033[0;34m'

info() { echo -e "${C_BLUE}INFO: $1${C_RESET}"; }
success() { echo -e "${C_GREEN}SUCCESS: $1${C_RESET}"; }
warn() { echo -e "${C_YELLOW}WARN: $1${C_RESET}"; }
error() { echo -e "${C_RED}ERROR: $1${C_RESET}" >&2; }

# --- Setup & Cleanup ---

setup_env() {
    local env_file="${1:-${SCRIPT_DIR}/.env}"

    if [ ! -f "$env_file" ]; then
        error "Environment file not found: $env_file"
        return 1
    fi

    if ! (readonly | grep -q "ENV_FILE"); then
        ENV_FILE="$env_file"
    fi
    
    set -o allexport
    source "$env_file"
    set +o allexport

    if ! (readonly | grep -q "COMPOSE_CMD"); then
        COMPOSE_CMD="docker compose"
        if [[ "${DEV_DOCKER:-false}" == "true" ]]; then
            info "Development mode enabled. Using local Dockerfile."
            COMPOSE_CMD="docker compose -f ${SCRIPT_DIR}/docker-compose.yaml -f ${SCRIPT_DIR}/docker-compose-dev.yaml"
        fi
    fi

    if ! (readonly | grep -q "OUTPUT_PATH"); then
        OUTPUT_PATH="${OUTPUT_PATH_HOST:-${SCRIPT_DIR}/evaluation_output}"
    fi
    
    info "Environment loaded from: $env_file"
    info "Compose command: $COMPOSE_CMD"
    info "Output directory: ${OUTPUT_PATH_HOST:-${SCRIPT_DIR}/evaluation_output}"

    return 0
}

register_cleanup() {
    if [[ "$CLEANUP_ON_EXIT" == "true" ]]; then
        trap cleanup_all EXIT
        info "Cleanup on exit registered."
    fi
}

cleanup_all() {
    info "Running cleanup... Stopping all containers."
    if [ -z "$COMPOSE_CMD" ]; then
        warn "COMPOSE_CMD not set; cannot cleanup."
        return 1
    fi
    $COMPOSE_CMD down -v --remove-orphans || warn "Cleanup had errors."
    success "Cleanup complete."
}

health_check() {
    local service="$1"
    local timeout="${2:-$DEFAULT_TIMEOUT_SLAM}"

    info "Health check for service '$service' (timeout: ${timeout}s)..."

    local elapsed=0
    while [ $elapsed -lt $timeout ]; do
        if $COMPOSE_CMD ps --services --filter "status=running" | grep -qw "$service"; then
            success "Service '$service' is running."
            return 0
        fi
        sleep 1
        elapsed=$((elapsed + 1))
    done

    error "Service '$service' failed to start within ${timeout}s."
    $COMPOSE_CMD logs --tail=20 "$service" || true
    return 1
}

# --- Stage Functions ---

run_stage_setup() {
    info "Setting up pipeline environment..."

    if [ -z "$OUTPUT_PATH" ]; then
        error "OUTPUT_PATH not configured. Call setup_env first."
        return 1
    fi

    info "Stopping any previous containers..."
    $COMPOSE_CMD down -v --remove-orphans || warn "Previous cleanup failed."

    info "Removing previous output directory: $OUTPUT_PATH"
    if [ -d "$OUTPUT_PATH" ]; then
        rm -rf "$OUTPUT_PATH"
        success "Previous output directory removed."
    fi

    info "Creating fresh output directory: $OUTPUT_PATH"
    mkdir -p "$OUTPUT_PATH"
    success "Setup complete."
    return 0
}

run_stage_slam() {
    info "Starting SLAM service..."

    if [ -z "$COMPOSE_CMD" ]; then
        error "COMPOSE_CMD not set. Call setup_env first."
        return 1
    fi

    if [ -z "$OUTPUT_PATH" ]; then
        error "OUTPUT_PATH not set. Call setup_env first."
        return 1
    fi

    mkdir -p "$OUTPUT_PATH" || {
        error "Could not create output directory: $OUTPUT_PATH"
        return 1
    }

    if ! touch "$OUTPUT_PATH/.write_test" 2>/dev/null; then
        error "Output directory is not writable: $OUTPUT_PATH"
        error "Fix ownership/permissions, e.g.: sudo chown -R $(id -u):$(id -g) $OUTPUT_PATH"
        return 1
    fi
    rm -f "$OUTPUT_PATH/.write_test"

    $COMPOSE_CMD up -d run_slam record_odometry
    success "SLAM and recorder services started in background."

    health_check "run_slam" "${DEFAULT_TIMEOUT_SLAM}"
    return $?
}

run_stage_slam_nvidia() {
    info "Starting SLAM service (NVIDIA GPU variant)..."

    if [ -z "$COMPOSE_CMD" ]; then
        error "COMPOSE_CMD not set. Call setup_env first."
        return 1
    fi

    if [ -z "$OUTPUT_PATH" ]; then
        error "OUTPUT_PATH not set. Call setup_env first."
        return 1
    fi

    mkdir -p "$OUTPUT_PATH" || {
        error "Could not create output directory: $OUTPUT_PATH"
        return 1
    }

    if ! touch "$OUTPUT_PATH/.write_test" 2>/dev/null; then
        error "Output directory is not writable: $OUTPUT_PATH"
        error "Fix ownership/permissions, e.g.: sudo chown -R $(id -u):$(id -g) $OUTPUT_PATH"
        return 1
    fi
    rm -f "$OUTPUT_PATH/.write_test"

    $COMPOSE_CMD up -d run_slam_nvidia record_odometry
    success "SLAM (GPU) and recorder services started in background."

    health_check "run_slam_nvidia" "${DEFAULT_TIMEOUT_SLAM}"
    return $?
}

run_stage_bag() {
    info "Starting bagfile playback..."

    if [ -z "$COMPOSE_CMD" ]; then
        error "COMPOSE_CMD not set. Call setup_env first."
        return 1
    fi

    # Bag playback stage requires SLAM to be already running.
    if ! $COMPOSE_CMD ps --services --filter "status=running" | grep -Eq "^(run_slam|run_slam_nvidia)$"; then
        error "SLAM service is not running. Start SLAM first (use --stages slam,bag in this order)."
        return 1
    fi

    if [[ "${DEV_DOCKER:-false}" == "true" ]]; then
        $COMPOSE_CMD up --build --no-deps --abort-on-container-exit --exit-code-from play_bag play_bag || {
            error "Bagfile playback failed."
            return 1
        }
    else
        $COMPOSE_CMD up --no-deps --abort-on-container-exit --exit-code-from play_bag play_bag || {
            error "Bagfile playback failed."
            return 1
        }
    fi

    success "Bagfile playback complete."
    return 0
}

run_stage_record_only() {
    info "Starting odometry recording (no playback)..."

    if [ -z "$COMPOSE_CMD" ]; then
        error "COMPOSE_CMD not set. Call setup_env first."
        return 1
    fi

    # record_odometry depends on run_slam, so SLAM must be running
    if ! $COMPOSE_CMD ps --services --filter "status=running" | grep -qw "run_slam"; then
        error "SLAM service must be running. Call run_stage_slam first."
        return 1
    fi

    info "Recorder is running as part of SLAM startup."
    info "Use Ctrl+C to stop, then call run_stage_evaluate."
    $COMPOSE_CMD logs -f record_odometry
    return 0
}

run_stage_evaluate() {
    info "Running trajectory evaluation..."

    if [ -z "$COMPOSE_CMD" ]; then
        error "COMPOSE_CMD not set. Call setup_env first."
        return 1
    fi

    if [ ! -f "${OUTPUT_PATH}/estimated_trajectory.txt" ]; then
        error "Estimated trajectory not found: ${OUTPUT_PATH}/estimated_trajectory.txt"
        return 1
    fi

    $COMPOSE_CMD up evaluate_trajectory || {
        error "Evaluation failed."
        return 1
    }

    success "Evaluation complete."
    return 0
}

run_stage_report() {
    local report_path="${OUTPUT_PATH}/trajectory_analysis.pdf"

    info "Attempting to open evaluation report..."

    if [ ! -f "$report_path" ]; then
        error "Evaluation report not found: $report_path"
        return 1
    fi

    xdg-open "$report_path" 2>/dev/null || open "$report_path" 2>/dev/null || {
        warn "Could not open report automatically. Available at: $report_path"
    }

    success "Report is available at: $report_path"
    return 0
}

# --- Utilities ---

get_service_logs() {
    local service="$1"
    local lines="${2:-50}"

    if [ -z "$COMPOSE_CMD" ]; then
        error "COMPOSE_CMD not set. Call setup_env first."
        return 1
    fi

    info "Logs for service '$service' (last $lines lines):"
    $COMPOSE_CMD logs --tail="$lines" "$service"
    return 0
}

stop_services() {
    info "Stopping all services..."

    if [ -z "$COMPOSE_CMD" ]; then
        error "COMPOSE_CMD not set."
        return 1
    fi

    $COMPOSE_CMD stop
    success "Services stopped."
    return 0
}

ps_services() {
    if [ -z "$COMPOSE_CMD" ]; then
        error "COMPOSE_CMD not set. Call setup_env first."
        return 1
    fi

    info "Current service status:"
    $COMPOSE_CMD ps
    return 0
}

# --- Configuration Overlay ---

set_env_var() {
    local key="$1"
    local value="$2"

    export "$key=$value"
    info "Set $key=$value"
}

load_config_overlay() {
    local overlay_file="$1"

    if [ ! -f "$overlay_file" ]; then
        error "Config overlay file not found: $overlay_file"
        return 1
    fi

    info "Loading config overlay from: $overlay_file"
    set -o allexport
    source "$overlay_file"
    set +o allexport
    success "Config overlay loaded."
    return 0
}

# --- Validation ---

validate_setup() {
    info "Validating pipeline setup..."

    if [ -z "$COMPOSE_CMD" ]; then
        error "setup_env not called."
        return 1
    fi

    if [ -z "$OUTPUT_PATH" ]; then
        error "OUTPUT_PATH not set."
        return 1
    fi

    if [ -z "$BAGFILES_PATH_HOST" ]; then
        error "BAGFILES_PATH_HOST not set in environment."
        return 1
    fi

    if [ -z "$SLAM_IMAGE" ] && [ -z "$CRL_SLAM_IMAGE" ]; then
        error "Neither SLAM_IMAGE nor CRL_SLAM_IMAGE set."
        return 1
    fi

    info "Rendering compose config to validate syntax..."
    if ! $COMPOSE_CMD config > /dev/null 2>&1; then
        error "Compose configuration is invalid."
        return 1
    fi

    success "Setup validation passed."
    return 0
}

# --- Batch Operations ---

run_pipeline_full() {
    info "Running full pipeline (setup → slam → bag → evaluate → report)..."

    run_stage_setup || return 1
    run_stage_slam || return 1
    run_stage_bag || return 1
    run_stage_evaluate || return 1
    run_stage_report || return 1

    success "Full pipeline completed successfully."
    return 0
}

run_pipeline_slam_and_bag() {
    info "Running SLAM and bagfile playback (no evaluation)..."

    run_stage_setup || return 1
    run_stage_slam || return 1
    run_stage_bag || return 1

    success "SLAM and bagfile playback completed."
    return 0
}

# Export functions for use by orchestrate.sh
export -f info success warn error
export -f setup_env register_cleanup cleanup_all health_check
export -f run_stage_setup run_stage_slam run_stage_slam_nvidia run_stage_bag run_stage_record_only run_stage_evaluate run_stage_report
export -f get_service_logs stop_services ps_services
export -f set_env_var load_config_overlay validate_setup
export -f run_pipeline_full run_pipeline_slam_and_bag
