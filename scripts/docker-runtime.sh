#!/usr/bin/env bash
# Explicit commands only: starting a container never starts motors implicitly.
set -eo pipefail
root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
image="${STACK_DOCKER_IMAGE:-rebotarm-stack:humble}"
action="${1:-help}"
[[ $# -eq 0 ]] || shift
case "$action" in
    build) exec docker build -f "$root/docker/runtime/Dockerfile" -t "$image" "$root" "$@" ;;
    run)
        [[ $# -gt 0 ]] || { echo 'run requires an explicit command' >&2; exit 2; }
        domain="${ROS_DOMAIN_ID:-0}"
        [[ "$domain" =~ ^[0-9]+$ ]] && ((domain <= 232)) || { echo 'Invalid ROS_DOMAIN_ID' >&2; exit 2; }
        args=(--rm --init --network host --env "ROS_DOMAIN_ID=$domain")
        case "${STACK_DOCKER_PRIVILEGED:-0}" in
            0) ;;
            1) args+=(--privileged --volume /dev:/dev --volume /run/udev:/run/udev:ro) ;;
            *) echo 'STACK_DOCKER_PRIVILEGED must be 0 or 1' >&2; exit 2 ;;
        esac
        if [[ -n "${ROBOT_EXTERNAL_ROOT:-}" ]]; then
            [[ "$ROBOT_EXTERNAL_ROOT" = /* && -d "$ROBOT_EXTERNAL_ROOT" ]] || { echo 'External root must be an existing absolute directory' >&2; exit 2; }
            # Same path preserves absolute symlink-install targets in the overlay.
            args+=(--volume "$ROBOT_EXTERNAL_ROOT:$ROBOT_EXTERNAL_ROOT:ro" --env "ROBOT_EXTERNAL_SETUP=${ROBOT_EXTERNAL_SETUP:?Set external setup path}")
        elif [[ -n "${ROBOT_EXTERNAL_SETUP:-}" ]]; then
            echo 'ROBOT_EXTERNAL_SETUP requires ROBOT_EXTERNAL_ROOT' >&2; exit 2
        fi
        exec docker run "${args[@]}" "$image" "$@"
        ;;
    *) echo "Usage: $0 build | run COMMAND [ARG ...]"; [[ "$action" = help ]] ;;
esac
