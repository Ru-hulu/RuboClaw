#!/usr/bin/env bash
set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
repo_root="$(cd "${script_dir}/../.." && pwd)"
display="${DISPLAY:-:0}"
xauthority="${XAUTHORITY:-/run/user/$(id -u)/gdm/Xauthority}"

if [[ ! -f "${xauthority}" ]]; then
    echo "Xauthority file not found: ${xauthority}" >&2
    exit 1
fi

if [[ $# -eq 0 ]]; then
    set -- bash
fi

docker_tty=()
if [[ -t 0 && -t 1 ]]; then
    docker_tty=(-it)
fi

exec docker run --rm "${docker_tty[@]}" \
    --network host \
    --user "$(id -u):$(id -g)" \
    --workdir /workspace/RoboClaw \
    --env DISPLAY="${display}" \
    --env HOME=/tmp \
    --env XAUTHORITY=/tmp/.docker.xauthority \
    --env QT_X11_NO_MITSHM=1 \
    --env LIBGL_ALWAYS_SOFTWARE=1 \
    --volume /tmp/.X11-unix:/tmp/.X11-unix:rw \
    --volume "${xauthority}:/tmp/.docker.xauthority:ro" \
    --volume "${repo_root}:/workspace/RoboClaw" \
    roboclaw-ros2:jazzy "$@"
