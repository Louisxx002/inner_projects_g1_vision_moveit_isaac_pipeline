#!/usr/bin/env bash
set -euo pipefail

g1_moveit_ws_root() {
  printf '%s\n' "${G1_MOVEIT_WS:-/home/louisxx/g1_moveit_ws}"
}

ros2_setup_base() {
  cd "$(g1_moveit_ws_root)"
  set +u
  source "/opt/ros/${G1_ROS_DISTRO:-jazzy}/setup.bash"
  set -u
  export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
}

ros2_setup_overlay() {
  ros2_setup_base
  set +u
  source install/setup.bash
  set -u
}

isaac_setup_common() {
  ISAAC_IMAGE="${ISAACSIM_IMAGE:-nvcr.io/nvidia/isaac-sim:5.1.0}"
  ISAAC_WORKSPACE_HOST="${G1_MOVEIT_WS:-/home/louisxx/g1_moveit_ws}"
  ISAAC_CACHE_HOST="${ISAACSIM_CACHE:-/home/louisxx/docker/isaac-sim}"
}

isaac_prepare_cache() {
  mkdir -p \
    "${ISAAC_CACHE_HOST}/cache/main/ov" \
    "${ISAAC_CACHE_HOST}/cache/main/warp" \
    "${ISAAC_CACHE_HOST}/cache/computecache" \
    "${ISAAC_CACHE_HOST}/config" \
    "${ISAAC_CACHE_HOST}/data/documents" \
    "${ISAAC_CACHE_HOST}/data/Kit" \
    "${ISAAC_CACHE_HOST}/logs"
}

isaac_build_headless_args() {
  ISAAC_RUN_ARGS=(--rm --gpus all --network=host --entrypoint bash)
  ISAAC_ENV_ARGS=(
    -e ACCEPT_EULA=Y
    -e PRIVACY_CONSENT=Y
    -e NVIDIA_VISIBLE_DEVICES="${NVIDIA_VISIBLE_DEVICES:-all}"
    -e NVIDIA_DRIVER_CAPABILITIES="${NVIDIA_DRIVER_CAPABILITIES:-all}"
    -e __GLX_VENDOR_LIBRARY_NAME="${__GLX_VENDOR_LIBRARY_NAME:-nvidia}"
  )
  ISAAC_MOUNT_ARGS=(
    -v "${ISAAC_WORKSPACE_HOST}:/workspace/g1_moveit_ws:rw"
    -v "${ISAAC_CACHE_HOST}/cache/main/ov:/isaac-sim/.cache/ov:rw"
    -v "${ISAAC_CACHE_HOST}/cache/main/warp:/isaac-sim/.cache/warp:rw"
    -v "${ISAAC_CACHE_HOST}/cache/computecache:/isaac-sim/.nv/ComputeCache:rw"
    -v "${ISAAC_CACHE_HOST}/config:/isaac-sim/.nvidia-omniverse/config:rw"
    -v "${ISAAC_CACHE_HOST}/data/documents:/isaac-sim/Documents:rw"
    -v "${ISAAC_CACHE_HOST}/data/Kit:/isaac-sim/.local/share/ov/data/Kit:rw"
    -v "${ISAAC_CACHE_HOST}/logs:/isaac-sim/.nvidia-omniverse/logs:rw"
  )
}

isaac_build_gui_args() {
  isaac_build_headless_args
  local display_value xauth_value
  local -a xauth_mount=()

  display_value="${DISPLAY:-:0}"
  xauth_value="${XAUTHORITY:-${HOME}/.Xauthority}"
  if [[ -f "${xauth_value}" ]]; then
    xauth_mount=(-v "${xauth_value}:/tmp/.docker.xauth:ro" -e XAUTHORITY=/tmp/.docker.xauth)
  fi

  ISAAC_RUN_ARGS=(--rm --gpus all --network=host --ipc=host --entrypoint bash)
  ISAAC_ENV_ARGS+=(
    -e DISPLAY="${display_value}"
    -e QT_X11_NO_MITSHM=1
  )
  ISAAC_MOUNT_ARGS+=(
    "${xauth_mount[@]}"
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw
  )
}
