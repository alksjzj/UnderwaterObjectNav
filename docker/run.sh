#!/usr/bin/env bash
set -Eeuo pipefail

project_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
compose_file="${project_dir}/docker/compose.yaml"

export HOST_UID="$(id -u)"
export HOST_GID="$(id -g)"
export BUILD_JOBS="${BUILD_JOBS:-2}"

compose() {
  docker compose -f "${compose_file}" "$@"
}

allow_x11() {
  if [[ -z "${DISPLAY:-}" ]]; then
    echo "DISPLAY is empty; graphical Stonefish cannot open a window." >&2
    return 1
  fi
  xhost "+SI:localuser:$(id -un)" >/dev/null
  trap 'xhost "-SI:localuser:$(id -un)" >/dev/null 2>&1 || true' EXIT
}

usage() {
  cat <<'EOF'
Usage: ./docker/run.sh COMMAND

Commands:
  build       Build ROS Noetic + Stonefish + Python 3.10 image
  compile     Build this Catkin workspace inside the container
  verify      Check ROS, Stonefish, GPU, Python and project tests
  shell       Open an interactive configured shell
  demo        Start the low-quality empty-pool Stonefish scene
  tracking    Start BricsBot tracking with thrusters disabled
  objectnav   Start online ObjectNav (requires model weights)
  expert      Start rule expert collection (requires detector_checkpoint:=...)

Extra arguments after objectnav or expert are passed to roslaunch.
EOF
}

command_name="${1:-}"
if (( $# > 0 )); then shift; fi
case "${command_name}" in
  build)
    if ! docker buildx version >/dev/null 2>&1; then
      echo "docker buildx is required (install the docker-buildx-plugin)." >&2
      exit 1
    fi
    # The build needs host networking: the host proxy listens on 127.0.0.1.
    # BuildKit therefore needs the explicit network.host entitlement.
    docker buildx build \
      --load \
      --file "${project_dir}/docker/Dockerfile" \
      --target full \
      --tag underwater-object-nav:noetic-stonefish-1.6 \
      --network host \
      --allow network.host \
      --build-arg USER_ID="${HOST_UID}" \
      --build-arg GROUP_ID="${HOST_GID}" \
      --build-arg BUILD_JOBS="${BUILD_JOBS}" \
      --build-arg HTTP_PROXY="${HTTP_PROXY:-}" \
      --build-arg HTTPS_PROXY="${HTTPS_PROXY:-}" \
      --build-arg NO_PROXY="${NO_PROXY:-localhost,127.0.0.0/8,::1}" \
      --build-arg PIP_INDEX_URL="${PIP_INDEX_URL:-https://pypi.tuna.tsinghua.edu.cn/simple}" \
      --build-arg TORCH_FIND_LINKS="${TORCH_FIND_LINKS:-https://mirrors.aliyun.com/pytorch-wheels/cu121/}" \
      "${project_dir}"
    ;;
  compile)
    compose run --rm underwater-nav bash -lc \
      'catkin_make -j"${BUILD_JOBS:-2}" -DPYTHON_EXECUTABLE=/usr/bin/python3 -DPYTHON3_EXECUTABLE=/usr/bin/python3 && source devel/setup.bash && rospack find stonefish_ros && rospack find objectnav && rospack find expert_data_rule'
    ;;
  verify)
    allow_x11
    compose run --rm underwater-nav bash -lc \
      'set -e; nvidia-smi --query-gpu=name,driver_version,memory.total --format=csv,noheader; glxinfo -B | grep -E "direct rendering|OpenGL vendor|OpenGL renderer|OpenGL core profile version"; test "$(rosversion -d)" = noetic; pkg-config --modversion stonefish; source devel/setup.bash; rospack find aquaflow_stonefish; rospack find expert_data_rule; conda run -n objectnav-gfl python -c "import torch, mmcv, mmengine, mmdet; print(\"PyTorch\", torch.__version__, \"CUDA\", torch.version.cuda, \"available\", torch.cuda.is_available()); print(\"MMCV\", mmcv.__version__, \"MMEngine\", mmengine.__version__, \"MMDetection\", mmdet.__version__)"; cd /workspace/src/algorithm/objectnav/ObjectNav; PYTHONPATH="$PWD" conda run -n objectnav-gfl python -m pytest -q tests; cd ../expert_data_rule; PYTHONPATH="$PWD" conda run -n objectnav-gfl python -m pytest -q tests'
    ;;
  shell)
    allow_x11
    compose run --rm underwater-nav bash
    ;;
  demo)
    allow_x11
    compose run --rm underwater-nav bash -lc \
      'source devel/setup.bash && roslaunch aquaflow_stonefish aquaflow_pool.launch scene:=empty_pool graphics_quality:=low'
    ;;
  tracking)
    allow_x11
    compose run --rm underwater-nav bash -lc \
      'source devel/setup.bash && roslaunch aquaflow_stonefish tracking.launch scene:=random_pillars vehicle_model:=bricsbot enable:=false'
    ;;
  objectnav)
    allow_x11
    compose run --rm underwater-nav bash -lc \
      'source devel/setup.bash && roslaunch objectnav objectnav_online.launch enable:=false conda_executable:=/opt/conda/bin/conda conda_env:=objectnav-gfl record_root:=/workspace/underwater_objectnav_online_runs graphics_quality:=low "$@"' -- "$@"
    ;;
  expert)
    if (( $# == 0 )); then
      echo 'Pass detector_checkpoint:=/workspace/path/to/checkpoint.pth (the trained weights are not included).' >&2
      exit 1
    fi
    allow_x11
    compose run --rm underwater-nav bash -lc \
      'source devel/setup.bash && roslaunch expert_data_rule continuous_expert_collection.launch enable_thrusters:=false conda_executable:=/opt/conda/bin/conda conda_env:=objectnav-gfl dataset_root:=/workspace/underwater_objectnav_dataset "$@"' -- "$@"
    ;;
  *)
    usage
    exit 1
    ;;
esac
