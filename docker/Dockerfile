ARG ROS_IMAGE=osrf/ros:noetic-desktop-full-focal
FROM ${ROS_IMAGE} AS simulation

SHELL ["/bin/bash", "-o", "pipefail", "-c"]

ARG DEBIAN_FRONTEND=noninteractive
ARG STONEFISH_COMMIT=b21eb8e194c570ff2f61e91aeffb38d73dc25f42
ARG BUILD_JOBS=2
ARG USER_ID=1000
ARG GROUP_ID=1000

ENV TZ=Asia/Shanghai \
    LANG=C.UTF-8 \
    LC_ALL=C.UTF-8 \
    STONEFISH_ROOT=/opt/stonefish \
    CMAKE_PREFIX_PATH=/opt/stonefish \
    LD_LIBRARY_PATH=/opt/stonefish/lib \
    NVIDIA_VISIBLE_DEVICES=all \
    NVIDIA_DRIVER_CAPABILITIES=compute,utility,graphics,display

RUN apt-get update && apt-get install -y --no-install-recommends \
        build-essential \
        ca-certificates \
        cmake \
        curl \
        git \
        libfreetype6-dev \
        libgl1-mesa-dev \
        libglm-dev \
        libsdl2-dev \
        mesa-utils \
        python3-catkin-tools \
        python3-numpy \
        python3-opencv \
        python3-pip \
        python3-pytest \
        python3-scipy \
        python3-yaml \
        ros-noetic-controller-manager \
        ros-noetic-cv-bridge \
        ros-noetic-image-transport \
        ros-noetic-message-filters \
        ros-noetic-pcl-conversions \
        ros-noetic-pcl-ros \
        ros-noetic-rqt-plot \
        ros-noetic-tf-conversions \
        xauth \
    && rm -rf /var/lib/apt/lists/*

# Proxy arguments are intentionally declared after apt installation so that
# host proxy changes do not invalidate the large Ubuntu/ROS dependency layer.
ARG HTTP_PROXY
ARG HTTPS_PROXY
ARG NO_PROXY

RUN git init /tmp/stonefish-src \
    && git -C /tmp/stonefish-src remote add origin https://github.com/patrykcieslak/stonefish.git \
    && git -C /tmp/stonefish-src fetch --depth 1 origin "${STONEFISH_COMMIT}" \
    && git -C /tmp/stonefish-src checkout --detach FETCH_HEAD \
    && cmake -S /tmp/stonefish-src -B /tmp/stonefish-build \
         -DCMAKE_BUILD_TYPE=Release \
         -DCMAKE_INSTALL_PREFIX="${STONEFISH_ROOT}" \
         -DBUILD_TESTS=OFF \
    && cmake --build /tmp/stonefish-build --parallel "${BUILD_JOBS}" \
    && cmake --install /tmp/stonefish-build \
    && rm -rf /tmp/stonefish-src /tmp/stonefish-build

RUN groupadd --gid "${GROUP_ID}" underwater \
    && useradd --uid "${USER_ID}" --gid "${GROUP_ID}" --create-home --shell /bin/bash underwater \
    && mkdir -p /workspace \
    && chown -R "${USER_ID}:${GROUP_ID}" /workspace /home/underwater

COPY docker/entrypoint.sh /usr/local/bin/underwater-entrypoint
RUN chmod +x /usr/local/bin/underwater-entrypoint

USER underwater
WORKDIR /workspace
ENTRYPOINT ["/usr/local/bin/underwater-entrypoint"]
CMD ["bash"]


FROM simulation AS full

USER root
ARG MINIFORGE_VERSION=24.11.3-0
ENV CONDA_ROOT=/opt/conda \
    PATH=/opt/conda/bin:${PATH}

RUN curl --fail --location --show-error \
      --retry 5 --retry-delay 3 --connect-timeout 20 \
      --speed-limit 1024 --speed-time 60 --progress-bar \
      "https://github.com/conda-forge/miniforge/releases/download/${MINIFORGE_VERSION}/Miniforge3-${MINIFORGE_VERSION}-Linux-x86_64.sh" \
      -o /tmp/miniforge.sh \
    && bash /tmp/miniforge.sh -b -p "${CONDA_ROOT}" \
    && rm /tmp/miniforge.sh \
    && "${CONDA_ROOT}/bin/conda" create -y -n objectnav-gfl python=3.10 pip \
    && "${CONDA_ROOT}/bin/conda" clean -afy

COPY src/image_process_ResNet50/requirements-ubuntu2004.txt /tmp/requirements-gfl.txt
COPY src/ObjectNav/objectnav_bc/requirements.txt /tmp/requirements-objectnav.txt

# The default index is the Tsinghua PyPI mirror, which is dramatically faster
# than the public index from mainland China. Override it with
#   --build-arg PIP_INDEX_URL=https://pypi.org/simple
# when building from a network where the mirror is not reachable.
ARG PIP_INDEX_URL=https://pypi.tuna.tsinghua.edu.cn/simple
# Aliyun mirrors the official cu121 wheel index. Override with
#   --build-arg TORCH_FIND_LINKS=https://download.pytorch.org/whl/cu121
# if the mirror is unavailable.
ARG TORCH_FIND_LINKS=https://mirrors.aliyun.com/pytorch-wheels/cu121/

RUN --mount=type=cache,target=/root/.cache/pip \
    "${CONDA_ROOT}/envs/objectnav-gfl/bin/python" -m pip install \
      --upgrade "pip<25" "setuptools<70" wheel

# Some domestic mirrors publish AAAA records whose route is much slower than
# IPv4. Prefer IPv4 for the large wheel downloads below.
RUN echo 'precedence ::ffff:0:0/96  100' >> /etc/gai.conf

RUN --mount=type=cache,target=/root/.cache/pip \
    env -u HTTP_PROXY -u HTTPS_PROXY -u ALL_PROXY \
        -u http_proxy -u https_proxy -u all_proxy \
    bash -c 'set -euo pipefail; \
      mkdir -p /tmp/torch-wheels; \
      curl -4 --fail --location --show-error \
        --retry 10 --retry-delay 3 --connect-timeout 20 \
        --speed-limit 1048576 --speed-time 120 --progress-bar \
        -o /tmp/torch-wheels/torch-2.1.0+cu121-cp310-cp310-linux_x86_64.whl \
        "${TORCH_FIND_LINKS%/}/torch-2.1.0%2Bcu121-cp310-cp310-linux_x86_64.whl"; \
      curl -4 --fail --location --show-error \
        --retry 10 --retry-delay 3 --connect-timeout 20 \
        --speed-limit 1048576 --speed-time 120 --progress-bar \
        -o /tmp/torch-wheels/torchvision-0.16.0+cu121-cp310-cp310-linux_x86_64.whl \
        "${TORCH_FIND_LINKS%/}/torchvision-0.16.0%2Bcu121-cp310-cp310-linux_x86_64.whl"; \
      "${CONDA_ROOT}/envs/objectnav-gfl/bin/python" -m pip install \
        /tmp/torch-wheels/*.whl \
        --index-url "${PIP_INDEX_URL}" \
        --retries 10 --timeout 120; \
      rm -rf /tmp/torch-wheels'

# MMCV's cu121/torch2.1 binary wheel matches the installed PyTorch. Its Python
# dependencies are covered by the project requirements installed below.
RUN --mount=type=cache,target=/root/.cache/pip \
    "${CONDA_ROOT}/envs/objectnav-gfl/bin/python" -m pip install \
      mmcv==2.1.0 --no-deps --only-binary=mmcv --no-index \
      --find-links https://download.openmmlab.com/mmcv/dist/cu121/torch2.1/index.html \
      --retries 10 --timeout 120

RUN --mount=type=cache,target=/root/.cache/pip \
    env -u HTTP_PROXY -u HTTPS_PROXY -u ALL_PROXY \
        -u http_proxy -u https_proxy -u all_proxy \
    "${CONDA_ROOT}/envs/objectnav-gfl/bin/python" -m pip install \
      --index-url "${PIP_INDEX_URL}" \
      --retries 10 --timeout 120 --prefer-binary \
      "opencv-python<5" pytest \
      -r /tmp/requirements-gfl.txt -r /tmp/requirements-objectnav.txt \
    && rm /tmp/requirements-gfl.txt /tmp/requirements-objectnav.txt \
    && chown -R underwater:underwater "${CONDA_ROOT}"

# ROS Noetic must use the system Python 3.8. Conda remains available at the
# end of PATH so that `conda run -n objectnav-gfl` still works in a shell.
ENV PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin:${CONDA_ROOT}/bin

USER underwater
