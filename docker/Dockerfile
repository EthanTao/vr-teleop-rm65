# Ubuntu container for XRoboToolkit PC Service + Python VR teleop (PICO 4 Ultra).
# On Apple Silicon Mac, builds natively with arm64 headless PC Service.

FROM ubuntu:22.04

ENV DEBIAN_FRONTEND=noninteractive
ENV LANG=C.UTF-8
ENV PATH="/opt/conda/bin:$PATH"

RUN apt-get update && apt-get install -y --no-install-recommends \
    ca-certificates curl git build-essential cmake \
    python3 python3-pip python3-venv \
    libgl1 libglib2.0-0 libegl1 libxext6 libxrender1 libsm6 \
    iproute2 net-tools \
    && rm -rf /var/lib/apt/lists/*

# Miniforge = conda + conda-forge only (no Anaconda ToS in non-interactive Docker builds)
RUN ARCH="$(uname -m)" \
    && curl -fsSL "https://github.com/conda-forge/miniforge/releases/latest/download/Miniforge3-Linux-${ARCH}.sh" -o /tmp/miniforge.sh \
    && bash /tmp/miniforge.sh -b -p /opt/conda \
    && rm /tmp/miniforge.sh \
    && /opt/conda/bin/conda create -n xrobo python=3.10 -y \
    && /opt/conda/bin/conda clean -afy

COPY docker/install_pc_service.sh /tmp/install_pc_service.sh
RUN chmod +x /tmp/install_pc_service.sh && /tmp/install_pc_service.sh

WORKDIR /workspace
COPY . /workspace/

SHELL ["/bin/bash", "-lc"]

RUN source /opt/conda/etc/profile.d/conda.sh \
    && conda activate xrobo \
    && pip install uv \
    && uv pip install numpy meshcat placo tyro \
    && chmod +x docker/install_sdk.sh \
    && docker/install_sdk.sh \
    && uv pip install -e . \
    && (uv pip uninstall -y xrobotoolkit-sdk-mock 2>/dev/null || true)

# docker exec does not inherit conda activation performed by entrypoint.sh.
# Use the same interpreter that received the project and XR SDK dependencies.
ENV PATH="/opt/conda/envs/xrobo/bin:$PATH"

COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

EXPOSE 63901/tcp 63901/udp 9090/tcp 9090/udp 8089/udp 7000/tcp 7001/tcp

ENTRYPOINT ["/entrypoint.sh"]
# Keep the XR service container alive. Motion starts only through an explicit
# sim/hardware command with a calibrated home pose.
CMD ["sleep", "infinity"]
