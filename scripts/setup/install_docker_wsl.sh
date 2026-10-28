#!/usr/bin/env bash
# Install/verify Docker Engine inside the dedicated Ubuntu 22.04 WSL2 distro.
# Run as root: bash /path/to/scripts/setup/install_docker_wsl.sh
# This script does not install Docker Desktop, run containers, contact a robot,
# change Windows networking/firewall settings, or delete Docker/container data.
# Official references checked 2026-10-25:
# https://docs.docker.com/engine/install/ubuntu/
# https://docs.docker.com/engine/install/linux-postinstall/
# https://learn.microsoft.com/en-us/windows/wsl/systemd
set -Eeuo pipefail
export PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin

STEP=preflight
task_tmp=""

log() { printf '[RM65 Docker] %s\n' "$*"; }
die() { printf '[RM65 Docker] ERROR: %s\n' "$*" >&2; exit 1; }
on_error() {
    printf '[RM65 Docker] ERROR: step=%s line=%s exit=%s; installation/verification incomplete.\n' \
        "$STEP" "$2" "$1" >&2
    exit "$1"
}
cleanup() {
    if [[ -n "$task_tmp" && -d "$task_tmp" ]]; then
        rm -f -- "$task_tmp/docker.asc" "$task_tmp/docker.sources"
        rmdir -- "$task_tmp" 2>/dev/null || true
    fi
}
trap 'on_error "$?" "$LINENO"' ERR
trap cleanup EXIT

if [[ $# -gt 0 ]]; then
    if [[ $# -eq 1 && ( "$1" == --help || "$1" == -h ) ]]; then
        log 'Usage: run this script as root in Ubuntu 22.04 WSL2 with systemd as PID 1.'
        log 'Installs missing official Docker Engine/Compose/Buildx packages; runs no container.'
        exit 0
    fi
    die 'No arguments are accepted except --help.'
fi
[[ "$(uname -s)" == Linux ]] || die 'A Linux shell is required; do not run this in Windows Bash emulation.'
[[ "$EUID" -eq 0 ]] || die 'Run as root, for example: wsl -d RM65-Servo -u root -- bash /path/to/this/script'
[[ -r /etc/os-release ]] || die '/etc/os-release is unavailable.'
# shellcheck disable=SC1091
. /etc/os-release
[[ "${ID:-}" == ubuntu && "${VERSION_ID:-}" == 22.04 ]] || \
    die 'Only Ubuntu 22.04 (jammy) is accepted; the current distro will not be modified.'
kernel_release="$(uname -r)"
[[ "${kernel_release,,}" == *microsoft* && "${kernel_release,,}" == *wsl2* ]] || \
    die "WSL2 is required; current kernel is $kernel_release. WSL1/native/custom kernels are not modified."
if [[ "$(cat /proc/1/comm)" != systemd || ! -d /run/systemd/system ]] || ! command -v systemctl >/dev/null; then
    cat >&2 <<'EOF'
[RM65 Docker] ERROR: systemd must be running as PID 1 before installing Docker.
Inside Ubuntu, install systemd and systemd-sysv if missing, and set the following
in /etc/wsl.conf while preserving its other sections:
    [boot]
    systemd=true
Then, in PowerShell, terminate ONLY the dedicated distro:
    wsl --terminate RM65-Servo
Reopen it and rerun this script. WSL 0.67.6 or newer is required by Microsoft.
This script does not terminate WSL or change Windows settings itself.
EOF
    exit 1
fi
log "Preflight OK: Ubuntu 22.04, WSL2 ($kernel_release), root, systemd."

if [[ -L /var/run/docker.sock ]]; then
    socket_target="$(readlink -f /var/run/docker.sock || true)"
    [[ "$socket_target" != *docker-desktop* && "$socket_target" != /mnt/wsl/* ]] || \
        die 'The Docker socket is supplied by Docker Desktop/another WSL integration; refusing to replace it.'
fi

# Starting an old stopped daemon can restore restart-policy containers. Do not
# inspect/modify their settings or start them implicitly in this provisioning step.
if ! systemctl is-active --quiet docker.service; then
    [[ ! -e /etc/docker/daemon.json && ! -L /etc/docker/daemon.json ]] || \
        die 'A stopped daemon has an existing /etc/docker/daemon.json; review its data root/restart behavior before starting it.'
    if [[ -d /var/lib/docker/containers ]] && \
        [[ -n "$(find /var/lib/docker/containers -mindepth 1 -maxdepth 1 -print -quit)" ]]; then
        die 'Existing stopped Docker containers were found; refusing a start that could restore them. Data is unchanged.'
    fi
    if systemctl cat docker.service 2>/dev/null | grep -Eq -- '--(data-root|config-file)([=[:space:]]|$)'; then
        die 'The stopped Docker service uses a custom data/config path; review it before starting the daemon.'
    fi
    for unit_root in /etc/systemd/system /run/systemd/system; do
        if [[ -e "$unit_root/docker.service" || -L "$unit_root/docker.service" ]] || \
            { [[ -d "$unit_root/docker.service.d" ]] && \
                [[ -n "$(find "$unit_root/docker.service.d" -mindepth 1 -maxdepth 1 -type f -print -quit)" ]]; }; then
            die 'The stopped Docker service has a local unit override; review its startup/data path before provisioning.'
        fi
    done
fi

package_installed() {
    [[ "$(dpkg-query -W -f='${Status}' "$1" 2>/dev/null || true)" == 'install ok installed' ]]
}
apt_install() {
    DEBIAN_FRONTEND=noninteractive apt-get -o DPkg::Lock::Timeout=120 -o Acquire::Retries=3 \
        install --yes --no-install-recommends --no-remove "$@"
}
apt_update() {
    apt-get -o DPkg::Lock::Timeout=120 -o Acquire::Retries=3 update
}

if command -v docker >/dev/null && docker --version >/dev/null 2>&1 && \
    docker compose version >/dev/null 2>&1 && docker buildx version >/dev/null 2>&1; then
    log 'Existing Docker CLI, Compose and Buildx found; verifying/reusing them without an upgrade.'
else
    conflicts=()
    for package in docker.io docker-compose docker-compose-v2 docker-doc docker-buildx podman-docker containerd runc; do
        if package_installed "$package"; then conflicts+=("$package"); fi
    done
    if [[ ${#conflicts[@]} -gt 0 ]]; then
        die "Conflicting/non-official packages need manual review: ${conflicts[*]}. No package or data was removed."
    fi
    STEP=bootstrap
    log 'Installing TLS/download prerequisites from the existing Ubuntu apt sources.'
    apt_update
    apt_install ca-certificates curl

    STEP=official-repository
    task_tmp="$(mktemp -d -t rm65-docker.XXXXXXXX)"
    architecture="$(dpkg --print-architecture)"
    [[ "$architecture" == amd64 || "$architecture" == arm64 ]] || die "Unsupported WSL architecture: $architecture"
    cat > "$task_tmp/docker.sources" <<EOF
Types: deb
URIs: https://download.docker.com/linux/ubuntu
Suites: jammy
Components: stable
Architectures: $architecture
Signed-By: /etc/apt/keyrings/docker.asc
EOF
    repository=/etc/apt/sources.list.d/docker.sources
    # Do not create duplicate sources with conflicting Signed-By keys, and do
    # not overwrite administrator-owned repository definitions.
    while IFS= read -r existing_source; do
        [[ "$existing_source" == "$repository" ]] || \
            die "Docker apt source already exists in $existing_source; review it manually rather than creating a duplicate."
    done < <(grep -RIlE --include='*.list' --include='*.sources' \
        '^[[:space:]]*[^#[:space:]].*download[.]docker[.]com/linux/ubuntu' \
        /etc/apt/sources.list /etc/apt/sources.list.d 2>/dev/null || true)
    if [[ -e "$repository" ]] && ! cmp -s "$task_tmp/docker.sources" "$repository"; then
        die "$repository already exists with different contents; it was not overwritten."
    fi
    install -m 0755 -d /etc/apt/keyrings /etc/apt/sources.list.d
    curl --fail --show-error --silent --location --retry 3 --connect-timeout 20 --max-time 120 \
        https://download.docker.com/linux/ubuntu/gpg -o "$task_tmp/docker.asc"
    grep -q -- '-----BEGIN PGP PUBLIC KEY BLOCK-----' "$task_tmp/docker.asc" || die 'Downloaded Docker signing key is invalid.'
    install -m 0644 "$task_tmp/docker.asc" /etc/apt/keyrings/docker.asc
    install -m 0644 "$task_tmp/docker.sources" "$repository"
    apt_update

    STEP=packages
    missing_packages=()
    for package in docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin; do
        if ! package_installed "$package"; then missing_packages+=("$package"); fi
    done
    if [[ ${#missing_packages[@]} -gt 0 ]]; then
        log "Installing missing packages: ${missing_packages[*]}"
        apt_install "${missing_packages[@]}"
    fi
fi

STEP=service
log 'Enabling Docker/containerd under systemd; no existing service is restarted.'
systemctl enable containerd.service docker.service
if ! systemctl start containerd.service docker.service; then
    systemctl --no-pager --full status docker.service containerd.service >&2 || true
    journalctl --unit docker.service --lines 50 --no-pager >&2 || true
    die 'Docker did not start. Review the service diagnostics above; no container was requested.'
fi
systemctl is-active --quiet docker.service || die 'docker.service is not active.'

STEP=verification
# Explicitly target this distro's local Unix socket. Ignore remote/desktop
# context environment variables; never validate a remote robot host by mistake.
docker --version
docker compose version
docker buildx version
server_summary="$(env -u DOCKER_HOST -u DOCKER_CONTEXT -u DOCKER_TLS_VERIFY -u DOCKER_CERT_PATH \
    timeout 60s docker --host unix:///var/run/docker.sock info \
    --format 'ServerVersion={{.ServerVersion}}; OS={{.OperatingSystem}}; Driver={{.Driver}}; Containers={{.Containers}}; Running={{.ContainersRunning}}')"
[[ "${server_summary,,}" != *"docker desktop"* ]] || die 'The local socket reports Docker Desktop; a distro-local Docker Engine is required.'
printf '%s\n' "$server_summary"
log 'Docker Engine, Compose and Buildx verified. No image was pulled and no container was run.'
log 'Use root/sudo for Docker; this script did not change user groups or Windows settings.'
