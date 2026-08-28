#!/usr/bin/env bash

set -euo pipefail

readonly PROGRAMMER_VERSION="2.23.0"
readonly BUNDLE_ID="programmer@${PROGRAMMER_VERSION}"
readonly DOWNLOAD_PAGE="https://www.st.com/en/development-tools/stm32cubeprog.html#st-get-software"
readonly DEFAULT_ARM64_PACKAGE_URL="https://drive.usercontent.google.com/download?id=1gX2pNBjZ8pFm_DzgV1kgPRpxuWnluyDC&export=download&confirm=t"
readonly DEFAULT_ARM64_PACKAGE_SHA256="46b844fd135627290d2d0af3d3897debfa4247ec6af7d1cea3e0e9b0fb0bd31d"

usage() {
  cat <<'EOF'
Usage: bootstrap_stm32_programmer.sh [--check-only]

Install the official STM32CubeProgrammer for the current host architecture.
On x86_64, the ST `cube` bundle CLI performs the download, license display,
integrity verification, and installation. On aarch64, the validated official
ARM64 Debian package is downloaded from the project Google Drive and installed
with apt.

STM32_PROGRAMMER_DEB can override the package with a local file.
STM32_PROGRAMMER_DEB_URL and STM32_PROGRAMMER_DEB_SHA256 can override the
download URL and required checksum together.

Options:
  --check-only  Verify the programmer without changing the machine.
  -h, --help    Show this help.
EOF
}

check_only=false
while (($#)); do
  case "$1" in
    --check-only)
      check_only=true
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      echo "Unknown option: $1" >&2
      usage >&2
      exit 2
      ;;
  esac
  shift
done

data_root="${XDG_DATA_HOME:-${HOME}/.local/share}"
default_programmer="${data_root}/stm32cube/bundles/programmer/${PROGRAMMER_VERSION}/bin/STM32_Programmer_CLI"
programmer="${STM32_PROGRAMMER:-}"

find_programmer() {
  local candidate

  if [[ -n "${programmer}" ]]; then
    return
  fi

  if command -v STM32_Programmer_CLI >/dev/null 2>&1; then
    programmer=$(command -v STM32_Programmer_CLI)
    return
  fi

  if [[ -x "${default_programmer}" ]]; then
    programmer="${default_programmer}"
    return
  fi

  if command -v dpkg-query >/dev/null 2>&1 \
      && dpkg-query -W -f='${Status}' stm32cubeprogrammer 2>/dev/null \
        | grep -q 'install ok installed'; then
    while IFS= read -r candidate; do
      if [[ "${candidate}" == */STM32_Programmer_CLI && -x "${candidate}" ]]; then
        programmer="${candidate}"
        return
      fi
    done < <(dpkg-query -L stm32cubeprogrammer 2>/dev/null)
  fi
}

check_programmer() {
  local version_output
  find_programmer
  [[ -n "${programmer}" ]] || return 1
  [[ -x "${programmer}" ]] || return 1
  version_output=$("${programmer}" --version 2>&1) || return 1
  [[ "${version_output}" == *"STM32CubeProgrammer"* \
    && "${version_output}" == *"${PROGRAMMER_VERSION}"* ]]
}

host_arch="${STM32_HOST_ARCH:-$(uname -m)}"

if check_programmer; then
  echo "STM32CubeProgrammer ${PROGRAMMER_VERSION} is available: ${programmer}"
  exit 0
fi

if [[ "${check_only}" == true ]]; then
  echo "No supported STM32 programmer is installed for ${host_arch}." >&2
  exit 1
fi

if [[ -n "${STM32_PROGRAMMER:-}" ]]; then
  echo "Configured STM32_PROGRAMMER is unavailable or is not version ${PROGRAMMER_VERSION}: ${STM32_PROGRAMMER}" >&2
  exit 1
fi

if [[ "${host_arch}" == "aarch64" || "${host_arch}" == "arm64" ]]; then
  package="${STM32_PROGRAMMER_DEB:-}"
  temporary_package=""
  package_url="${STM32_PROGRAMMER_DEB_URL:-${DEFAULT_ARM64_PACKAGE_URL}}"
  package_sha256="${STM32_PROGRAMMER_DEB_SHA256:-}"

  if [[ -z "${package}" ]]; then
    command -v curl >/dev/null 2>&1 || {
      echo "curl is required to download STM32CubeProgrammer." >&2
      exit 1
    }
    if [[ -z "${package_sha256}" ]]; then
      if [[ "${package_url}" == "${DEFAULT_ARM64_PACKAGE_URL}" ]]; then
        package_sha256="${DEFAULT_ARM64_PACKAGE_SHA256}"
      else
        echo "STM32_PROGRAMMER_DEB_SHA256 is required with a custom download URL." >&2
        exit 1
      fi
    fi
    temporary_package=$(mktemp --suffix=_arm64.deb)
    trap 'rm -f "${temporary_package}"' EXIT
    echo "Downloading official STM32CubeProgrammer ARM64 package from Google Drive"
    curl --fail --location --silent --show-error \
      --retry 3 --retry-delay 2 \
      --output "${temporary_package}" "${package_url}"
    package="${temporary_package}"
    actual_sha256=$(sha256sum "${package}" | awk '{print $1}')
    if [[ "${actual_sha256}" != "${package_sha256}" ]]; then
      echo "STM32CubeProgrammer package checksum mismatch." >&2
      echo "Expected: ${package_sha256}" >&2
      echo "Actual:   ${actual_sha256}" >&2
      exit 1
    fi
    chmod a+r "${package}"
  fi

  if [[ -z "${package}" || ! -f "${package}" ]]; then
    cat >&2 <<EOF
STM32CubeProgrammer ${PROGRAMMER_VERSION} for Linux ARM64 is distributed by ST
as a Debian package behind its license/export-control download confirmation.
Download stm32cubeprogrammer_*_arm64.deb once from:
  ${DOWNLOAD_PAGE}
Then set STM32_PROGRAMMER_DEB=/absolute/path/to/package.deb and rerun this
target.
EOF
    exit 1
  fi

  package=$(realpath "${package}")
  package_name=$(dpkg-deb --field "${package}" Package 2>/dev/null || true)
  package_arch=$(dpkg-deb --field "${package}" Architecture 2>/dev/null || true)
  package_version=$(dpkg-deb --field "${package}" Version 2>/dev/null || true)
  if [[ "${package_name}" != "stm32cubeprogrammer" ]]; then
    echo "Unexpected Debian package: ${package} (Package=${package_name:-unknown})" >&2
    exit 1
  fi
  if [[ "${package_arch}" != "arm64" ]]; then
    echo "Not an ARM64 Debian package: ${package} (Architecture=${package_arch:-unknown})" >&2
    exit 1
  fi
  if [[ "${package_version}" != "${PROGRAMMER_VERSION}"* ]]; then
    echo "Expected STM32CubeProgrammer ${PROGRAMMER_VERSION}, got ${package_version:-unknown}: ${package}" >&2
    exit 1
  fi

  privileged=()
  if ((EUID != 0)); then
    if ! command -v sudo >/dev/null 2>&1; then
      echo "sudo is required to install STM32CubeProgrammer as a non-root user." >&2
      exit 1
    fi
    # Authenticate before stdin is reserved for the package's SLA0048 prompt.
    sudo -v
    privileged=(sudo)
  fi

  echo "Installing official STM32CubeProgrammer ${package_version} ARM64 package"
  echo "Accepting the bundled ST SLA0048 license for this explicit setup target"
  printf 'A\n' | "${privileged[@]}" env DEBIAN_FRONTEND=noninteractive \
    apt-get install --yes "${package}"
  programmer=""
  if ! check_programmer; then
    echo "ARM64 package installation completed, but STM32_Programmer_CLI ${PROGRAMMER_VERSION} is unavailable." >&2
    exit 1
  fi
  echo "STM32CubeProgrammer ${PROGRAMMER_VERSION} installed: ${programmer}"
  exit 0
fi

if [[ "${host_arch}" != "x86_64" && "${host_arch}" != "amd64" ]]; then
  echo "Unsupported STM32CubeProgrammer host architecture: ${host_arch}" >&2
  exit 1
fi

cube_cli="${STM32_CUBE_CLI:-}"
if [[ -z "${cube_cli}" ]] && command -v cube >/dev/null 2>&1; then
  cube_cli=$(command -v cube)
fi

if [[ -z "${cube_cli}" ]]; then
  shopt -s nullglob
  candidates=(
    "${HOME}"/.vscode/extensions/stmicroelectronics.stm32cube-ide-core-*-linux-x64/resources/binaries/linux/x86_64/cube
  )
  shopt -u nullglob
  if ((${#candidates[@]})); then
    cube_cli="${candidates[${#candidates[@]} - 1]}"
  fi
fi

if [[ -z "${cube_cli}" || ! -x "${cube_cli}" ]]; then
  cat >&2 <<'EOF'
The official ST `cube` bundle CLI was not found.
Install the "STM32 VS Code Extension Pack" once, put `cube` on PATH, or set
STM32_CUBE_CLI=/absolute/path/to/cube. In a disposable Docker container, mount
an already installed programmer bundle read-only instead.
EOF
  exit 1
fi

echo "Installing official ST bundle ${BUNDLE_ID} with ${cube_cli}"
"${cube_cli}" bundle install --yes "${BUNDLE_ID}"

if ! check_programmer; then
  echo "Bundle installation completed but ${programmer} is unavailable." >&2
  exit 1
fi

echo "STM32CubeProgrammer ${PROGRAMMER_VERSION} installed: ${programmer}"
