#!/usr/bin/env bash

set -euo pipefail

PACKAGES=(
  ca-certificates
  curl
  gcc-arm-none-eabi
  binutils-arm-none-eabi
  libnewlib-arm-none-eabi
  libstdc++-arm-none-eabi-newlib
)

usage() {
  cat <<'EOF'
Usage: bootstrap_stm32_tools.sh [--check-only] [--no-update]

Install the STM32 cross-compiler from Ubuntu/Debian packages.
STM32CubeProgrammer is handled separately by bootstrap_stm32_programmer.sh.

Options:
  --check-only  Verify tools without changing the machine.
  --no-update   Skip apt-get update before installing packages.
  -h, --help    Show this help.
EOF
}

check_only=false
update_package_index=true

while (($#)); do
  case "$1" in
    --check-only)
      check_only=true
      ;;
    --no-update)
      update_package_index=false
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

missing=()
for command_name in \
  arm-none-eabi-gcc \
  arm-none-eabi-g++ \
  arm-none-eabi-objcopy \
  arm-none-eabi-size; do
  if ! command -v "${command_name}" >/dev/null 2>&1; then
    missing+=("${command_name}")
  fi
done

if ! command -v curl >/dev/null 2>&1; then
  missing+=(curl)
fi

if command -v arm-none-eabi-g++ >/dev/null 2>&1; then
  for library_name in libstdc++.a libsupc++.a; do
    library_path=$(arm-none-eabi-g++ -print-file-name="${library_name}" 2>/dev/null || true)
    if [[ -z "${library_path}" || ! -f "${library_path}" ]]; then
      missing+=("ARM C++ runtime (${library_name})")
    fi
  done
fi

if [[ "${check_only}" == true ]]; then
  if ((${#missing[@]})); then
    printf 'Missing STM32 tool: %s\n' "${missing[@]}" >&2
    exit 1
  fi
  echo "STM32 build tools are available."
  exit 0
fi

if ((${#missing[@]} == 0)); then
  echo "STM32 build tools are already available."
  exit 0
fi

if ! command -v apt-get >/dev/null 2>&1; then
  echo "Automatic setup supports apt-based Ubuntu/Debian hosts only." >&2
  exit 1
fi

privileged=()
if ((EUID != 0)); then
  if ! command -v sudo >/dev/null 2>&1; then
    echo "sudo is required to install packages as a non-root user." >&2
    exit 1
  fi
  privileged=(sudo)
fi

if [[ "${update_package_index}" == true ]]; then
  "${privileged[@]}" env DEBIAN_FRONTEND=noninteractive apt-get update
fi
"${privileged[@]}" env DEBIAN_FRONTEND=noninteractive apt-get install \
  --yes --no-install-recommends "${PACKAGES[@]}"

exec "$0" --check-only
