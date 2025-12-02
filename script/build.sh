#!/bin/bash
set -e

CPUTYPE="$1"   # arm / x86 / ndk
BUILD_TYPE="Debug" # [Debug|Release|RelWithDebInfo|MinSizeRel]

# ===== CPU 参数检查 =====
if [[ -z "$CPUTYPE" ]]; then
    echo "❌ ERROR: missing CPU type parameter"
    echo "Usage: ./build.sh [arm|x86|ndk]"
    exit 1
fi

case "$CPUTYPE" in
    arm|ARM)   CPUTYPE="arm" ;;
    x86|X86)   CPUTYPE="x86" ;;
    ndk|NDK)   CPUTYPE="ndk" ;;
    *)
        echo "❌ ERROR: invalid CPU type '$CPUTYPE'"
        echo "Allowed: arm | x86 | ndk"
        exit 1
        ;;
esac

# ===== 模块定义 =====
MODULES_ORDERED=(
    "services              apa_services"
    "commons               apa_common"
    "apa_nav_${CPUTYPE}    nav_common/src"
)

SCRIPT_DIR=$(cd "$(dirname "$0")"; pwd)
BASEDIR=$(realpath "${SCRIPT_DIR}/..")

# ===== 交给 common.sh =====
source "${BASEDIR}/apa_runtime/common.sh"
