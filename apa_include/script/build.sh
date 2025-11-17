#!/bin/bash

set -e
echo "CPU type: $1"

# 初始化变量
CPUTYPE=""
GENERATOR="Unix Makefiles"
BUILD_SUBDIR=""

# 根据传入的参数设置交叉编译环境
if [[ $1 == "arm" || $1 == "ARM" ]]; then
    CPUTYPE=arm
    export PATH=/opt/gcc-linaro-6.5.0-2018.12-x86_64_aarch64-linux-gnu/bin:$PATH
    export LD_LIBRARY_PATH=/opt/gcc-linaro-6.5.0-2018.12-x86_64-aarch64-linux-gnu/lib:$LD_LIBRARY_PATH
    export CC=aarch64-linux-gnu-gcc
    export CXX=aarch64-linux-gnu-g++
    BUILD_SUBDIR=build_arm

elif [[ $1 == "x86" || $1 == "X86" ]]; then
    CPUTYPE=x86
    export CC=gcc
    export CXX=g++
    BUILD_SUBDIR=build_x86

elif [[ $1 == "ndk" || $1 == "NDK" ]]; then
    CPUTYPE=ndk
    export NDK_HOME=/opt/android-ndk-r26b
    export PATH=$NDK_HOME/toolchains/llvm/prebuilt/linux-x86_64/bin:$PATH
    export CC=aarch64-linux-android31-clang
    export CXX=aarch64-linux-android31-clang++
    BUILD_SUBDIR=build_ndk

else
    echo "Unknown CPU type: $1"
    exit 1
fi

# 获取当前脚本路径和项目根目录路径
SCRIPT_DIR=$(cd "$(dirname "$0")"; pwd)
BASEDIR=$(realpath "${SCRIPT_DIR}/..")

# 定义所有模块及其相对路径
# 有序二维数组，每一项是 "模块名 路径"
MODULES_ORDERED=(
    "common            apa_common/common"
    "location          apa_location"
    "odometry          apa_odometry"
    "rte               apa_rte"
    "canrecv           apa_device/can"
    "eth               apa_device/eth"
    "pk_uart           apa_device/uart"
    "apa_dds           apa_device/dds"
    "avmcomm           apa_statemanage/avmcomm"
    "statemanage       apa_statemanage/statemanage"
    "uicomm            apa_statemanage/uicomm"
    "slotdetect        apa_slotdetect"
    "sensorfusion      apa_sensorfusion/sensorfusion"
    "sensorpreproc     apa_sensorfusion/sensorpreproc"
    "sensort2s         apa_sensorfusion/sensort2s"
    "objavoid          apa_sensorfusion/ultronicobjavoid"
    "pathexecute       apa_pathplan/pathexec"
    "pathplan          apa_pathplan/pathplan"
    "hybrid            apa_pathplan/pathplan/hybrid_a_star"
    "distctrl          apa_control/distancectrl"
    "apa_pk_${CPUTYPE} apa_common/main"
)


# 保存每个 build 目录用于合并 compile_commands.json
build_dirs=()
for item in "${MODULES_ORDERED[@]}"; do
    read -r target module_path <<< "$item"
    build_dir=${BASEDIR}/${BUILD_SUBDIR}/${module_path}
    build_dirs+=("$build_dir")

    mkdir -p "$build_dir"
    cd "$build_dir"

    cmake "${BASEDIR}/${module_path}" \
        -G "${GENERATOR}" \
        -DCMAKE_BUILD_TYPE=Debug \
        -DCMAKE_C_COMPILER=${CC} \
        -DCMAKE_CXX_COMPILER=${CXX} \
        -DPROJECT_BASEDIR="$BASEDIR" \
        -DBUILDTYPE=$CPUTYPE

    cmake --build . --target "$target" -j$(nproc)
    echo "build completed: ${target}"

    cmake --install . --prefix "$BASEDIR"
done

# 合并所有 compile_commands.json 为一个文件，供 IDE 使用
FINAL_COMPILE_COMMANDS="${BASEDIR}/script/compile_commands.json"
rm -f "$FINAL_COMPILE_COMMANDS"
echo "[" > "$FINAL_COMPILE_COMMANDS"
first=1
for dir in "${build_dirs[@]}"; do
    f="${dir}/compile_commands.json"
    if [[ -f "$f" ]]; then
        content=$(sed '1d;$d' "$f")
        if [[ -n "$content" ]]; then
            [[ $first -eq 0 ]] && echo "," >> "$FINAL_COMPILE_COMMANDS"
            echo "$content" >> "$FINAL_COMPILE_COMMANDS"
            first=0
        fi
    fi
done
echo "]" >> "$FINAL_COMPILE_COMMANDS"
echo "Merged compile_commands.json created at $FINAL_COMPILE_COMMANDS"
