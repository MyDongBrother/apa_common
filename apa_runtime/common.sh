#!/bin/bash

# 初始化变量
GENERATOR=""
BUILD_SUBDIR=""
TOOLCHAIN_OPT=""

# 根据传入的参数设置交叉编译环境
if [[ ${CPUTYPE} == "arm" ]]; then
    export PATH=/opt/aarch64-linux-gnu/bin:$PATH
    export LD_LIBRARY_PATH=/opt/aarch64-linux-gnu/lib:$LD_LIBRARY_PATH
    export CC=aarch64-linux-gnu-gcc
    export CXX=aarch64-linux-gnu-g++
    GENERATOR="Unix Makefiles"
    BUILD_SUBDIR=build_arm
    TOOLCHAIN_OPT=""

elif [[ ${CPUTYPE} == "x86" ]]; then
    export CC=gcc
    export CXX=g++
    GENERATOR="Unix Makefiles"
    BUILD_SUBDIR=build_x86
    TOOLCHAIN_OPT=""

elif [[ ${CPUTYPE} == "ndk" ]]; then
    export NDK_HOME=/opt/android-ndk-r26b
    export PATH=$NDK_HOME/toolchains/llvm/prebuilt/linux-x86_64/bin:$PATH
    export CC=aarch64-linux-android31-clang
    export CXX=aarch64-linux-android31-clang++
    GENERATOR="Unix Makefiles"
    BUILD_SUBDIR=build_ndk
    TOOLCHAIN_OPT="-DCMAKE_TOOLCHAIN_FILE=$NDK_HOME/build/cmake/android.toolchain.cmake \
                   -DANDROID_ABI=arm64-v8a \
                   -DANDROID_PLATFORM=android-31"
else
    echo "❌ Unknown CPU type: ${CPUTYPE}"
    exit 1
fi

# 构建目录记录
build_dirs=()

for item in "${MODULES_ORDERED[@]}"; do
    read -r target module_path <<< "$item"
    build_dir="${BASEDIR}/${BUILD_SUBDIR}/${module_path}"
    build_dirs+=("$build_dir")

    mkdir -p "$build_dir"
    cd "$build_dir"

    rm -f CMakeCache.txt

    cmake "${BASEDIR}/${module_path}" \
        -G "${GENERATOR}" \
        ${TOOLCHAIN_OPT} \
        -DCMAKE_BUILD_TYPE=${BUILD_TYPE} \
        -DCMAKE_C_COMPILER=${CC} \
        -DCMAKE_CXX_COMPILER=${CXX} \
        -DPROJECT_BASEDIR="$BASEDIR" \
        -DBUILDTYPE=$CPUTYPE

    cmake --build . --target "$target" -j$(nproc)
    echo "✅ build completed: ${target}"

    cmake --install . --prefix "$BASEDIR"
done

# 合并 compile_commands.json
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
echo "🎯 Merged compile_commands.json → $FINAL_COMPILE_COMMANDS"
