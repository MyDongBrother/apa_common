#!/bin/bash

set -e

LIB_REMOTE="/data/local/tmp/apa_vis/lib"
EXLIB_REMOTE="/data/local/tmp/apa_vis/extlib"
BIN_REMOTE="/data/local/tmp/apa_vis"
COMMLIB_REMOTE="/data/local/tmp/lib64"
CONFIG_REMOTE="/data/local/tmp/apa_vis/config"
# 如果在 WSL 环境下，用 adb.exe，否则用 adb
if [[ ! -z "$WSL_DISTRO_NAME" ]]; then
    ADB_CMD="adb.exe"
else
    ADB_CMD="adb"
fi
# 构建工程
cd script
./build.sh ndk
cd ..

# 定义同步函数
sync_file() {
    local local_file="$1"
    local remote_file="$2"

    if [ ! -f "$local_file" ]; then
        echo "❌ Local file not found: $local_file"
        return
    fi

    local_md5=$(md5sum "$local_file" | awk '{print $1}')
    remote_md5=$($ADB_CMD shell "md5sum '$remote_file' 2>/dev/null || true" | awk '{print $1}')

    if [ "$local_md5" != "$remote_md5" ]; then
        echo "📤 Pushing $local_file → $remote_file"
        $ADB_CMD push "$local_file" "$remote_file"
    else
        echo "✅ Up-to-date: $local_file"
    fi
}

# 同步 .so 文件
for lib in ./vis_common/lib/ndk/*.so; do
    base=$(basename "$lib")
    sync_file "$lib" "$LIB_REMOTE/$base"
done

# 同步可执行文件
for bin in ./vis_common/bin/ndk/apa_vis_ndk; do
    [ -f "$bin" ] || continue
    base=$(basename "$bin")
    sync_file "$bin" "$BIN_REMOTE/$base"
done

# # 同步 run.sh 文件
for sh in ./vis_common/script/run.sh; do
    [ -f "$sh" ] || continue
    base=$(basename "$sh")
    sync_file "$sh" "$BIN_REMOTE/$base"
done

# # 同步 stop.sh 文件
for sh in ./vis_common/script/stop.sh; do
    [ -f "$sh" ] || continue
    base=$(basename "$sh")
    sync_file "$sh" "$BIN_REMOTE/$base"
done

# # 同步 json 配置文件
for json in ./vis_common/config/*.json; do
    [ -f "$json" ] || continue
    base=$(basename "$json")
    sync_file "$json" "$CONFIG_REMOTE/$base"
done

# # 同步 libservices.so 文件
for lib in ./apa_runtime/apa_services/ndk/libservices.so; do
    [ -f "$lib" ] || continue
    base=$(basename "$lib")
    sync_file "$lib" "$COMMLIB_REMOTE/$base"
done

# # 同步 libcommons.so 文件
for lib in ./apa_runtime/apa_common/ndk/libcommons.so; do
    [ -f "$lib" ] || continue
    base=$(basename "$lib")
    sync_file "$lib" "$COMMLIB_REMOTE/$base"
done


# # 同步 config 目录（递归）
# echo "📁 Syncing config directory..."
# $ADB_CMD push ./vis_common/config/ai "$BIN_REMOTE"
# $ADB_CMD push ./vis_common/config/avm "$BIN_REMOTE"

# # # 同步 三方so 文件
# for lib in ./extlib/lib/ndk/*.so; do
#     [ -f "$lib" ] || continue
#     base=$(basename "$lib")
#     sync_file "$lib" "$EXLIB_REMOTE/$base"
# done