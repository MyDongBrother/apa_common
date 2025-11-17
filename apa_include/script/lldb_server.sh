#!/bin/bash

set -e

LIB_REMOTE="/data/local/tmp/playctrl/lib"
EXLIB_REMOTE="/data/local/tmp/playctrl/extlib"
BIN_REMOTE="/data/local/tmp/playctrl"
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
for lib in ./apa_include/lib/ndk/*.so; do
    base=$(basename "$lib")
    sync_file "$lib" "$LIB_REMOTE/$base"
done

# 同步可执行文件
for bin in ./apa_include/bin/ndk/*; do
    [ -f "$bin" ] || continue
    base=$(basename "$bin")
    sync_file "$bin" "$BIN_REMOTE/$base"
done

# # 同步 .json 文件
# for json in ./apa_include/config/yuan/*.json; do
#     [ -f "$json" ] || continue
#     base=$(basename "$json")
#     sync_file "$json" "$BIN_REMOTE/$base"
# done

# # 同步 run.sh 文件
# for sh in ./apa_include/script/run.sh; do
#     [ -f "$sh" ] || continue
#     base=$(basename "$sh")
#     sync_file "$sh" "$BIN_REMOTE/$base"
# done


# 同步 三方so 文件
for lib in ./extlib/lib/ndk/*.so; do
    [ -f "$lib" ] || continue
    base=$(basename "$lib")
    sync_file "$lib" "$EXLIB_REMOTE/$base"
done

# # 同步 opencv so 文件
# for lib in ./extlib/opencv/ndk/sdk/native/libs/arm64-v8a/*.so; do
#     [ -f "$lib" ] || continue
#     base=$(basename "$lib")
#     sync_file "$lib" "$EXLIB_REMOTE/$base"
# done

$ADB_CMD shell sync

# 启动 lldb-server
# $ADB_CMD root
# $ADB_CMD forward tcp:9098 tcp:9098
# $ADB_CMD shell pkill lldb-server
# $ADB_CMD shell "nohup /data/local/tmp/tools/lldb-server platform --listen *:9098 --server > /data/local/tmp/lldb-server.log 2>&1 &"
