#!/bin/bash
set -e
if [[ ! -z "$WSL_DISTRO_NAME" ]]; then
    ADB_CMD="adb.exe"
else
    ADB_CMD="adb"
fi
# Android 目标目录
TARGET_DIR="/data/local/tmp/lib64"

echo "[INFO] push libs to $TARGET_DIR"

# 创建目录
$ADB_CMD shell "mkdir -p $TARGET_DIR"

# 推送公共库
$ADB_CMD push apa_msgs/ndk/libbmi_msgs.so*               $TARGET_DIR/
$ADB_CMD push apa_services/ndk/libservices.so*           $TARGET_DIR/


# FastDDS 依赖
$ADB_CMD push fastdds/ndk/lib/*.so                       $TARGET_DIR/
$ADB_CMD push fastcdr/ndk/lib/*.so                       $TARGET_DIR/

# OpenCV 依赖
$ADB_CMD push opencv/ndk/lib/*.so  $TARGET_DIR/

# 其他系统三方 依赖
$ADB_CMD push extlib/ndk/*.so  $TARGET_DIR/

# 设置权限
$ADB_CMD shell "chmod 755 $TARGET_DIR/*"

echo "[OK] install finished."
