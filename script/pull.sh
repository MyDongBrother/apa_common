#!/bin/bash

set -e

# 自动确保 remote 存在
ensure_remote() {
    local name=$1
    local url=$2

    if git remote get-url "$name" >/dev/null 2>&1; then
        echo "Remote '$name' 已存在：$(git remote get-url $name)"
    else
        echo "Remote '$name' 不存在，正在添加：$url"
        git remote add "$name" "$url"
    fi
}

# subtree 自动 add/pull
pull_subtree() {
    local prefix=$1
    local remote=$2
    local branch=$3

    echo "处理 subtree: $prefix ← $remote/$branch"

    if [ ! -d "$prefix" ]; then
        echo "目录 '$prefix' 不存在，自动执行 subtree add"
        git subtree add --prefix="$prefix" "$remote" "$branch" --squash
    else
        echo "目录 '$prefix' 已存在，执行 subtree pull"
        git subtree pull --prefix="$prefix" "$remote" "$branch" --squash
    fi

    echo
}

###=== 配置区 ===###

ensure_remote "apa_services" "https://github.com/MyDongBrother/apa_services.git"
ensure_remote "apa_runtime"  "https://github.com/MyDongBrother/apa_runtime.git"
ensure_remote "apa_common"   "https://github.com/MyDongBrother/apa_common.git"

###=== 执行 subtree 逻辑 ===###

pull_subtree "apa_services" "apa_services" "master"
pull_subtree "apa_runtime"  "apa_runtime"  "public"
pull_subtree "apa_common"   "apa_common"   "main"

echo "全部 subtree 更新完成！"
