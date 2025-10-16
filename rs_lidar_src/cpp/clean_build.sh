#!/bin/bash

# clean_build.sh - 清理 Python 构建产物

echo "开始清理构建文件..."

# 定义要删除的目录
DIRECTORIES=("build")

# 遍历并删除目录
for dir in "${DIRECTORIES[@]}"; do
    if [ -d "$dir" ]; then
        rm -rf "$dir"
        echo "✅ 已删除: $dir/"
    else
        echo "ℹ️  不存在: $dir/"
    fi
done

echo "清理完成！"