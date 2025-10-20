#!/bin/bash

# setup.sh - 构建 Python wheel 包
# 用法: ./setup.sh

# 设置严格模式
set -euo pipefail

# 检查是否在项目根目录（存在 setup.py）
if [ ! -f "setup.py" ]; then
    echo "❌ 错误: 当前目录下未找到 setup.py 文件！"
    echo "请确保在包含 setup.py 的项目根目录中运行此脚本。"
    exit 1
fi

# 检查是否安装了 wheel 包
if ! python -c "import wheel" &> /dev/null; then
    echo "⚠️  'wheel' 包未安装，正在安装..."
    pip install wheel
    if [ $? -ne 0 ]; then
        echo "❌ 安装 'wheel' 失败，请检查 pip 和网络设置。"
        exit 1
    fi
fi

echo "📦 正在构建 wheel 包..."
python setup.py bdist_wheel

# 检查构建是否成功
if [ $? -eq 0 ]; then
    echo "✅ 构建成功！生成的 wheel 文件位于 ./dist/ 目录中。"
else
    echo "❌ 构建失败，请检查上面的错误信息。"
    exit 1
fi