#!/bin/bash

# cmake_make.sh - CMake 构建脚本
# 用法: ./cmake_make.sh

# 启用严格模式：出错即退出，未定义变量报错
set -euo pipefail

# 定义构建目录
BUILD_DIR="build"

echo "🔧 开始构建流程..."

# 检查是否安装了必要的工具
if ! command -v cmake &> /dev/null; then
    echo "❌ 错误: 'cmake' 未安装。"
    echo "请运行: sudo apt update && sudo apt install cmake"
    exit 1
fi

if ! command -v make &> /dev/null; then
    echo "❌ 错误: 'make' 未安装。"
    echo "请运行: sudo apt install build-essential"
    exit 1
fi

# 创建 build 目录（如果不存在）
if [ ! -d "$BUILD_DIR" ]; then
    echo "📁 创建 build 目录..."
    mkdir -p "$BUILD_DIR"
else
    echo "📁 使用已存在的 build 目录。"
fi

# 进入 build 目录
cd "$BUILD_DIR"

# 执行 cmake 并构建
echo "⚙️  运行 CMake 配置..."
cmake ./../../src/

if [ $? -ne 0 ]; then
    echo "❌ CMake 配置失败，请检查 src/ 路径和 CMakeLists.txt"
    exit 1
fi

# echo "🔨 开始编译..."
make -j$(nproc)

if [ $? -eq 0 ]; then
    echo "✅ 构建成功！"
else
    echo "❌ 编译失败，请查看上面的错误信息。"
    exit 1
fi