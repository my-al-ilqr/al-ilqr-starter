#!/usr/bin/env bash
set -euo pipefail

TOTAL_STEPS=3

echo "============================================"
echo "  MY-AL-iLQR 依赖安装脚本"
echo "============================================"

if ! command -v apt-get &>/dev/null; then
  echo "[错误] 当前仅支持 apt-get (Ubuntu / Debian)。"
  echo "请手动安装以下依赖："
  echo "  - cmake >= 3.16"
  echo "  - g++ (支持 C++17) 或 clang++"
  echo "  - Eigen3 开发库"
  echo "  - OpenGL + X11/Wayland 开发库 (用于 ImGui 前端)"
  exit 1
fi

echo ""
echo "需要 sudo 权限来安装系统包，请输入密码："
sudo -v || { echo "[错误] sudo 认证失败"; exit 1; }

echo ""
echo "[$((1))/${TOTAL_STEPS}] 更新包索引 ..."
sudo apt-get update

echo ""
echo "[$((1))/${TOTAL_STEPS}] 安装编译工具和核心依赖 ..."
echo "  → build-essential, cmake, libeigen3-dev"
sudo apt-get install -y \
  build-essential \
  cmake \
  libeigen3-dev
echo "  ✓ 编译工具和 Eigen3 安装完成"

echo ""
echo "[$((2))/${TOTAL_STEPS}] 安装 ImGui 前端所需的图形库 ..."
echo "  → libgl-dev, libx11-dev, libxrandr-dev, libxinerama-dev,"
echo "    libxcursor-dev, libxi-dev, libwayland-dev, libxkbcommon-dev"
sudo apt-get install -y \
  libgl-dev \
  libx11-dev \
  libxrandr-dev \
  libxinerama-dev \
  libxcursor-dev \
  libxi-dev \
  libwayland-dev \
  libxkbcommon-dev
echo "  ✓ 图形库安装完成"

echo ""
echo "[$((3))/${TOTAL_STEPS}] 安装 Python 可视化依赖 (可选) ..."
echo "  → python3, python3-matplotlib, python3-numpy"
sudo apt-get install -y \
  python3 \
  python3-matplotlib \
  python3-numpy
echo "  ✓ Python 可视化依赖安装完成"

echo ""
echo "============================================"
echo "  全部 ${TOTAL_STEPS}/${TOTAL_STEPS} 步安装完成！"
echo ""
echo "  下一步："
echo ""
echo "  1. 编译项目："
echo "     cmake -B build -DCMAKE_BUILD_TYPE=Release"
echo "     cmake --build build -j\$(nproc)"
echo ""
echo "  2. 运行测试："
echo "     cd build && ctest --output-on-failure"
echo ""
echo "  3. 启动交互式前端："
echo "     ./build/imgui_frontend"
echo "============================================"
