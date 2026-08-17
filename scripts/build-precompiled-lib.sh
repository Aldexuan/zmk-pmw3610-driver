#!/bin/bash
# 将 PMW3610 驱动编译成预编译静态库
# 这样可以隐藏源码,只提供 .a 库文件

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
OUTPUT_DIR="$PROJECT_ROOT/precompiled"

echo "=========================================="
echo "编译 PMW3610 驱动为预编译库"
echo "=========================================="

# 清理旧的输出
rm -rf "$OUTPUT_DIR"
mkdir -p "$OUTPUT_DIR/lib"
mkdir -p "$OUTPUT_DIR/include"

# 检查是否在 Zephyr 环境中
if [ -z "$ZEPHYR_BASE" ]; then
    echo "错误: 未找到 Zephyr 环境"
    echo "请先运行: source zephyr/zephyr-env.sh"
    exit 1
fi

# 目标架构列表(ARM Cortex-M)
TARGETS=(
    "arm/cortex-m0"      # nRF51
    "arm/cortex-m4"      # nRF52832
    "arm/cortex-m4f"     # nRF52840
)

# 为每个目标编译
for target in "${TARGETS[@]}"; do
    target_name=$(echo "$target" | tr '/' '_')
    echo "编译目标: $target_name"
    
    # 创建临时构建目录
    BUILD_DIR="/tmp/pmw3610-build-$target_name"
    rm -rf "$BUILD_DIR"
    mkdir -p "$BUILD_DIR"
    
    cd "$BUILD_DIR"
    
    # 创建 CMake 配置
    cat > CMakeLists.txt <<EOF
cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS \$ENV{ZEPHYR_BASE})
project(pmw3610_driver)

# 编译驱动源码为静态库
zephyr_library_named(pmw3610)
zephyr_library_sources(
    $PROJECT_ROOT/src/pmw3610.c
    $PROJECT_ROOT/src/behaviors/behavior_pmw3610_setting.c
)
zephyr_library_include_directories(
    $PROJECT_ROOT/include
    $PROJECT_ROOT/src
)
EOF

    # 创建最小化配置
    cat > prj.conf <<EOF
CONFIG_SPI=y
CONFIG_GPIO=y
CONFIG_SENSOR=y
EOF
    
    # 使用 west 构建
    west build -b nrf52840dk_nrf52840 --build-dir "$BUILD_DIR" -- \
        -DCONFIG_COMPILER_OPTIMIZATION_SIZE=y \
        -DCONFIG_DEBUG=n
    
    # 提取静态库
    if [ -f "$BUILD_DIR/zephyr/libpmw3610.a" ]; then
        cp "$BUILD_DIR/zephyr/libpmw3610.a" "$OUTPUT_DIR/lib/libpmw3610-$target_name.a"
        echo "✅ 生成: libpmw3610-$target_name.a"
    else
        echo "❌ 失败: 未找到静态库"
    fi
    
    # 清理临时目录
    rm -rf "$BUILD_DIR"
done

# 复制头文件(API 接口)
echo "复制公开头文件..."
cp -r "$PROJECT_ROOT/include/zmk" "$OUTPUT_DIR/include/"
cp -r "$PROJECT_ROOT/include/dt-bindings" "$OUTPUT_DIR/include/"

# 复制 DTS 绑定和行为定义
echo "复制 DTS 文件..."
cp -r "$PROJECT_ROOT/dts" "$OUTPUT_DIR/"

# 创建预编译版本的 CMakeLists.txt
cat > "$OUTPUT_DIR/CMakeLists.txt" <<'EOF'
# PMW3610 驱动 - 预编译版本
# 此版本使用预编译静态库,不包含源码

if(CONFIG_PMW3610)
    # 检测目标架构
    if(CONFIG_CPU_CORTEX_M0)
        set(PMW3610_LIB "libpmw3610-arm_cortex-m0.a")
    elseif(CONFIG_CPU_CORTEX_M4 AND NOT CONFIG_FPU)
        set(PMW3610_LIB "libpmw3610-arm_cortex-m4.a")
    elseif(CONFIG_CPU_CORTEX_M4 AND CONFIG_FPU)
        set(PMW3610_LIB "libpmw3610-arm_cortex-m4f.a")
    else()
        message(FATAL_ERROR "不支持的目标架构")
    endif()
    
    # 链接预编译库
    zephyr_library_import(pmw3610_precompiled ${CMAKE_CURRENT_SOURCE_DIR}/lib/${PMW3610_LIB})
    
    # 添加头文件路径
    zephyr_include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)
    
    # 添加 DTS 绑定
    list(APPEND DTS_ROOT ${CMAKE_CURRENT_SOURCE_DIR})
endif()
EOF

# 复制 Kconfig(保持不变)
cp "$PROJECT_ROOT/Kconfig" "$OUTPUT_DIR/"

# 创建 module.yml
cat > "$OUTPUT_DIR/zephyr/module.yml" <<EOF
build:
  cmake: .
  kconfig: Kconfig
  settings:
    dts_root: .
EOF

# 创建 README
cat > "$OUTPUT_DIR/README.md" <<'EOF'
# PMW3610 驱动 - 预编译版本

⚠️ **本驱动为预编译版本,不包含源码**

## 包含内容

- ✅ 预编译静态库 (lib/*.a)
- ✅ 公开 API 头文件 (include/*)
- ✅ DTS 绑定和行为定义 (dts/*)
- ❌ 驱动实现源码 (已编译)

## 使用方法

将此目录作为 Zephyr module 集成到 ZMK 构建中。

## 许可

本驱动为商业软件,仅供授权用户使用。
EOF

# 生成校验和
echo "生成文件清单..."
cd "$OUTPUT_DIR"
find . -type f -exec sha256sum {} \; > FILES.sha256

echo "=========================================="
echo "✅ 预编译库构建完成"
echo "输出目录: $OUTPUT_DIR"
echo "=========================================="
ls -lhR "$OUTPUT_DIR"
