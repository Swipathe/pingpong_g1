#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
CONFIG_PATH="$SCRIPT_DIR/config.env"

usage() {
    echo "用法: $0 [--config PATH]"
    echo "检查手机视频转 HITTER NPZ 流水线所需的外部路径和 Python 依赖。"
}

while (($# > 0)); do
    case "$1" in
        --config)
            if (($# < 2)); then
                echo "[ERROR] --config 缺少路径参数" >&2
                exit 2
            fi
            CONFIG_PATH="$2"
            shift 2
            ;;
        -h|--help)
            usage
            exit 0
            ;;
        *)
            echo "[ERROR] 未知参数: $1" >&2
            usage >&2
            exit 2
            ;;
    esac
done

if [[ ! -f "$CONFIG_PATH" ]]; then
    echo "[ERROR] 配置文件不存在: $CONFIG_PATH" >&2
    echo "        先执行: cp \"$SCRIPT_DIR/config.example.env\" \"$SCRIPT_DIR/config.env\"" >&2
    exit 1
fi

# 只加载用户明确指定的本地配置文件；不要使用来源不可信的配置文件。
# shellcheck disable=SC1090
source "$CONFIG_PATH"

errors=0

ok() {
    echo "[OK] $*"
}

error() {
    echo "[ERROR] $*" >&2
    errors=$((errors + 1))
}

required_variables=(
    GVHMR_ROOT
    GMR_ROOT
    PIPELINE_PYTHON
    SMPLX_ROOT
    HITTER_URDF
    REFERENCE_ROOT
    VIDEO_INPUT_ROOT
    GVHMR_OUTPUT_ROOT
    NPZ_OUTPUT_ROOT
)

for variable_name in "${required_variables[@]}"; do
    if [[ -n "${!variable_name:-}" ]]; then
        ok "$variable_name=${!variable_name}"
    else
        error "$variable_name 未配置"
    fi
done

check_directory() {
    local label="$1"
    local path="$2"
    if [[ -d "$path" ]]; then
        ok "$label 目录存在: $path"
    else
        error "$label 目录不存在: $path"
    fi
}

check_file() {
    local label="$1"
    local path="$2"
    if [[ -f "$path" ]]; then
        ok "$label 文件存在: $path"
    else
        error "$label 文件不存在: $path"
    fi
}

if [[ -n "${PIPELINE_PYTHON:-}" ]]; then
    if [[ -x "$PIPELINE_PYTHON" ]]; then
        ok "PIPELINE_PYTHON 可执行: $PIPELINE_PYTHON"
    else
        error "PIPELINE_PYTHON 不存在或不可执行: $PIPELINE_PYTHON"
    fi
fi

if [[ -n "${GVHMR_ROOT:-}" ]]; then
    check_directory "GVHMR_ROOT" "$GVHMR_ROOT"
    check_file "GVHMR 入口" "$GVHMR_ROOT/tools/demo/demo.py"
fi
if [[ -n "${GMR_ROOT:-}" ]]; then
    check_directory "GMR_ROOT" "$GMR_ROOT"
    check_directory "GMR Python 包" "$GMR_ROOT/general_motion_retargeting"
fi
if [[ -n "${SMPLX_ROOT:-}" ]]; then
    check_file "SMPL-X 模型" "$SMPLX_ROOT/smplx/SMPLX_NEUTRAL.npz"
fi
if [[ -n "${HITTER_URDF:-}" ]]; then
    check_file "HITTER URDF" "$HITTER_URDF"
fi
if [[ -n "${REFERENCE_ROOT:-}" ]]; then
    check_directory "参考动作" "$REFERENCE_ROOT"
fi
if [[ -n "${VIDEO_INPUT_ROOT:-}" ]]; then
    check_directory "输入视频" "$VIDEO_INPUT_ROOT"
fi

if [[ -x "${PIPELINE_PYTHON:-}" ]]; then
    module_output=""
    if module_output="$(
        PYTHONPATH="${GVHMR_ROOT:-}:${GMR_ROOT:-}${PYTHONPATH:+:$PYTHONPATH}" \
            "$PIPELINE_PYTHON" -c \
            'import cv2, torch, hydra, pytorch3d, numpy, pybullet, smplx, general_motion_retargeting' \
            2>&1
    )"; then
        ok "Python 依赖可导入: cv2 torch hydra pytorch3d numpy pybullet smplx general_motion_retargeting"
    else
        error "Python 依赖不完整（PIPELINE_PYTHON=$PIPELINE_PYTHON）"
        if [[ -n "$module_output" ]]; then
            echo "$module_output" | sed 's/^/        /' >&2
        fi
    fi
fi

if ((errors > 0)); then
    echo "[FAIL] 环境检查发现 $errors 个问题；未启动推理。" >&2
    exit 1
fi

echo "[PASS] 环境检查通过。"
