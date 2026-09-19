#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
CONFIG_PATH="$SCRIPT_DIR/config.env"
OVERWRITE=0
STATIC_CAM=1
RENDER=0

usage() {
    cat <<'EOF'
用法:
  ./run_pipeline.sh [选项]

选项:
  --config PATH       指定配置文件（默认: 包目录/config.env）
  --overwrite         重跑每个视频的 GVHMR，并覆盖已有 NPZ
  --static-cam        使用静态相机模式（默认）
  --no-static-cam     不使用静态相机模式
  --render            保留 GVHMR 预览渲染（默认跳过）
  -h, --help          显示帮助
EOF
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
        --overwrite)
            OVERWRITE=1
            shift
            ;;
        --static-cam)
            STATIC_CAM=1
            shift
            ;;
        --no-static-cam)
            STATIC_CAM=0
            shift
            ;;
        --render)
            RENDER=1
            shift
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

"$SCRIPT_DIR/check_environment.sh" --config "$CONFIG_PATH"

# 环境检查通过后，再加载同一份可信配置并创建输出目录。
# shellcheck disable=SC1090
source "$CONFIG_PATH"

mkdir -p -- "$GVHMR_OUTPUT_ROOT" "$NPZ_OUTPUT_ROOT"
GVHMR_OUTPUT_ROOT="$(cd -- "$GVHMR_OUTPUT_ROOT" && pwd -P)"
NPZ_OUTPUT_ROOT="$(cd -- "$NPZ_OUTPUT_ROOT" && pwd -P)"

mapfile -d '' -t videos < <(
    find "$VIDEO_INPUT_ROOT" -type f \
        \( -iname '*.mp4' -o -iname '*.mov' -o -iname '*.m4v' \) \
        -print0 |
        sort -z
)

if ((${#videos[@]} == 0)); then
    echo "[ERROR] 输入目录中没有 .mp4/.mov/.m4v 视频: $VIDEO_INPUT_ROOT" >&2
    exit 1
fi

for video in "${videos[@]}"; do
    filename="$(basename -- "$video")"
    stem="${filename%.*}"
    case "$stem" in
        forehand_*|backhand_*) ;;
        *)
            echo "[ERROR] 视频名必须以 forehand_ 或 backhand_ 开头: $video" >&2
            exit 1
            ;;
    esac

    result_dir="$GVHMR_OUTPUT_ROOT/$stem"
    result_file="$result_dir/hmr4d_results.pt"

    if [[ -f "$result_file" && "$OVERWRITE" -eq 0 ]]; then
        echo "[SKIP] 已有 GVHMR 结果: $result_file"
        continue
    fi

    if [[ -e "$result_dir" && "$OVERWRITE" -eq 1 ]]; then
        case "$result_dir/" in
            "$GVHMR_OUTPUT_ROOT"/*/)
                echo "[OVERWRITE] 删除该视频的旧 GVHMR 中间目录: $result_dir"
                rm -rf -- "$result_dir"
                ;;
            *)
                echo "[ERROR] 拒绝删除不在 GVHMR_OUTPUT_ROOT 下的路径: $result_dir" >&2
                exit 1
                ;;
        esac
    fi

    gvhmr_args=(
        tools/demo/demo.py
        --video "$video"
        --output_root "$GVHMR_OUTPUT_ROOT"
    )
    if [[ "$STATIC_CAM" -eq 1 ]]; then
        gvhmr_args+=(-s)
    fi
    if [[ "$RENDER" -eq 0 ]]; then
        gvhmr_args+=(--skip-render)
    fi

    echo "[GVHMR] $video"
    (
        cd -- "$GVHMR_ROOT"
        "$PIPELINE_PYTHON" "${gvhmr_args[@]}"
    )

    if [[ ! -f "$result_file" ]]; then
        echo "[ERROR] GVHMR 未生成预期结果: $result_file" >&2
        exit 1
    fi
done

mapfile -d '' -t gvhmr_results < <(
    find "$GVHMR_OUTPUT_ROOT" -type f -name 'hmr4d_results.pt' -print0 |
        sort -z
)
if ((${#gvhmr_results[@]} == 0)); then
    echo "[ERROR] 没有可转换的 hmr4d_results.pt: $GVHMR_OUTPUT_ROOT" >&2
    exit 1
fi

convert_args=(
    "$SCRIPT_DIR/scripts/convert_gvhmr_results_to_hitter_npz.py"
    --input-root "$GVHMR_OUTPUT_ROOT"
    --clip-root "$VIDEO_INPUT_ROOT"
    --output-root "$NPZ_OUTPUT_ROOT"
    --gmr-root "$GMR_ROOT"
    --smplx-root "$SMPLX_ROOT"
    --urdf-path "$HITTER_URDF"
    --reference-root "$REFERENCE_ROOT"
    --compressed
)
if [[ "$OVERWRITE" -eq 1 ]]; then
    convert_args+=(--overwrite)
fi

echo "[NPZ] 转换 ${#gvhmr_results[@]} 个 GVHMR 结果"
"$PIPELINE_PYTHON" "${convert_args[@]}"

mapfile -d '' -t npz_results < <(
    find "$NPZ_OUTPUT_ROOT" -type f -name '*.npz' -print0 |
        sort -z
)
if ((${#npz_results[@]} == 0)); then
    echo "[ERROR] 转换结束但没有生成 NPZ: $NPZ_OUTPUT_ROOT" >&2
    exit 1
fi

echo "[DONE] NPZ 数量: ${#npz_results[@]}"
echo "[DONE] 输出目录: $NPZ_OUTPUT_ROOT"
