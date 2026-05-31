#!/bin/bash
# 保存 SLAM 构建的地图
# 用法:
#   rosrun rm_ep_navigation save_map.sh [地图名称]
#   ./scripts/save_map.sh [地图名称]

MAP_NAME=${1:-default_map}
SCRIPT_DIR=$(cd "$(dirname "$0")" && pwd)
MAP_DIR="$(cd "$SCRIPT_DIR/../maps" 2>/dev/null && pwd || echo "$SCRIPT_DIR/../maps")"

if [ ! -d "$MAP_DIR" ]; then
    mkdir -p "$MAP_DIR"
fi

echo "保存地图到: ${MAP_DIR}/${MAP_NAME}"

rosrun map_server map_saver -f "${MAP_DIR}/${MAP_NAME}"

if [ -f "${MAP_DIR}/${MAP_NAME}.pgm" ] && [ -f "${MAP_DIR}/${MAP_NAME}.yaml" ]; then
    echo "地图保存成功:"
    echo "  ${MAP_DIR}/${MAP_NAME}.pgm"
    echo "  ${MAP_DIR}/${MAP_NAME}.yaml"
else
    echo "地图保存失败！请确认 gmapping 正在运行。"
    exit 1
fi
