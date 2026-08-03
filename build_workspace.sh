#!/usr/bin/env bash
set -euo pipefail

WORKSPACE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

if [[ ! -f /opt/ros/noetic/setup.bash ]]; then
  echo "ROS1 Noetic을 찾을 수 없습니다: /opt/ros/noetic/setup.bash" >&2
  exit 1
fi

source /opt/ros/noetic/setup.bash
cd "${WORKSPACE_DIR}"

rosdep install --from-paths src --ignore-src -r -y
catkin_make

echo
echo "빌드 완료. 다음 명령으로 환경을 적용하세요:"
echo "source ${WORKSPACE_DIR}/devel/setup.bash"
