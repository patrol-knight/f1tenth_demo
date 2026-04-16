#!/usr/bin/env bash
set -e
sudo -v

RANGELIBC_DIR="${RANGELIBC_DIR:-$HOME/range_libc}"
RANGELIBC_REPO="${RANGELIBC_REPO:-https://github.com/f1tenth/range_libc.git}"
RANGELIBC_BRANCH="${RANGELIBC_BRANCH:-humble-devel}"

echo "[INFO] Python executable: $(command -v python3)"
python3 --version

if ! python3 -m pip --version >/dev/null 2>&1; then
  echo "[ERROR] python3-pip not found. Install first:"
  echo "  sudo apt-get update && sudo apt-get install -y python3-pip"
  exit 1
fi

# 安裝 Cython（不要升級 setuptools / wheel，避免 colcon 衝突）
echo "[INFO] Ensuring Cython is installed in this python3 environment"
python3 -m pip install --user transform3d numpy scipy
python3 -m pip install --user -U Cython

echo "[INFO] Verifying Cython import"
python3 - <<'PY'
import sys, Cython
print("Cython OK:", Cython.__version__)
print("Python:", sys.executable)
PY

echo "[INFO] Ensuring range_libc repo at: ${RANGELIBC_DIR}"
if [ ! -d "${RANGELIBC_DIR}/.git" ]; then
  mkdir -p "$(dirname "${RANGELIBC_DIR}")"
  git clone -b "${RANGELIBC_BRANCH}" "${RANGELIBC_REPO}" "${RANGELIBC_DIR}"
else
  git -C "${RANGELIBC_DIR}" fetch --all --prune
  git -C "${RANGELIBC_DIR}" checkout "${RANGELIBC_BRANCH}"
  git -C "${RANGELIBC_DIR}" pull --ff-only
fi

# 自動找 pywrapper/pywrappers 目錄（repo 結構可能不同）
PYWRAPPER_DIR="$(find "${RANGELIBC_DIR}" -maxdepth 4 -type d \( -name pywrapper -o -name pywrappers \) | head -n 1 || true)"

if [ -z "${PYWRAPPER_DIR}" ]; then
  echo "[ERROR] Cannot find pywrapper or pywrappers under ${RANGELIBC_DIR}"
  echo "[INFO] Try checking repo layout:"
  echo "  find ${RANGELIBC_DIR} -maxdepth 4 -type d | sort"
  exit 1
fi

echo "[INFO] Using wrapper dir: ${PYWRAPPER_DIR}"

# CPU-only + 使用目前 python 環境（避免 build isolation 找不到 Cython）
echo "[INFO] Installing range_libc pywrapper (CPU-only)"
export CMAKE_ARGS="${CMAKE_ARGS:-} -DWITH_CUDA=OFF"
# (
#   cd "${PYWRAPPER_DIR}"
#   SETUPTOOLS_USE_DISTUTILS=stdlib WITH_CUDA=OFF python3 setup.py install --user
# )
(
  cd "${PYWRAPPER_DIR}"
  WITH_CUDA=OFF python3 setup.py install --user
)

echo "[INFO] Verifying range_libc import"
python3 - <<'PY'
import sys, range_libc
print("range_libc import OK")
print("range_libc path:", range_libc.__file__)
print("Python:", sys.executable)
PY

echo "[INFO] Checking known required apt packages"
if ! dpkg -s ros-humble-asio-cmake-module >/dev/null 2>&1; then
  echo "[INFO] Installing ros-humble-asio-cmake-module"
  sudo apt-get update
  sudo apt-get install -y ros-humble-asio-cmake-module
else
  echo "[INFO] ros-humble-asio-cmake-module already installed"
fi
sudo apt-get update
sudo apt-get install -y ccache
sudo apt-get install -y ros-humble-rosbridge-server
sudo apt-get install -y ros-humble-tf-transformations
echo "[INFO] Installing ROS package dependencies from src/ via rosdep"

# 確保 rosdep 存在
if ! command -v rosdep >/dev/null 2>&1; then
  echo "[ERROR] rosdep not found. Install first:"
  echo "  sudo apt-get update && sudo apt-get install -y python3-rosdep"
  exit 1
fi

# rosdep init (只需一次；重跑安全)
if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
  echo "[INFO] Initializing rosdep (first time only)"
  sudo rosdep init
fi

echo "[INFO] Updating rosdep database"
rosdep update

# 回到 workspace 根目錄（假設 script 從 workspace 根目錄執行）
WS_DIR="$(cd "$(dirname "$0")" && pwd)"
cd "${WS_DIR}"

if [ ! -d "${WS_DIR}/src" ]; then
  echo "[ERROR] Cannot find src/ under workspace: ${WS_DIR}"
  exit 1
fi

# 安裝 src 裡所有 package.xml 宣告的系統依賴（apt等）
# --ignore-src: 不安裝你自己 source code 裡的 package
# -r: 繼續處理其他套件，即使某些套件失敗
# -y: 自動 yes
rosdep install --from-paths src --ignore-src --rosdistro humble -r -y

echo "[INFO] Done."
