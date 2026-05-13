#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Keep rqt on the system ROS/Python/Qt stack even if Conda, Snap, or an IDE
# injected library paths into the shell environment.
export PATH="/opt/ros/humble/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin"
unset CONDA_DEFAULT_ENV
unset CONDA_EXE
unset CONDA_PREFIX
unset CONDA_PROMPT_MODIFIER
unset CONDA_PYTHON_EXE
unset CONDA_SHLVL
unset _CONDA_EXE
unset _CONDA_ROOT
unset LD_LIBRARY_PATH
unset LD_PRELOAD
unset PYTHONPATH
unset PYTHONHOME
unset SNAP
unset SNAP_ARCH
unset SNAP_COMMON
unset SNAP_CONTEXT
unset SNAP_COOKIE
unset SNAP_DATA
unset SNAP_INSTANCE_NAME
unset SNAP_LIBRARY_PATH
unset SNAP_NAME
unset SNAP_REAL_HOME
unset SNAP_REVISION
unset SNAP_USER_COMMON
unset SNAP_USER_DATA
unset SNAP_VERSION
unset GTK_PATH
unset GIO_EXTRA_MODULES
unset QT_PLUGIN_PATH
unset QT_QPA_PLATFORM_PLUGIN_PATH

source /opt/ros/humble/setup.bash
source "${SCRIPT_DIR}/install_ros2/setup.bash"

exec /opt/ros/humble/bin/rqt "$@"
