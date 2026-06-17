#!/bin/bash
set -euo pipefail

WORKSPACE_ROOT="/workspace"
SRC_DIR="$WORKSPACE_ROOT/src"
UNITREE_SDK2_DIR="$SRC_DIR/unitree_sdk2"
UNITREE_SDK2_PYTHON_DIR="$SRC_DIR/unitree_sdk2_python"
UNITREE_ROS2_DIR="$SRC_DIR/unitree_ros2"
UNITREE_MUJOCO_DIR="$SRC_DIR/unitree_mujoco"
LOCAL_EXAMPLE_G1_DIR="$WORKSPACE_ROOT/example_g1"
COPIED_EXAMPLE_G1_DIR="$SRC_DIR/example_g1"
LOCAL_UNITREE_PYTHON_EXAMPLES_DIR="$WORKSPACE_ROOT/examples_unitree_python_sdk"
COPIED_UNITREE_PYTHON_EXAMPLES_DIR="$SRC_DIR/examples_unitree_python_sdk"
LOCAL_BASIC_ROS2_EXAMPLE_DIR="$WORKSPACE_ROOT/unitree_ros2_basic_example"
ROS2_BASIC_WS_DIR="$WORKSPACE_ROOT/unitree_ros2_basic_example_ws"
ROS2_BASIC_WS_SRC_DIR="$ROS2_BASIC_WS_DIR/src"
COPIED_BASIC_ROS2_EXAMPLE_DIR="$ROS2_BASIC_WS_SRC_DIR/unitree_ros2_basic_example"
CYCLONEDDS_WS_DIR="$UNITREE_ROS2_DIR/cyclonedds_ws"
CYCLONEDDS_SRC_DIR="$CYCLONEDDS_WS_DIR/src"
EXAMPLE_WS_DIR="$UNITREE_ROS2_DIR/example"
MUJOCO_VERSION="mujoco-3.3.6"
MUJOCO_TARBALL="mujoco-3.3.6-linux-x86_64.tar.gz"
MUJOCO_DOWNLOAD_URL="https://github.com/google-deepmind/mujoco/releases/download/3.3.6/$MUJOCO_TARBALL"
MUJOCO_LOCAL_DIR_DEFAULT="$WORKSPACE_ROOT/mujoco-3.3.6-linux-x86_64/$MUJOCO_VERSION"
MUJOCO_HOME_DIR="$HOME/.mujoco"
UNITREE_MUJOCO_SIM_DIR="$UNITREE_MUJOCO_DIR/simulate"
UNITREE_MUJOCO_SIM_PYTHON_DIR="$UNITREE_MUJOCO_DIR/simulate_python"
UNITREE_MUJOCO_LINK="$UNITREE_MUJOCO_SIM_DIR/mujoco"

ensure_repo() {
  local url="$1"
  local ref="$2"
  local dest="$3"

  if [ -d "$dest/.git" ]; then
    return
  fi

  git clone --branch "$ref" --depth 1 "$url" "$dest"
}

run_as_root() {
  if [ "$(id -u)" -eq 0 ]; then
    "$@"
  else
    sudo "$@"
  fi
}

ensure_apt_packages() {
  run_as_root apt-get update
  run_as_root apt-get install -y "$@"
}

system_pip3_install() {
  ensure_apt_packages python3-pip

  if pip3 install --help | grep -q -- '--break-system-packages'; then
    run_as_root pip3 install --break-system-packages "$@"
  else
    run_as_root pip3 install "$@"
  fi
}

copy_local_example_g1_assets() {
  if [ ! -d "$LOCAL_EXAMPLE_G1_DIR" ]; then
    return 0
  fi

  rm -rf "$COPIED_EXAMPLE_G1_DIR"
  cp -a "$LOCAL_EXAMPLE_G1_DIR" "$COPIED_EXAMPLE_G1_DIR"
}

copy_local_unitree_python_examples() {
  if [ ! -d "$LOCAL_UNITREE_PYTHON_EXAMPLES_DIR" ]; then
    return 0
  fi

  rm -rf "$COPIED_UNITREE_PYTHON_EXAMPLES_DIR"
  cp -a "$LOCAL_UNITREE_PYTHON_EXAMPLES_DIR" "$COPIED_UNITREE_PYTHON_EXAMPLES_DIR"
}

copy_local_basic_ros2_example() {
  if [ ! -d "$LOCAL_BASIC_ROS2_EXAMPLE_DIR" ]; then
    return 0
  fi

  mkdir -p "$ROS2_BASIC_WS_SRC_DIR"
  rm -rf "$COPIED_BASIC_ROS2_EXAMPLE_DIR"
  cp -a "$LOCAL_BASIC_ROS2_EXAMPLE_DIR" "$COPIED_BASIC_ROS2_EXAMPLE_DIR"
}

resolve_mujoco_dir() {
  local candidate

  for candidate in \
    "${MUJOCO_LOCAL_DIR:-}" \
    "$MUJOCO_LOCAL_DIR_DEFAULT" \
    "$HOME/.mujoco/$MUJOCO_VERSION"
  do
    if [ -n "$candidate" ] && [ -d "$candidate" ]; then
      printf '%s\n' "$candidate"
      return 0
    fi
  done

  return 1
}

ensure_mujoco_available() {
  local mujoco_home_target="$MUJOCO_HOME_DIR/$MUJOCO_VERSION"
  local archive_path="$MUJOCO_HOME_DIR/$MUJOCO_TARBALL"

  if [ -d "$mujoco_home_target" ]; then
    return 0
  fi

  mkdir -p "$MUJOCO_HOME_DIR"

  if [ ! -f "$archive_path" ]; then
    curl -L "$MUJOCO_DOWNLOAD_URL" -o "$archive_path"
  fi

  tar -xzf "$archive_path" -C "$MUJOCO_HOME_DIR"
}

ensure_line_in_file() {
  local line="$1"
  local file="$2"

  grep -qxF "$line" "$file" || echo "$line" >> "$file"
}

ensure_conditional_source_in_file() {
  local script_path="$1"
  local file="$2"
  local legacy_line="source $script_path"
  local guarded_line="[ -f $script_path ] && source $script_path"

  if [ -f "$file" ]; then
    sed -i "\|^${legacy_line}$|d" "$file"
  fi

  ensure_line_in_file "$guarded_line" "$file"
}

source_setup_script() {
  local script_path="$1"
  local had_nounset=0

  if [ ! -f "$script_path" ]; then
    echo "Missing setup script: $script_path" >&2
    return 1
  fi

  case $- in
    *u*)
      had_nounset=1
      set +u
      ;;
  esac

  export AMENT_TRACE_SETUP_FILES="${AMENT_TRACE_SETUP_FILES-}"
  export AMENT_RETURN_ENVIRONMENT_HOOKS="${AMENT_RETURN_ENVIRONMENT_HOOKS-}"
  export COLCON_TRACE="${COLCON_TRACE-}"

  # shellcheck source=/dev/null
  source "$script_path"
  local status=$?

  if [ "$had_nounset" -eq 1 ]; then
    set -u
  fi

  return "$status"
}

patch_unitree_setup_scripts() {
  python3 /workspace/.devcontainer/patch_unitree_setup_scripts.py
}

build_unitree_sdk2() {
  cd "$UNITREE_SDK2_DIR"
  mkdir -p build
  cd build
  cmake .. -DCMAKE_INSTALL_PREFIX=/opt/unitree_robotics
  make -j"$(nproc)"
  make install || true
}

build_unitree_ros2_for_foxy() {
  # Build Cyclone DDS using the same workspace/layout recommended by Unitree.
  mkdir -p "$CYCLONEDDS_SRC_DIR"
  cd "$CYCLONEDDS_SRC_DIR"
  ensure_repo "https://github.com/ros2/rmw_cyclonedds.git" "foxy" \
    "$CYCLONEDDS_SRC_DIR/rmw_cyclonedds"
  ensure_repo "https://github.com/eclipse-cyclonedds/cyclonedds.git" "releases/0.10.x" \
    "$CYCLONEDDS_SRC_DIR/cyclonedds"
  cd "$CYCLONEDDS_WS_DIR"

  # Foxy's Cyclone build is sensitive to a pre-sourced ROS environment.
  # If the build fails manually, the usual fallback is:
  # export LD_LIBRARY_PATH=/opt/ros/foxy/lib
  env -u AMENT_PREFIX_PATH \
      -u CMAKE_PREFIX_PATH \
      -u COLCON_PREFIX_PATH \
      -u LD_LIBRARY_PATH \
      -u PYTHONPATH \
      -u ROS_DISTRO \
      -u ROS_PYTHON_VERSION \
      -u ROS_VERSION \
      -u RMW_IMPLEMENTATION \
      LD_LIBRARY_PATH=/opt/ros/foxy/lib \
      colcon build --packages-select cyclonedds

  source_setup_script /opt/ros/foxy/setup.bash
  export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
  colcon build --packages-select unitree_go unitree_hg unitree_api

  cd "$EXAMPLE_WS_DIR"
  source_setup_script /opt/ros/foxy/setup.bash
  source_setup_script "$CYCLONEDDS_WS_DIR/install/setup.bash"
  export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
  colcon build --packages-select unitree_ros2_example
}

setup_unitree_mujoco() {
  local mujoco_local_dir

  ensure_repo "https://github.com/unitreerobotics/unitree_mujoco.git" "main" \
    "$UNITREE_MUJOCO_DIR"

  # unitree_mujoco links against the system fmt package on Ubuntu/Foxy.
  ensure_apt_packages libboost-all-dev libfmt-dev libglfw3-dev

  ensure_mujoco_available

  if ! mujoco_local_dir="$(resolve_mujoco_dir)"; then
    echo "MuJoCo $MUJOCO_VERSION must exist inside the container before linking." >&2
    echo "Checked: $MUJOCO_LOCAL_DIR_DEFAULT and $HOME/.mujoco/$MUJOCO_VERSION" >&2
    echo "If your MuJoCo files are only on the host, mount or copy them into the container first." >&2
    return 1
  fi

  mkdir -p "$UNITREE_MUJOCO_SIM_DIR" "$UNITREE_MUJOCO_SIM_PYTHON_DIR"
  cp -f "$LOCAL_EXAMPLE_G1_DIR/config.yaml" "$UNITREE_MUJOCO_SIM_DIR/config.yaml"
  cp -f "$LOCAL_EXAMPLE_G1_DIR/config.yaml" "$UNITREE_MUJOCO_SIM_PYTHON_DIR/config.yaml"

  if [ -L "$UNITREE_MUJOCO_LINK" ]; then
    local current_target
    current_target="$(readlink -f "$UNITREE_MUJOCO_LINK")"
    if [ "$current_target" != "$mujoco_local_dir" ]; then
      rm -f "$UNITREE_MUJOCO_LINK"
      ln -s "$mujoco_local_dir" "$UNITREE_MUJOCO_LINK"
    fi
  elif [ -e "$UNITREE_MUJOCO_LINK" ]; then
    echo "Refusing to replace existing non-symlink path: $UNITREE_MUJOCO_LINK" >&2
    return 1
  else
    ln -s "$mujoco_local_dir" "$UNITREE_MUJOCO_LINK"
  fi

  cd "$UNITREE_MUJOCO_SIM_DIR"
  mkdir -p build
  cd build
  cmake ..
  make -j"$(nproc)"
}

build_local_basic_ros2_workspace() {
  if [ ! -d "$COPIED_BASIC_ROS2_EXAMPLE_DIR" ]; then
    echo "Skipping local ROS 2 example workspace build: missing $COPIED_BASIC_ROS2_EXAMPLE_DIR" >&2
    return 0
  fi

  mkdir -p "$ROS2_BASIC_WS_SRC_DIR"
  cd "$ROS2_BASIC_WS_DIR"
  source_setup_script /opt/ros/foxy/setup.bash
  source_setup_script "$CYCLONEDDS_WS_DIR/install/setup.bash"
  export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
  colcon build --packages-select unitree_ros2_basic_example
}

install_python_sim_packages() {
  echo "Installing Python simulation packages..."
  system_pip3_install mujoco pygame
  python3 -c "import mujoco, pygame; print(mujoco.__file__); print(pygame.__file__)"
  echo "✓ Python simulation packages installed successfully"
}

install_unitree_sdk2_python() {
  echo "Installing unitree_sdk2_python package..."
  
  # Clone and modify the Python SDK to use flexible cyclonedds version
  ensure_repo "https://github.com/unitreerobotics/unitree_sdk2_python.git" "master" \
    "$UNITREE_SDK2_PYTHON_DIR"
  
  # Relax the cyclonedds version requirement to allow compatible versions
  sed -i 's/"cyclonedds==0.10.2"/"cyclonedds>=0.10.2"/g' "$UNITREE_SDK2_PYTHON_DIR/setup.py"
  
  # Install the SDK into the system Python package path.
  cd "$UNITREE_SDK2_PYTHON_DIR"
  system_pip3_install .
  run_as_root mkdir -p /usr/local/lib/python3.8/dist-packages
  run_as_root cp -r unitree_sdk2py /usr/local/lib/python3.8/dist-packages/
  python3 -c "import unitree_sdk2py; print(unitree_sdk2py.__file__)"
  
  echo "✓ unitree_sdk2_python installed successfully"
}

cd "$WORKSPACE_ROOT"
mkdir -p "$SRC_DIR"
cd "$SRC_DIR"

if [ ! -d "$UNITREE_SDK2_DIR" ]; then
  git clone https://github.com/unitreerobotics/unitree_sdk2.git
fi

if [ ! -d "$UNITREE_ROS2_DIR" ]; then
  git clone https://github.com/unitreerobotics/unitree_ros2.git
fi

patch_unitree_setup_scripts

copy_local_example_g1_assets
copy_local_unitree_python_examples
copy_local_basic_ros2_example

build_unitree_sdk2

ensure_line_in_file 'export LD_LIBRARY_PATH=/usr/local/lib:$LD_LIBRARY_PATH' ~/.bashrc
ensure_line_in_file 'export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp' ~/.bashrc
ensure_conditional_source_in_file '/workspace/src/unitree_ros2/cyclonedds_ws/install/setup.bash' ~/.bashrc
ensure_conditional_source_in_file '/workspace/src/unitree_ros2/example/install/setup.bash' ~/.bashrc
ensure_conditional_source_in_file '/workspace/unitree_ros2_basic_example_ws/install/setup.bash' ~/.bashrc

source_setup_script /opt/ros/foxy/setup.bash
rosdep install --from-paths "$UNITREE_ROS2_DIR/cyclonedds_ws/src/unitree" "$EXAMPLE_WS_DIR/src" \
  --ignore-src -r -y

build_unitree_ros2_for_foxy
setup_unitree_mujoco
export LD_LIBRARY_PATH=/opt/unitree_robotics/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}
ensure_line_in_file 'export LD_LIBRARY_PATH=/opt/unitree_robotics/lib:$LD_LIBRARY_PATH' ~/.bashrc
build_local_basic_ros2_workspace


echo "=== Setup Complete ==="
