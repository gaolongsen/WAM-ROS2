# Barrett WAM ROS 2 Humble Driver

ROS 2 Humble driver for controlling a Barrett WAM arm over CAN bus through
`libbarrett`.

This repository ports the original ROS 1 Barrett WAM interface to ROS 2 while
keeping the low-level hardware access in `libbarrett`. It has been validated on
a local control PC with a 7-DOF WAM, SocketCAN, ROS 2 Humble, and Ubuntu
22.04-style system dependencies.

![Barrett WAM](https://github.com/gaolongsen/picx-images-hosting/raw/master/barrett_arm_hand.pfx4ph3f4.webp)

## Highlights

- ROS 2 Humble `ament_cmake` build.
- CAN-bus WAM control through patched `libbarrett`.
- ROS 2 interfaces for WAM state, realtime commands, and service commands.
- Optional BarrettHand and force/torque sensor support when discovered by
  `libbarrett`.
- Conservative launch defaults for first-time hardware bring-up.
- Helper tools for `rqt` and safe single-joint motion diagnostics.

## Repository Layout

```text
wam_node/              ROS 2 WAM hardware node
barrett_hand_node/     ROS 2 standalone BarrettHand node
wam_msgs/              ROS 2 custom messages
wam_srvs/              ROS 2 custom services
wam_teleop/            ROS 2 joystick teleoperation
scripts/               Small user-facing test utilities
libbarrett/            Patched libbarrett source, if vendored in this repo
wam_demos/             Legacy ROS 1 package, ignored by colcon
perception_palm/       Legacy ROS 1 package, ignored by colcon
```

## Tested Platform

Recommended target:

- Ubuntu 22.04
- ROS 2 Humble
- SocketCAN or PEAK PCAN CAN interface exposed as `can0`
- Barrett WAM with safety pendant
- Patched `libbarrett` built from this repository or from your patched
  `libbarrett` fork

Notes:

- ROS 2 Humble officially targets Ubuntu 22.04.
- The WAM is real hardware. Keep the E-stop reachable, clear the workspace, and
  do not run motion commands until the arm is physically safe.
- The original ROS 1 demo and Perception Palm packages are not ported in this
  tree and are intentionally ignored by `colcon`.

## Install Dependencies

Install ROS 2 Humble first by following the official ROS 2 installation guide
for Ubuntu 22.04. Then install the build and runtime packages used by this
driver:

```bash
sudo apt update
sudo apt install -y \
  build-essential \
  cmake \
  git \
  can-utils \
  libboost-system-dev \
  libboost-thread-dev \
  libconfig++-dev \
  libeigen3-dev \
  libgsl-dev \
  libncurses5-dev \
  python3-colcon-common-extensions \
  ros-humble-joy \
  ros-humble-rqt \
  ros-humble-rqt-common-plugins \
  ros-humble-rqt-service-caller
```

If Conda is active, use the system ROS tools when building and running:

```bash
conda deactivate || true
source /opt/ros/humble/setup.bash
```

## Build and Install Patched libbarrett

This ROS 2 port requires a patched `libbarrett` for modern Ubuntu/libconfig/C++
toolchains. The patch set includes:

- Compatibility with stock Ubuntu `libconfig++`.
- C++17-compatible public headers.
- SocketCAN build support for non-realtime Linux control.
- A more robust CAN receive buffer during recovery/reconnect.
- `COLCON_IGNORE` so `colcon` does not try to build `libbarrett` as a ROS
  package.

If this repository contains the patched `libbarrett/` folder, build it with:

```bash
cd ~/Dropbox/Barrett-7-DoF-Robot-Arm-ROS1/libbarrett
mkdir -p build
cd build
cmake .. \
  -DNON_REALTIME=true \
  -DWITH_PYTHON=OFF \
  -DCMAKE_BUILD_TYPE=Release
make -j$(nproc)
sudo make install
sudo ldconfig
```

Verify installation:

```bash
ldconfig -p | grep libbarrett
test -f /usr/local/share/barrett/barrett-config.cmake && echo "Barrett CMake config OK"
test -f /usr/local/include/barrett/detail/libconfig_c_setting.h && echo "Patched headers OK"
```

## Configure CAN

Bring up the CAN interface before starting the WAM node:

```bash
sudo modprobe can
sudo modprobe can_raw
sudo modprobe can_dev

sudo ip link set can0 down || true
sudo ip link set can0 up type can bitrate 1000000 restart-ms 100
ip -details -statistics link show can0
```

Optional passive bus check:

```bash
candump -L can0
```

Do not leave `candump` or any other CAN reader/writer running while starting
`wam_node`.

## Build the ROS 2 Workspace

From the repository root:

```bash
cd ~/Dropbox/Barrett-7-DoF-Robot-Arm-ROS1
source /opt/ros/humble/setup.bash
./build.sh
source install_ros2/setup.bash
```

Expected ROS 2 packages:

```bash
ros2 pkg list | grep -E 'wam_|barrett_hand_node'
ros2 pkg executables wam_node
ros2 pkg executables wam_teleop
```

## First WAM Bring-Up

For first contact, run the executable directly so libbarrett can read terminal
input during the zeroing prompt:

```bash
cd ~/Dropbox/Barrett-7-DoF-Robot-Arm-ROS1
source /opt/ros/humble/setup.bash
source install_ros2/setup.bash
./install_ros2/wam_node/lib/wam_node/wam_node
```

Follow the safety pendant sequence:

1. Clear the workspace and keep E-stop reachable.
2. Release E-stop.
3. When prompted, press `SHIFT + IDLE`.
4. If prompted to zero, move the WAM to its physical home/zero pose and press
   `Enter`.
5. When prompted, press `SHIFT + ACTIVATE`.

After startup, verify ROS discovery:

```bash
ros2 node list
ros2 topic list -t
ros2 service list -t
ros2 topic echo --once /wam/joint_states
```

## Launching wam_node

After the first bring-up is working, launch through ROS 2:

```bash
ros2 launch wam_node wam_node.launch.py
```

Launch arguments:

```bash
ros2 launch wam_node wam_node.launch.py --show-args
```

Important defaults:

- `auto_gravity_comp:=true`
  - Matches the original ROS 1 behavior.
  - Required for stable joint-position control on gravity-loaded joints.
- `initialize_hand_on_startup:=false`
  - Avoids automatic BarrettHand initialization during first WAM startup.
- `hand_clearance_move_on_startup:=false`
  - Avoids the original automatic joint-4 clearance move.
- `home_velocity:=0.10`
- `home_acceleration:=0.10`

For a slower home motion:

```bash
ros2 launch wam_node wam_node.launch.py home_velocity:=0.05 home_acceleration:=0.05
```

## ROS 2 Topics

State topics:

```text
/wam/joint_states       sensor_msgs/msg/JointState
/wam/pose               geometry_msgs/msg/PoseStamped
/wam/move_is_done       std_msgs/msg/Bool
/fts/fts_states         geometry_msgs/msg/Wrench, if F/T sensor is present
```

Command topics:

```text
/wam/cart_vel_cmd       wam_msgs/msg/RTCartVel
/wam/ortn_vel_cmd       wam_msgs/msg/RTOrtnVel
/wam/jnt_vel_cmd        wam_msgs/msg/RTJointVel
/wam/jnt_pos_cmd        wam_msgs/msg/RTJointPos
/wam/cart_pos_cmd       wam_msgs/msg/RTCartPos
```

Realtime command topics are timeout-based. If messages stop, the node returns
to holding the current joint position.

## ROS 2 Services

Core WAM services:

```text
/wam/gravity_comp       wam_srvs/srv/GravityComp
/wam/go_home            std_srvs/srv/Empty
/wam/hold_joint_pos     wam_srvs/srv/Hold
/wam/hold_cart_pos      wam_srvs/srv/Hold
/wam/hold_ortn          wam_srvs/srv/Hold
/wam/joint_move         wam_srvs/srv/JointMove
/wam/pose_move          wam_srvs/srv/PoseMove
/wam/cart_move          wam_srvs/srv/CartPosMove
/wam/ortn_move          wam_srvs/srv/OrtnMove
```

Enable gravity compensation:

```bash
ros2 service call /wam/gravity_comp wam_srvs/srv/GravityComp "{gravity: true}"
```

Move to an absolute 7-DOF joint pose:

```bash
ros2 service call /wam/joint_move wam_srvs/srv/JointMove \
  "{joints: [0.0, -1.5, 0.0, 2.0, 0.0, 0.0, 0.0]}"
```

Move to the configured libbarrett home pose:

```bash
ros2 service call /wam/go_home std_srvs/srv/Empty "{}"
```

Important: `/wam/go_home` is not all-zero joints. For a typical 7-DOF wrist WAM
the configured home pose is:

```text
[0, -2, 0, 3.13, 0, 0, 0]
```

This value comes from `/etc/barrett/calibration_data/wam7w/zerocal.conf` or the
corresponding WAM configuration selected by `libbarrett`.

## Safe Joint Motion Test

Use the helper script to test one joint at a time. It reads the current joint
state, adds a small delta to one joint, calls `/wam/joint_move`, waits for
`/wam/move_is_done`, and prints final error.

```bash
cd ~/Dropbox/Barrett-7-DoF-Robot-Arm-ROS1
source /opt/ros/humble/setup.bash
source install_ros2/setup.bash

scripts/wam_joint_nudge.py 1 0.02
scripts/wam_joint_nudge.py 1 -0.02
```

Test joints 1 through 7 slowly. Do not send the next command until the previous
one finishes.

If Fast DDS prints shared-memory warnings such as `Failed init_port`, disable
Fast DDS shared memory for the current terminal:

```bash
export RMW_FASTRTPS_USE_SHM=0
```

## rqt

Launch `rqt` from the repository helper so it uses the same ROS 2 workspace
environment and avoids Conda/Snap/Qt library pollution:

```bash
cd ~/Dropbox/Barrett-7-DoF-Robot-Arm-ROS1
./run_rqt_wam.sh --clear-config --force-discover
```

Open:

```text
Plugins -> Services -> Service Caller
```

For `/wam/joint_move`, edit the `joints` expression from the default empty
array to a 7-value Python list:

```python
[0.0, -1.5, 0.0, 2.0, 0.0, 0.0, 0.0]
```

Do not publish to `/wam/joint_states`; it is a feedback topic, not a command
topic.

## BarrettHand Node

For standalone BarrettHand control over CAN:

```bash
ros2 launch barrett_hand_node barrett_hand_node.launch.py
```

Example services:

```bash
ros2 service call /bhand/initialize std_srvs/srv/Empty "{}"
ros2 service call /bhand/open_grasp std_srvs/srv/Empty "{}"
ros2 service call /bhand/close_grasp std_srvs/srv/Empty "{}"
ros2 service call /bhand/grasp_pos wam_srvs/srv/BHandGraspPos "{radians: 1.0}"
```

## Joystick Teleoperation

Start the WAM node first, then launch teleoperation:

```bash
ros2 launch wam_teleop wam_joystick_teleop.launch.py
```

The teleop node subscribes to `/joy` and publishes WAM realtime velocity
commands.

## Recovery After E-stop or Shaking

If the WAM shakes, behaves unexpectedly, or E-stop is pressed:

1. Press E-stop.
2. Stop `wam_node`.
3. Reset the CAN interface:

   ```bash
   sudo ip link set can0 down
   sudo ip link set can0 up type can bitrate 1000000 restart-ms 100
   ```

4. Restart `wam_node`.
5. Press `SHIFT + IDLE`.
6. Zero if prompted.
7. Press `SHIFT + ACTIVATE`.
8. Re-enable gravity compensation if needed:

   ```bash
   ros2 service call /wam/gravity_comp wam_srvs/srv/GravityComp "{gravity: true}"
   ```

Do not continue sending commands after serious shaking without restarting the
node and recovering the safety state.

## Troubleshooting

### `librcl_interfaces__rosidl_typesupport_cpp.so` not found

You likely overwrote `LD_LIBRARY_PATH`. Source ROS 2 and prepend any local
library path instead of replacing it:

```bash
source /opt/ros/humble/setup.bash
source install_ros2/setup.bash
LD_LIBRARY_PATH=$PWD/libbarrett/build/src:$LD_LIBRARY_PATH ./install_ros2/wam_node/lib/wam_node/wam_node
```

### `rqt` loads Snap or Conda libraries

Use:

```bash
./run_rqt_wam.sh --clear-config --force-discover
```

If needed, use a fresh non-Snap terminal and deactivate Conda.

### `BusManager::storeMessage: Buffer overflow`

Use the patched `libbarrett` in this repository. Rebuild and install it:

```bash
cd libbarrett/build
make -j$(nproc)
sudo make install
sudo ldconfig
```

Then reset CAN before reconnecting:

```bash
sudo ip link set can0 down
sudo ip link set can0 up type can bitrate 1000000 restart-ms 100
```

### Joint commands are accepted but gravity-loaded joints do not move

Ensure gravity compensation is enabled:

```bash
ros2 service call /wam/gravity_comp wam_srvs/srv/GravityComp "{gravity: true}"
```

The default launch file enables it automatically with `auto_gravity_comp:=true`.

### `rqt` sends `0-DOF request received`

The Service Caller sent an empty array. For `/wam/joint_move`, the `joints`
field must be a 7-element list.

### WAM home target is unexpected

`/wam/go_home` uses the home pose from libbarrett configuration, usually in:

```text
/etc/barrett/calibration_data/wam7w/zerocal.conf
```

Check that the installed Barrett calibration files match your physical WAM.

## Citation

If this repository is useful for your work, please cite or acknowledge the
project repository and Barrett Technology's original ROS/libbarrett software.

## License

This repository is derived from Barrett ROS/libbarrett software. Keep the
original license files and notices from upstream components when redistributing
or modifying this code.
