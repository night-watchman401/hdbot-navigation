# 安装与构建

## 支持环境

目标环境为 Ubuntu 24.04、ROS 2 Jazzy、C++17。Ubuntu 22.04 / ROS 2 Humble 未验证。以下命令假定 ROS 2 Jazzy 的 apt 源及基础环境已按官方说明安装：

https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html

先安装构建工具和常用依赖（这些命令会安装系统软件，应由用户在自己的机器执行）：

```bash
sudo apt update
sudo apt install -y build-essential cmake git python3-colcon-common-extensions \
  python3-rosdep python3-pytest libboost-all-dev libtbb-dev libmetis-dev \
  libsuitesparse-dev ros-jazzy-navigation2 ros-jazzy-nav2-bringup \
  ros-jazzy-slam-toolbox ros-jazzy-cv-bridge ros-jazzy-tf-transformations \
  python3-opencv
source /opt/ros/jazzy/setup.bash
```

首次使用 rosdep 的机器执行 `sudo rosdep init`，已初始化则跳过；随后执行 `rosdep update`。

## 外部 GTSAM

本模块依赖官方 `DopplerFactorArm` 的局部导航坐标系构造函数，不能仅按版本号选择任意 GTSAM。已验证提交为：

```text
https://github.com/borglab/gtsam
71a25ca36c084cbad1f872e812d6d97fbadfdb05
```

依赖放在项目仓库之外，安装到独立目录；不用系统管理员权限安装 GTSAM：

```bash
mkdir -p "$HOME/hdbot-deps"
git clone --recursive https://github.com/borglab/gtsam.git "$HOME/hdbot-deps/gtsam"
git -C "$HOME/hdbot-deps/gtsam" checkout --detach 71a25ca36c084cbad1f872e812d6d97fbadfdb05
git -C "$HOME/hdbot-deps/gtsam" submodule update --init --recursive
cmake -S "$HOME/hdbot-deps/gtsam" -B "$HOME/hdbot-deps/gtsam-build" \
  -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX="$HOME/.local/hdbot-deps/gtsam" \
  -DGTSAM_BUILD_TESTS=OFF -DGTSAM_BUILD_EXAMPLES_ALWAYS=OFF \
  -DGTSAM_BUILD_UNSTABLE=OFF -DGTSAM_BUILD_PYTHON=OFF \
  -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF
cmake --build "$HOME/hdbot-deps/gtsam-build" --parallel 2
cmake --install "$HOME/hdbot-deps/gtsam-build"
```

构建后端时指定安装配置目录 `GTSAM_DIR`。内存不足时将并行数改为 1。上述步骤是外部依赖的可执行安装说明，尚未在全新机器上完整复核；本机现有依赖已通过接口编译/链接探测。若安装布局不同，可用 `find "$HOME/.local/hdbot-deps/gtsam" -name GTSAMConfig.cmake` 定位并取其所在目录。

## 构建五个模块

在本项目的 `chapt8_ws` 中执行。明确列出源码范围，避免本地保留的演示包影响依赖解析：

```bash
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths \
  src/hdbot_gtsam_backend src/hdbot_gnss_msgs src/nav2_custom_planner \
  src/nav2_custom_controller src/autopartol_robot \
  --ignore-src --rosdistro jazzy --skip-keys gtsam -y
colcon build --base-paths \
  src/hdbot_gtsam_backend src/hdbot_gnss_msgs src/nav2_custom_planner \
  src/nav2_custom_controller src/autopartol_robot \
  --parallel-workers 2 --cmake-args -DCMAKE_BUILD_TYPE=Release \
  -DGTSAM_DIR="$HOME/.local/hdbot-deps/gtsam/lib/cmake/GTSAM"
source install/setup.bash
```

`--skip-keys gtsam` 只跳过已手动安装的 GTSAM，不表示它是可选依赖。CMake 会探测官方多普勒因子接口，接口不兼容时明确停止。

若只使用规划器、控制器或巡检包，可用 `colcon build --packages-select nav2_custom_planner nav2_custom_controller autopartol_robot`，无需构建 GNSS 后端或安装 GTSAM。

## 验证

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
colcon test --packages-select hdbot_gtsam_backend
colcon test-result --test-result-base build/hdbot_gtsam_backend --verbose
python3 -m pytest src/autopartol_robot/test/test_patrol_utils.py \
  src/autopartol_robot/test/test_patrol_flow.py -q
```

这些是功能测试命令，不代表全仓库 lint、实车安全或完整导航系统均已通过。
