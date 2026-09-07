#!/bin/bash

# systemd 下偶发 HOME/USER 为空，先钉死
export HOME="${HOME:-/home/hiwonder}"
export USER="${USER:-hiwonder}"

source "$HOME/ros_ws/.typerc"

export CUDA_HOME=/usr/local/cuda
export PATH=$CUDA_HOME/bin:$PATH
export LD_LIBRARY_PATH=$CUDA_HOME/lib64:$LD_LIBRARY_PATH

# 对齐终端 .hiwonderrc：缺这两项时 numpy/OpenBLAS 会 SIGILL（exit -4）
export OPENBLAS_CORETYPE=ARMV8
if [ -f /usr/lib/aarch64-linux-gnu/libgomp.so.1 ]; then
  export LD_PRELOAD="/usr/lib/aarch64-linux-gnu/libgomp.so.1${LD_PRELOAD:+:$LD_PRELOAD}"
fi
export AUDIODRIVER=alsa
export XDG_RUNTIME_DIR="${XDG_RUNTIME_DIR:-/run/user/$(id -u)}"
export PULSE_SERVER="${PULSE_SERVER:-unix:$XDG_RUNTIME_DIR/pulse/native}"

# 对齐手开成功的那套：不要用 localhost，也不要设 ROS_HOSTNAME
export ROS_IP=127.0.0.1
export ROS_MASTER_URI=http://127.0.0.1:11311
unset ROS_HOSTNAME

if [ $ZSH_VERSION ]; then
  . /opt/ros/melodic/setup.zsh
  . $HOME/ros_ws/devel/setup.zsh
elif [ $BASH_VERSION ]; then
  . /opt/ros/melodic/setup.bash
  . $HOME/ros_ws/devel/setup.bash
else
  . /opt/ros/melodic/setup.sh
  . $HOME/ros_ws/devel/setup.sh
fi
export DISPLAY=:0.0
exec "$@"
