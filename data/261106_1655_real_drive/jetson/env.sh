# 새 터미널마다 (Jetson): source ~/real_drive/env.sh
source /opt/ros/lyrical/setup.bash
[ -f "$HOME/Lv2_Cartlidar_Assignment/workspace/ros2_ws/install/setup.bash" ] && source "$HOME/Lv2_Cartlidar_Assignment/workspace/ros2_ws/install/setup.bash"
[ -d "$HOME/onnxruntime/lib" ] && export LD_LIBRARY_PATH="$HOME/onnxruntime/lib:${LD_LIBRARY_PATH:-}"
