# export ROS_HOME="/home/tup/ros2_ws"
export ROS_HOME="/home/liyuhang/TUP-Engineer-2023"
cd /home/liyuhang/TUP-Engineer-2023
# 设置需要启动和监视的ros命令
declare -a commands=("ros2 launch global_user engineer_exchange_stone_bringup.launch.py")

# 定义函数来启动ros命令
start_commands() {
  for cmd in "${commands[@]}"
  do
	gnome-terminal -- bash -c "source /opt/intel/openvino_2022/setupvars.sh;source /opt/ros/galactic/setup.bash;cd $(pwd);source install/setup.bash;$cmd;exec bash;"
  done
}

# 定义函数来监视ros进程是否在运行
watch_commands() {
  while true
  do
    for cmd in "${commands[@]}"
    do
      process_name=$(echo $cmd | awk '{print $1}') # 提取进程名
      if ! pgrep -f "$cmd" > /dev/null
      then
        echo "$cmd 未在运行, 重新启动"
	gnome-terminal -- bash -c "source /opt/intel/openvino_2022/setupvars.sh;source /opt/ros/galactic/setup.bash;cd $(pwd);source install/setup.bash;$cmd;exec bash;"
      fi
    done
    sleep 1 # 设置睡眠时间，以便在下一次检查之前给进程足够的时间运行
  done
}

# 启动ros命令和监视进程
start_commands
watch_commands


