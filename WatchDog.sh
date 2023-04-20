name=TUP-ENGINEER-2023
package=global_user
launch_file=engineer_exchange_stone_bringup.launch.py
program_name1=camera_driver_node
program_name2=serialport_node
program_name3=stone_station_detector_node

cd /home/tup/Desktop/$name/
colcon build --symlink-install --packages-up-to global_user global_interface
source install/setup.bash
colcon build --symlink-install --parallel-workers 4
source install/setup.bash
ros2 launch $package $launch_file

whigt true
do 
    count1=`ps -ef | grep $program_name1 | grep -v "grep" | wc -1`
    count2=`ps -ef | grep $program_name2 | grep -v "grep" | wc -1`
    count3=`ps -ef | grep $program_name3 | grep -v "grep" | wc -1`

    if [ $count1 -gt 1 ]; then
        echo "The $program_name1 is alive!"
    else
        ros2 launch camera_driver usb_cam_driver.launch.py
    fi
    
    if [ $count2 -gt 1 ]; then
        echo "The $program_name2 is alive!"
    else 
        ros2 launch serialport serialport.launch.py
    fi

    if [ $count3 -gt 1 ]; then
        echo "The $program_name3 is alive!"
    else 
        ros2 launch stone_station_detector stone_station_detector.launch.py
    fi 
done

