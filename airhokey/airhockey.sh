# Run the Python script
python3 /home/"user name"/catkin_ws/src/indy_driver/src/indy_set_velocity.py

# Define a function to run a command in a new Terminator window and arrange it
function run_in_terminator {
    local command=$1
    # Use Terminator's --geometry option to specify the window position and size (format: WIDTHxHEIGHT+X+Y)
    # Adjust values as needed
    terminator --geometry=800x600+0+0 -e "$command" &
    sleep 5
}

# Run the commands
run_in_terminator "bash -c 'source devel/setup.bash; roslaunch indy10_moveit_config moveit_planning_execution.launch robot_ip:=192.168.0.8; exec bash'"
run_in_terminator "bash -c 'source devel/setup.bash; rosrun indy_driver camera.py; exec bash'"
run_in_terminator "bash -c 'source devel/setup.bash; rosrun indy_driver move_robot.py; exec bash'"

# Wait for user input to close all windows
read -p "Press Enter to close all Terminator windows"

# Close all Terminator windows
killall terminator
