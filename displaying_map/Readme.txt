https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Introducing-Turtlesim/Introducing-Turtlesim.html
sudo apt update
sudo apt install ros-humble-turtlesim
ros2 run turtlesim turtlesim_node
ros2 run turtlesim turtle_teleop_key
sudo apt update
sudo apt install ros-humble-rosbridge-suite
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
python3 -m http.server (not neccesry)
