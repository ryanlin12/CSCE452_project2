# CSCE452_project2
CSCE452_project2

1. First, source ROS2 as a whole by running: "source /opt/ros/humble/setup.bash"

2. AFTER MAKING CHANGES TO CODE: Go to project's root directory (~/CSCE452_project2/) and build by running "colcon build --packages-select my_package" 

3. TO RUN: After building, go to a NEW TERMINAL and cd to the project's root directory and then source by running "<. install/setup.bash>". To run the publisher: "ros2 run my_package talker". To run the subscriber: "ros2 run my_package listener"

4. Go to ~/CSCE452_project2/src/my_package/my_package to access the publisher/subscriber nodes