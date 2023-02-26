# CSCE452_project2
CSCE452_project2

There are two packages in this workspace. One is called "my_package" which is the publisher/subscriber directory for controlling the turtle. The other is called "py_srvcli" which is the client/server package for initializing/changing the backgorund color and clearing the turtle line/setting turtle line to white.

1. First, source ROS2 as a whole by running: "source /opt/ros/humble/setup.bash"

2. AFTER MAKING CHANGES TO CODE: Go to project's root directory (~/CSCE452_project2/) and build by running "colcon build --packages-select (package)" (my_package for pubsub or py_srvcli for srvcli)

3. TO RUN: After building, go to a NEW TERMINAL and cd to the project's root directory and then source by running ". install/setup.bash". To run the publisher: "ros2 run my_package talker". To run the subscriber: "ros2 run my_package listener". ~~To run service: "ros2 run py_srvcli service". To run client: "ros2 run py_srvcli client"~~
To run set_color_client: "ros2 run py_srvcli set_color_client". To run set_color_service: "ros2 run py_srvcli set_color_service".
To run clear_line_client: "ros2 run py_srvcli clear_line_client". To run clear_line_service: "ros2 run py_srvcli clear_line_service".

4. Go to ~/CSCE452_project2/src/my_package/my_package to access the publisher/subscriber nodes

5. Go to ~/CSCE452_project2/src/py_srvcli/py_srvcli to access the service/client nodes

To run turtlesim: ros2 run turtlesim turtlesim_node
To run teleop_key: ros2 run turtlesim turtle_teleop_key