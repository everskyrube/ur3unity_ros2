
#!/bin/bash

source /opt/ros/humble/setup.bash                                                                         
ros2 launch rosbridge_server rosbridge_websocket_launch.xml

# ws://localhost:9090