# Disable Ubuntu firewall (ufw)
sudo ufw disable

# Set the MTU of the camera’s network interface (enp47s0) to 7200 bytes
sudo ifconfig enp47s0 mtu 7200

# Launch the Multisense S7 stereo camera ROS 2 node, specifying model S7 and MTU 7200
ros2 launch multisense_ros multisense_launch.py sensor:=S7 mtu:=7200

# Run the rqt_reconfigure GUI to enable HDR and set the max frame rate to 30 Hz
ros2 run rqt_reconfigure rqt_reconfigure
