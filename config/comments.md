Steps:
adjust wheel size and wheel base and gear ratio in esp32 code
flash code
add config yaml for speed x and yaw
drive

Problems:
Ros adjust pid topic


Comments:

launch teleop:
ros2 launch teleop_twist_joy teleop-launch.py joy_config:='xbox' config_filepath:=config/bot135smallWheel_teleop.config.yaml


