# igd_fr3_control

## Requires
- `apt install libspnav-dev spacenavd; systemctl start spacenav`
- `pip install # Download from https://github.com/cheng-chi/spnav/archive/c1c938ebe3cc542db4685e0d13850ff1abfdb943.tar.gz`
- fr3 packages including moveit_congid

## How to run
- Terminal 1: `ros2 launch franka_fr3_moveit_config moveit.launch.py   robot_ip:=192.170.10.101   robot_type:=fr3   use_fake_hardware:=false load_gripper:=true`
- Terminal 2: `ros2 launch igd_fr3_control fr3_spacemouse_servocontrol.launch.py `
- Terminal 3: `ros2 run igd_fr3_control spacemouse_twiststamped_publisher `
