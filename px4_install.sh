cd
git clone https://github.com/PX4/PX4-Autopilot.git --recursive
bash ./PX4-Autopilot/Tools/setup/ubuntu.sh
cd PX4-Autopilot/
make px4_sitl
cd ~/PX4-Autopilot/ && colcon build
cd ~/PX4-Autopilot && make px4_sitl gz_x500
