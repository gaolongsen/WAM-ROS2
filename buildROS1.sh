source /opt/ros/melodic/setup.bash
cd ~/barrett-ros2-pkg
catkin_make install --pkg wam_msgs
catkin_make install --pkg wam_srvs
catkin_make install --pkg bhand_msgs
catkin_make install --pkg bhand_srvs
source ~/barrett-ros2-pkg/devel/setup.bash
catkin_make install --pkg wam_sim_node
catkin_make install --pkg wam_demos
catkin_make install --pkg perception_palm
catkin_make
