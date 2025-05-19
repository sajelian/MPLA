# TTK4192: Mission Planning for Autonomous Systems

### Setup:
Run the following commands in a terminal (assuming all dependencies are installed):

- `export TURTLEBOT3_MODEL=waffle`

- `export ROS_WS_NAME=<your_ros_workspace_name>`

- `cd $ROS_WS_NAME`

- `source devel/setup.bash`

### Turtlebot3 Bringup:
For the **simulation** run `roslaunch assignment4_ttk4192 turtlebot3_ttk4192_sim.launch`.
This will start the simulation in Gazebo with the Turtlebot3 Waffle model and manipulator. 

When using **real hardware**, run the following command instead: `roslaunch assignment4_ttk4192 turtlebot3_ttk4192_real.launch`.


### Mission Execution:
`rosrun assigment4_ttk4192 mission_execution.py`

Check `scripts/files_description.md` for further structural explanation.
