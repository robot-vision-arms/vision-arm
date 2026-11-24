# Team 2 native lifecycle executables

This isolated package does not replace `/opt/ros/jazzy` or the other team's Python modules.

- `team2_move_group`: upstream MoveIt 2.12.4 `move_group.cpp`, BSD license retained in the source and installed source copy. Planning, controller configuration and capabilities are unchanged. An outer capability loader and controller-library loader keep plugin code alive until nodes/action clients have been released. The external executor is destroyed before capabilities. Libraries are released normally; there is no `dlclose` interception, `_exit`, SIGSEGV handler or forced-success exit.
- `team2_gz_server`: a small lifecycle wrapper around installed Gazebo Harmonic `gz::sim::Server`. It initializes the shared ROS context before controller plugins, records SIGINT/SIGTERM, joins the two ros2_control statistics publisher threads while ROS time is still available, shuts the ROS context down before destroying the Gazebo server and joins through the normal server destructor. This avoids leaving controller executors alive while waiting for plugin destruction. Existing world, URDF, physics and controller plugins are reused.

References:
- https://github.com/moveit/moveit2/tree/2.12.4/moveit_ros/move_group/src
- https://github.com/moveit/moveit2/issues/3680
- https://github.com/moveit/moveit2/pull/3828
- https://github.com/gazebosim/ros_gz/blob/jazzy/ros_gz_sim/src/gzserver.cpp

Build with `source env.sh && colcon build --symlink-install` from the workspace root.
The Team 2 runner selects these executables. Ordinary sim.launch.py defaults continue to select the installed upstream executables; they are not globally patched.

The Team 2 launch adapter imports the installed UR launch and changes only its Gazebo process action. It fails explicitly if the expected include structure changes. No installed package file is modified.

The runner keeps the clock bridge alive until the Gazebo wrapper has released its statistics threads. Python worker pools are joined before executor guards are destroyed. Child launch stdin is detached from the manager TTY so interactive Ctrl+C and automated completion use the same shutdown path.
