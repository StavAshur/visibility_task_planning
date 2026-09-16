=================================================================
===================== SSH access to robot =======================
=================================================================

Test communication using command
"ping 192.168.131.1"

SSH using the command
"ssh administrator@192.168.131.1"
password: clearpath

Copying files from pc into the robot is done using "scp":
scp <file_to_copy> administrator@192.168.131.1:<location_to_copy_to>
Example:
scp ~/catkin_ws/src/tau01_husky/tau01_moveit_config/config/husky.srdf  administrator@192.168.131.1:~/catkin_ws/src/tau01_husky/tau01_moveit_config/config/husky.srdf

First time using a new computer add the line:
"192.168.131.1    cpr-a200-0738"
to the file:
/etc/hosts

If using distrobox might need to use the command"
" sudo sysctl -w net.ipv4.ping_group_range="0 2147483647" "
to allow distrobox VMs to access ethernet socket

=================================================================
==================== Setting up pc + robot ======================
=================================================================

1. Turn robot on: 
  1.1 Turn Husky on
  1.2. Wait for comm light to turn green, then press 1 in the emergency remote (remote with 4 round black buttons + red round emergency button) to unlock
  1.3. Turn UR on
  1.4. Wait for screen to allow initialization, go to initialization screen, turn on and start UR5 (same button in initialization screen)

2. Open terminal in PC, SSH into robot ("ssh administrator@192.168.131.1") and start bringup launch file:
  "roslaunch tau01_bringup tau01_bringup.launch"

3. In UR screen 
  3.1. Press "file" (top left)
  3.2. "load program"
  3.3. Choose "external control"
  3.4. Press play icon (bottom center)

  Command "rostopic echo /ur/ur_hardware_interface/robot_program_running" from PC terminal should return "true" if working properly.

4. Open terminal in PC, SSH into robot ("ssh administrator@192.168.131.1") and start moveit launch file:
  "roslaunch tau01_moveit_config tau01_moveit_planning_execution.launch"

5. Open how ever many terminals you need in PC:
  5.0 enter distrobox using "distrobox enter <vm_name>" (example: distrobox "enter ubuntu1804-husky")
  5.1 Got to catkin_ws and source ("source devel/setup.bash")
  5.2 If we want the commands to run from a non-ssh'd terminal - need to set the robot as ros master 
    "export ROS_MASTER_URI=http://192.168.131.1:11311"
    "export ROS_IP=192.168.131.X" where X depends on the ethernet socket in the Husky?

    The two addresses I've seen are 100 and 101:
    "export ROS_IP=192.168.131.100"
    "export ROS_IP=192.168.131.101"

  For convinience:
    "export ROS_MASTER_URI=http://192.168.131.1:11311 && export ROS_IP=192.168.131.100"
    "export ROS_MASTER_URI=http://192.168.131.1:11311 && export ROS_IP=192.168.131.101"


  5.3 For visualization of the robot in RViz:
  "roslaunch husky_viz view_robot.launch"

  5.4 To run visual-based motion planning:
    5.4.1 Launch service: "roslaunch visual_based_planning visual_planner.launch". See next section for parameters.
    5.4.2 Set up scene: "roslaunch scene_builder load_scene.launch". Only AABBs for now.
      edit file catkin_ws/src/scene_builder/config/obstacles.yaml to change environment
    5.4.3 Launch client with visual planning task "rosrun visual_based_planning visual_planning_client"
      Targets are defined in catkin_ws/src/visual_based_planning/config/targets.yaml


  5.5 To run the VisTSP experiments (global_plan steps 3-4), everything in one command:
      "roslaunch visual_based_planning vistsp_experiments_full.launch"
      This starts MoveIt for the SIMULATED MOBILE UR5, RViz (with the target spheres
      displayed), the double-room scene, the planner node and the experiment runner.
      Start small the first time:
        "roslaunch visual_based_planning vistsp_experiments_full.launch num_trials:=1 targets_per_trial:=3"
      Results: vistsp_experiments.txt and .csv in the package directory.

      It must be the MOBILE UR5 MoveIt config, not tau01: planner_config.yaml plans for
      the group "whole_robot", which is defined only in mobile_ur_moveit_config's
      mobile_ur5.srdf. The scene must be scene_builder's double_room_1.yaml, whose
      shelves are the bands config/sampling_regions_double_room.yaml samples from.

      Every trial starts from the SRDF group_state "vertical" (base at (0, -3, 0),
      arm up), and the runner parks the simulated robot there first. Change it with
      start_state:=<another group_state>.

      To keep the planner node warm between runs (the debugging loop), start the stack
      once and then re-run the runner alone:
        "roslaunch visual_based_planning vistsp_experiments.launch launch_planner:=false num_trials:=1"

  6. To use simulated robot instead of real robot replace all above robot instructions with:
    6.1 Open and source two terminals
    6.2 In first terminal "roscore"
    6.3 Launch simulation in second "roslaunch tau01_moveit_config demo.launch use_gui:=true"
      gui is used to move the robot away from first position if invalid
      For the VisTSP experiments use the mobile UR5 instead:
        "roslaunch mobile_ur_moveit_config demo.launch use_gui:=true"


The order no longer matters (since 2026-09-16): visual_planning_node pulls the planning
scene from move_group at startup and before every request, so the scene may be loaded
before or after the service. Loading it afterwards used to be mandatory, and forgetting
it meant planning in an empty world with no error.

  7. Zed camera is set to observe the workspace and populate the parameter /target_points
    with the coordinates of the targets w.r.t. the robot's pose, which is determined by
    observing the QR code. Some important details:
    7.1 Activation of the setup is "roslaunch zed_color_tracker tracker.launch"
    7.2 The QR code is ID 42 in ArUco DICT_6X6_250
    7.3 The target points are all pixels with some color, where the color is declared in colors.yaml
      as a range of values. Currently set to red. 
    7.4 For debugging use the depth viewer to see what the camera sees: "/usr/local/zed/tools/ZED_Depth_Viewer"
    7.5 The USB extension is the cause of many bugs. Make sure it is properly connected.



roslaunch viz_markers_pkg markers.launch 


=================================================================
=================== Planner parameterization ====================
=================================================================


1. Locate the Configuration File: catkin_ws/src/visual_based_planning/config/planner_config.yaml

2. Core Behavior Settings:
  2.1 mode: Selects the planning algorithm. Set to "VisRRT" (Tree-based, good for single queries)
    or "VisPRM" (Graph-based, good for multiple queries).
  2.2 use_visual_ik: Set to true to allow the planner to "snap" the tool orientation to face the target automatically.
    Set false for standard random sampling.
  2.3 use_visibility_integrity: Set to true to create the visibility integrity data structure before running RRT/PRM.
    Pre-processing time is long.
  2.4 enable_shortcutting: Set to true to post-process the path, removing unnecessary waypoints and creating smoother motion.
  2.5 workspace_bounds: Sets the environment bounding box for visibility integrity data structure and sampling 3D points. 

3. Robot & Constraints:
  3.1 group_name: The MoveIt planning group to control (e.g., "manipulator").
  3.2 ee_link_name: The link name of the flashlight/camera (e.g., "tool0").
  3.3 resolution: Step size for collision checking in radians (suggested 0.05). Smaller is safer but slower.
  3.4 visibility_threshold: Fraction of the target ball (0.0 to 1.0, suggested 0.8) that must be illuminated to consider the goal "reached".

4. Algorithm Tuning (RRT):
  4.1 rrt/goal_bias: Probability (0.0 - 1.0, suggested 0.01) of sampling the goal state directly. Higher values converge faster in open spaces.
  4.2 rrt/max_extension: Maximum distance (in joint space radians) the tree can grow in one step. Keep small (e.g., 0.1) for cluttered environments.
  4.3 rrt/max_iterations: Maximum number of samples to try before reporting failure (suggested 5000).

5. Visibility Integrity Parameters:
    5.1 visibility_integrity/num_samples: The number of valid points that will make up the data structure (1000 - fast, 5000 - takes time)
    5.2 visibility_integrity/vi_threshold: Integrity threshold for visibility clusters (range 0.0 - 1.0, suggested 0.8). Higher value will create more, but more cohesive cluster. 
    5.3 visibility_integrity/k_neighbors: Used to label query points and associate them with a cluster (suggested 5).
    5.4 visibility_integrity/limit_diameter_factor: Higher value will create more clusters but smaller and more convex (suggested 2.0).

=================================================================
========== Changes made in the Husky onboard computer ===========
=================================================================

I have changed the file /opt/ros/melodic/share/husky_description/urdf/tau01_husky_extras.urdf.xacro IN THE ROBOT COMPUTER:
Removed the gripper links and camera link because they were causing collisions in moveit for some reason,
and also not actually physically installed at the moment.

OMPL original lib files are stored in a dir. Different place in each pc I'm using, but main one is:
/opt/ros/melodic/lib/temp_ompl_lib_files

In order to attach stl files of luminating object - changed original_urdf_tau01.xacro,
and saved a backup of revious version as backup_original_urdf_tau01.xacro
Also added collision disables to /home/roblab22/catkin_ws/src/tau01_husky/tau01_moveit_config/config/husky.srdf:
<disable_collisions link1="ur_arm_wrist_3_link" link2="flashlight" reason="Adjacent" />
<disable_collisions link1="flashlight" link2="light_beam" reason="Adjacent" />

Some more collision disabling was added to the srdf file because the planning would not be performed otherwise:
  <disable_collisions link1="base_link" link2="front_bumper_extension_link" reason="Adjacent" />
  <disable_collisions link1="base_link" link2="rear_bumper_extension_link" reason="Adjacent" />
  <disable_collisions link1="front_bumper_extension_link" link2="front_bumper_link" reason="Adjacent" />
  <disable_collisions link1="front_bumper_extension_link" link2="top_chassis_link" reason="Adjacent" />
  <disable_collisions link1="rear_bumper_extension_link" link2="rear_bumper_link" reason="Adjacent" />
  <disable_collisions link1="rear_bumper_extension_link" link2="top_chassis_link" reason="Adjacent" />

Also added a "cable_link" which is supposed to protect the cables from manipulator movements

=================================================================
================ Using distrobox (Ubuntu 24.04) =================
=================================================================

To install vscode on an ubuntu 18.04 virutal machine:
sudo apt update
sudo apt install wget gpg
wget https://update.code.visualstudio.com/1.74.3/linux-deb-x64/stable -O code_1.74.3-1668896808_amd64.deb
sudo apt install ./code_1.74.3-1668896808_amd64.deb

This is a way of creating a virutal machine that should, in theory, be able to visualize. Getting libboost issues though.
distrobox create --image ubuntu:18.04 --name ubuntu-1804 --nvidia

To allow distrobox port access I use the command:
sudo sysctl -w net.ipv4.ping_group_range="0 2147483647"



=================================================================
============================= Misc. =============================
=================================================================


Joint states can be seen by
"rostopic echo /ur/joint_states"

home position is:
[0.0, -1.57, 0.0, -1.57, 0.0, 0.0]

Origin is center of the Husky by default.
Any x,y,z positions given to the UR are w.r.t the center of the Husky.

To find current tool position I used:
"rosrun tf tf_echo base_link ur_arm_tool0"

Was able to move the arm using this message:
"
rostopic pub /ur/ur_arm_scaled_pos_joint_traj_controller/command trajectory_msgs/JointTrajectory "header:
  stamp: now
  frame_id: ''
joint_names: ['ur_arm_shoulder_pan_joint','ur_arm_shoulder_lift_joint','ur_arm_elbow_joint','ur_arm_wrist_1_joint','ur_arm_wrist_2_joint','ur_arm_wrist_3_joint']
points:
- positions: [1.57, -1.57, 0.0, -1.57, 0.0, 0.0]
  time_from_start: {secs: 5, nsecs: 0}"
"


Using the moveit interface from the terminal to move the robot to start position:

python -c "import sys, rospy, moveit_commander; \
moveit_commander.roscpp_initialize(sys.argv); \
rospy.init_node('cmd_line_mover', anonymous=True); \
group = moveit_commander.MoveGroupCommander('manipulator'); \
group.set_max_velocity_scaling_factor(0.1); \
group.set_max_acceleration_scaling_factor(0.1); \
goal = dict(zip(['ur_arm_shoulder_pan_joint','ur_arm_shoulder_lift_joint','ur_arm_elbow_joint','ur_arm_wrist_1_joint','ur_arm_wrist_2_joint','ur_arm_wrist_3_joint'], [0.0, -1.57, 0.0, -1.57, 3.14, 0.0])); \
group.go(goal, wait=True); \
moveit_commander.roscpp_shutdown()"



=================================================================
=========== Old irrelevant stuff I did when used OMPL ===========
=================================================================


In order to change OMPL implementations:
Install OMPL on pc
  1.Open some file:
  "sudo nano /etc/apt/sources.list.d/ros-latest.list"

  2. add this line (can actually write the distribution in the previous file for ROS instead of the parameter)
  "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main"

  3. important to update after step 2!
  "sudo apt update"

  4. get the OPML files by going to the directory where you want the OMPL dir and:
  "sudo apt source ros-${ROS_DISTRO}-ompl"

  5. Need to give permissions to do stuff in the OMPL dir:
  "sudo chown -R $USER:$USER ros-XXXXX-ompl-1.X.X"
  (replace X's so it would be the dir that was created by step 4)

  6. Go into the OMPL dir (ros-XXXXX-ompl-1.X.X or whatever)
  "mkdir build && cd build"
  "cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=$HOME/ompl_custom/install"
  "make -j$(nproc)"
  "make install"

  The path to where the ompl_costum dir is not very important in my opinion.
  Also the $(nproc) thing can be replaced with the number of threads deemed enough. I use 4.

Change OMPL implemnentations in ros-XXXXX-ompl-1.X.X/src/ompl
Dont forget to go into build and compile:
"make -j$(nproc)" 

Replace original OMPL files with new ones 

In ros-XXXXX-ompl-1.X.X/build/lib you have the binaries
libompl.so  libompl.so.1.X.X  libompl.so.XX
replace the old ones (after the originals have been copied for backup) in /opt/ros/XXXXX/lib (X's are ROS distro):
"sudo cp libompl.so* /opt/ros/melodic/lib/"


OMPL original lib files are stored in a dir. Different place in each pc I'm using, but main one is:
/opt/ros/melodic/lib/temp_ompl_lib_files



The OMPL moveit plugin also needs something similar.
For this I have created the ROS workspace ompl_bridge_ws which contains the single package I need.
Compiling is done the normal way (sourceing ROS and calling catkin_make)
sudo cp ~/ompl_bridge_ws/devel/lib/*.so* /opt/ros/melodic/lib/
original files are in /opt/ros/melodic/lib/temp_ompl_moveit_plugin


Commands to copy stuff to husky:

ompl:
scp ~/ros-melodic-ompl-1.4.2/build/lib/libompl.so* administrator@192.168.131.1:~/
sudo cp ~/libompl.so* /opt/ros/melodic/lib/

ompl-moveit interface:
scp ~/ompl_bridge_ws/devel/lib/libmoveit_ompl* administrator@192.168.131.1:~/
sudo cp ~/libmoveit_ompl* /opt/ros/melodic/lib/

VisibilityOracle.h:
scp ~/ros-melodic-ompl-1.4.2/src/ompl/datastructures/VisibilityOracle.h administrator@192.168.131.1:~/
sudo cp ~/VisibilityOracle.h /opt/ros/melodic/include/ompl-1.4/ompl/datastructures


Maybe need to do the same with RRTConnect.h
sudo cp ~/ros-melodic-ompl-1.4.2/src/ompl/geometric/planners/rrt/RRTConnect.h /opt/ros/melodic/include/ompl-1.4/ompl/geometric/planners/rrt/


I added  /opt/ros/melodic/include/ompl-1.4/ompl/datastructures/VisibilityOracle.h to get the ompl_interface to compile.
"sudo cp ~/ros-melodic-ompl-1.4.2/src/ompl/datastructures/VisibilityOracle.h /opt/ros/melodic/include/ompl-1.4/ompl/datastructures"

I switched RRTConnect.h in /opt/ros/melodic/... for the same reason.
Backup file is RRTConnect.h_old
"sudo cp ~/ros-melodic-ompl-1.4.2/src/ompl/geometric/planners/rrt/RRTConnect.h /opt/ros/melodic/include/ompl-1.4/ompl/geometric/planners/rrt/"



I did the exact same thing with trac_ik_kinematics_plugin:
"sudo apt source ros-melodic-trac-ik-kinematics-plugin
sudo chown -R $USER:$USER ros-melodic-trac-ik-kinematics-plugin_1.X.X
cd ros-melodic-trac-ik-kinematics-plugin_1.X.X
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/opt/ros/melodic \
  -DCMAKE_PREFIX_PATH=/opt/ros/melodic
make -j$(nproc)
sudo make install"

Only difference is that the lib file is now in:
~/trac_ik_custom/install/lib/

And also I built the file on a system (my pc) with libmoveit_kinematics_base.so.1.0.11,
but the robot has libmoveit_kinematics_base.so.1.0.10 installed.
I created a symbolic link in the robot's /opt/ros/melodic/lib that made it work:
sudo ln -s libmoveit_kinematics_base.so.1.0.10 libmoveit_kinematics_base.so.1.0.11
I can probably change it to 1.0.10 by checking out the right branch.

This is obviously very bad, but I hope it's good enough.

