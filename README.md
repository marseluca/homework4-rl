<h2>Commands to launch</h2>

<h3>Running Gazebo, Rviz and rqt_steering</h3>
<code>roslaunch fra2mo_2dnav fra2mo_nav_bringup.launch</code>

<h3>Running the navigator</h3>
<code>rosrun fra2mo_2dnav tf_nav</code><br>
<b>NOTE:</b> Running this node will complete different tasks between this branch and the "first4goals-only" branch

<h3>Running the camera for Aruco marker recognition</h3>
<code>rlaunch aruco_ros usb_cam_aruco.launch</code><br>
<code>rqt_image_view</code>
