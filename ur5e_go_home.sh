#!/bin/bash
# UR5e: 드라이버 시작 시점의 원위치로 복귀 (8초에 걸쳐 이동)
source /opt/ros/humble/setup.bash
ros2 action send_goal /scaled_joint_trajectory_controller/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory "{
  trajectory: {
    joint_names: [shoulder_pan_joint, shoulder_lift_joint, elbow_joint, wrist_1_joint, wrist_2_joint, wrist_3_joint],
    points: [{positions: [-1.5623, -1.5682, -1.5762, -1.5672, 1.5724, -0.0027], time_from_start: {sec: 8}}]
  }}"
