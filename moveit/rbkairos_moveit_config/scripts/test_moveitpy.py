#! /usr/bin/env python

import rospy
from moveit_python import MoveGroupInterface
from moveit_msgs.msg import MoveItErrorCodes

rospy.init_node('moveit_python_tutorial', anonymous=True)

move_group = MoveGroupInterface("arm", "robot_base_link")

joints = ["robot_arm_elbow_joint", "robot_arm_shoulder_lift_joint", "robot_arm_shoulder_pan_joint",
                  "robot_arm_wrist_1_joint", "robot_arm_wrist_2_joint", "robot_arm_wrist_3_joint"]

#pose = [-2.10, -0.45, 0.0, -1.0, 2.3, 1.3]
#pose = [-1.43, -0.73, 0.99, -1.44, 1.49, 1.70]
pose = [-1.95, -0.43, 0.0, -1.51, 2.32, 1.07]

while not rospy.is_shutdown():

    result = move_group.moveToJointPosition(joints, pose, 0.02)
    if result:

        if result.error_code.val == MoveItErrorCodes.SUCCESS:
            rospy.loginfo("Trajectory successfully executed!")
        else:
            rospy.logerr("Arm goal in state: %s",
                            move_group.get_move_action().get_state())
    else:
        rospy.logerr("MoveIt failure! No result returned.")

move_group.get_move_action().cancel_all_goals()
