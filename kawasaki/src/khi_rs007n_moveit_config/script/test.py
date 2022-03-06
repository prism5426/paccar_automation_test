#!/usr/bin/env python


from re import X
import rospy
import tf, tf_conversions
from geometry_msgs.msg import PoseStamped, Pose
from tf.transformations import quaternion_from_euler

from moveit_commander import MoveGroupCommander

if __name__ == '__main__':

    #init_node()
    rospy.init_node('message', anonymous=True)
    group = MoveGroupCommander("manipulator")
    exec_vel = 0.5

    rospy.loginfo("returning to home")
    group.set_named_target('home')
    group.go()
    rospy.loginfo("arm is at home position")
    rospy.sleep(2)

    end_effector_link = group.get_end_effector_link()

    # set reference frame to be base of arm
    reference_frame = "base_link"
    group.set_pose_reference_frame(reference_frame)

    group.allow_replanning(True)

    # set goal position tolerance
    group.set_goal_position_tolerance(0.01)
    group.set_goal_orientation_tolerance(0.05)

    # set target pose
    rospy.loginfo("pose start")
    target_pose = PoseStamped()
    target_pose.header.frame_id = reference_frame

    x_pos = [0, -0.2, 0, 0.2]
    y_pos = [0.2, 0, -0.2, 0]
    counter = 0

    # RPY to convert: 90deg, 0, -90deg
    r = [-1.57, 0, 1.57, 0]
    p = [0, -1.57, 0, 1.57]
    y = [0, 0, 0, 0]


    for _ in range(25):
        # go to x=0, y=0.2, z=0.8
        target_pose.header.stamp = rospy.Time.now()     
        target_pose.pose.position.x = x_pos[counter]
        target_pose.pose.position.y = y_pos[counter]
        target_pose.pose.position.z = 0.8

        q = quaternion_from_euler(r[counter], p[counter], y[counter])
        target_pose.pose.orientation.x = q[0]
        target_pose.pose.orientation.y = q[1]
        target_pose.pose.orientation.z = q[2]
        target_pose.pose.orientation.w = q[3]

        group.set_start_state_to_current_state()
        group.set_pose_target(target_pose, end_effector_link)
        group.go()

        if counter == 3:
            counter = 0
        else:
            counter = counter + 1

    rospy.loginfo("pose end")