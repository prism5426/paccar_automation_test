#! /usr/bin/env python
# -*- coding: utf-8 -*-

import rospy, sys
import moveit_commander
from moveit_commander import MoveGroupCommander
from copy import deepcopy

if __name__ == '__main__':
    moveit_commander.roscpp_initialize(sys.argv)

    rospy.init_node('test2', anonymous=True)

    cartesian = rospy.get_param('~cartesian', True)

    arm = MoveGroupCommander('manipulator')

    arm.allow_replanning(True)

    rospy.loginfo("returning to home")
    arm.set_named_target('home')
    arm.go()
    rospy.loginfo("arm is at home position")
    rospy.sleep(2)

    arm.set_pose_reference_frame('base_link')

    arm.set_goal_position_tolerance(0.01)
    arm.set_goal_orientation_tolerance(0.01)

    end_effector_link = arm.get_end_effector_link()
    rospy.loginfo("end_link name = %s", end_effector_link)

    start_pose = arm.get_current_pose(end_effector_link).pose

    waypoints = []

    waypoints.append(start_pose)

    # second pose
    wpose = deepcopy(start_pose)
    wpose.position.z -= 0.2
    waypoints.append(deepcopy(wpose))

    # 3rd pose
    wpose.position.x += 0.45
    waypoints.append(deepcopy(wpose))

    # 4th pose
    wpose.position.y -= 0.45
    waypoints.append(deepcopy(wpose))

    fraction = 0.0   #路径规划覆盖率
    maxtries = 100   #最大尝试规划次数
    attempts = 0     #已经尝试规划次数
    
    # 设置机器臂当前的状态作为运动初始状态
    arm.set_start_state_to_current_state()

    # 尝试规划一条笛卡尔空间下的路径，依次通过所有路点
    while fraction < 1.0 and attempts < maxtries:
        (plan, fraction) = arm.compute_cartesian_path (
                                waypoints,   # waypoint poses，路点列表
                                0.01,        # eef_step，终端步进值
                                0.0,         # jump_threshold，跳跃阈值
                                True)        # avoid_collisions，避障规划
        
        # 尝试次数累加
        attempts += 1
        
        # 打印运动规划进程
        if attempts % 10 == 0:
            rospy.loginfo("Still trying after " + str(attempts) + " attempts...")
                    
    # 如果路径规划成功（覆盖率100%）,则开始控制机械臂运动
    if fraction == 1.0:
        rospy.loginfo("Path computed successfully. Moving the arm.")
        arm.execute(plan)
        rospy.loginfo("Path execution complete.")
    # 如果路径规划失败，则打印失败信息
    else:
        rospy.loginfo("Path planning failed with only " + str(fraction) + " success after " + str(maxtries) + " attempts.")  
