#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy, roslib, sys
import moveit_commander
from moveit_msgs.msg import RobotTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

from geometry_msgs.msg import PoseStamped, Pose
from tf.transformations import euler_from_quaternion, quaternion_from_euler


class MoveItPlanningDemo:
    def __init__(self):
        moveit_commander.roscpp_initialize(sys.argv)

        rospy.init_node("moveit_kinova_demo")

        self.scene = moveit_commander.PlanningSceneInterface()
        self.scene.remove_world_object()
        rospy.sleep(2)

        self.arm = moveit_commander.MoveGroupCommander("arm")
        self.robot = moveit_commander.RobotCommander()

        self.end_effector_link = self.arm.get_end_effector_link()

        self.reference_frame = "joint_1"
        self.arm.set_pose_reference_frame(self.reference_frame)

        self.arm.allow_replanning(True)

        self.arm.set_goal_position_tolerance(0.01)
        self.arm.set_goal_orientation_tolerance(0.05)

    def add_one_box(self):
        box1_pose = [0.6, 0.0, 0.35, 0, 0, 0, 1]
        box1_dimensions = [0.16, 0.16, 0.16]

        self.add_box_object("box1", box1_dimensions, box1_pose)
        rospy.sleep(2)

    def add_two_box(self):
        box1_pose = [0.6, 0.0, 0.1, 0, 0, 0, 1]
        box1_dimensions = [0.16, 0.16, 0.16]

        self.add_box_object("box2", box1_dimensions, box1_pose)
        rospy.sleep(2)

    def add_box_object(self, name, dimensions, pose):
        p = PoseStamped()
        p.header.frame_id = self.robot.get_planning_frame()
        p.header.stamp = rospy.Time.now()
        p.pose.position.x = pose[0]
        p.pose.position.y = pose[1]
        p.pose.position.z = pose[2]
        p.pose.orientation.x = pose[3]
        p.pose.orientation.y = pose[4]
        p.pose.orientation.z = pose[5]
        p.pose.orientation.w = pose[6]

        self.scene.add_box(name, p, (dimensions[0], dimensions[1], dimensions[2]))

    def src_pos(self):
        joint_goal = self.arm.get_current_joint_values()
        joint_goal[0] = 1.0048
        joint_goal[1] = 1.5924
        joint_goal[2] = -2.9866
        joint_goal[3] = 0.3215
        joint_goal[4] = 2.9796
        joint_goal[5] = 1.8668
        joint_goal[6] = 0.9537

        self.arm.go(joint_goal, wait=True)
        self.arm.stop()        

    def dst_pos(self):
        joint_goal = self.arm.get_current_joint_values()
        joint_goal[0] = -0.9423
        joint_goal[1] = 1.6384
        joint_goal[2] = -0.1420
        joint_goal[3] = -0.0048
        joint_goal[4] = -2.3003
        joint_goal[5] = 2.0447
        joint_goal[6] = -2.0016

        self.arm.go(joint_goal, wait=True)
        self.arm.stop()        

    def run(self):
        self.add_one_box()
        self.add_two_box()
        self.src_pos()
        self.dst_pos()

        moveit_commander.roscpp_shutdown()
        moveit_commander.os._exit(0)


if __name__ == "__main__":
    o = MoveItPlanningDemo()
    o.run()