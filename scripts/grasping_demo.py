#!/usr/bin/env python3

import sys
import signal
import rospy
import moveit_commander
import copy
from gazebo_ros_link_attacher.srv import Attach, AttachRequest, AttachResponse


class LinkAttacher:
    def __init__(self):
        self.attach_srv = rospy.ServiceProxy('/link_attacher_node/attach', Attach)
        self.attach_srv.wait_for_service()
        rospy.loginfo("Created ServiceProxy to /link_attacher_node/attach")

        self.detach_srv = rospy.ServiceProxy('/link_attacher_node/detach', Attach)
        self.detach_srv.wait_for_service()
        rospy.loginfo("Created ServiceProxy to /link_attacher_node/detach")

        self.req = AttachRequest()
        self.req.model_name_1 = "panda"
        self.req.link_name_1 = "panda_leftfinger"
        self.req.model_name_2 = "unit_cylinder"
        self.req.link_name_2 = "link"
        rospy.sleep(1)

    def attach(self):
        try:
            self.attach_srv.call(self.req)
        except rospy.ServiceException as e:
            print(f"Service did not process request: {e}")

    def detach(self):
        try:
            self.detach_srv.call(self.req)
        except rospy.ServiceException as e:
            print(f"Service did not process request: {e}")


class Gripper:
    def __init__(self):
        self.gripper_group = moveit_commander.MoveGroupCommander("hand")
        self.max_aperture = 0.04
        self.min_aperture = 0.001

    def go_to_percent(self, percentage):
        """percentage should be between [0,100]"""
        percentage = 0 if percentage < 0 else 100 if percentage > 100 else percentage
        aperture = (percentage / 100) * (self.max_aperture - self.min_aperture) + self.min_aperture
        self.gripper_group.go([aperture, aperture])

    def open(self):
        self.go_to_percent(100)

    def close(self):
        self.go_to_percent(0)


def signal_handler(sig, frame):
    print('\nSIGINT or CTRL-C detected. Exiting gracefully')
    sys.exit(0)


def main():
    signal.signal(signal.SIGINT, signal_handler)
    rospy.init_node('grasping_demo')
    arm_group_name = "panda_arm"
    arm_group = moveit_commander.MoveGroupCommander(arm_group_name)
    # print(arm_group.get_current_rpy())
    # print(arm_group.get_current_pose())
    link_attacher = LinkAttacher()
    gripper = Gripper()
    
    rospy.loginfo('Opening the gripper')
    gripper.open()

    rospy.loginfo('Moving to the top of the object')
    arm_group.set_pose_target([0.152, -0.481, 0.6157, 1.609, 0.821,  0.06838])
    arm_group.go()

    rospy.loginfo('Moving to the grasping location')
    arm_group.set_pose_target([0.134, -0.475, 0.35, 1.6104, 0.8213, 0.0688])
    arm_group.go()

    rospy.loginfo('Creating a virtual fixed joint between the gripper and the object')
    link_attacher.attach()

    rospy.loginfo('Moving up')
    arm_group.set_pose_target([0.152, -0.481, 0.6157, 1.609, 0.821, 0.06838])
    arm_group.go()

    rospy.loginfo('Moving to the top of the goal')
    arm_group.set_pose_target([0.1252, 0.4813, 0.6289, 1.6011, 0.8063, -3.124])
    arm_group.go()

    rospy.loginfo('Moving to the goal location')
    arm_group.set_pose_target([0.1201, 0.55384, 0.42712, 1.6004, 0.8065, -3.1235])
    arm_group.go()

    rospy.loginfo('Detaching the object')
    link_attacher.detach()

    arm_group.set_named_target('ready')
    arm_group.go()


if __name__ == '__main__':
    main()
