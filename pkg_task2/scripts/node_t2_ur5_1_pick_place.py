#! /usr/bin/env python

import rospy
import sys
import copy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import actionlib
import math

from pkg_vb_sim.srv import vacuumGripper
from pkg_vb_sim.srv import vacuumGripperRequest
from pkg_vb_sim.srv import vacuumGripperResponse
from gazebo_ros_link_attacher.srv import Attach, AttachRequest, AttachResponse


class Ur5Moveit:

    # Constructor
    def __init__(self):
        
        rospy.init_node('node_t2_ur5_1_pick_place', anonymous=True)

        self._planning_group = "ur5_1_planning_group"
        self._commander = moveit_commander.roscpp_initialize(sys.argv)
        self._robot = moveit_commander.RobotCommander()
        self._scene = moveit_commander.PlanningSceneInterface()
        self._group = moveit_commander.MoveGroupCommander(self._planning_group)
        self._display_trajectory_publisher = rospy.Publisher(
            '/move_group/display_planned_path', 
            moveit_msgs.msg.DisplayTrajectory, queue_size=1)

        self._exectute_trajectory_client = actionlib.SimpleActionClient(
            'execute_trajectory', moveit_msgs.msg.ExecuteTrajectoryAction)
        self._exectute_trajectory_client.wait_for_server()

        self._planning_frame = self._group.get_planning_frame()
        self._eef_link = self._group.get_end_effector_link()
        self._group_names = self._robot.get_group_names()
        self._box_name = ''

        # Current State of the Robot is needed to add box to planning scene
        self._curr_state = self._robot.get_current_state()

        rospy.loginfo(
            '\033[94m' + "Planning Group: {}".format(self._planning_frame)
             + '\033[0m')
        rospy.loginfo(
            '\033[94m' + "End Effector Link: {}".format(self._eef_link)
             + '\033[0m')
        rospy.loginfo(
            '\033[94m' + "Group Names: {}".format(self._group_names)
             + '\033[0m')

        rospy.loginfo('\033[94m' + " >>> Ur5Moveit init done." + '\033[0m')

    def go_to_pose(self, arg_pose):

        pose_values = self._group.get_current_pose().pose
        rospy.loginfo('\033[94m' + ">>> Current Pose:" + '\033[0m')
        rospy.loginfo(pose_values)

        self._group.set_pose_target(arg_pose)
        flag_plan = self._group.go(wait=True)  # wait=False for Async Move

        pose_values = self._group.get_current_pose().pose
        rospy.loginfo('\033[94m' + ">>> Final Pose:" + '\033[0m')
        rospy.loginfo(pose_values)

        list_joint_values = self._group.get_current_joint_values()
        rospy.loginfo('\033[94m' + ">>> Final Joint Values:" + '\033[0m')
        rospy.loginfo(list_joint_values)

        if (flag_plan == True):
            rospy.loginfo(
                '\033[94m' + ">>> go_to_pose() Success" + '\033[0m')
        else:
            rospy.logerr(
                '\033[94m' + ">>> go_to_pose() Failed. Solution for Pose not Found."
                 + '\033[0m')

        return flag_plan

    def go_to_predefined_pose(self, arg_pose_name):
        rospy.loginfo('\033[94m' + "Going to Pose: {}".format(arg_pose_name)
         + '\033[0m')
        self._group.set_named_target(arg_pose_name)
        plan = self._group.plan()
        goal = moveit_msgs.msg.ExecuteTrajectoryGoal()
        goal.trajectory = plan
        self._exectute_trajectory_client.send_goal(goal)
        self._exectute_trajectory_client.wait_for_result()
        rospy.loginfo('\033[94m' + "Now at Pose: {}".format(arg_pose_name)
         + '\033[0m')
    

    def wait_for_state_update(self, box_is_known=False, box_is_attached=False,
                              timeout=4):
        box_name = self._box_name
        scene = self._scene
        
        start = rospy.get_time()
        seconds = rospy.get_time()
        while (seconds - start < timeout) and not rospy.is_shutdown():
            # Test if the box is in attached objects
            attached_objects = scene.get_attached_objects([box_name])
            is_attached = len(attached_objects.keys()) > 0

            # Test if the box is in the scene.
            is_known = box_name in scene.get_known_object_names()

            # Test if we are in the expected state
            if (box_is_attached == is_attached) and (box_is_known == is_known):
                return True
           
            rospy.sleep(0.1)
            seconds = rospy.get_time()
        # If we exited the while loop without returning then we timed out
        return False

    def add_box(self, timeout=4):
        box_name = self._box_name
        scene = self._scene
        
        box_pose = geometry_msgs.msg.PoseStamped()
        box_pose.header.frame_id = "world"
        box_pose.pose.position.y = 0.455
        box_pose.pose.position.z = 1.917919 
        box_name = "box"
        scene.add_box(box_name, box_pose, size=(0.15, 0.15, 0.15))
        self._box_name=box_name
        return self.wait_for_state_update(box_is_known=True, timeout=timeout)

    def attach_box(self, timeout=4):
        box_name = self._box_name
        robot = self._robot
        scene = self._scene
        eef_link = self._eef_link
        group_names = self._group_names

        grasping_group = self._planning_group
        touch_links = robot.get_link_names(group=grasping_group)
        scene.attach_box(eef_link, box_name, touch_links=touch_links)
        
        return self.wait_for_state_update(box_is_attached=True,
                                          box_is_known=False, timeout=timeout)

    def detach_box(self, timeout=4):
        box_name = self._box_name
        scene = self._scene
        eef_link = self._eef_link
        return self.wait_for_state_update(box_is_known=True,
               box_is_attached=False, timeout=timeout)


    def remove_box(self, timeout=4):
        box_name = self._box_name
        scene = self._scene

        scene.remove_world_object(box_name)
        return self.wait_for_state_update(box_is_attached=False,
                     box_is_known=False, timeout=timeout)

    # Destructor
    def __del__(self):
        moveit_commander.roscpp_shutdown()
        rospy.loginfo(
            '\033[94m' + "Object of class Ur5Moveit Deleted." + '\033[0m')

    def callback_service_on_request(req):
        if req.activate_vacuum_gripper == True:
            req.activate_vacuum_gripper = False
        else:
            req.activate_vacuum_gripper = False

    def callback_to_activate(req):
        req.activate_vacuum_gripper = True
    
    def callback_to_deactivate(req):
        req.activate_vacuum_gripper = False


# Function to activate or deactivate the gripper
def activate_gripper(status, attach_srv):
    """
    @status: True - Activate vacuum gripper.
             False - Deactivate vacuum gripper.
    """    
    if status:
        resp = attach_srv(activate_vacuum_gripper = True)
        attach_srv.wait_for_service()
    else:
        resp = attach_srv(activate_vacuum_gripper = False)
        attach_srv.wait_for_service()


def main():
    ur5 = Ur5Moveit()
    
    # Create server proxy
    rospy.loginfo("Creating ServiceProxy to /eyrc/vb/ur5_1/activate_vacuum_gripper")
    attach_srv = rospy.ServiceProxy('/eyrc/vb/ur5_1/activate_vacuum_gripper',
                                      vacuumGripper)
    #To reach the box
    ur5_pose_1 = geometry_msgs.msg.Pose()
    ur5_pose_1.position.x = 0.002648
    ur5_pose_1.position.y = 0.25018
    ur5_pose_1.position.z = 1.917855
    ur5_pose_1.orientation.x = 0.000796
    ur5_pose_1.orientation.y = 0.999999
    ur5_pose_1.orientation.z = 0.00024155
    ur5_pose_1.orientation.w = -3.049716

    #Towards destination
    ur5_pose_2 = geometry_msgs.msg.Pose()
    ur5_pose_2.position.x = -0.737039
    ur5_pose_2.position.y = -0.077709
    ur5_pose_2.position.z = 1.177639
    ur5_pose_2.orientation.x = 0.000796
    ur5_pose_2.orientation.y = 0.999999
    ur5_pose_2.orientation.z = 0.00024155
    ur5_pose_2.orientation.w = -3.049716

    ur5.go_to_pose(ur5_pose_1)
    rospy.sleep(1)
    ur5.add_box()
    ur5.attach_box()
    activate_gripper(True, attach_srv)

    ur5.go_to_pose(ur5_pose_2)
    rospy.sleep(1)
    ur5.detach_box()
    ur5.remove_box()
    activate_gripper(False, attach_srv)

    ur5.go_to_predefined_pose("allZeros")
    rospy.sleep(1)

    del ur5

if __name__ == '__main__':
    main()
