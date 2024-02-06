#!/usr/bin/python3.8.10
from __future__ import print_function
from six.moves import input
from sensor_msgs.msg import JointState
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from moveit_msgs.msg import Constraints, PositionConstraint
from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import RobotState
try:
    from math import pi, tau, dist, fabs, cos
except:  # For Python 2 compatibility
    from math import pi, fabs, cos, sqrt

    tau = 2.0 * pi

    def dist(p, q):
        return sqrt(sum((p_i - q_i) ** 2.0 for p_i, q_i in zip(p, q)))


from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list

class MoveitInterface:
    def __init__(self, robot):
        print("hello")
        self.robot=robot
        self.group_name = "arm"
        self.move_group = moveit_commander.MoveGroupCommander(self.group_name)
        
        self.arm_name="gripper"
        self.arm_group = moveit_commander.MoveGroupCommander(self.arm_name)


        # We can get the name of the reference frame for this robot:
        planning_frame = self.move_group.get_planning_frame()
        print("============ Planning frame: %s" % planning_frame)

        # We can also print the name of the end-effector link for this group:
        eef_link = self.move_group.get_end_effector_link()
        print("============ End effector link: %s" % eef_link)

        # We can get a list of all the groups in the robot:
        group_names = self.robot.get_group_names()
        print("============ Available Planning Groups:", self.robot.get_group_names())

        # Sometimes for debugging it is useful to print the entire state of the
        # robot:
        print("============ Printing robot state")
        print(self.robot.get_current_state())

    def close_gripper(self):
        s_msg=JointState()

        joint_goal = self.arm_group.get_current_joint_values()
        
        joint_goal[0] = 0.15
        joint_goal[1] = 0.15
        joint_goal[2] = 0.15
        s_msg.position=joint_goal
        s_msg.velocity=len(joint_goal)*[1]
        print(len(s_msg.position),len(s_msg.velocity))
        # The go command can be called with joint values, poses, or without any
        # parameters if you have already set the pose or joint target for the group
        self.arm_group.set_joint_value_target(joint_goal)
        self.arm_group.set_goal_joint_tolerance(0.05)
        self.arm_group.go()

        # Calling ``stop()`` ensures that there is no residual movement
        self.arm_group.stop()

    def pick(self):
        eef_link = self.move_group.get_end_effector_link()
        print(eef_link)
        print(self.move_group.get_current_joint_values())
        self.move_group.set_pose_target(self.move_group.get_current_joint_values(),"Close",eef_link)
        self.move_group.go()

        # Calling ``stop()`` ensures that there is no residual movement
        self.move_group.stop()


    def pose_goal(self,w,x,y,z):
        
        move_group = self.move_group

        pose_goal = geometry_msgs.msg.Pose()
        pose_goal.orientation.w = w
        pose_goal.position.x = x
        pose_goal.position.y = y
        pose_goal.position.z = z

        move_group.set_pose_target(pose_goal)

        ## Now, we call the planner to compute the plan and execute it.
        # `go()` returns a boolean indicating whether the planning and execution was successful.
        success = move_group.go(wait=True)
        # Calling `stop()` ensures that there is no residual movement
        move_group.stop()
        # It is always good to clear your targets after planning with poses.
        # Note: there is no equivalent function for clear_joint_value_targets().
        move_group.clear_pose_targets()

        ## END_SUB_TUTORIAL

        
if __name__ == "__main__":
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node("move_group_python_interface_tutorial", anonymous=True)
    robot = moveit_commander.RobotCommander()
    mv=MoveitInterface(robot)
    pose_x=[0.0,0,-0.7,-0.5,0,0.5,0.7]
    pose_y=[0.0,0.4,0.4,0.4,0.4,0.4,0.4]
    pose_z=[0.7,0.5,0.3,0.3,0.3,0.3,0.3]
    pose_w=7*[0.0]
    for i in range(2,7):
        print("going to home \n")
        mv.pose_goal(pose_w[0],pose_x[0],pose_y[0],pose_z[0])
        print("going to pick the object "+str(i-1)+"\n")
        mv.pose_goal(pose_w[i],pose_x[i],pose_y[i],pose_z[i])
        print("going to pose\n")
        mv.pose_goal(pose_w[1],pose_x[1],pose_y[1],pose_z[1])
    
    