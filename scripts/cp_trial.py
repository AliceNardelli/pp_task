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
    def __init__(self, move_group):
        print("hello")
        self.move_group=move_group
        # We can get the name of the reference frame for this robot:
        planning_frame = move_group.get_planning_frame()
        print("============ Planning frame: %s" % planning_frame)

        # We can also print the name of the end-effector link for this group:
        eef_link = move_group.get_end_effector_link()
        print("============ End effector link: %s" % eef_link)

        # We can get a list of all the groups in the robot:
        group_names = robot.get_group_names()
        print("============ Available Planning Groups:", robot.get_group_names())

        # Sometimes for debugging it is useful to print the entire state of the
        # robot:
        print("============ Printing robot state")
        print(robot.get_current_state())

        print(self.move_group.get_current_pose())
        print(self.move_group.get_current_joint_values())
        print("THEREEEE")
    
    def joint_goal(self):
        # We get the joint values from the group and change some of the values:
        s_msg=JointState()

        joint_goal = self.move_group.get_current_joint_values()
        print(joint_goal)
        joint_goal[0] = joint_goal[6]+1
        
        s_msg.position=joint_goal
        s_msg.velocity=len(joint_goal)*[1]
        print(len(s_msg.position),len(s_msg.velocity))
        # The go command can be called with joint values, poses, or without any
        # parameters if you have already set the pose or joint target for the group
        self.move_group.go(s_msg, wait=True)

        # Calling ``stop()`` ensures that there is no residual movement
        self.move_group.stop()
    
    def pose_goal(self):
        pose_goal = geometry_msgs.msg.Pose()
        pose_goal.orientation.w = 1.0
        pose_goal.position.x = 0.4
        pose_goal.position.y = 0.1
        pose_goal.position.z = 1

        self.move_group.set_pose_target(pose_goal)
        self.move_group.set_max_velocity_scaling_factor(1)
        self.move_group.set_max_acceleration_scaling_factor(1)
        # `go()` returns a boolean indicating whether the planning and execution was successful.
        success = self.move_group.go(wait=True)
        print(success)
        # Calling `stop()` ensures that there is no residual movement
        self.move_group.stop()
        # It is always good to clear your targets after planning with poses.
        # Note: there is no equivalent function for clear_joint_value_targets().
        self.move_group.clear_pose_targets()

    def constraint(self):
        
        # Set the planner to use RRTConnect
        self.move_group.set_planner_id("RRTConnect")

        # Define the plane on which you want to constrain the end effector
        plane_origin = PoseStamped()
        planning_frame = self.move_group.get_planning_frame()
        eef_link = self.move_group.get_end_effector_link()
        plane_origin.header.frame_id = planning_frame
        plane_origin.pose.position.x = 0.5
        plane_origin.pose.position.y = 0.0
        plane_origin.pose.position.z = 0.7

        # Define the normal vector of the plane
        plane_normal = PoseStamped()
        plane_normal.header.frame_id = planning_frame
        plane_normal.pose.orientation.w = 1.0  # No rotation, representing a normal vector along the z-axis

        # Define the position constraint
        c=Constraints()
        position_constraint = PositionConstraint()
        position_constraint.header.frame_id = planning_frame
        position_constraint.link_name = eef_link
        position_constraint.target_point_offset.x = 0.02
        position_constraint.target_point_offset.y = 0.02
        position_constraint.target_point_offset.z = 0.02
        position_constraint.weight = 1.0
        position_constraint.constraint_region.primitive_poses.append(plane_origin.pose)
        position_constraint.constraint_region.primitive_poses.append(plane_normal.pose)
        c.position_constraints=position_constraint
        # Set the constraints in the MoveGroupCommander
        self.move_group.set_path_constraints(c)

        # Set a target pose
        target_pose = PoseStamped()
        target_pose.header.frame_id = planning_frame
        target_pose.pose.position.x = 0.7
        target_pose.pose.position.y = 0.0
        target_pose.pose.position.z = 0.7
        self.move_group.set_pose_target(target_pose)

        # Plan and execute
        plan = self.move_group.plan()
        if plan:
            self.move_group.execute(plan)
        else:
            rospy.loginfo("Planning failed")

        # Clear constraints
        self.move_group.clear_path_constraints()

        

if __name__ == "__main__":
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node("move_group_python_interface_tutorial", anonymous=True)
    robot = moveit_commander.RobotCommander()
    group_name = "arm"
    move_group = moveit_commander.MoveGroupCommander(group_name)
    #publish trajectory on rviz
    
    mv=MoveitInterface(move_group)
    mv.constraint()