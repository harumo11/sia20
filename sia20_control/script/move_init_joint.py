import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
moveit_commander.roscpp_initialize(sys.argv)
rospy.init_node('move_group', anonymous=True)
robot = moveit_commander.RobotCommander()
scene = moveit_commander.PlanningSceneInterface()
group_name = "manipulator"
group = moveit_commander.MoveGroupCommander(group_name)
                                               
# We can get the name of the reference frame for this robot:
planning_frame = group.get_planning_frame()
#print "============ Reference frame: %s" % planning_frame

# We can also print the name of the end-effector link for this group:
eef_link = group.get_end_effector_link()
#print "============ End effector: %s" % eef_link

# We can get a list of all the groups in the robot:
group_names = robot.get_group_names()
#print "============ Robot Groups:", robot.get_group_names()

# Sometimes for debugging it is useful to print the entire state of the
# robot:
#print "============ Printing robot state"
#print robot.get_current_state()
#print ""

joint_goal = [0, 0, 0, 0, 0, 0, 0]
joint_goal[0] = 0.0
joint_goal[1] = -0.529410898685
joint_goal[2] = -1.70442326635e-05
joint_goal[3] = 1.45146977901
joint_goal[4] = 0.0
joint_goal[5] = -0.42356094718
joint_goal[6] = -6.01561150688e-05

group.set_joint_value_target(joint_goal)
rospy.loginfo("Start to move the target : %s", joint_goal)
group.go()
rospy.loginfo("Finish to move")
rospy.sleep(3)
rospy.loginfo("bye bye!")
