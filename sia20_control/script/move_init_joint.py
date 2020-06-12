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

rospy.loginfo("Those joints configuration is for harumo's broom puresure test")

joint_goal = [0, 0, 0, 0, 0, 0, 0]
# short brush 70deg
joint_goal[0] = 0.000528371194378 
joint_goal[1] = -1.29529345036
joint_goal[2] = 0.000221575013711
joint_goal[3] = 1.43981146812
joint_goal[4] = 3.03758570226e-05
joint_goal[5] = -1.51197338104 
joint_goal[6] = 0.0015039028367
# short broom 70deg
#joint_goal[0] = 0.000681769277435 
#joint_goal[1] = -0.992639005184
#joint_goal[2] = 6.81769306539e-05
#joint_goal[3] = 1.47650766373
#joint_goal[4] = -0.000592329190113
#joint_goal[5] = -0.532534301281 
#joint_goal[6] = 0.00156405894086

group.set_joint_value_target(joint_goal)
rospy.loginfo("Start to move the target : %s", joint_goal)
group.go()
rospy.loginfo("Finish to move")
rospy.sleep(3)
rospy.loginfo("bye bye!")
