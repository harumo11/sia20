import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from std_srvs.srv import Trigger
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
rospy.loginfo("Servo on")

rospy.wait_for_service("robot_enable")
service_client = rospy.ServiceProxy("robot_enable", Trigger)
service_client.call()

joint_goal = [0, 0, 0, 0, 0, 0, 0]
# short broom 
#joint_goal[0] = -0.3186
#joint_goal[1] =  0.2539
#joint_goal[2] =  0.0008
#joint_goal[3] = -1.6698
#joint_goal[4] =  0.77011
#joint_goal[5] =  0.51063
#joint_goal[6] = -0.9972
# long broom
joint_goal[0] = -0.324965327978
joint_goal[1] = 0.226790547371
joint_goal[2] = -8.52211596793e-05
joint_goal[3] = -1.0992847681
joint_goal[4] = -1.0992847681
joint_goal[5] = -0.349322348833
joint_goal[6] = 0.958707988262

group.set_joint_value_target(joint_goal)
rospy.loginfo("Start to move the target : %s", joint_goal)
group.go()
rospy.loginfo("Finish to move")
rospy.sleep(3)
rospy.loginfo("bye bye!")
