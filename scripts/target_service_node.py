#!/usr/bin/env python

import rospy
from assignment2_part1.srv import GetLastTarget, GetLastTargetResponse
from assignment_2_2024.msg import PlanningActionGoal 
from geometry_msgs.msg import PoseStamped

# Global variables to store last target coordinates
last_target_x = 0.0
last_target_y = 0.0

def planningCallback(msg):
    """Callback function to update the last target coordinates from the PlanningActionGoal."""
    global last_target_x, last_target_y
    # Access the target pose from the goal and update the coordinates
    last_target_x = msg.goal.target_pose.pose.position.x
    last_target_y = msg.goal.target_pose.pose.position.y

def handleTargetRequest(req):
    """Callback function to handle service requests and return the last target."""
    global last_target_x, last_target_y
    res = GetLastTargetResponse()
    res.x = last_target_x
    res.y = last_target_y
    return res

def targetServiceNode():
    rospy.init_node('target_service_node', anonymous=True)

    # Subscribe to the topic
    rospy.Subscriber('/reaching_goal/goal', PlanningActionGoal, planningCallback)

    # Advertise the service
    rospy.Service('last_target', GetLastTarget, handleTargetRequest)

    rospy.loginfo("Target service is ready.")
    rospy.spin()

if __name__ == "__main__":
    try:
        targetServiceNode()
    except rospy.ROSInterruptException:
        rospy.loginfo("Target service node terminated.")
