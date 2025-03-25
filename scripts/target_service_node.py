#!/usr/bin/env python

"""
.. module:: target_service_node
   :platform: ROS
   :synopsis: ROS service node that tracks and provides the last target coordinates

.. moduleauthor:: Sarvenaz Ashoori

This ROS node maintains the last received target coordinates and provides them via a service.

Subscribes to:
    - /reaching_goal/goal (assignment_2_2024/PlanningActionGoal): Receives new target goals

Services:
    - /last_target (assignment2_part1/GetLastTarget): Service to query the last received target coordinates

Dependencies:
    - assignment_2_2024 package
    - assignment2_part1 package
"""

import rospy
from assignment2_part1.srv import GetLastTarget, GetLastTargetResponse
from assignment_2_2024.msg import PlanningActionGoal 

# Global variables to store last target coordinates
last_target_x = 0.0
last_target_y = 0.0

def planningCallback(msg):
    """Callback function that updates the last known target coordinates.
    
    Args:
        msg (PlanningActionGoal): Contains the goal message with target pose information.
        
    Updates:
        Modifies the global variables last_target_x and last_target_y with new coordinates.
    """
    global last_target_x, last_target_y
    last_target_x = msg.goal.target_pose.pose.position.x
    last_target_y = msg.goal.target_pose.pose.position.y
    rospy.logdebug(f"Updated target coordinates: x={last_target_x}, y={last_target_y}")

def handleTargetRequest(req):
    """Service handler that returns the last stored target coordinates.
    
    Args:
        req (GetLastTargetRequest): Empty service request (no parameters needed)
        
    Returns:
        GetLastTargetResponse: Contains the last known x and y target coordinates
    """
    global last_target_x, last_target_y
    response = GetLastTargetResponse()
    response.x = last_target_x
    response.y = last_target_y
    rospy.loginfo(f"Service called - Returning target: x={response.x}, y={response.y}")
    return response

def targetServiceNode():
    """Main function that initializes and runs the target service node.
    
    Initializes the ROS node, sets up the subscriber for goal updates,
    and advertises the last_target service.
    """
    rospy.init_node('target_service_node', log_level=rospy.INFO)
    
    # Subscriber for goal updates
    rospy.Subscriber('/reaching_goal/goal', PlanningActionGoal, planningCallback)
    
    # Service provider
    rospy.Service('last_target', GetLastTarget, handleTargetRequest)
    
    rospy.loginfo("Target coordinate service is ready and waiting for requests...")
    rospy.spin()

if __name__ == "__main__":
    try:
        targetServiceNode()
    except rospy.ROSInterruptException:
        rospy.logwarn("Service node shutdown requested.")