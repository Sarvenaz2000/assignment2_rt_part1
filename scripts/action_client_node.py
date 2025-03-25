#!/usr/bin/env python

"""
.. module:: action_client_node
   :platform: ROS
   :synopsis: ROS action client node for robot navigation control

.. moduleauthor:: Sarvenaz Ashoori

This node provides interactive control for sending navigation goals to a robot via actionlib.
It continuously monitors and publishes the robot's current position and velocity,
and allows real-time goal cancellation through user input.

Subscribes to:
    - /odom (nav_msgs/Odometry): Receives the robot's position and velocity updates

Publishes to:
    - /robot_status (assignment2_part1/LastTarget): Publishes the robot's current state

Action Clients:
    - /reaching_goal (assignment_2_2024/PlanningAction): Action interface for goal management

Dependencies:
    - assignment_2_2024 package
    - assignment2_part1 package
"""

import rospy
import actionlib
from assignment_2_2024.msg import PlanningAction, PlanningGoal
from nav_msgs.msg import Odometry
from assignment2_part1.msg import LastTarget
from actionlib_msgs.msg import GoalStatus

# Global state variables
current_x = 0.0  #: Current x position of the robot (meters)
current_y = 0.0  #: Current y position of the robot (meters)
vel_x = 0.0      #: Current linear velocity in x direction (m/s)
vel_z = 0.0      #: Current angular velocity around z-axis (rad/s)
goal_active = False  #: Flag indicating if a goal is currently active
status_pub = None    #: Publisher for robot status messages

def feedback_callback(feedback):
    """Processes feedback from the action server.
    
    Args:
        feedback (PlanningFeedback): Contains status messages about goal progress.
        
    Note:
        Logs a message when the target is reached and provides user instructions.
    """
    if feedback.stat == "Target reached!":
        rospy.loginfo("Goal successfully reached! Press 'Enter' to continue...")

def odom_callback(msg):
    """Updates and publishes the robot's current state from odometry data.
    
    Args:
        msg (Odometry): Contains the robot's latest pose and twist information.
        
    Publishes:
        Updates the /robot_status topic with position and velocity data.
    """
    global current_x, current_y, vel_x, vel_z
    
    # Update global position and velocity variables
    current_x = msg.pose.pose.position.x
    current_y = msg.pose.pose.position.y
    vel_x = msg.twist.twist.linear.x
    vel_z = msg.twist.twist.angular.z
    
    # Publish status message
    status_msg = LastTarget()
    status_msg.x = current_x
    status_msg.y = current_y
    status_msg.vel_x = vel_x
    status_msg.vel_z = vel_z
    status_pub.publish(status_msg)

def cancel_current_goal(ac):
    """Cancels the active goal if requested by the user.
    
    Args:
        ac (SimpleActionClient): The active action client instance.
        
    Returns:
        bool: True if cancellation was successful, False otherwise.
    """
    global goal_active
    if goal_active:
        ac.cancel_goal()
        goal_active = False
        rospy.loginfo("Goal cancellation requested.")
        return True
    return False

def get_user_coordinates():
    """Handles user input for target coordinates with validation.
    
    Returns:
        tuple: (x, y) coordinates if valid, None if input is invalid.
    """
    try:
        target_x = float(input("Enter target x coordinate: "))
        target_y = float(input("Enter target y coordinate: "))
        return target_x, target_y
    except ValueError:
        rospy.logwarn("Invalid input! Please enter numeric values only.")
        return None

def action_client_node():
    """Main node function managing the action client lifecycle.
    
    Initializes ROS components, handles user interaction, and manages goal states.
    """
    global goal_active, status_pub
    
    rospy.init_node('action_client_node')
    
    # Initialize action client
    ac = actionlib.SimpleActionClient('/reaching_goal', PlanningAction)
    rospy.loginfo("Connecting to action server...")
    ac.wait_for_server()
    rospy.loginfo("Action server connected!")
    
    # Setup ROS communications
    rospy.Subscriber('/odom', Odometry, odom_callback)
    status_pub = rospy.Publisher('/robot_status', LastTarget, queue_size=10)
    
    # Main interaction loop
    while not rospy.is_shutdown():
        coords = get_user_coordinates()
        if coords is None:
            continue
            
        target_x, target_y = coords
        
        # Create and send new goal
        goal = PlanningGoal()
        goal.target_pose.pose.position.x = target_x
        goal.target_pose.pose.position.y = target_y
        rospy.loginfo(f"Sending new goal: ({target_x}, {target_y})")
        
        ac.send_goal(goal, feedback_cb=feedback_callback)
        goal_active = True
        
        # Goal monitoring loop
        while not rospy.is_shutdown() and goal_active:
            state = ac.get_state()
            
            # Check for goal completion
            if state in [GoalStatus.SUCCEEDED, GoalStatus.ABORTED, GoalStatus.REJECTED]:
                goal_active = False
                break
                
            # Check for cancellation request
            user_input = input("Enter 'cancel' to abort current goal or press Enter to continue: ").strip().lower()
            if user_input == 'cancel':
                cancel_current_goal(ac)
                break
                
            rospy.sleep(0.1)  # Prevent CPU overload

if __name__ == "__main__":
    try:
        action_client_node()
    except rospy.ROSInterruptException:
        rospy.loginfo("Node shutdown requested.")