#!/usr/bin/env python

import rospy
import actionlib
from assignment_2_2024.msg import PlanningAction, PlanningGoal, PlanningFeedback
from nav_msgs.msg import Odometry
from assignment2_part1.msg import LastTarget

# Global variables to store the robot's position and velocity
current_x = 0.0
current_y = 0.0
vel_x = 0.0
vel_z = 0.0
cancel_goal_flag = False 
goal_active = False 

def feedback_callback(feedback):
    if feedback.stat == "Target reached!":
        rospy.loginfo("Goal Reached!! Press 'Enter' to continue!!")

# Odometry callback to get robot position and velocity
def odom_callback(msg):
    global current_x, current_y, vel_x, vel_z
    current_x = msg.pose.pose.position.x
    current_y = msg.pose.pose.position.y
    vel_x = msg.twist.twist.linear.x
    vel_z = msg.twist.twist.angular.z

    # Publish custom message with position and velocity
    status_msg = LastTarget()
    status_msg.x = current_x
    status_msg.y = current_y
    status_msg.vel_x = vel_x
    status_msg.vel_z = vel_z

    status_pub.publish(status_msg)

def cancel_goal(ac):
    """Cancel the goal if the cancel flag is set and the robot is moving."""
    global cancel_goal_flag, goal_active
    if cancel_goal_flag and goal_active:
        ac.cancel_goal()
        goal_active = False

# Main function to send goals and monitor feedback
def action_client_node():
    global cancel_goal_flag, goal_active, status_pub

    rospy.init_node('action_client_node', anonymous=True)

    # Create an action client
    ac = actionlib.SimpleActionClient('/reaching_goal', PlanningAction)

    # Wait for the action server to start
    rospy.loginfo("Waiting for action server to start...")
    ac.wait_for_server()
    rospy.loginfo("Action server started!")

    # Subscribe to /odom for robot's position and velocity
    rospy.Subscriber('/odom', Odometry, odom_callback)

    status_pub = rospy.Publisher('robot_status', LastTarget, queue_size=10)

    # Request user input for the target coordinates
    while not rospy.is_shutdown():
        try:
            # Get user input for target coordinates
            target_x = float(input("Enter target x: "))
            target_y = float(input("Enter target y: "))

            goal = PlanningGoal()  # Create a goal instance
            goal.target_pose.pose.position.x = target_x  # Set the target x-coordinate
            goal.target_pose.pose.position.y = target_y  # Set the target y-coordinate
            rospy.loginfo(f"Sending goal: x={target_x}, y={target_y}")
        
            # Send the goal to the action server and listen for feedback
            ac.send_goal(goal, done_cb=None, active_cb=None, feedback_cb=feedback_callback)
            goal_active = True  

            # Monitor the goal state and robot's position while the goal is in progress
            while not rospy.is_shutdown() and ac.get_state() not in [actionlib.GoalStatus.SUCCEEDED, actionlib.GoalStatus.ABORTED, actionlib.GoalStatus.REJECTED]:
                # Check for user input to cancel the goal
                user_input = input("Robot running!! Enter 'cancel' to cancel the current goal: ").strip().lower()
                if user_input == 'cancel':
                    cancel_goal_flag = True 
                    cancel_goal(ac)
                    break

                rospy.sleep(0.5)  # Sleep for 500 ms to avoid spamming messages

            # Check the outcome of the goal
            if ac.get_state() == actionlib.GoalStatus.SUCCEEDED:
                rospy.loginfo("Please Enter new coordinates !!")
            elif cancel_goal_flag:
                rospy.loginfo("Goal was canceled by the user.")
            else:
                rospy.loginfo("Goal did not succeed. Status: %d", ac.get_state())

            cancel_goal_flag = False
            goal_active = False  # Reset goal_active after processing the goal

        except ValueError:
            rospy.logerr("Invalid input! Please enter valid numbers for the target coordinates.")
        
        # After handling the goal or cancelation, return to accepting new coordinates
        cancel_goal_flag = False  # Reset the cancel flag before the next iteration
        rospy.sleep(0.5)  # Sleep for 1 second before allowing another input

if __name__ == "__main__":
    try:
        action_client_node()
    except rospy.ROSInterruptException:
        rospy.loginfo("Action client node terminated.")
