#!/usr/bin/env python
#coding=utf-8

from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
import rospy
import math
from geometry_msgs.msg import PointStamped,PoseStamped
import actionlib
from move_base_msgs.msg import *
from actionlib_msgs.msg import *


def position_callback(msg):

    Goalstring = msg.status.goal_id.id
   
def status_callback(msg):

    global goal_pub, index,markerArray
    global add_more_pose,try_again
    print 'log1 index ',index,' count ',count    
    if(msg.status.status == 3):
        try_again = 0
        if add_more_pose == 0:
            print 'Goal reached'

        if index < count:

            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.header.stamp = rospy.Time.now()
            pose.pose.position.x = markerArray.markers[index].pose.position.x
            pose.pose.position.y = markerArray.markers[index].pose.position.y
            pose.pose.orientation.w = markerArray.markers[index].pose.orientation.w
            pose.pose.orientation.z = markerArray.markers[index].pose.orientation.z
            print 'successfully ! next goal: pos.x',pose.pose.position.x,' pos.y ',pose.pose.position.y, ' orien.w ',pose.pose.orientation.w,' orien.z ',pose.pose.orientation.z
            goal_pub.publish(pose)

            index += 1
        elif index == count:
            add_more_pose = 1

    else:
    # uint8 PENDING         = 0   # The goal has yet to be processed by the action server
    # uint8 ACTIVE          = 1   # The goal is currently being processed by the action server
    # uint8 PREEMPTED       = 2   # The goal received a cancel request after it started executing
    #                             #   and has since completed its execution (Terminal State)
    # uint8 SUCCEEDED       = 3   # The goal was achieved successfully by the action server (Terminal State)
    # uint8 ABORTED         = 4   # The goal was aborted during execution by the action server due
    #                             #    to some failure (Terminal State)
    # uint8 REJECTED        = 5   # The goal was rejected by the action server without being processed,
    #                             #    because the goal was unattainable or invalid (Terminal State)
    # uint8 PREEMPTING      = 6   # The goal received a cancel request after it started executing
    #                             #    and has not yet completed execution
    # uint8 RECALLING       = 7   # The goal received a cancel request before it started executing,
    #                             #    but the action server has not yet confirmed that the goal is canceled
    # uint8 RECALLED        = 8   # The goal received a cancel request before it started executing
    #                             #    and was successfully cancelled (Terminal State)
    # uint8 LOST            = 9   # An action client can determine that a goal is LOST. This should not be
    #                             #    sent over the wire by an action server
        
        if msg.status.status == 11 :
            print 'stop : 11' 
            index = len(markerArray.markers)  
            goal_cancel = GoalID()
            goal_cancel.id = Goalstring
            goal_cancel.stamp = rospy.Time.now()
            goal_cancel_pub.publish(goal_cancel)			
            
        elif msg.status.status == 10:
            print 'stop : 10'
            goal_cancel = GoalID()
            goal_cancel.id = Goalstring
            goal_cancel.stamp = rospy.Time.now()
            goal_cancel_pub.publish(goal_cancel)	
             		
        
        else :
          print 'Goal cannot reached has some error :',msg.status.status," try again!!!!"
          if try_again == 1:
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.header.stamp = rospy.Time.now()
            pose.pose.position.x = markerArray.markers[index-1].pose.position.x
            pose.pose.position.y = markerArray.markers[index-1].pose.position.y
            pose.pose.orientation.w = markerArray.markers[index-1].pose.orientation.w
            pose.pose.orientation.z = markerArray.markers[index-1].pose.orientation.z
            goal_pub.publish(pose)
            try_again = 0
          else:
            print 'markers',len(markerArray.markers),'index',index
            if index < len(markerArray.markers):
                pose = PoseStamped()
                pose.header.frame_id = 'map'
                pose.header.stamp = rospy.Time.now()
                pose.pose.position.x = markerArray.markers[index].pose.position.x
                pose.pose.position.y = markerArray.markers[index].pose.position.y
                pose.pose.orientation.w = markerArray.markers[index].pose.orientation.w
                pose.pose.orientation.z = markerArray.markers[index].pose.orientation.z
                print 'fail! next goal:  x ',pose.pose.position.x,' y ',pose.pose.position.y
                goal_pub.publish(pose)
                index += 1
            else : 
			    add_more_pose = 1

def click_callback(msg):
    global markerArray,count,MARKERS_MAX
    global goal_pub,index
    global add_more_pose

    marker = Marker()
    marker.header.frame_id = 'map'
    marker.type = marker.TEXT_VIEW_FACING
    marker.action = marker.ADD
    marker.scale.x = 1
    marker.scale.y = 1
    marker.scale.z = 1
    marker.color.a = 1.0
    marker.color.r = 1.0
    marker.color.g = 0.0
    marker.color.b = 0.0
    marker.pose.orientation.w = msg.pose.orientation.w
    marker.pose.orientation.z = msg.pose.orientation.z
    marker.pose.orientation.x = msg.pose.orientation.x
    marker.pose.orientation.y = msg.pose.orientation.y
    marker.pose.position.x = msg.pose.position.x
    marker.pose.position.y = msg.pose.position.y
    marker.pose.position.z = msg.pose.position.z
    print 'add: pos.x ',marker.pose.position.x,' pos.y ',marker.pose.position.y, ' orien.w ',marker.pose.orientation.w,' orien.z ',marker.pose.orientation.z
    marker.text = str(count)
    # We add the new marker to the MarkerArray, removing the oldest
    # marker from it when necessary
    # if(count > MARKERS_MAX):
    #    markerArray.markers.pop(0)

    markerArray.markers.append(marker)

    # Renumber the marker IDs
    id = 0
    for m in markerArray.markers:
       m.id = id
       id += 1

    # Publish the MarkerArray
    mark_pub.publish(markerArray)
    print 'add_more_pose'
    print  add_more_pose
    #first goal
    if count==0:
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = rospy.Time.now()
        pose.pose.position.x = msg.pose.position.x
        pose.pose.position.y = msg.pose.position.y
        pose.pose.orientation.w = msg.pose.orientation.w
        pose.pose.orientation.z = msg.pose.orientation.z
        goal_pub.publish(pose)
        index += 1

    if add_more_pose and count > 0:
        add_more_pose = 0
        move =MoveBaseActionResult()
        move.status.status = 3
        move.header.stamp = rospy.Time.now()
        goal_status_pub.publish(move)

    count += 1
    print 'add a path goal pose'
    print 'index',index,'count',count
   
markerArray = MarkerArray()
#CurGoal = GoalStatusArray()
CurPose = PoseStamped()
count = 0       #total goal num
index = 0       #current goal pose index
add_more_pose = 0 # after all goal arrive, if add some more goal
try_again = 0  # try the fail goal once again
Goalstring = ""
rospy.init_node('path_pose_demo')

mark_pub = rospy.Publisher('/path_pose', MarkerArray,queue_size=100)
#click_sub = rospy.Subscriber('/clicked_point',PoseStamped,click_callback)
click_sub = rospy.Subscriber('/PoseStamped',PoseStamped,click_callback)
goal_pub = rospy.Publisher('/move_base_simple/goal',PoseStamped,queue_size=1)
goal_status_sub = rospy.Subscriber('/move_base/result',MoveBaseActionResult,status_callback)
position_status_sub = rospy.Subscriber('move_base/feedback',MoveBaseActionFeedback,position_callback)
#goal_id_sub  = rospy.Subscriber('/move_base/status',GoalStatusArray,goalID_callback)
#after all goal arrive, if add some more goal
#we deleberate pub a topic to trig the goal sent
goal_status_pub = rospy.Publisher('/move_base/result',MoveBaseActionResult,queue_size=1)
goal_cancel_pub = rospy.Publisher('move_base/cancel',GoalID,queue_size=1)
rospy.spin()

# sac = actionlib.SimpleActionClient('move_base', MoveBaseAction )
# goal = MoveBaseGoal()
# goal.target_pose.pose.position.x = 0.0
# goal.target_pose.pose.position.y = 0.0
# goal.target_pose.pose.orientation.w = 1.0
# goal.target_pose.header.frame_id = 'map'
# goal.target_pose.header.stamp = rospy.Time.now()
#
# #start listner
# sac.wait_for_server()
#
# #send goal
# sac.send_goal(goal)
# #finish
# sac.wait_for_result()
#
# rate = rospy.Rate(10)
#
# while not rospy.is_shutdown():
#     print sac.get_result()
#     print 'ada'
#     rate.sleep()

