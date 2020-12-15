#!/usr/bin/env python
'''
make planning scene services available so the moveit stuff we aren't
using can start properly
'''

from moveit_msgs.srv import GetPlanningScene, GetPlanningSceneResponse
from moveit_msgs.srv import ApplyPlanningScene, ApplyPlanningSceneResponse
import rospy

def handle_get_planning_scene(req):
    return GetPlanningSceneResponse()

def handle_apply_planning_scene(req):
    return ApplyPlanningSceneResponse()

def add_two_ints_server():
    rospy.init_node('get_planning_scene_spoofer')
    s = rospy.Service('/get_planning_scene', GetPlanningScene, handle_get_planning_scene)
    s = rospy.Service('/apply_planning_scene', ApplyPlanningScene, handle_apply_planning_scene)
    rospy.spin()

if __name__ == "__main__":
    add_two_ints_server()
