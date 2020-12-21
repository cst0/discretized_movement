#! /usr/bin/env python
import rospy
from discretized_movement.msg import worldstate, worldobject
import sys
import rospy
import moveit_commander
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2, Image
import cv2
from cv_bridge import CvBridge, CvBridgeError
import ros_numpy
import ctypes
import struct
from geometry_msgs.msg import PoseStamped

yellow_point = None
red_point = None
blue_point = None

bridge = CvBridge()
scene = None
robot = None
image_pub = None

def image_callback(msg):
    try:
      cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")
    except CvBridgeError as e:
      rospy.logerr(e)
      return
    (rows,cols,channels) = cv_image.shape
    colors = [[0]*10]*10
    hsvImage = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
    check_range = 10
    yellow_hsv = 24
    red_hsv = 5
    blue_hsv = 208/2
    red_points = []
    blue_points = []
    yellow_points = []
    for x in range(0, 10):
        for y in range(0, 10):
            x_point = 140+(x*45)
            y_point = 215+(y*23)
            colors[y][x] = hsvImage[y_point][x_point][0]
            if -check_range < (hsvImage[y_point][x_point][0] - yellow_hsv) < check_range:
                yellow_points.append((y+1, x+1))
            if -check_range < (hsvImage[y_point][x_point][0] - red_hsv) < check_range:
                red_points.append((y+1, x+1))
            if -check_range < (hsvImage[y_point][x_point][0] - blue_hsv) < check_range:
                blue_points.append((y+1, x+1))

            cv2.circle(
                    cv_image,
                    (x_point, y_point),
                    5,
                    (0,0,255),
                    2)

    try:
        global yellow_point
        global red_point
        global blue_point
        yellow_point = yellow_points[0]
        red_point = red_points[0]
        blue_point = blue_points[0]
        rospy.loginfo_throttle(1, str(yellow_point)+" "+str(red_point)+" "+str(blue_point))
    except:
        pass

    pubimage = bridge.cv2_to_imgmsg(cv_image, "passthrough")
    pubimage.header.frame_id = msg.header.frame_id
    image_pub.publish(bridge.cv2_to_imgmsg(cv_image, "passthrough"))
#    cv2.imwrite("/home/cst/Pictures/cvimg.png", cv_image)
#    cv2.waitKey(3)

def main():
    rospy.init_node('depth_to_worldstate', anonymous=False)
    global image_pub
    image_pub = rospy.Publisher("image_topic", Image, queue_size=5)
    world_pub = rospy.Publisher("world_state_status", worldstate, queue_size=5)
    rospy.Subscriber('/camera/color/image_raw', Image, image_callback)
    world_state_msg = worldstate()

    rospy.loginfo("spinning now...")
    rospy.spin()

    while red_point is None or blue_point is None or yellow_point is None:
        rospy.loginfo_throttle(1, "Haven't seen all items yet...")

    output_str = ""
    output_str += "- {name: agent          , x: "+str(5)+" , y: "+str(5)+"}\n"
    output_str += "- {name: cube1          , x: "+str(red_point[0])+" , y: "+str(red_point[1])+"}\n"
    output_str += "- {name: cube2          , x: "+str(yellow_point[0])+" , y: "+str(yellow_point[1])+"}\n"
    output_str += "- {name: crafting_table , x: "+str(blue_point[0])+" , y: "+str(blue_point[1])+"}"
    with open("./data/data.yaml", "w") as outputfile:
        outputfile.write(output_str)

def move_out_of_vision():
    group_name = "arm"
    group = moveit_commander.MoveGroupCommander(group_name)

    # We can get the joint values from the group and adjust some of the values.
    # Here's a spot out-of-the-way:
    joint_goal = [1.6063205779373169, -0.14542388296539308, -1.593849874534607,
                  1.557513295687256, -1.7560851869842529, -1.5511037020385743, -2.681811961767578]
    # The go command can be called with joint values, poses, or without any
    # parameters if you have already set the pose or joint target for the group
    group.go(joint_goal, wait=True)


if __name__ == "__main__":
    main()
