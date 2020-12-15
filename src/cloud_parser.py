#! /usr/bin/env python

import rospy
import roslib; roslib.load_manifest('laser_assembler')
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2, PointField
from laser_assembler.srv import *
import ctypes
import struct

TF_LISTENER = None
PUBLISHER = None
_X = 0
_Y = 1
_Z = 2

MOST_RECENT_POINTS = dict()


def get_point_rgb(point):
    # cast float32 to int so that bitwise operations are possible
    s = struct.pack('>f', point[3])
    i = struct.unpack('>l', s)[0]
    # you can get back the float value by the inverse operations
    pack = ctypes.c_uint32(i).value
    r = (pack >> 16) & 0x0000ff
    g = (pack >> 8) & 0x0000ff
    b = (pack) & 0x0000ff
    # x,y,z can be retrieved from the x[0],x[1],x[2]
    return r, g, b

def pack_point_rgb(point):
    r, g, b = get_point_rgb(point)
    a = 255
    rgb = struct.unpack('I', struct.pack('BBBB', b, g, r, a))[0]
    return rgb

def point_cloud_callback(msg):
    gen = pc2.read_points(msg, skip_nans=True, field_names=("x", "y", "z", "rgb"))
    data = list(gen)
    table_layer_height = rospy.get_param("table_layer_height")
    obstacle_layer_height = rospy.get_param("obstacle_layer_height")

    points = []
    for point in data:
        if True: #point[_Z] >= table_layer_height and point[_Z] <= obstacle_layer_height:
            points.append([
                point[_X],
                point[_Y],
                point[_Z],
                pack_point_rgb(point)
                ])
    fields = [
        PointField('x', 0, PointField.FLOAT32, 1),
        PointField('y', 4, PointField.FLOAT32, 1),
        PointField('z', 8, PointField.FLOAT32, 1),
        PointField('rgba', 12, PointField.UINT32, 1),
    ]

    newmsg = pc2.create_cloud(msg.header, fields, points)
    MOST_RECENT_POINTS[msg.header.frame_id] = newmsg

if __name__ == "__main__":
    rospy.init_node("cloud_parser", anonymous=False)
    rospy.loginfo("Hello! Starting up...")
    rospy.wait_for_service("assemble_scans2")
    n = 1
    more_inputs = True
    while more_inputs:
        if rospy.has_param("depth_input_"+str(n)):
            rospy.loginfo("attempting to connect to source "+str(rospy.get_param("depth_input_"+str(n))))
            rospy.Subscriber(rospy.get_param("depth_input_"+str(n)), PointCloud2, point_cloud_callback)
            n += 1
        else:
            more_inputs = False

    publisher = rospy.Publisher("cloud", PointCloud2, queue_size=1)
    PUBLISHER = publisher
    rospy.Subscriber("/camera/depth_registered/points", PointCloud2, point_cloud_callback)
#    rospy.Subscriber("/head_camera/depth_registered/points", PointCloud2, point_cloud_callback)
    assembled_publisher = rospy.Publisher("assembled_cloud", PointCloud2, queue_size=10)
    scan_assembler = rospy.ServiceProxy('assemble_scans2', AssembleScans2)

    rospy.loginfo("Ready to go!")

    rate = rospy.Rate(5)
    while not rospy.is_shutdown():
        rate.sleep()
        for item in MOST_RECENT_POINTS.values():
            publisher.publish(item)

#        try:
#            cloud = scan_assembler(rospy.Time(0,0), rospy.get_rostime()).cloud
#            cloud.header.frame_id = "base_link"
#            assembled_publisher.publish(cloud)
#        except rospy.ServiceException as e:
#            rospy.logwarn_throttle(1, "failed: "+str(e))


