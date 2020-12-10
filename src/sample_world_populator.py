#!/usr/bin/env python
import rospy
import yaml
import argparse
import sys
from discretized_movement.msg import worldstate, worldobject


def main():
    # arg parse and other setup
    parser = argparse.ArgumentParser()
    parser.add_argument('file_path', type=str, help='path to input file')
    clean_argv = rospy.myargv(argv=sys.argv)[1:]
    args = parser.parse_args(clean_argv)

    # read yaml as input
    inputfile = open(args.file_path)
    inputyaml = yaml.load(inputfile)

    # assign values
    newmsg = worldstate()
    for single_input in inputyaml:
        newobj = worldobject()
        newobj.x= single_input['x']
        newobj.y= single_input['y']
        newobj.name = single_input['name']
        newmsg.observed_objects.append(newobj)

    # publish and finish
    rospy.init_node('world_state_publisher', anonymous=False)
    publisher = rospy.Publisher('/world_state_status', worldstate, queue_size=1, latch=True)


    rate = rospy.Rate(1)  # node persists for 2 seconds
    while not rospy.is_shutdown():
        rate.sleep()
        publisher.publish(newmsg)
        break;


if __name__ == "__main__":
    main()
