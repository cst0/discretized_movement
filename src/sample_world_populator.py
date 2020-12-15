#!/usr/bin/env python
import rospy
import yaml
import argparse
import sys
from discretized_movement.msg import worldstate, worldobject


def main():
    # arg parse and other setup
    parser = argparse.ArgumentParser()
    parser.add_argument('file_path', type=str, help='path to input file', default="/home/cst/ws_rlproject/src/discretized_movement/src/data/data.yaml")
    clean_argv = rospy.myargv(argv=sys.argv)[1:]
    args = parser.parse_args(clean_argv)

    # read yaml as input
    inputfile = open(args.file_path)
    inputyaml = yaml.load(inputfile)

    # assign values
    newmsg = worldstate()
    for single_input in inputyaml:
        if single_input['name'] == 'agent':
            newmsg.robot_state.x = single_input['x']
            newmsg.robot_state.y = single_input['y']
        else:
            newobj = worldobject()
            newobj.x= single_input['x']
            newobj.y= single_input['y']
            newobj.name = single_input['name']
            newmsg.observed_objects.append(newobj)

    # publish and finish
    rospy.init_node('world_state_publisher', anonymous=False)
    publisher = rospy.Publisher('/world_state_status', worldstate, queue_size=1, latch=True)


    rate = rospy.Rate(.2)
    while not rospy.is_shutdown():
        rate.sleep()
        publisher.publish(newmsg)
        if publisher.get_num_connections() > 0:
            rospy.loginfo("Looks like something's received that message. All done! Exiting node.")
            break
        rospy.loginfo_throttle(2, "Nothing's received this message yet, will keep trying...")


if __name__ == "__main__":
    main()
