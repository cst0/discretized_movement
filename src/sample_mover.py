#!/usr/bin/env python3
import rospy
import sys
import actionlib
import discretized_movement.msg

class _Getch:
    """Gets a single character from standard input.  Does not echo to the
screen."""
    def __init__(self):
        self.impl = _GetchUnix()

    def __call__(self): return self.impl()


class _GetchUnix:
    def __init__(self):
        import tty, sys

    def __call__(self):
        import sys, tty, termios
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch

def movement_client():
    client = actionlib.SimpleActionClient('simplified_kinematics', discretized_movement.msg.MoveAction)
    client.wait_for_server()

    rospy.loginfo("client is ready: use WASD to control positioning, or Q to quit.")

    keep_running = True
    while keep_running:
        goal = discretized_movement.msg.MoveGoal()
        getch = _Getch()
        ch = getch()
        if ch == 'w':
            goal.move.direction = goal.move.UP
            print("\u2191", end=" ")
        if ch == 'a':
            goal.move.direction = goal.move.LEFT
            print("\u2190", end=" ")
        if ch == 's':
            goal.move.direction = goal.move.DOWN
            print("\u2193", end=" ")
        if ch == 'd':
            goal.move.direction = goal.move.RIGHT
            print("\u2192", end=" ")
        if ch == 'q':
            keep_running = False
            break;

        client.send_goal_and_wait(goal)
        print(client.get_result())


def main():
    try:
        rospy.init_node('sample_mover')
        movement_client()
    except rospy.ROSInterruptException:
        sys.exit(0)


if __name__ == "__main__":
    main()
