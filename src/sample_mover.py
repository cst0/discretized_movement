#!/usr/bin/env python3
import rospy
import sys
import actionlib
import discretized_movement.msg

class _Getch:
    """
    Gets a single character from standard input.  Does not echo to the screen.
    """
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
    kinematics_client = actionlib.SimpleActionClient('simplified_kinematics', discretized_movement.msg.MoveAction)
    kinematics_client.wait_for_server()
    interation_client = actionlib.SimpleActionClient('simplified_interaction', discretized_movement.msg.InteractAction)
    interation_client.wait_for_server()


    rospy.loginfo("client is ready: use WASD to control positioning, o to grab, p to release, any other key to quit.")

    keep_running = True
    while keep_running:
        move_goal = discretized_movement.msg.MoveGoal()
        interact_goal = discretized_movement.msg.InteractGoal()
        move_goal_set = False
        interact_goal_set = False
        getch = _Getch()
        ch = getch()
        if ch == 'w':
            move_goal.move.direction = move_goal.move.UP
            print("\u2191", end=" ")
            move_goal_set = True
        elif ch == 'a':
            move_goal.move.direction = move_goal.move.LEFT
            print("\u2190", end=" ")
            move_goal_set = True
        elif ch == 's':
            move_goal.move.direction = move_goal.move.DOWN
            print("\u2193", end=" ")
            move_goal_set = True
        elif ch == 'd':
            move_goal.move.direction = move_goal.move.RIGHT
            print("\u2192", end=" ")
            move_goal_set = True
        elif ch == 'o':
            interact_goal.action.interact = interact_goal.action.GRAB
            print("grab", end=" ")
            interact_goal_set = True
        elif ch == 'p':
            interact_goal.action.interact = interact_goal.action.RELEASE
            print("release", end=" ")
            interact_goal_set = True
        else:
            keep_running = False
            print("bye!")

        if move_goal_set:
            kinematics_client.send_goal_and_wait(move_goal)
            print(kinematics_client.get_result())
        if interact_goal_set:
            interation_client.send_goal_and_wait(interact_goal)
            print(interation_client.get_result())


def main():
    try:
        rospy.init_node('sample_mover')
        movement_client()
    except rospy.ROSInterruptException:
        sys.exit(0)


if __name__ == "__main__":
    main()
