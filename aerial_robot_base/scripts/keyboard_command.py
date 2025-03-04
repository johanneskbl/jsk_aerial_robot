#!/usr/bin/env python
import sys, select, termios, tty
import socket

import rospy
import rosgraph
from std_msgs.msg import Empty
from std_msgs.msg import Int8
from rostopic import ROSTopicIOException

# Function to get the key value from keyboard
def getKey():
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

if __name__=="__main__":
    settings = termios.tcgetattr(sys.stdin)
    rospy.init_node("keyboard_command")
    robot_ns = rospy.get_param("~robot_ns", "")

    if not robot_ns:
        master = rosgraph.Master('/rostopic')
        try:
            _, subs, _ = master.getSystemState()

        except socket.error:
            raise ROSTopicIOException("Unable to communicate with master!")

        teleop_topics = [topic[0] for topic in subs if 'teleop_command/start' in topic[0]]
        if len(teleop_topics) == 1:
            robot_ns = teleop_topics[0].split('/teleop')[0]

    # Define actions for the robot
    ns = robot_ns + "/teleop_command"   # General namespace for teleoperation commands
    start_pub = rospy.Publisher(ns + '/start', Empty, queue_size=1)
    takeoff_pub = rospy.Publisher(ns + '/takeoff', Empty, queue_size=1)
    land_pub = rospy.Publisher(ns + '/land', Empty, queue_size=1)
    halt_pub = rospy.Publisher(ns + '/halt', Empty, queue_size=1)
    force_landing_pub = rospy.Publisher(ns + '/force_landing', Empty, queue_size=1)
    ctrl_mode_pub = rospy.Publisher(ns + '/ctrl_mode', Int8, queue_size=1)
    motion_start_pub = rospy.Publisher('task_start', Empty, queue_size=1)

    # Define key values and publish the corresponding action
    comm=Int8()
    try:
        while(True):
            key = getKey()
            print("the key value is {}".format(ord(key)))
            if key == 'r':
                start_pub.publish(Empty())  # Arm the robot
            if key == 't':
                takeoff_pub.publish(Empty())
            if key == 'l':
                land_pub.publish(Empty())
            if key == 'h':
                halt_pub.publish(Empty())
            if key == 'f':
                force_landing_pub.publish(Empty())
            if key == 'x':
                motion_start_pub.publish()
            if key == 'p':
                # x/y position control mode
                # Set current pos as target
                comm.data = 0
                ctrl_mode_pub.publish(comm)
            if key == 'v':
                # x/y veloctiy control mode
                # Set current vel = 0 as target
                comm.data = 1
                ctrl_mode_pub.publish(comm)
            if key == 'a':
                # x/y acceleration control mode
                # Set current acc = 0 as target
                comm.data = 2
                ctrl_mode_pub.publish(comm)
            if key == '\x03':
                break
            rospy.sleep(0.001)

    except Exception as e:
        print(repr(e))
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)


