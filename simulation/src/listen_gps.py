#!/usr/bin/env python3
import numpy as np
import rospy
from nav_msgs.msg import Odometry
import matplotlib.pyplot as plt
import time
poses = []

def get_headings(msg:Odometry):
    x = msg.pose.pose.position.x
    y = msg.pose.pose.position.y
    yaw=msg.twist.twist.angular.z
    print('listen yaw', yaw)
    poses.append([x,y,yaw])

if __name__ == '__main__':
    rospy.init_node('listen_gps', anonymous=True)

    rospy.Subscriber('car/gps',Odometry, get_headings)

    rate = rospy.Rate(10)
    time.sleep(79)

    headings_np = np.array(poses)
    np.savetxt('poses3.txt', headings_np)

    plt.plot(headings_np[:, 0] ,headings_np[:, 1] , 'r-')
    plt.show()