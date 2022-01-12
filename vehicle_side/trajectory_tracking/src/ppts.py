#!/usr/bin/env python3
'''
修改为右手系. 0111
PIX增加刹车判定
'''
# import matplotlib.pyplot as plt
import numpy as np
import rospy
from rospy.core import rospywarn
from std_msgs.msg import Int16MultiArray, Int32
from nav_msgs.msg import Odometry
import math
import time
import sys
import math
from geometry_msgs.msg import Point
from math import sin,cos
from trajectory_tracking.msg import RoadPoint
from trajectory_tracking.msg import Trajectory


previewPoint = Point(0, 0, 0)
# 配置参数：GPS安装偏移量
bias = [0, 0.4]

def read_file(roadMapPath):
    
    local_trajectory = Trajectory()
    road_type=[]

    with open(roadMapPath, "r") as f:
        for line in f.readlines():
                line = line.strip()
                line = line.split()
                roadpoints = RoadPoint()
                # rospy.logwarn(line)
                roadpoints.x = float(line[1])
                roadpoints.y = float(line[2])
                roadpoints.v = float(line[6])
                id = int(line[4])
                road_type.append(id)
                local_trajectory.roadpoints.append(roadpoints)

    return local_trajectory, road_type

class PurePursuit():
    def __init__(self, hostname):
        self.rate = rospy.Rate(60)
        self.error_ending = True

        self.is_gps_ready = False
        self.is_local_trajectory_ready = False
        self.host = int(hostname)
        # 停车功能
        self.road_type = []
        self.stop_place = None
        self.stop_time = -5
        
        dt_build_up = 4 
        self.stop_time_queue_dict = {
            1: [4.25, 18+4.54 + 1 * dt_build_up],
            3: [ 3.29, 18+4.39 + 4 * dt_build_up],
            2: [5.58, 18+4.16],
            4: [6.71, 18+3.75 + 3 * dt_build_up],
            5: [7.58, 18+4.56 + 2 * dt_build_up]
        }

        self.stop_time_queue = self.stop_time_queue_dict[self.host]
        self.road_type_list = []
        self.gps_msg = Odometry()   
        self.local_traj = Trajectory()
        self.get_local_trajectory()
        self.posture = np.zeros(3)  # x,y,yaw。x+正东,y+正北,yaw逆时针为正，正东为零，角度制
        self.vel = np.zeros(2)  
        self.stage = 0
        
        self.preview_point = Point()

        # 输入 gnss, 规划的局部避撞轨迹
        rospy.Subscriber('/car'+'/gps', Odometry, self.compute_cmd)
        # rospy.Subscriber('/stage_cmd', Int32, self.get_stage_cmd)
        rospy.Subscriber('/local_trajectory', Trajectory, self.get_stage_cmd)

        # 输出控制指令
        self.pub_cmd = rospy.Publisher('control_cmd', Int16MultiArray, queue_size=1)

        self.pub_preview_point = rospy.Publisher('purepusuit/preview_point', Point, queue_size=1)

        #  control_cmd initialize
        self.cmd = Int16MultiArray()
        self.cmd.data = np.zeros([13], dtype=int).tolist()
        self.cmd.data[3], self.cmd.data[8] = 1, 1
        self.cmd.data[11] = 1       # 4 wheel turning
        self.rate = rospy.Rate(60)

    def run(self):

        while not rospy.is_shutdown():

            if self.is_local_trajectory_ready and self.is_gps_ready:
                # pix 速度为0时候使用刹车达到制动
                if abs(self.cmd.data[0]) < 1:     
                    self.cmd.data[2] = 1024
                else:
                    self.cmd.data[2] = 0
                self.pub_cmd.publish(self.cmd)
                self.pub_preview_point.publish(self.preview_point)
            self.rate.sleep()

    def get_stage_cmd(self, msg):

        self.stage = int(msg.roadpoints[0].kappa)

    def compute_cmd(self, msg):

        #switch case 
        if self.stage == 0 :
            self.cmd.data[0] = 0
            self.cmd.data[1] = 0
            rospy.logwarn('Stage cmd is not received yet!')
            return
        elif self.stage > 0 :
            rospy.logwarn(self.stage)
            self.local_traj = self.trajectory_list[self.stage -1]
            self.road_type = self.road_type_list[self.stage -1]

        self.is_gps_ready = True

        # 赋值
        self.gps_msg = msg
        self.posture[0] = (self.gps_msg.pose.pose.position.x)
        self.posture[1] = (self.gps_msg.pose.pose.position.y)

        self.posture[2] = (self.gps_msg.twist.twist.angular.z)
        yaw = self.posture[2] / 180 * np.pi

        new_bias_x = bias[0] * sin(yaw) + bias[1 ]* cos(yaw)
        new_bias_y= -bias[0] * cos(yaw) + bias[1] * sin(yaw)
        self.posture[0] = self.posture[0] + new_bias_x
        self.posture[1] = self.posture[1] + new_bias_y

        self.vel[0] = (self.gps_msg.twist.twist.linear.x)
        self.vel[1] = (self.gps_msg.twist.twist.linear.y)

        if not self.is_local_trajectory_ready:
            rospy.logwarn('waiting for local trajectory')
            return

        n_roadpoint = len(self.local_traj.roadpoints)
        if n_roadpoint <= 0:
            # stop for None trajectory:
            self.cmd.data[0] = 0
            return

        # local_traj_xy
        local_traj_xy = np.zeros([n_roadpoint, 2])
        for i in range(n_roadpoint):
            local_traj_xy[i, 0] = self.local_traj.roadpoints[i].x
            local_traj_xy[i, 1] = self.local_traj.roadpoints[i].y

        # find the current waypoint according to distance.
        id_current, distance_current = self.get_current_roadpoint(local_traj_xy, self.posture)

        position_current = local_traj_xy[id_current,:]
        stop_duration = time.time() - self.stop_time

        if self.road_type[id_current] == 1:
            if self.stop_place is not None:
                pre_stop_distance = np.linalg.norm(position_current - self.stop_place, axis=0)
                if pre_stop_distance > 1 and stop_duration  > self.stop_time_queue[0]:
                    rospy.logwarn('next stop place')
                    self.cmd.data[0] = 0
                    self.cmd.data[1] = 0
                    self.stop_time = time.time()
                    self.stop_place = position_current
                    self.stop_time_queue.pop(0)
                    return
            else:
                rospy.logwarn('first stop place')
                self.cmd.data[0] = 0
                self.cmd.data[1] = 0
                self.stop_time = time.time()
                self.stop_place = position_current
                return
    
        if stop_duration < self.stop_time_queue[0]:
            rospy.logwarn('waiting until'+str(self.stop_time_queue[0])+'s')
            # rospy.logwarn('stop time: '+str(self.stop_time))
            # rospy.logwarn('now time: '+str(time.time()))
            self.cmd.data[0] = 0
            self.cmd.data[1] = 0
            return

        # 最近点太远直接停车
        if distance_current > 1.0:
            rospy.logwarn('*'+10*'-'+'偏离路线太远，停车'+10*'-'+'*')
            self.cmd.data[0] = 0
            self.cmd.data[1] = 0
            return

        preview_distance = 1.5
        
        # find the preview roadpoint according to preview distance
        id_preview, preview_distance_real = self.get_preview_roadpoint(local_traj_xy, id_current, preview_distance,
                                                                       self.posture)
        yaw = self.posture[2]
        preview_x, preview_y = local_traj_xy[id_preview, 0], local_traj_xy[id_preview, 1]

        self.preview_point.x, self.preview_point.y = local_traj_xy[id_preview, 0], local_traj_xy[id_preview, 1]

        # delta_y：纯追踪算法中关键量。预瞄点 距离 车质心 车身右方侧向距离。
        delta_y = (preview_x - self.posture[0]) * np.sin(yaw * np.pi / 180) - (preview_y - self.posture[1]) * np.cos(
            yaw * np.pi / 180)
        # delta_y = -(preview_x - self.posture[0]) * np.sin(yaw * np.pi / 180) +   (preview_y - self.posture[1]) * np.cos(yaw * np.pi / 180);

        # 算法公式。曲率 kappa = 2*y/ 距离平方。
        preview_curvature = 2 * delta_y / (preview_distance_real**2+0.0000001)

        # 车辆轴距为0.8。前轮转角 angle = L *kappa
        angle = math.atan(0.8/2 * preview_curvature) * 180 / np.pi
        rospy.logwarn("angle:"+ str(angle))

        #angle = (math.atan(0.8 * preview_curvature) * 180 / np.pi)/2
        #rospy.logwarn("angle2: " + str(angle))

        if np.abs(angle) > 30:
            angle = np.sign(angle) * 30

        # 转角右转为正
        if np.isnan(angle):
            rospy.logwarn('purepursuit. 135 angle is None')
            angle = 0
        self.cmd.data[1] = int(angle * 1024 / 30)

        vel_target = self.local_traj.roadpoints[id_current].v
        vel_current = np.linalg.norm(self.vel)
        vel_cmd = vel_target

        if vel_cmd == 0:
            rospy.logwarn('current roadpoint 0 velocity.Stop!')

        if preview_distance_real < 0.5:
            rospy.logwarn('pp preview distance too short')
            vel_cmd = 0
            self.cmd.data[1] = 0
    

        self.cmd.data[0] = int(vel_cmd * 36)
        rospy.logwarn(str(id_current)+"  "+str(id_preview)+" "+str(vel_cmd)+" "+str(self.road_type[id_current]))
        # 正常运行时，从此处完成
        self.error_ending = False
        return

    def get_current_roadpoint(self, local_traj_xy, posture):
        position = posture[:2]
        distance = np.linalg.norm(local_traj_xy - position, axis=1)
        index = np.argmin(distance)
        min_distance = distance[index]

        index = np.where(np.abs(min_distance - distance) < 0.05)
        index = index[0][-1]
        if min_distance > 0:
            rospy.logwarn('当前偏差: \t' + str(min_distance) )
            rospy.logwarn('当前位置:\t'+str(position))
            deviation = local_traj_xy[index, :] - position
            rospy.logwarn('调整方向: \t'+str(deviation))

        return index, min_distance

    def get_preview_roadpoint(self, local_traj_xy, id_current, preview_distance, posture):
        position = local_traj_xy[id_current,:]#posture[:2]
        # 距离自车距离再增加1m
        distance = np.linalg.norm(local_traj_xy[id_current:, :] - position, axis=1) - preview_distance

        id_preview = np.argmin(np.abs(distance))
        id_preview = id_preview + id_current

        preview_distance_error = distance[id_preview - id_current]
        if preview_distance_error > 0.05:
            rospy.loginfo('not long enough for preview. preview the road end point instead')

        return id_preview, preview_distance_error + preview_distance

    def get_local_trajectory(self):

        self.is_local_trajectory_ready = True

        roadMapPath_zhencha = rospy.get_param("/roadMapPath/zhencha")
        roadMapPath_rest = rospy.get_param("/roadMapPath/rest")
        rospy.logwarn('Roadmap file '+roadMapPath_zhencha+' is loaded!')
        rospy.logwarn('Roadmap file '+roadMapPath_rest+' is loaded!')

        traj_zhencha, road_type_zhencha = read_file(roadMapPath_zhencha)
        traj_rest, road_type_rest = read_file(roadMapPath_rest)

        self.trajectory_list = [traj_zhencha, traj_rest]
        self.road_type_list = [road_type_zhencha, road_type_rest]


if __name__ == '__main__':
    rospy.init_node('pure_pursuit_py', anonymous=True)
    pure_pursuit = PurePursuit(sys.argv[1])
    #pure_pursuit = PurePursuit()
    pure_pursuit.run()
    # rospy.spin()
