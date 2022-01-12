#!/usr/bin/env python3
# -*- coding: utf-8 -*-
'''
grpc2ros。 input
接收辅助决策指令grpc，转成ros消息发布。
仅会收到一次grpc指令，ros话题需要持续发布。
'''

import rospy
from grpc_ad_msgs.msg import Task, TasksRequest, AttackRequest
from std_msgs.msg import Bool
from trajectory_tracking.msg import Trajectory, RoadPoint


from concurrent import futures
import logging

import grpc

import  libproto.adm2ctrl_pb2  as adm2ctrl_pb2
import  libproto.adm2ctrl_pb2_grpc as adm2ctrl_pb2_grpc
import threading

'''
Config: IP以及端口配置。
与中心节点传输数据所用的IP与端口号
'''
IP_CONTROL ='192.168.1.162'         # 本机IP地址
RCV_ADM_PORT = '20208'                  # 接收辅助决策的端口
CTRL_RCV_ADDR =  IP_CONTROL+':'+RCV_ADM_PORT


class GlobalTaskInfo():
    ''' 全局任务信息。
    '''
    def __init__(self):
        self.search_started_lock = threading.Lock()
        self.is_search_started = Bool()
        self.is_search_started.data = False

        self.task_msg = TasksRequest()
        self.task_msg_lock = threading.Lock()
        
        self.attack_msg = AttackRequest()
        self.attack_msg_lock = threading.Lock()

        # 增加feed back.
        rospy.Subscriber('/car1/local_trajectory', Trajectory, self.get_client_feedback)

    def pub_search_info(self):
        pub = rospy.Publisher('SearchRequest', Bool, queue_size=1)
        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            self.search_started_lock.acquire()
            pub.publish(self.is_search_started)
            self.search_started_lock.release()
            rate.sleep()


    def pub_task_info(self):
        pub = rospy.Publisher('TasksRequest', TasksRequest, queue_size=1)
        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            if  len(self.task_msg.tasks)==0:
                rate.sleep()
            else:
                self.task_msg_lock.acquire()
                pub.publish(self.task_msg)
                self.task_msg_lock.release()
                rate.sleep()

    def pub_attack_info(self):
        pub = rospy.Publisher('AttackRequest', AttackRequest, queue_size=1)    
        rate = rospy.Rate(1)
        while not rospy.is_shutdown():
            if  len(self.attack_msg.attack_pairs)==0:
                rate.sleep()
            else:
                self.attack_msg_lock.acquire()
                pub.publish(self.attack_msg)
                self.attack_msg_lock.release()
                rate.sleep()

    def get_client_feedback(self, msg):
        point = msg.roadpoints[0]
        stage = int(point.kappa)
        # 执行到发送轨迹收到stage
        if stage == 1:
            self.search_started_lock.acquire()
            self.is_search_started.data = False
            self.search_started_lock.release()
        elif stage ==2:
            self.attack_msg_lock.acquire()
            self.attack_msg = AttackRequest()
            self.attack_msg_lock.release()

            self.task_msg_lock.acquire()
            self.task_msg = TasksRequest()
            self.task_msg_lock.release()



gti = GlobalTaskInfo()

class DtoC(adm2ctrl_pb2_grpc.DtoCServicer):

    def ExecZhncha(self, request:adm2ctrl_pb2.ZhnchaRequest, context):
        print('enter ExecZhncha')
        gti.search_started_lock.acquire()
        
        gti.is_search_started.data = bool(request.start_zhncha)
        gti.search_started_lock.release()

        return adm2ctrl_pb2.ZhnchaReply(zhncha_flag="OK")

    def ExecTasks(self, request:TasksRequest, context):
        print('enter ExecTasks')
        global gti

        gti.attack_msg_lock.acquire()
        gti.task_msg.init_timestamp = request.init_timestamp
        for task in request.tasks:
            t = Task()
            t.order = task.order
            t.type = task.type
            t.pos.posx = task.pos.posx
            t.pos.posy = task.pos.posy
            t.pos.posz = task.pos.posz
            for agent in task.agent_us:
                a = agent
                t.agent_us.append(a)
            for agent in task.agent_enemy:
                a = agent
                t.agent_us.append(a)
            t.start_time = task.start_time
            t.duration = task.duration
            gti.task_msg.tasks.append(t)

        gti.attack_msg_lock.release()
        print("control server received: " + str(request.tasks))
        return adm2ctrl_pb2.TasksReply(tasks_flag="tasks received")


    def ExecAttack(self, request:AttackRequest, context):
        print('enter ExecAttack')
        global gti
        
        gti.attack_msg_lock.acquire()
        gti.attack_msg = AttackRequest()
        for attack_pair in request.attack_pairs:
            a = str(attack_pair)
            gti.attack_msg.attack_pairs.append(a)

        gti.attack_msg_lock.release()

        print("control server received: " + str(request.attack_pairs))
        
        return adm2ctrl_pb2.AttackReply(attack_flag="attack pairs received")


def receive_ad_info_server():
    global CTRL_RCV_ADDR
    server = grpc.server(futures.ThreadPoolExecutor(max_workers=10))
    adm2ctrl_pb2_grpc.add_DtoCServicer_to_server(DtoC(), server)
    server.add_insecure_port(CTRL_RCV_ADDR)
    server.start()
    print("Waiting for request")
    server.wait_for_termination()


threads = []
thd_ctrl = threading.Thread(target=receive_ad_info_server)
threads.append(thd_ctrl)

for pubs in [gti.pub_search_info, gti.pub_task_info, gti.pub_attack_info]:
    threads.append(threading.Thread(target=pubs))



if __name__ == '__main__':
    logging.basicConfig()
    rospy.init_node('DtoCServer', anonymous=True)

    for thd in threads:
        thd.start()