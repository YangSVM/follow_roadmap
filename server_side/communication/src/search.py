#!/usr/bin/env python3
# -*- coding: utf-8 -*-
'''接收到上层侦查，发布grpc控制无人机起飞，并且发布话题开始侦查

'''
from codecs import BOM_LE
from threading import Thread, Lock
import threading

import rospy
import grpc
from concurrent import futures

import adm2ctrl_pb2 as adm2ctrl_pb2
import adm2ctrl_pb2_grpc as adm2ctrl_pb2_grpc
# import libproto.data_transf_pb2 as data_transf_pb2
# import libproto.data_transf_pb2_grpc as data_transf_pb2_grpc

from std_msgs.msg import Bool

'''
与中心节点传输数据所用的IP与端口号
'''
IP_CORE = '166.111.188.213'  # FIXME: 修改为运行数据交互节点的IP
IP_CONTROL = '183.173.68.101'  # FIXME: 修改为运行目前节点的主机IP
PORT_ASK_DATA = '19330'
PORT_CORE2CTRL = '40015'  # FIXME: 修改为本节点预备接受回传数据的端口号

ADDR_DECISION = '183.173.68.101:20208'

search_started_lock = threading.Lock()
is_search_started = Bool()
is_search_started.data = False



class Search(adm2ctrl_pb2_grpc.DtoCServicer):

    def ExecZhncha(self, request:adm2ctrl_pb2.ZhnchaRequest, context):
        print('enter ExecZhncha')
        search_started_lock.acquire()
        
        is_search_started.data = bool(request.start_zhncha)
        search_started_lock.release()

        return adm2ctrl_pb2.ZhnchaReply(zhncha_flag="OK")


def SearchServer():
    server = grpc.server(futures.ThreadPoolExecutor(max_workers=10))
    adm2ctrl_pb2_grpc.add_DtoCServicer_to_server(Search(), server)
    server.add_insecure_port(ADDR_DECISION)
    server.start()
    print("Waiting for request")
    server.wait_for_termination()


def ros4loop():
    pub = rospy.Publisher('SearchRequest', Bool, queue_size=1)
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        search_started_lock.acquire()
        pub.publish(is_search_started)
        search_started_lock.release()
        rate.sleep()



if __name__ == '__main__':
    rospy.init_node('search_command_grpc', anonymous=True)

    threads = []
    thd_ros = threading.Thread(target=ros4loop)
    threads.append(thd_ros)
    thd_server= threading.Thread(target=SearchServer)
    threads.append(thd_server)

    for thd in threads:
        thd.start()
