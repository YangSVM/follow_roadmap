#!/usr/bin/env python3
# -*- coding: utf-8 -*-
'''请求gps数据
grpc2ros
'''
from threading import Thread

import rospy
from nav_msgs.msg import Odometry

import grpc
import logging
from concurrent import futures

import libproto.data_transf_pb2 as data_transf_pb2
import libproto.data_transf_pb2_grpc as data_transf_pb2_grpc
import sys

'''
与中心节点传输数据所用的IP与端口号
'''
IP_DATA_SERVER = '166.111.188.213'  # FIXME: 修改为运行数据交互节点的IP
IP_CONTROL = '183.173.66.49'  # FIXME: 修改为运行目前节点的主机IP
PORT_ASK_DATA = '19330'
PORT_CORE2CTRL = '40015'  # FIXME: 修改为本节点预备接受回传数据的端口号


class DataTransfServicer(data_transf_pb2_grpc.DataTransfServiceServicer):
    def ZNTStatusReceive(self, request, context):
        print("Receive agent", request.zntCode)
        car_id = request.zntCode[-1]
        pub = rospy.Publisher('/car'+str(car_id)+'/gps', Odometry, queue_size=1)

        gps_msg = Odometry()
        print(request.zntPosition[0], request.zntPosition[1], request.hxAngle)
        gps_msg.pose.pose.position.x = request.zntPosition[0]
        gps_msg.pose.pose.position.y = request.zntPosition[1]
        gps_msg.twist.twist.angular.z = request.hxAngle
    
        pub.publish(gps_msg)

        response = data_transf_pb2.BaseRespInfo(
            code='200',
            msg="SUCCESS",
            # serverTime= # FIXME: 时间戳应该写啥
        )
        return response


def Rec_from_Core():
    '''
    接收数据交互模块所发来的信息，具体方法实现在类DataTransfServicer()中
    '''
    server = grpc.server(futures.ThreadPoolExecutor(max_workers=10))    
    data_transf_pb2_grpc.add_DataTransfServiceServicer_to_server(
        DataTransfServicer(), server)
    server.add_insecure_port(IP_CONTROL+':'+PORT_CORE2CTRL)
    server.start()
    print("Waiting for request")
    server.wait_for_termination()

def Ask_Data():
    '''
    向数据交互模块申请发送智能体状态信息
    '''
    print('Asking for data...')
    with grpc.insecure_channel(IP_DATA_SERVER+':'+PORT_ASK_DATA) as channel:
        stub = data_transf_pb2_grpc.DataTransfServiceStub(channel)
        msg = data_transf_pb2.SubscribeInfo(
            moduleCode='XT',
            operType='DY',
            moduleHost=IP_CONTROL,
            modulePort=int(PORT_CORE2CTRL),
            dataCodes='ZNTZT',
            # sendTime=0 # FIXME: sendTime写啥？
        )
        response = stub.SubscribeData(msg)
        if response.code == '200':
            print('Success')
        else:
            logging.ERROR(response.msg)
            sys.exit()

if __name__ == '__main__':
    logging.basicConfig()
    Ask_Data()
    rospy.init_node('receive_gnss_grpc2ros', anonymous=True)

    thd_rec = Thread(target=Rec_from_Core)
    thd_rec.start()