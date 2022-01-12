#!/usr/bin/env python3
'''
1229    sub Search/Attack request & pub local_trajectory
0108    add ResetRequest for loop control
'''

import rospy
from std_msgs.msg import Bool
from grpc_ad_msgs.msg import TasksRequest
from trajectory_tracking.msg import Trajectory, RoadPoint


class StageControl():
    def __init__(self):
        self.rate = rospy.Rate(1)

        self.stage_search = 0
        self.stage_attack = 0
        self.stage_reset = 0

        rospy.Subscriber('/SearchRequest', Bool, self.get_stage_search)
        rospy.Subscriber('/TasksRequest', TasksRequest, self.get_stage_attack)

        rospy.Subscriber('/ResetRequest', Bool, self.get_stage_reset)


        self.pub_list = [rospy.Publisher('/car'+str(i)+'/local_trajectory', Trajectory, queue_size=1) for i in range(1,9)]

    def run(self):
        
        while not rospy.is_shutdown():
            self.pub_logic()
            # rospy.spin()
            self.rate.sleep()

    def pub_logic(self):

        # if(self.stage_search == 1 and self.stage_attack == 1):
        #     rospy.logwarn("stage error!")

        if (self.stage_search == 0 and self.stage_attack == 0):
            rospy.logwarn("stage stop")
            self.stuff_info(0)
        
        elif self.stage_attack == 1:
            rospy.logwarn("stage rest")
            self.stuff_info(2)

        elif self.stage_search == 1:
            # 可能有残余信息进行发送
            rospy.logwarn("stage zhencha")
            self.stuff_info(1)

        else:
            rospy.logwarn('unknow stage'+str(self.stage_search)+str(self.stage_attack))


    def stuff_info(self, stage):

        local_trajectory = Trajectory()
        roadpoint = RoadPoint()
        roadpoint.kappa = stage
        local_trajectory.roadpoints.append(roadpoint)
        for i in range(8):
            self.pub_list[i].publish(local_trajectory)


    def get_stage_search(self, msg:Bool):
        
        if msg.data is True:
            self.stage_search = 1


    def get_stage_attack(self, msg:TasksRequest):
        
        if self.stage_attack == 0 and len(msg.tasks) > 0:
            self.stage_attack = 1
            self.stage_search = 0

    def get_stage_reset(self, msg:Bool):
        if msg.data is True:
            self.stage_reset = 1
            self.stage_search=0
            self.stage_attack = 0

if __name__ == '__main__':

    rospy.init_node('StageControl', anonymous=True)
    stage_control = StageControl()

    stage_control.run()
    



        

