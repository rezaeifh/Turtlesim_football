#! /usr/bin/env python3

# import usefull packages
import rospy
from turtlesim.srv import Spawn, SpawnRequest


class EnvironmentClass():

    def __init__(self):


        # wait for the service
        rospy.wait_for_service('/spawn')

        # create turtles for the goal
        #####################T1###################
        self.T1_service_call = rospy.ServiceProxy(
            '/spawn', Spawn)
        self.T1_object = SpawnRequest()

        self.T1_object.x = 2.5

        self.service_client_T1_result = self.T1_service_call(
            self.T1_object)
            
        #####################T2###################
        self.T2_service_call = rospy.ServiceProxy(
            '/spawn', Spawn)
        self.T2_object = SpawnRequest()

        self.T2_object.x = 3.5

        self.service_client_T2_result = self.T2_service_call(
            self.T2_object)
            
        #####################T3###################
        self.T3_service_call = rospy.ServiceProxy(
            '/spawn', Spawn)
        self.T3_object = SpawnRequest()

        self.T3_object.x = 7.5

        self.service_client_T3_result = self.T3_service_call(
            self.T3_object)
        
        #####################T4###################
        self.T4_service_call = rospy.ServiceProxy(
            '/spawn', Spawn)
        self.T4_object = SpawnRequest()

        self.T4_object.x = 8.5

        self.service_client_T4_result = self.T4_service_call(
            self.T4_object)
            
        #####################T5###################
        self.T5_service_call = rospy.ServiceProxy(
            '/spawn', Spawn)
        self.T5_object = SpawnRequest()

        self.T5_object.x = 9.5

        self.service_client_T5_result = self.T5_service_call(
            self.T5_object)
        
        #####################T6###################
        self.T6_service_call = rospy.ServiceProxy(
            '/spawn', Spawn)
        self.T6_object = SpawnRequest()

        self.T6_object.x = 1.5

        self.service_client_T6_result = self.T6_service_call(
            self.T6_object)


        rospy.loginfo("Successfully build the goal")



if __name__ == '__main__':
    rospy.init_node('environment_node')
    track_object = EnvironmentClass()
    rospy.spin()
