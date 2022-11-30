#! /usr/bin/env python3

# import usefull packages
import rospy
import math
import random
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from turtlesim.srv import Spawn, SpawnRequest, TeleportAbsolute, TeleportAbsoluteRequest, Kill, KillRequest, SetPen, SetPenRequest


class TurtleGameClass():

    def __init__(self):
        
        # Required parameters
        self.count = 0
        self.i = 1
        self.i_new = 1
        self.touch = 0
        
        # wait for the environment
        rospy.wait_for_service("/turtle7/teleport_absolute")

        # get number of round user wants to play
        self.r = input('How many rounds do you want to play?\n')
        if self.r.isdecimal() == False :
            print("you will play 2 rounds since your input was wrong!")
            self.r = '2'

        # wait for the spawn service
        rospy.wait_for_service('/spawn')
        
        # wait for the kill service
        rospy.wait_for_service('/kill')

        # create the ball
        self.create_ball()

        # wait for the service
        rospy.wait_for_service('/ball/teleport_absolute')
        self. set_angle()

        # Start subscribing the pose of the player and the ball
        self.sub = rospy.Subscriber(
            '/ball/pose', Pose, self.pose_ball_callback)
        self.sub = rospy.Subscriber(
            '/turtle1/pose', Pose, self.pose_player_callback)
        
        # cleaning the path of the player
        self.clear_path("turtle1")

        self.pub = rospy.Publisher('/ball/cmd_vel', Twist, queue_size=1000)
        self.speed_player = Twist()  # Twist Object
        self.speed_player.linear.x = 0

    def myhook(self):
        pass    

    def clear_path(self, name):
        service_name = "/"+ name + "/set_pen"
        
        self.clear_service_call = rospy.ServiceProxy(service_name, SetPen)
        self.clear_object = SetPenRequest()

        self.clear_object.off = 1
   
        self.service_client_clear_result = self.clear_service_call(self.clear_object)

    def create_ball(self):

        self.ball_service_call = rospy.ServiceProxy('/spawn', Spawn)
        self.ball_object = SpawnRequest()
	
	#random location will be considered for the ball
        self.ball_object.x = random.uniform(3, 9)
        self.ball_object.y = random.uniform(4, 9)
        self.ball_object.name = "ball"

        self.service_client_ball_result = self.ball_service_call(self.ball_object)
        self.clear_path("ball")
     
    def kill_ball(self):

        self.kill_service_call = rospy.ServiceProxy('/kill', Kill)
        self.kill_object = KillRequest()
        self.kill_object.name = "ball"
        self.service_client_kill_result = self.kill_service_call(self.kill_object)

    def set_angle(self):
        self.angle_service_call = rospy.ServiceProxy(
            '/ball/teleport_absolute', TeleportAbsolute)
        self.angle_object = TeleportAbsoluteRequest()

    # calculate distance between ball and player
    def distance(self, c1, c2):
        x1 = c1.x
        x2 = c2.x
        y1 = c1.y
        y2 = c2.y
        return math.sqrt(((x1-x2)**2)+((y1-y2)**2))
    
    # claculate the angle of kicking the ball
    def calculate_angle(self, c1, c2):
        x1 = c1.x
        x2 = c2.x
        y1 = c1.y
        y2 = c2.y

        if x1 > x2 and y1 > y2:
            return -(math.pi - math.atan((y1 - y2)/(x1 - x2)))
        elif x1 < x2 and y1 > y2:
            return -math.atan((y1 - y2)/(x2 - x1))
        elif x1 > x2 and y1 < y2:
            return (math.pi - math.atan((y2 - y1)/(x1 - x2)))
        elif x1 < x2 and y1 < y2:
            return math.atan((y2 - y1)/(x2 - x1))
        elif x1 == x2 and y1 > y2:
            return -math.pi/2
        elif x1 == x2 and y1 < y2:
            return math.pi/2

    # ball subscriber callback function
    def pose_ball_callback(self, posi) :
        self.pose_ball = posi

    # player subscriber callback function
    def pose_player_callback(self, pos) :
        
        if int(self.r) > 1 :
            at_string = " attempts."
        else:
            at_string = " attempt."
            
        if self.i > 1 :
            ati_string = " attempts."
        else:
            ati_string = " attempt."
        
        self.pose_player = pos
        if self.i_new > self.i :
            self.create_ball()
            self.i += 1
            
            rospy.sleep(2)
            if self.i <= int(self.r) :
                rospy.loginfo("Round " + str(self.i))
        
        if self.count > 1 :
            g_string = " goals"
        else :
            g_string = " goal"
        
        if self.i > int(self.r):
            rospy.loginfo_once("End of the game")
            rospy.loginfo_once("You scored " + str(self.count) + g_string + " in "+ self.r + at_string)
            rospy.on_shutdown(self.myhook)
            
        Tol = 0.1
        rospy.loginfo_once("Start the game")
        # start rounds
        if self.distance(self.pose_player, self.pose_ball) < 0.7 and self.touch == 0:
            self.theta = self.calculate_angle(self.pose_player, self.pose_ball)
            self.angle_object.theta = self.theta
            self.angle_object.x = self.pose_ball.x
            self.angle_object.y = self.pose_ball.y
            self.speed_player.linear.x = 2
            self.service_client_angle_result = self.angle_service_call(self.angle_object)
            self.touch = 1
            rospy.sleep(1)
        elif self.pose_ball.y < 11+Tol and self.pose_ball.y > 11-Tol:
            self.angle_object.theta = -self.pose_ball.theta
            self.angle_object.x = self.pose_ball.x
            self.angle_object.y = self.pose_ball.y
            self.service_client_angle_result = self.angle_service_call(self.angle_object)
            rospy.sleep(1)
        elif self.pose_ball.x < Tol:
            self.angle_object.theta = math.pi-self.pose_ball.theta
            self.angle_object.x = self.pose_ball.x
            self.angle_object.y = self.pose_ball.y
            self.service_client_angle_result = self.angle_service_call(self.angle_object)
            rospy.sleep(1)
        elif self.pose_ball.x < 11+Tol and self.pose_ball.x > 11-Tol:
            self.angle_object.theta = math.pi-self.pose_ball.theta
            self.angle_object.x = self.pose_ball.x
            self.angle_object.y = self.pose_ball.y
            self.service_client_angle_result = self.angle_service_call(self.angle_object)
            rospy.sleep(1)
            
        self.pub.publish(self.speed_player)
        
        if self.pose_ball.y < Tol:
        
            if self.pose_ball.x > 4.1 and self.pose_ball.x < 6.8:
                self.count += 1
                
            self.touch = 0
            self.i_new = self.i+1    
            rospy.loginfo(str(self.count) + g_string + " in " + str(self.i) + ati_string)    
            self.kill_ball()
            rospy.sleep(2)
            self.speed_player.linear.x = 0.0
                


if __name__ == '__main__':
    rospy.init_node('turtlesim_game_node')
    game = TurtleGameClass()
    rospy.spin()
