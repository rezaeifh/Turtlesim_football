#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <turtlesim/Pose.h>
#include <turtlesim/Spawn.h>
#include <turtlesim/TeleportAbsolute.h>
#include <turtlesim/Kill.h>
#include <turtlesim/SetPen.h>
#include <cmath>
#include <iostream>
#include <string.h>
#include <random>

class TurtleGameClass
{
protected:

	// ROS Objects
        ros::NodeHandle nh_;
        
        // ROS Publisher
        ros::Publisher pub1;
        geometry_msgs::Twist speed_player;
        turtlesim::Pose pose_ball, pose_player;
        
        //ROS Subscriber
        ros::Subscriber sub1, sub2;
        
        //ROS Service
        ros::ServiceClient clear_service_call, ball_service_call, angle_service_call, kill_service_call;
        turtlesim::Spawn ball_object;
        turtlesim::SetPen clear_object;
        turtlesim::TeleportAbsolute angle_object;
        turtlesim::Kill kill_object;  
        
               
        int count= 0;
        int loop = 0;
        int touch = 0;
        int direction = 1;
        int i = 1; 
        int i_new = 1; 
        const double pi = 3.14159265358979323846;
        std::string r;
public:
  TurtleGameClass()
  {
        // wait for the environment
        ros::service::waitForService("/turtle7/teleport_absolute");
  
   	// get number of round user wants to play
  	std::cout<< "How many rounds do you want to play?\n";
  	std::cin >> r;
  
  	if (this->is_number(r) != true)
  	{
      	    std::cout<< "You will play 2 rounds since your input was wrong!";
      	    r = "2";
  	}
        
  	// wait for the spawn service
        ros::service::waitForService("/spawn");
        
        // wait for the kill service
        ros::service::waitForService("/kill");

        // create the ball
        this->create_ball();
        ros::service::waitForService("/ball/teleport_absolute");

        // wait for the service
        ros::service::waitForService("/ball/teleport_absolute");
        angle_service_call = nh_.serviceClient<turtlesim::TeleportAbsolute>("/ball/teleport_absolute");

        // Start subscribing the pose of the player and the ball
        sub1 = nh_.subscribe<turtlesim::Pose>("/ball/pose", 1, boost::bind(&TurtleGameClass::pose_ball_callback, this, _1));
        
        sub2 = nh_.subscribe<turtlesim::Pose>("/turtle1/pose", 1, boost::bind(&TurtleGameClass::pose_player_callback, this, _1));
        
        // cleaning the path of the player
        this->clear_path("turtle1");

        pub1 = nh_.advertise<geometry_msgs::Twist>("/ball/cmd_vel", 1);
    }

    ~TurtleGameClass(void)
    {
    }
    
    bool is_number(const std::string& s)
    {
        std::string::const_iterator it = s.begin();
        while (it != s.end() && std::isdigit(*it)) ++it;
        return !s.empty() && it == s.end();
    }
    
    void clear_path(const std::string& name)
    {
        std::string service_name = "/"+ name + "/set_pen";
        clear_service_call = nh_.serviceClient<turtlesim::SetPen>(service_name);

        clear_object.request.off = 1;
   
        clear_service_call.call(clear_object);
    }

    void create_ball()
    {

        ball_service_call = nh_.serviceClient<turtlesim::Spawn>("/spawn");
        	
	// random location will be considered for the ball
        ball_object.request.x = (3 + 1) + (((float) rand()) / (float) RAND_MAX) * (9 - (3 + 1));
        ball_object.request.y = (4 + 1) + (((float) rand()) / (float) RAND_MAX) * (9 - (4 + 1));
        
        ball_object.request.name = "ball";
        ball_service_call.call(ball_object);
        
        this->clear_path("ball");
    }
     
    void kill_ball()
    {
        kill_service_call = nh_.serviceClient<turtlesim::Kill>("/kill");
        kill_object.request.name = "ball";
        kill_service_call.call(kill_object);
    }

    // calculate distance between ball and player
    double distance(turtlesim::Pose c1, turtlesim::Pose c2)
    {
        double x1 = c1.x;
        double x2 = c2.x;
        double y1 = c1.y;
        double y2 = c2.y;
        return sqrt(pow((x1-x2),2)+pow((y1-y2),2));
    }
    
    // claculate the angle of kicking the ball
    double calculate_angle(turtlesim::Pose c1, turtlesim::Pose c2)
    {
        double x1 = c1.x;
        double x2 = c2.x;
        double y1 = c1.y;
        double y2 = c2.y;
        
        if (x1 > x2 && y1 > y2)
        {
            return -(pi - atan((y1 - y2)/(x1 - x2)));
        }
        else if (x1 < x2 && y1 > y2)
        {
            return -atan((y1 - y2)/(x2 - x1));
        }
        else if (x1 > x2 && y1 < y2)
        {
            return (pi - atan((y2 - y1)/(x1 - x2)));
        }
        else if (x1 < x2 && y1 < y2)
        {
            return atan((y2 - y1)/(x2 - x1));
        }
        else if (x1 == x2 && y1 > y2)
        {
            return -pi/2;
        }
        else if (x1 == x2 && y1 < y2)
        {
            return pi/2;
        }
        
        return 0.00;
    }

    // ball subscriber callback function
    void pose_ball_callback(const turtlesim::Pose::ConstPtr& poo)
    {
        pose_ball.x = poo->x;
        pose_ball.y = poo->y;
        pose_ball.theta = poo->theta; 
        
        ros::Duration(0.1).sleep();  
    }

    void pose_player_callback(const turtlesim::Pose::ConstPtr& pos)
    {
    pose_player.x = pos->x;
    pose_player.y = pos->y;
    pose_player.theta = pos->theta;
    ros::Duration(0.1).sleep();
    }
    void game_play()
    {
    std::string at_string, ati_string, g_string;

    	
        if (stoi(r) > 1)
        {
            at_string = " attempts.";
        }
        else
        {
            at_string = " attempt.";
        }
           
        if (i > 1)
        {
            ati_string = " attempts.";
        }
        else
        {
            ati_string = " attempt.";
        }
        
        
        if (i_new > i)
        {
            this->create_ball();
            i += 1;
            
            sleep(2);
            
            if (i <= stoi(r))
            {
                ROS_INFO("Round %d" ,i);
            }
        }
        
        if (count > 1)
        {
            g_string = " goals";
        }
        else
        {
            g_string = " goal";
        }
        
        if (i > stoi(r))
        {
            ROS_INFO_ONCE("End of the game");
            ROS_INFO_ONCE("You scored %d%s in %s%s ", count, g_string.c_str(), r.c_str(), at_string.c_str());
            ros::shutdown();
        }      
            
        double Tol = 0.1;
        
        if (loop==0)
        {
        sleep(2);
        }
        ROS_INFO_ONCE("Start the game");
        // start rounds
        if (pose_player.y !=0.00 && this->distance(pose_player, pose_ball) < 0.7 && touch == 0)
        {
            double theta = this->calculate_angle(pose_player, pose_ball);
            angle_object.request.theta = theta;
            angle_object.request.x = pose_ball.x;
            angle_object.request.y = pose_ball.y;
            speed_player.linear.x = 2;
            angle_service_call.call(angle_object);
            touch = 1;
        }
        else if (pose_ball.y < 11+Tol && pose_ball.y > 11-Tol)
        {
            angle_object.request.theta = -pose_ball.theta;
            angle_object.request.x = pose_ball.x;
            angle_object.request.y = pose_ball.y;
            angle_service_call.call(angle_object);
        }
        else if (pose_ball.x < Tol && loop>6)
        {
            angle_object.request.theta = pi-pose_ball.theta;
            angle_object.request.x = pose_ball.x;
            angle_object.request.y = pose_ball.y;
            angle_service_call.call(angle_object);
        }
        else if (pose_ball.x < 11+Tol && pose_ball.x > 11-Tol)
        {
            angle_object.request.theta = pi-pose_ball.theta;
            angle_object.request.x = pose_ball.x;
            angle_object.request.y = pose_ball.y;
            angle_service_call.call(angle_object); 
        }
            
        pub1.publish(speed_player);
       
        
        if (pose_ball.y < Tol && loop>6)
        {
            if (pose_ball.x > 4.1 && pose_ball.x < 6.8)
            {
                count += 1;
            }    
            touch = 0;
            i_new = i+1;    
            ROS_INFO("%d%s in %d%s", count, g_string.c_str(), i, ati_string.c_str());    
            this->kill_ball();
            sleep(2);
            speed_player.linear.x = 0.0;
            loop = -1;
        }
      loop++;
    }             
};  

int main(int argc, char** argv)
{
  ros::init(argc, argv, "turtlesim_game_node");
  
  TurtleGameClass track_object;
  
  while(ros::ok())
  {
  track_object.game_play();
  ros::spinOnce();
  }
  
  return 0;
}
