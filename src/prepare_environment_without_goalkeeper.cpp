#include <ros/ros.h>
#include <turtlesim/Spawn.h>

class EnvironmentClass
{
    protected:
    
    // ROS Objects
    ros::NodeHandle nh_;
    
    //ROS Service
    ros::ServiceClient T1_service_call; 
    std::string srv_name_;
    turtlesim::Spawn T1_object;
    ros::ServiceClient T2_service_call;
    turtlesim::Spawn T2_object;
    ros::ServiceClient T3_service_call; 
    turtlesim::Spawn T3_object;
    ros::ServiceClient T4_service_call; 
    turtlesim::Spawn T4_object;
    ros::ServiceClient T5_service_call; 
    turtlesim::Spawn T5_object;
    ros::ServiceClient T6_service_call; 
    turtlesim::Spawn T6_object;
   
    
    public:
    	EnvironmentClass(std::string name): srv_name_(name)
    	{
    	
    	    ros::service::waitForService(name);
	    
	    ////////////////T1//////////////////
            T1_service_call = nh_.serviceClient<turtlesim::Spawn>(name);
            ros::Duration(0.1).sleep();
            T1_object.request.x = 1.5;
            T1_service_call.call(T1_object);
            
            ////////////////T2//////////////////
            T2_service_call = nh_.serviceClient<turtlesim::Spawn>(name);
            ros::Duration(0.1).sleep();
            T2_object.request.x = 2.5;
            T2_service_call.call(T2_object);
            
            ////////////////T3//////////////////
            T3_service_call = nh_.serviceClient<turtlesim::Spawn>(name);
            ros::Duration(0.1).sleep();
            T3_object.request.x = 3.5;
            T3_service_call.call(T3_object);
            
            ////////////////T4//////////////////
            T4_service_call = nh_.serviceClient<turtlesim::Spawn>(name);
            ros::Duration(0.1).sleep();
            T4_object.request.x = 7.5;
            T4_service_call.call(T4_object);
            
            ////////////////T5//////////////////
            T5_service_call = nh_.serviceClient<turtlesim::Spawn>(name);
            ros::Duration(0.1).sleep();
            T5_object.request.x = 8.5;
            T5_service_call.call(T5_object);
            
            ////////////////T6//////////////////
            T6_service_call = nh_.serviceClient<turtlesim::Spawn>(name);
            ros::Duration(0.1).sleep();
            T6_object.request.x = 9.5;
            T6_service_call.call(T6_object);
            
            ROS_INFO("Successfully build the goal");
            
        }
};
        

int main(int argc, char** argv)
{
  ros::init(argc, argv, "environment_node");
  EnvironmentClass track_object("/spawn");
  
  
  return 0;
}
