#include <ros/ros.h>
#include <laser_assembler/AssembleScans2.h>
#include <sensor_msgs/PointCloud2.h>
#include <signal.h>

void mySigintHandler(int sig)
{
  // Do some custom action.
  // For example, publish a stop message to some other nodes.
  ROS_INFO("Node has finished cleanly\n");
  
  // All the default sigint handler does is call shutdown()
  ros::shutdown();
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "assemble_scans_node");
    ros::NodeHandle nh;
    ros::service::waitForService("assemble_scans2");
    ros::ServiceClient client = nh.serviceClient<laser_assembler::AssembleScans2>("assemble_scans2");
    ros::Publisher pub = nh.advertise<sensor_msgs::PointCloud2> ("points2", 1);
    
    // Override the default ros sigint handler.
    // This must be set after the first NodeHandle is created.
    signal(SIGINT, mySigintHandler);
  
    sensor_msgs::PointCloud2 pointcloud2;
    laser_assembler::AssembleScans2 srv;
    srv.request.begin = ros::Time(0,0);

    if(ros::Time::waitForValid()) {
        srv.request.end = ros::Time::now();
        ROS_INFO("The sim time is: %f\n", ros::Time::now().toSec());
    }
    else 
        ROS_ERROR("No valid time available\n");
    
    
  
    ros::WallRate loop_rate(0.25);
    while (nh.ok())
    {
        laser_assembler::AssembleScans2 srv;
        srv.request.begin = ros::Time(0,0);
        srv.request.end = ros::Time::now();
            
        if (client.call(srv)) {
            pointcloud2 = srv.response.cloud;
            pub.publish (pointcloud2);
            ROS_INFO("Got cloud with %u points\n", pointcloud2.row_step*pointcloud2.height);
        }
        else
            ROS_ERROR("Service call failed\n");
        
        ros::spinOnce ();
        loop_rate.sleep (); // When /use_sim_time is set and no time provider is running, all ROS sleep methods will just block infinitely -> Solution: use ros::WallRate instead of ros::Rate
    }
    
    return 0;
}
