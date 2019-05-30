#include <ros/ros.h>
#include <laser_assembler/AssembleScans2.h>
#include <sensor_msgs/PointCloud2.h>

using namespace laser_assembler;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "assemble_scans_node");
    ros::NodeHandle nh;
    ros::service::waitForService("assemble_scans2");
    ros::ServiceClient client = nh.serviceClient<laser_assembler::AssembleScans2>("assemble_scans2");
    ros::Publisher pub = nh.advertise<sensor_msgs::PointCloud2> ("points2", 1);
  
    sensor_msgs::PointCloud2 pointcloud2;
    laser_assembler::AssembleScans2 srv;
    srv.request.begin = ros::Time(0,0);

    if(ros::Time::waitForValid()) {
        srv.request.end = ros::Time::now();
        ROS_INFO("The sim time is: %f\n", ros::Time::now().toSec());
    }
    else 
        ROS_ERROR("No valid time available\n");
    
    if (client.call(srv)) {
        pointcloud2 = srv.response.cloud;
        printf("Got cloud with %lu points\n", pointcloud2.row_step*pointcloud2.height);
    }
    else
        ROS_ERROR("Service call failed\n");
    
  
    ros::Rate loop_rate(4);
    while (nh.ok())
    {
        pub.publish (srv.response.cloud);
        ROS_INFO("The sim time is: %f", ros::Time::now().toSec());
        
        ros::spinOnce ();
        loop_rate.sleep ();
    }
            
    return 0;
}
