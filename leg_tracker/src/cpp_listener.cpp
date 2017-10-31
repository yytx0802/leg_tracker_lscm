#include "ros/ros.h"
#include "std_msgs/String.h"
#include <tf/transform_listener.h>
#include <visualization_msgs/Marker.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>

// Local headers
#include <leg_tracker/laser_processor.h>
#include <leg_tracker/cluster_features.h>

// Custom messages
#include <leg_tracker/Person.h>
#include <leg_tracker/PersonArray.h>

class RoutinePlanners
{
public:
    RoutinePlanners() {
	std::string listen_topic;
        nh.param("accelerater_param", accelerater_param, 2.0);
	nh.param("listen_topic", listen_topic, std::string("people_tracked"));

        sub = nh.subscribe(listen_topic, 1000, &RoutinePlanners::chatterCallback,this);
        pub = nh.advertise<std_msgs::String>("FFFFF",20);
	pub2 = nh.advertise<geometry_msgs::Twist>("/cmd_vel",50);
    }
private:
    ros::NodeHandle nh;
    ros::Subscriber sub;
    ros::Publisher pub;
    ros::Publisher pub2;

    double accelerater_param;
    

    void chatterCallback(const leg_tracker::PersonArray::ConstPtr& msg)
    {
      //ROS_INFO("%d",msg->header.seq);
	std_msgs::String ifo;
	geometry_msgs::Twist drivecom;
	drivecom.linear.x = 0.2;
	drivecom.linear.y = 0;
	drivecom.linear.z = 0;
	drivecom.angular.x = 0;
	drivecom.angular.y = 0;
	drivecom.angular.z = 0.5;

        ROS_INFO("x is : %f, angle z is : %f",msg->people[0].pose.position.x,msg->people[0].pose.orientation.z);
        if(msg->people[0].pose.position.y>= 0){
	    ifo.data = std::string("Left!");
            //pub.publish(ifo);
	    pub2.publish(drivecom);
	    
	}
        else if(msg->people[0].pose.position.y < 0){
            ifo.data = std::string("Right");
	    drivecom.linear.x = -0.2;
	    drivecom.angular.z = -0.5;
            //pub.publish(ifo);
	    pub2.publish(drivecom);
	}
	else {
	    ifo.data = std::string("Invalid!");
            //pub.publish(ifo);
	    pub2.publish(drivecom);
        }

    }

};


int main(int argc, char **argv)
{
  ros::init(argc, argv, "cpp_listener");
  RoutinePlanners test;
  ros::spin();
  return 0;
}

