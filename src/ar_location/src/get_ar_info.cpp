#include <ros/ros.h>
#include <ar_track_alvar_msgs/AlvarMarkers.h>

#define SUBSCRIBER_BUFFER_SIZE 1

void markersCallback(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr& msg);


int main(int argc, char** argv)
{
    ros::init(argc, argv, "get_ar_info");
    ros::NodeHandle nh;

    ros::Subscriber pose_sub = nh.subscribe(
        "/ar_pose_marker", SUBSCRIBER_BUFFER_SIZE, markersCallback);

    ros::spin();

    return 0;
}


void markersCallback(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr& msg)
{
    // ʶ�𵽶�ά��
    if (msg->markers.size() > 0)
    {
        // ��ά�����ĵ�����������ϵ��x���ꡢy�����z����
        double x_ar, y_ar, z_ar;  
        x_ar = msg->markers[0].pose.pose.position.x;
        y_ar = msg->markers[0].pose.pose.position.y;
        z_ar = msg->markers[0].pose.pose.position.z;
        ROS_INFO("x_ar: %.4f, y_ar: %.4f, z_ar: %.4f", x_ar, y_ar, z_ar);
    }
}


