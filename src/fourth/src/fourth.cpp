#include <ros/ros.h>
#include <ar_track_alvar_msgs/AlvarMarkers.h>
#include <geometry_msgs/Twist.h>


#define SUBSCRIBER_BUFFER_SIZE 1
#define PI 3.1415926535
#define RATE 10
#define SPEED 0.2


void markersCallback(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr& msg);
void odd();
void even();
double radians(double deg);

bool isMoving = false;
double length;  
double width; 
ros::Publisher vel_pub;
ros::Subscriber pose_sub;


int main(int argc, char** argv)
{
    ros::init(argc, argv, "fourth");
    ros::NodeHandle nh;

    nh.getParam("/length", length);
    nh.getParam("/width", width);

    // 创建一个Publisher，主题名为 /mobile_base/commands/velocity
    vel_pub = nh.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 10);


    pose_sub = nh.subscribe("/ar_pose_marker", SUBSCRIBER_BUFFER_SIZE, markersCallback);

    ros::spin();

    return 0;
}


void markersCallback(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr& msg)
{
    // 识别到二维码
    if (msg->markers.size() > 0 && !isMoving)
    {
        isMoving = true;

        // 二维码中心点相对相机坐标系的x坐标、y坐标和z坐标
        double x_ar, y_ar, z_ar;  
        x_ar = msg->markers[0].pose.pose.position.x;
        y_ar = msg->markers[0].pose.pose.position.y;
        z_ar = msg->markers[0].pose.pose.position.z;
        ROS_INFO("x_ar: %.4f, y_ar: %.4f, z_ar: %.4f", x_ar, y_ar, z_ar);

        int id;
        id = msg->markers[0].id;
        ROS_INFO("id: %d", id);

        double x, y, z, w;
        x = msg->markers[0].pose.pose.orientation.x;
        y = msg->markers[0].pose.pose.orientation.y;
        z = msg->markers[0].pose.pose.orientation.z;
        w = msg->markers[0].pose.pose.orientation.w;
        ROS_INFO("x: %.4f, y: %.4f, z: %.4f, w: %.4f", x, y, z, w);
    
        // even
        if (id % 2 == 0) {
            even();
        } else {
            odd();
        }

        // 扫描二维码
    }
}

void odd() {
    int count = 0;
    while (ros::ok())
    {
        ROS_INFO("count: %d", count);
	    int isTurnRight = 1;
        int time = 0;
        if (count % 6 == 0)  // first edge
        {
	        isTurnRight = -1;
            double seconds = length / (2 * SPEED);
            time = int(seconds * RATE);
        }
        else if (count % 6 == 1)               // second edge
        {
            double seconds = width / (2 * SPEED);
            time = int(seconds * RATE);
        }
	    else if (count % 6 == 2)               // 3ed edge
        {
            double seconds = width / (2 * SPEED);
            time = int(seconds * RATE);
        }
	    else if (count % 6 == 3)               // 4th edge
        {
            double seconds = width / SPEED;
            time = int(seconds * RATE);
        }
        else if (count % 6 == 4)               // 5th edge
        {
            double seconds = width / SPEED;
            time = int(seconds * RATE);
        }
        else			               // 6th edge
        {
            double seconds = width / (2 * SPEED);
            time = int(seconds * RATE);
        }

        // 设置循环频率
        ros::Rate rate(RATE);


        // 前进
        ROS_INFO("Going Straight");
        for (int i = 0; i < time; i++)
        {
            geometry_msgs::Twist vel_msg_move;
            vel_msg_move.linear.x = SPEED;
            vel_msg_move.angular.z = 0.0;
            vel_pub.publish(vel_msg_move);
            rate.sleep();
        }

        // 转弯
        ROS_INFO("Turning");
        for (int i = 0; i < 2 * RATE; i++){
            geometry_msgs::Twist vel_msg_turn;
            vel_msg_turn.linear.x = 0.0;
            vel_msg_turn.angular.z = isTurnRight * radians(45);
            vel_pub.publish(vel_msg_turn);
            rate.sleep();
        }


        count = (count + 1) % 6;
        if (count == 0)
        {
            ROS_INFO("Moving is over");
            break;
        }
    }
}

void even() {
    int count = 0;
    while (ros::ok())
    {
        ROS_INFO("count: %d", count);
	    int isTurnRight = 1;
        int time = 0;
        if (count % 6 == 0)  // first edge
        {
	        isTurnRight = -1;
            double seconds = length / (2 * SPEED);
            time = int(seconds * RATE);
        }
        else if (count % 6 == 1)               // second edge
        {
            double seconds = width / (2 * SPEED);
            time = int(seconds * RATE);
        }
	    else if (count % 6 == 2)               // 3ed edge
        {
            double seconds = width / (2 * SPEED);
            time = int(seconds * RATE);
        }
	    else if (count % 6 == 3)               // 4th edge
        {
            double seconds = width / SPEED;
            time = int(seconds * RATE);
        }
        else if (count % 6 == 4)               // 5th edge
        {
            double seconds = width / SPEED;
            time = int(seconds * RATE);
        }
        else			               // 6th edge
        {
            double seconds = width / (2 * SPEED);
            time = int(seconds * RATE);
        }

        // 设置循环频率
        ros::Rate rate(RATE);


        // 前进
        ROS_INFO("Going Straight");
        for (int i = 0; i < time; i++)
        {
            geometry_msgs::Twist vel_msg_move;
            vel_msg_move.linear.x = SPEED;
            vel_msg_move.angular.z = 0.0;
            vel_pub.publish(vel_msg_move);
            rate.sleep();
        }

        // 转弯
        ROS_INFO("Turning");
        for (int i = 0; i < 2 * RATE; i++){
            geometry_msgs::Twist vel_msg_turn;
            vel_msg_turn.linear.x = 0.0;
            vel_msg_turn.angular.z = isTurnRight * radians(-45);
            vel_pub.publish(vel_msg_turn);
            rate.sleep();
        }


        count = (count + 1) % 6;
        if (count == 0)
        {
            ROS_INFO("Moving is over");
            break;
        }
    }
}

double radians(double deg) 
{
    double rad = deg * PI / 180;
    return rad;
}
