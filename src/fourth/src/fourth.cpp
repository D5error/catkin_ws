#include <ros/ros.h>
#include <ar_track_alvar_msgs/AlvarMarkers.h>
#include <geometry_msgs/Twist.h>
#include <math.h>

#define SUBSCRIBER_BUFFER_SIZE 1
#define PI 3.1415926535
#define RATE 10
#define SPEED 0.2


void odd();
void even();
double radians(double deg);
void goStraight(double length);
void turnAround(double angular);
int getQRCodeId(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr& msg);
double getQRCodeX(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr& msg);
double getQRCodeY(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr& msg);
double getQRCodeZ(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr& msg);
void markersCallback(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr& msg);
double square(double n);

bool isMoving = false; // 是否正在移动
bool isFinish = false; // 是否运动结束
double x_start; // 二维码起始点的x坐标
double y_start; // 二维码起始点的y坐标
double z_start; // 二维码起始点的z坐标
double x_end; // 二维码起始点的x坐标
double y_end; // 二维码起始点的y坐标
double z_end; // 二维码起始点的z坐标
ros::Publisher vel_pub; // 速度发布
ros::Subscriber pose_sub; // 二维码订阅

int main(int argc, char** argv)
{
    // 初始化
    ros::init(argc, argv, "fourth");
    ros::NodeHandle nh;

    // 发布和订阅
    vel_pub = nh.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 10);
    pose_sub = nh.subscribe("/ar_pose_marker", SUBSCRIBER_BUFFER_SIZE, markersCallback);

    ros::spin();

    return 0;
}

// 二维码回调函数
void markersCallback(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr& msg)
{
    // 识别到二维码
    if (msg->markers.size() > 0 && !isMoving)
    {
        isMoving = true;
        x_start = getQRCodeX(msg);
        y_start = getQRCodeY(msg);
        z_start = getQRCodeZ(msg);
        ROS_INFO("start: x = %.2f, y = %.2f, z = %.2f", x_start, y_start, z_start);
        int id = getQRCodeId(msg);
        ROS_INFO("id = %d", id);
        if (id % 2 == 0) {
            even();
        } else {
            odd();
        }
        isFinish = true;
    } else if (msg->markers.size() > 0 && isFinish) {
        x_end = getQRCodeX(msg);
        y_end = getQRCodeY(msg);
        z_end = getQRCodeZ(msg);
        ROS_INFO("end: x = %.2f, y = %.2f, z = %.2f", x_end, y_end, z_end);

        double error = sqrt(square(x_start - x_end) + square(y_start - y_end) + square(z_start - z_end));
        ROS_INFO("error: %.2f mm", error * 1000);
        
        ros::shutdown();
    }
}

// 奇数运动
void odd() {
    ROS_INFO("Odd move");
    // 1
    goStraight(1.4);
    turnAround(-90);
    // 2
    goStraight(1.4);
    turnAround(90);
    // 3
    goStraight(1.4);
    turnAround(90);
    // 4
    goStraight(2.8);
    turnAround(90);
    // 5
    goStraight(2.8);
    turnAround(90);
    // 6
    goStraight(1.4);
    turnAround(90);

    ROS_INFO("Moving is over");
}

// 偶数运动
void even() {
    ROS_INFO("Even move");
    // 1
    goStraight(1.4);
    turnAround(90);
    // 2
    goStraight(1.4);
    turnAround(-90);
    // 3
    goStraight(1.4);
    turnAround(-90);
    // 4
    goStraight(2.8);
    turnAround(-90);
    // 5
    goStraight(2.8);
    turnAround(-90);
    // 6
    goStraight(1.4);
    turnAround(-90);

    ROS_INFO("Moving is over");
}

// 获取二维码id
int getQRCodeId(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr& msg) {
    return msg->markers[0].id;
}

// 获取二维码中心点相对相机坐标系的x坐标
double getQRCodeX(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr& msg) {
    return msg->markers[0].pose.pose.position.x;
}

// 获取二维码中心点相对相机坐标系的y坐标
double getQRCodeY(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr& msg) {
    return msg->markers[0].pose.pose.position.y;
}

// 获取二维码中心点相对相机坐标系的z坐标
double getQRCodeZ(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr& msg) {
        return msg->markers[0].pose.pose.position.z;
}

// 直走length
void goStraight(double length) {
    ros::Rate rate(RATE);
    int time = 0;
    double seconds = length / SPEED;
    time = int(seconds * RATE);

    ROS_INFO("go straight %.5f m", length);

    for (int i = 0; i < time; i++)
    {
        geometry_msgs::Twist vel_msg_move;
        vel_msg_move.linear.x = SPEED;
        vel_msg_move.angular.z = 0.0;
        vel_pub.publish(vel_msg_move);
        rate.sleep();
    }
}

// 逆时针转动angular（角度制）
void turnAround(double angular) {
    ros::Rate rate(RATE);
    ROS_INFO("Turn counterclockwise %.2f degrees", angular);
    geometry_msgs::Twist vel_msg_turn;
    vel_msg_turn.linear.x = 0.0;
    vel_msg_turn.angular.z = radians(angular / 2);

    for (int i = 0; i < 2 * RATE; i++){
        vel_pub.publish(vel_msg_turn);
        rate.sleep();
    }
}

// 角度制转弧度制
double radians(double deg) {
    double rad = deg * PI / 180;
    return rad;
}

double square(double n) {
    return n * n;
}