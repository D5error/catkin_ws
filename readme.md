# 智能机器人综合实验
* 编译
    cd ~/catkin_ws
    catkin_make

* 打开世界
    * 空世界
        roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=/home/haiden/worlds/empty.world

    * 二维码世界
        roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=/home/haiden/worlds/marker.world

* 运行节点
    * velocity publisher
        rosrun learning_topic velocity_publisher

    * odom subscriber
        rosrun learning_topic odom_subscriber

* 键盘控制
    roslaunch turtlebot_teleop keyboard_teleop.launch

* 运行启动文件
    * learning topic
        roslaunch learning_topic learning_topic.launch

    * 矩形
        roslaunch trajectory_following rectangle.launch

    * fourth
        1. 默认（仿真用）
            roslaunch fourth fourth.launch
        2. 指定二维码尺寸（实际用）
            roslaunch fourth fourth.launch marker_size:=14

    * 识别二维码信息
        roslaunch ar_location get_ar_info.launch

    * 向二维码走
        roslaunch ar_location ar_location.launch

* USB连接机器人
    * 检查机器人是否连接成功
        ls /dev/kobuki

    * 机器人连接失败，运行这个
        rosrun kobuki_ftdi create_udev_rules

    * 机器人，启动
        roslaunch turtlebot_bringup minimal.launch

    * RGB-D相机，启动
        roslaunch openni_launch openni.launch

    * 实时显示相机画面
        rosrun image_view image_view image:=/camera/rgb/image_color

    * 跟着人走
        roslaunch turtlebot_follower follower.launch

    * 向二维码走
        roslaunch ar_location ar_location.launch marker_size:=14

    * 实际场景建图
        roslaunch turtlebot_bringup minimal.launch
        roslaunch turtlebot_navigation gmapping_demo.launch
        roslaunch turtlebot_rviz_launchers view_navigation.launch
        roslaunch turtlebot_teleop keyboard_teleop.launch
