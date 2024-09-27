* compile
    cd ~/catkin_ws
    catkin_make

* create a world
    * create a empty world
        roslaunch turtlebot_gazebo turtlebot_world.launch world_file:=/home/haiden/worlds/empty.world

* run a node
    * velocity publisher
        rosrun learning_topic velocity_publisher

    * odom subscriber
        rosrun learning_topic odom_subscriber