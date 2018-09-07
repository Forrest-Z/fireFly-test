# navigation_notes
### 主要涉及的包

+ map_server 发布map话题，需要yaml文件
+ amcl  蒙特卡罗自适应算法？
+ move_base 导航包
+ joint_state_publisher
+ robot_state_publisher
+ gazebo
+ rviz


`chapter6_configuration_gazebo.launch`

``` xml
<?xml version="1.0"?>
<launch>

  	<param name="/use_sim_time" value="true" />

	<remap from="robot/laser/scan" to="/scan" />  
  	<!-- start up wg world -->
	<include file="$(find gazebo_ros)/launch/willowgarage_world.launch" >
	</include>

  	<arg name="model" default="$(find chapter6_tutorials)/urdf/robot1_base_04.xacro"/>
  	<param name="robot_description" command="$(find xacro)/xacro.py $(arg model)" />
	<!--robot_description 会在joint_state_publisher and robot_state_publisher中用到-->
  	<node name="joint_state_publisher" pkg="joint_state_publisher" type="joint_state_publisher" ></node>
	<!--joint_state_publisher会发布机器人的关节信息-->
  	<!-- start robot state publisher -->
 	<node pkg="robot_state_publisher" type="robot_state_publisher" name="robot_state_publisher" 
	 output="screen" />
	<!--robot_state_publisher会接收关节信息，计算出所有tf-->
	<node name="spawn_robot" pkg="gazebo_ros" type="spawn_model" args="-urdf -param robot_description -z 0.1 -model robot_model" respawn="false" output="screen" />

	<node name="rviz" pkg="rviz" type="rviz" args="-d $(find chapter6_tutorials)/launch/navigation.rviz" />

</launch>
```

`move_base.launch`
```xml
<?xml version="1.0"?>

<launch>

  <!-- Run the map server -->
   <node name="map_server" pkg="map_server" type="map_server" args="$(find chapter6_tutorials)/maps/map.yaml" output="screen"/>
  <include file="$(find chapter6_tutorials)/launch/chapter6_configuration_gazebo.launch" >
	</include>
  <!--启动gazebo rviz joint_state_publisher robot_state_publisher xacro文件-->
  <include file="$(find amcl)/examples/amcl_diff.launch" >
  </include> 

  <node pkg="move_base" type="move_base" respawn="false" name="move_base" output="screen">
    <param name="controller_frequency" value="10.0"/>
    <param name="controller_patiente" value="15.0"/>
    <rosparam file="$(find chapter6_tutorials)/launch/costmap_common_params.yaml" command="load" ns="global_costmap" />
    <rosparam file="$(find chapter6_tutorials)/launch/costmap_common_params.yaml" command="load" ns="local_costmap" />
    <rosparam file="$(find chapter6_tutorials)/launch/local_costmap_params.yaml" command="load" />
    <rosparam file="$(find chapter6_tutorials)/launch/global_costmap_params.yaml" command="load" />
    <rosparam file="$(find chapter6_tutorials)/launch/base_local_planner_params.yaml" command="load" />
  </node>

</launch>
```
### 遇到的问题
- [x] 构建地图的时候提示，找不到从odom到base_footprint的tf
>解决方案的发现，尝试着自己添加静态的odom到base_footprint的tf

>`rosrun tf static_transform_publisher 0 0 0 0 0 0 odom base_footprint 100`

>发现在rviz里面从找不到link，成功实现了地图的构建

>> 1. 解决方案之一，自己编写补充从odom到base_footprint的变换
```c++
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

geometry_msgs::TransformStamped odom_trans;
tf::TransformBroadcaster *broadcaster_p;
void odomCallback(const nav_msgs::Odometry& msg)
{
  odom_trans.header.stamp = ros::Time::now();
  odom_trans.transform.translation.x = msg.pose.pose.position.x;
  odom_trans.transform.translation.y = msg.pose.pose.position.y; 
  odom_trans.transform.translation.z = 0.0;
  odom_trans.transform.rotation = msg.pose.pose.orientation;
  broadcaster_p->sendTransform(odom_trans);
}

int main(int argc,char* argv[])
{
    ros::init(argc,argv,"odomToBase");
    ros::NodeHandle n;
    odom_trans.header.frame_id = "odom";
	odom_trans.child_frame_id = "base_footprint";
    tf::TransformBroadcaster broadcaster;
    broadcaster_p=&broadcaster;
    ros::Subscriber sub=n.subscribe("odom",1000,odomCallback);
    ros::spin();
    return 0;
}
```
>>2.解决方案之二，教程里面的xacro文件的libgazebo_ros_skid_steer_drive.so插件本身已经提供这个功能，只是教程有bug，没选上

>>将`<broadcastTF>0</broadcastTF>`改为`<broadcastTF>1</broadcastTF>`即可

## 最重要的心得
1. 遇到不认识的包，先上ros wiki看介绍，搞清楚功能，以及input和output的topic
2. 看懂roslaunch每个节点的功能，可以有助于解决问题
3. 解决问题的过程就是在学习？
4. 教材有坑
