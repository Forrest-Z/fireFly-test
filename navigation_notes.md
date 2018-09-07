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

