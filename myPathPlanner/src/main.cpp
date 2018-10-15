#include<iostream>
#include<ros/ros.h>
#include<tf/transform_broadcaster.h>
#include<string>
#include<tf/tf.h>
#include<vector>
#include <nav_msgs/Path.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>
#include<geometry_msgs/PointStamped.h>
#include<geometry_msgs/PoseArray.h>
double degree2rad(double deg)
{
    return deg*M_PI/180.0f;
}
double rad2degree(double rad)
{
    return rad/M_PI*180.0f;
}
using namespace std;
struct point2d{
    double x;
    double y;
    point2d(double x=0,double y=0)
    {
        this->x=x;
        this->y=y;
    }
};
struct tpose{
    point2d p;
    double yaw;
    tpose(point2d p,double yaw)
    {
        this->p=p;
        this->yaw=yaw;
    }
};
void showPath(vector<point2d> &pointSet,ros::Publisher &pub);
void showPose(vector<point2d> p,vector<double> yaw,ros::Publisher& pub);
void showPoint(point2d &p,ros::Publisher & pub);
int main(int argc,char**argv)
{
    ros::init(argc,argv,"myPathPlanner");
    ros::NodeHandle n;
    ros::Publisher path_pub = n.advertise<nav_msgs::Path>("trajectory",100);
    ros::Publisher point_pub=n.advertise<geometry_msgs::PointStamped>("aim",100);
    ros::Publisher pose_pub=n.advertise<geometry_msgs::PoseArray>("mypose",100);
    tf::TransformBroadcaster broadcaster;
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.frame_id="odom";
    odom_trans.child_frame_id="base_link";
    ros::Rate loop_rate(30);
    bool init=false;
    tpose start_point(point2d(0,0),degree2rad(45));
    tpose end_point(point2d(50,50),degree2rad(90));
    vector<point2d> path={start_point.p,end_point.p};
    vector<double> yaw={start_point.yaw,end_point.yaw};
    while(ros::ok())
    {
    
        showPath(path,path_pub);
        showPose(path,yaw,pose_pub);
        
        odom_trans.header.stamp=ros::Time::now();
        odom_trans.transform.translation.x=0;
        odom_trans.transform.translation.y=0;
        odom_trans.transform.translation.z=0.0;
        odom_trans.transform.rotation=tf::createQuaternionMsgFromYaw(start_point.yaw);//姿态
        broadcaster.sendTransform(odom_trans);
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
void showPath(vector<point2d> &pointSet,ros::Publisher &pub)
{
    geometry_msgs::PoseStamped this_pose_stamped;
    nav_msgs::Path path;
    path.header.stamp=ros::Time::now();
    path.header.frame_id="odom";
    for(size_t i=0;i<pointSet.size();i++)
    {
        this_pose_stamped.pose.position.x = pointSet[i].x;
        this_pose_stamped.pose.position.y = pointSet[i].y;
        
        this_pose_stamped.header.stamp=ros::Time::now();
        this_pose_stamped.header.frame_id="odom";
        path.poses.push_back(this_pose_stamped);        
    }
    pub.publish(path);
}
void showPoint(point2d &p,ros::Publisher & pub)
{
    geometry_msgs::PointStamped gp;
    gp.header.frame_id="odom";
    gp.header.stamp=ros::Time::now();
    gp.point.x=p.x;
    gp.point.y=p.y;
    gp.point.z=0;
    pub.publish(gp);
}
void showPose(vector<point2d> p,vector<double> yaw,ros::Publisher& pub)
{
    geometry_msgs::PoseArray gpsa;
    gpsa.header.frame_id="odom";
    gpsa.header.stamp=ros::Time::now();
    gpsa.poses.resize(p.size());
    for(size_t i=0;i<p.size();i++)
    {
        gpsa.poses[i].position.x=p[i].x;
        gpsa.poses[i].position.y=p[i].y;
        gpsa.poses[i].position.z=0;
        gpsa.poses[i].orientation=tf::createQuaternionMsgFromYaw(yaw[i]);
    }
    pub.publish(gpsa);
}