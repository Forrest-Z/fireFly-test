#include<iostream>
#include<ros/ros.h>
#include<tf/transform_broadcaster.h>
#include<string>
#include<tf/tf.h>
#include<vector>
#include<nav_msgs/Path.h>
#include<std_msgs/String.h>
#include<geometry_msgs/Quaternion.h>
#include<geometry_msgs/PoseStamped.h>
#include<geometry_msgs/PointStamped.h>
#include<geometry_msgs/PoseArray.h>
#include"spline.h"
#include"hybridAstar.h"
#include"path.h"
#include<ctime>
#include<sys/time.h>

using namespace std;
using namespace PT;
unsigned long long GetCurrentTimeMsec()
{
#ifdef _WIN32
		struct timeval tv;
		time_t clock;
		struct tm tm;
		SYSTEMTIME wtm;

		GetLocalTime(&wtm);
		tm.tm_year = wtm.wYear - 1900;
		tm.tm_mon = wtm.wMonth - 1;
		tm.tm_mday = wtm.wDay;
		tm.tm_hour = wtm.wHour;
		tm.tm_min = wtm.wMinute;
		tm.tm_sec = wtm.wSecond;
		tm.tm_isdst = -1;
		clock = mktime(&tm);
		tv.tv_sec = clock;
		tv.tv_usec = wtm.wMilliseconds * 1000;
		return ((unsigned long long)tv.tv_sec * 1000 + (unsigned long long)tv.tv_usec / 1000);
#else
        struct timeval tv;
        gettimeofday(&tv,NULL);
        return ((unsigned long long)tv.tv_sec * 1000 + (unsigned long long)tv.tv_usec / 1000);
#endif
    }



void showPath(vector<point2d> &pointSet,vector<double> &yaw,ros::Publisher &pub);
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
    odom_trans .child_frame_id="base_link";
    ros::Rate loop_rate(30);
    bool init=false;
    tpose end_point(point2d(0,15),degree2rad(15));
    tpose start_point(point2d(0,0),degree2rad(270));


    vector<point2d> poseP={start_point.p,end_point.p};
    vector<double> yaw={start_point.yaw,end_point.yaw};
    PT::pathPlanner myPathPlanner;
    myPathPlanner.setConstraints(start_point,end_point,10);
    tpath path=myPathPlanner.getOptimalPath();

    int playerIndex=0;
    while(ros::ok())
    {
    
        showPath(path.pointS,path.yaw,path_pub);
        showPose(poseP,yaw,pose_pub);
        
        odom_trans.header.stamp=ros::Time::now();
        odom_trans.transform.translation.x=path.pointS[playerIndex].x;
        odom_trans.transform.translation.y=path.pointS[playerIndex].y;
        odom_trans.transform.translation.z=0.0;
        odom_trans.transform.rotation=tf::createQuaternionMsgFromYaw(path.yaw[playerIndex]);//姿态
        broadcaster.sendTransform(odom_trans);
        ros::spinOnce();
        loop_rate.sleep();
        if(playerIndex<path.pointS.size()-1)
            playerIndex++;
        else
            playerIndex=0;
    }
    return 0;
}
void showPath(vector<point2d> &pointSet,vector<double> &yaw,ros::Publisher &pub)
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

        this_pose_stamped.pose.orientation=tf::createQuaternionMsgFromYaw(yaw[i]);
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
