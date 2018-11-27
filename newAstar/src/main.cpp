#include "AStar.h"
#include <cmath>
#include <ctime>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <iostream>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <stdio.h>
#include <string>
#include <sys/time.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <vector>

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
    gettimeofday(&tv, NULL);
    return ((unsigned long long)tv.tv_sec * 1000 + (unsigned long long)tv.tv_usec / 1000);
#endif
}

void showPath(path myPath,
    ros::Publisher& pub);
void showPose(vector<point> p, vector<double> yaw, ros::Publisher& pub);
void showPoint(point& p, ros::Publisher& pub);
int main(int argc, char** argv)
{
    cout.precision(3);
    ros::init(argc, argv, "myPathPlanner");
    ros::NodeHandle n;
    ros::Publisher path_pub = n.advertise<nav_msgs::Path>("trajectory", 100);
    ros::Publisher point_pub = n.advertise<geometry_msgs::PointStamped>("aim", 100);
    ros::Publisher pose_pub = n.advertise<geometry_msgs::PoseArray>("mypose", 100);
    tf::TransformBroadcaster broadcaster;
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";
    ros::Rate loop_rate(30);
    bool init = false;
    pxy end_point(
        degree2rad(15.0f),
        point(8, 8)); // end_point(point(0, 15), degree2rad(15)); 有bug
    pxy start_point(point(3, 3), degree2rad(15.0f));

    vector<point> poseP = { start_point.xy, end_point.xy };
    vector<double> yaw = { start_point.pose, end_point.pose };

    PT::ASMapCreator ASM;
    ASM.setNewAim(start_point, end_point, 3); //必须满足R>0.035dis(start,aim)
    ASM.update();

    PT::AStar myPathPlanner;
    //PT::map tempM = vector<vector<float>>(25, vector<float>(25, 1));
    //myPathPlanner.setMap(tempM);
    myPathPlanner.setMap(ASM.getASMap());
    myPathPlanner.setMinR(ASM.getMinR());
    myPathPlanner.setNewAim(ASM.getMapAim()[0], ASM.getMapAim()[1]);
    //myPathPlanner.setNewAim(start_point, end_point);
    myPathPlanner.setPlanFunc(ASM.getPlanFunc());
    auto t1 = GetCurrentTimeMsec();
    path myPath = myPathPlanner.getPath();
    cout << "cost time:" << GetCurrentTimeMsec() - t1 << "ms" << endl;
    cout << myPath[0].xy << " " << rad2degree(myPath[0].pose) << endl
         << myPath[myPath.size() - 1].xy << " " << rad2degree(myPath[myPath.size() - 1].pose) << endl;
    myPath = ASM.getOriginalPath(myPath);
    if (!myPath.size()) {
        cout << "bye bye" << endl;
        return 0;
    }
    int playerIndex = 0;
    while (ros::ok()) {
        showPath(myPath, path_pub);
        showPose(poseP, yaw, pose_pub);

        odom_trans.header.stamp = ros::Time::now();
        odom_trans.transform.translation.x = myPath[playerIndex].xy.x;
        odom_trans.transform.translation.y = myPath[playerIndex].xy.y;
        odom_trans.transform.translation.z = 0.0;
        odom_trans.transform.rotation = tf::createQuaternionMsgFromYaw(myPath[playerIndex].pose); //姿态
        broadcaster.sendTransform(odom_trans);
        ros::spinOnce();
        loop_rate.sleep();
        if (playerIndex < myPath.size() - 1)
            playerIndex++;
        else
            playerIndex = 0;
    }
    return 0;
}
void showPath(path myPath,
    ros::Publisher& pub)
{
    geometry_msgs::PoseStamped this_pose_stamped;
    nav_msgs::Path path;
    path.header.stamp = ros::Time::now();
    path.header.frame_id = "odom";
    for (size_t i = 0; i < myPath.size(); i++) {
        this_pose_stamped.pose.position.x = myPath[i].xy.x;
        this_pose_stamped.pose.position.y = myPath[i].xy.y;

        this_pose_stamped.header.stamp = ros::Time::now();
        this_pose_stamped.header.frame_id = "odom";
        this_pose_stamped.pose.orientation = tf::createQuaternionMsgFromYaw(myPath[i].pose);
        path.poses.push_back(this_pose_stamped);
    }
    pub.publish(path);
}
void showPoint(point& p, ros::Publisher& pub)
{
    geometry_msgs::PointStamped gp;
    gp.header.frame_id = "odom";
    gp.header.stamp = ros::Time::now();
    gp.point.x = p.x;
    gp.point.y = p.y;
    gp.point.z = 0;
    pub.publish(gp);
}
void showPose(vector<point> p, vector<double> yaw, ros::Publisher& pub)
{
    geometry_msgs::PoseArray gpsa;
    gpsa.header.frame_id = "odom";
    gpsa.header.stamp = ros::Time::now();
    gpsa.poses.resize(p.size());
    for (size_t i = 0; i < p.size(); i++) {
        gpsa.poses[i].position.x = p[i].x;
        gpsa.poses[i].position.y = p[i].y;
        gpsa.poses[i].position.z = 0;
        gpsa.poses[i].orientation = tf::createQuaternionMsgFromYaw(yaw[i]);
    }
    pub.publish(gpsa);
}
