#ifndef NEW_A_STAR_ROS_H__
#define NEW_A_STAR_ROS_H__
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <algorithm>
#include <cmath>
#include <iostream>
#include <list>
#include <opencv2/core.hpp>
#include <vector>

namespace PT {
using std::cout;
using std::endl;
using std::list;
using std::min;
using std::vector;
#define idx(var) (int(floor(var)))
#define minDifferAngle (M_PI / 180)
//角度统一使用弧度制，[0,2PI)
typedef cv::Point2f point;
typedef std::vector<std::vector<float>> map;

struct pxy {
    float pose;
    point xy;
    pxy(){};
    pxy(float p, point txy)
    {
        xy = txy;
        pose = p;
    }
    pxy(point txy, float p)
    {
        xy = txy;
        pose = p;
    }
};
typedef std::vector<pxy> path;
typedef vector<float> (*planFunc)(pxy now);

struct maze_s {
    float f; //目前的真实代价+到达目标的预估代价
    float g; //到目前为止的真实代价
    float x;
    float y;
    float theta; //此刻姿态
    int dir; //正向开车为1 反向为-1
    int asternTimes; //倒车次数
};
struct close_s {
    float angle;
    float cx;
    float cy;
    float cangle;
};

float absf(float s);
float degree2rad(float s);
float rad2degree(float s);
float absAngle(float a1, float a2);
void legalizeAngle(float& angle);
float getDis(point s);
float getDis(float x, float y);
float getDis(point p1, point p2);
class AStar {

public:
    AStar();
    void setMinR(float minR);
    void setNewAim(pxy start, pxy aim); //假设start和aim已经转化到对角线上
    void setMap(map _map); //假设_map为已经分割符合规则的地图，0为可行 -1为不可行 其他数字为代价系数
    path getPath();
    void setPlanFunc(planFunc);

private:
    bool isInit();
    float getPredictCost(float x, float y, float th, float th_l);
    bool CloseInsert(vector<vector<list<close_s>>>& closed, int x, int y, float angle, float come_x, float come_y, float come_angle);
    pxy getPxyFromClosed(vector<vector<list<close_s>>>& closed, pxy p);
    bool isValid(int x, int y);
    vector<maze_s> expand(maze_s cur);
    map _map;
    float minR;
    pxy start;
    pxy aim;
    int map_r, map_c; //地图的尺寸
    bool astern; //是否允许倒车
    float perThetas; //把一份maxda分割成多少分
    float maxda; //根据length动态计算的值 每步进length长度最大的角度变化 maxda=2*asin(l/(2*minR))
    float length; //动态计算的变量
    planFunc _planfunc;
};

class ASMapCreator {
#define sqrtCellSize 29
#define externDis 7
public:
    void setNewAim(pxy start, pxy aim, float minR);
    void setNewMap(map _map); //暂时不用
    bool update();
    planFunc getPlanFunc();
    map getASMap();
    path getOriginalPath(path p);
    vector<pxy> getMapAim();
    float getMinR();

private:
    Eigen::Isometry3f T;
    float s; //缩放因子
    map ASMap;
    map _map; //暂时不用
    pxy start;
    pxy aim;
    float minR;
    pxy MapStart;
    pxy MapAim;
    float dTheta;
    float newMinR;
};
} // namespace PT
#endif