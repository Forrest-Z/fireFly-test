#ifndef PATH_H__
#define PATH_H__

#include<iostream>
#include"spline.h"
#include<cmath>
#include<vector>
#include"hybridAstar.h"

using std::cout;
using std::endl;
using std::vector;


namespace PT
{

class pathPlanner{
public:
    ~pathPlanner();
    //返回一条从Pos1到pos2的路径
    //minR为最小拐弯半径
    //cell_size为格子大小
    void setConstraints(tpose pos1,tpose pos2,double minR);
    tpath getAstarPath();
    tpath getEightPath();
    tpath getOptimalPath();
private:
    double minR;
    tpose pos1;
    tpose pos2;
    vector<point2d> redress1;
    vector<point2d> redress2;
    double shift_theta;
    vector<point2d> CircleCenter1;//顺时针 逆时针
    vector<point2d> CircleCenter2;
    tpath AstarPath;
    vector<tpath> allEightPath;
    hbf *As;
};

}

#endif