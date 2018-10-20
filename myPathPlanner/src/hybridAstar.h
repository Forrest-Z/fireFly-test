#ifndef HYBRIDASTAR_H__
#define HYBRIDASTAR_H__
#include<vector>
#include<iostream>
#include<cmath>

using namespace std;
namespace PT{
    struct point2d{
    double x;
    double y;
    point2d(double x=0,double y=0)
    {
        this->x=x;
        this->y=y;
    }
    point2d operator-(point2d b)
    {
        double tx,ty;
        tx=this->x-b.x;
        ty=this->y-b.y;
        return point2d(tx,ty);
    }
    point2d operator+(point2d b)
    {
        double tx,ty;
        tx=this->x+b.x;
        ty=this->y+b.y;
        return point2d(tx,ty);
    }
    
};
struct tpose{
    point2d p;
    double yaw;
    tpose()
    {

    }
    tpose(point2d p,double yaw)
    {
        this->p=p;
        this->yaw=yaw;
    }
};
struct tpath{
    vector<point2d> pointS;
    vector<double> yaw; 
};


inline double degree2rad(double deg)
{
    return deg*M_PI/180.0f;
}
inline double rad2degree(double rad)
{
    return rad/M_PI*180.0f;
}

//把p旋转sita角度
void rotatePoint(point2d &p,double sita);

//返回一个圆上的一个点
//p为圆心，R为半径，sita为圆心角，shift表示从圆心角多少度开始算0度
point2d getCirclePoint(point2d p,double R,double sita,double shift);

//获取一个圆上的点
//p为圆心
//R为半径
//begin为起始点的圆心角
//end为结束点的圆心角
//shift为多少度开始算0度
//isClockwise为方向，1为顺时针，0为逆时针，nums为采样的点的数量
//res为存放结果，注意不会清空该容器
void getCircle(point2d p,double R,double begin,double end,double shift,int isClockwise,int nums,vector<point2d> &res);

//获得一个圆的圆心，并且确保pos的位置在圆上，切pos的方向在该点的切向上
//返回两个圆心坐标，0为顺时针得到，1为逆时针得到
vector<point2d> getCircleCenter(tpose pos,double R);

tpath getPath(tpose pos1,tpose pos2,double minR);


    //-1代表不可行，自然数代表代价
    double getRightParam(point2d a, point2d b);
    typedef vector<vector<int> > map;
    struct maze_s
    {
        double f;
        double g;
        double x;
        double y;
        double theta;
        int dir;//1正向开车 0反向开车
        int asternTimes;
    };
    class hbf
    {
    public:
        hbf(double minR,double thetaNum,bool isAstern=true);  
        void setNewDemand(tpose start,tpose end);
        //void setNewMap(map *searchSpace);
        tpath getPath();
        ~hbf();
    private:
        map *myMap;
        bool isValid(int x,int y);
        double getPredictCost(double x,double y,double th);
        vector<maze_s> expand(maze_s cur);
        tpose start,end;
        point2d trans;
        double roll_theta;
        double LENGTH;
        double minDtheta[2];//0表示正向范围 1表示反向范围
        double maxDtheta[2];
        bool astern;
        int thetas;
        int all_theta;
        double cell_size;
        double minR;
    };
}

#endif