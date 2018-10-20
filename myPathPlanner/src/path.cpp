#include"path.h"
namespace PT
{
    void rotatePoint(point2d &p,double sita)
    {
        point2d tempP;
        tempP.x=cosf(sita)*p.x-sinf(sita)*p.y;
        tempP.y=sinf(sita)*p.x+cosf(sita)*p.y;
        p=tempP;
    }
    vector<point2d> getCircleCenter(tpose pos,double R)
    {
        point2d temp=pos.p;
        pos.p.x=cosf(pos.yaw)*R;
        pos.p.y=sinf(pos.yaw)*R;
        rotatePoint(pos.p,degree2rad(-90));//顺时针
        point2d ClockwiseP=pos.p;
        rotatePoint(pos.p,degree2rad(180));//逆时针
        point2d Counterclockwise=pos.p;
        ClockwiseP=ClockwiseP+temp;
        Counterclockwise=Counterclockwise+temp;
        return {ClockwiseP,Counterclockwise};
    }
    void getCircle(point2d p,double R,double begin,double end,double shift,int isClockwise
    ,int nums,vector<point2d> &res)
    {
        while(!(begin>=0 && begin<=degree2rad(360)))
        {
            if(begin<0)
                begin+=degree2rad(360);
            if(begin>=degree2rad(360))
                begin-=degree2rad(360);
        }
        while(!(end>=0 && end<=degree2rad(360)))
        {
            if(end<0)
                end+=degree2rad(360);
            if(end>=degree2rad(360))
                end-=degree2rad(360);
        }
        double perSita=0;
        if(isClockwise>0)
        {
            if(begin<=end)
                perSita=(begin-end+degree2rad(360))/nums;
            else
                perSita=(begin-end)/nums;
            perSita=-perSita;
        }
        else
        {
            if(begin<=end)
                perSita=(end-begin)/nums;
            else
                perSita=(end-begin+degree2rad(360))/nums;
        }
        for(size_t i=0;i<nums;i++)
        {
            point2d temp;
            temp.x=R*cosf(begin+perSita*i+shift)+p.x;
            temp.y=R*sinf(begin+perSita*i+shift)+p.y;
            res.push_back(temp);
        }
    }
    point2d getCirclePoint(point2d p,double R,double sita,double shift)
    {
        point2d res;
        res.x=R*cosf(sita+shift)+p.x;
        res.y=R*sinf(sita+shift)+p.y;
        return res; 
    }

    tpath getPath(tpose pos1,tpose pos2,double minR)
    {
        cout<<pos1.p.x<<" "<<pos1.p.y<<" "<<pos1.yaw<<" "<<endl;
        cout<<pos2.p.x<<" "<<pos2.p.y<<" "<<pos2.yaw<<" "<<endl;
        cout<<minR<<endl;
        double shift_sita=atan2(pos2.p.y-pos1.p.y,pos2.p.x-pos1.p.x);
        point2d ref_point=pos1.p;
        pos1.p=pos1.p-ref_point;
        pos2.p=pos2.p-ref_point;
        rotatePoint(pos1.p,-shift_sita);
        rotatePoint(pos2.p,-shift_sita);
        pos1.yaw=pos1.yaw-shift_sita;
        pos2.yaw=pos2.yaw-shift_sita;
        vector<point2d> circle1=getCircleCenter(pos1,minR);//顺时针 逆时针
        vector<point2d> circle2=getCircleCenter(pos2,minR);
        
        tpath res;

        vector<double> X,Y;
        point2d begin_temp,end_temp;
        tk::spline s;
        
        //正向開車去目的地 1
        getCircle(circle1[0],minR,degree2rad(90),degree2rad(90)-pos1.yaw,pos1.yaw,1,360,res.pointS);//正向开车
        for(int i=0;i<res.pointS.size()-1;i++)
        {
            res.yaw.push_back(atan2(res.pointS[i+1].y-res.pointS[i].y,res.pointS[i+1].x-res.pointS[i].x));
        }
        res.yaw.push_back(0);
        begin_temp=getCirclePoint(circle1[0],minR,degree2rad(90)-pos1.yaw,pos1.yaw);
        end_temp=getCirclePoint(circle2[0],minR,degree2rad(90)-pos2.yaw,pos2.yaw);
        X={begin_temp.x,begin_temp.x+0.1,end_temp.x-0.1,end_temp.x};
        Y={begin_temp.y,begin_temp.y,end_temp.y,end_temp.y};
        s.set_points(X,Y);
        for(double x=begin_temp.x;x<end_temp.x;x+=0.1)
            res.pointS.push_back(point2d(x,s(x)));
        for(int i=res.yaw.size();i<res.pointS.size()-1;i++)
        {
            res.yaw.push_back(atan2(res.pointS[i+1].y-res.pointS[i].y,res.pointS[i+1].x-res.pointS[i].x));
        }
        res.yaw.push_back(0);
        getCircle(circle2[0],minR,degree2rad(90)-pos2.yaw,degree2rad(90),pos2.yaw,1,360,res.pointS); 
        for(int i=res.yaw.size();i<res.pointS.size()-1;i++)
        {
            res.yaw.push_back(atan2(res.pointS[i+1].y-res.pointS[i].y,res.pointS[i+1].x-res.pointS[i].x));
        }
        res.yaw.push_back(pos2.yaw);

        for(int i=0;i<res.pointS.size();i++)
        {
            rotatePoint(res.pointS[i],shift_sita);
            res.pointS[i]=res.pointS[i]+ref_point;
            res.yaw[i]+=shift_sita;
        }
        return res;
    }

    void pathPlanner::setConstraints(tpose pos1,tpose pos2,double minR)
    {
        this->pos1=pos1;
        this->pos2=pos2;
        this->minR=minR;
        shift_theta=atan2(pos2.p.y-pos1.p.y,pos2.p.x-pos1.p.x);
        CircleCenter1=getCircleCenter(pos1,minR);
        CircleCenter2=getCircleCenter(pos2,minR);
        redress1.clear();
        redress2.clear();

        redress1.push_back(getCirclePoint(CircleCenter1[0],minR,shift_theta,0));
        redress1.push_back(getCirclePoint(CircleCenter1[1],minR,shift_theta,0));

        redress2.push_back(getCirclePoint(CircleCenter2[0],minR,shift_theta,0));
        redress2.push_back(getCirclePoint(CircleCenter2[1],minR,shift_theta,0));
        As=new hbf(minR,1);
    }
    tpath pathPlanner::getOptimalPath()
    {
        point2d line=pos2.p-pos1.p;
        double line_normal=sqrt(line.x*line.x+line.y*line.y);
        if(line_normal<=2*minR)
        {
            cout<<1<<endl;
            return getAstarPath();
        }
        if(line_normal>4*minR)
        {
            cout<<2<<endl;
            return getEightPath();
        }    
        cout<<3<<endl;
        tpath res;
        getCircle(CircleCenter1[0],minR,pos1.yaw+M_PI_2,shift_theta+M_PI_2,0,1,360,res.pointS);
        for(int i=0;i<res.pointS.size()-1;i++)
        {
            res.yaw.push_back(atan2(res.pointS[i+1].y-res.pointS[i].y,res.pointS[i+1].x-res.pointS[i].x));
        }
        res.yaw.push_back(0);
        point2d begin_temp=getCirclePoint(CircleCenter1[0],minR,shift_theta+M_PI_2,0);
        tpose begin_pos;
        begin_pos.p=begin_temp;
        begin_pos.yaw=shift_theta;
        As->setNewDemand(begin_pos,pos2);
        tpath res2=As->getPath();
        for(int i=0;i<res2.pointS.size();i++)
        {
            res.pointS.push_back(res2.pointS[i]);
            res.yaw.push_back(res2.yaw[i]);
        } 
        return res;

    }

    tpath pathPlanner::getAstarPath()
    {
        As->setNewDemand(pos1,pos2);
        return As->getPath();
    }
    tpath pathPlanner::getEightPath()
    {
        return getPath(pos1,pos2,minR);
    }
    pathPlanner::~pathPlanner()
    {
        delete As;
    }
}
