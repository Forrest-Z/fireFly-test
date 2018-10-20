#include"hybridAstar.h"
#include<list>
using namespace PT;
#define idx(var) (int(floor(var)))
double absf(double s)
{
    if(s<0)
    return -s;
    return s;
}
void legalizeAngle(double &angle)
{
    while(angle<0 || angle >=M_PI*2)
    {
        if(angle<0)
            angle+=M_PI*2;
        if(angle>=M_PI*2)
            angle-=M_PI*2;
    }
}
double dAngle(double a,double b)
{
    double re=absf(a-b);
    if(re>M_PI)
        re=M_PI*2-re;
    return re;
}
int theta_to_stack_number(double theta,int NUM_THETA_CELLS)
{
    double new_theta = fmod((theta + 2 * M_PI),(2 * M_PI));
    int stack_number = (int)(round(new_theta * NUM_THETA_CELLS / (2*M_PI))) % NUM_THETA_CELLS;
    return stack_number;
}
bool compare_maze_s(const maze_s & lhs, const maze_s & rhs) {
    return lhs.f < rhs.f;
}

hbf::hbf(double minR,double thetaNum,bool isAstern)
{
    this->astern=isAstern;
    this->minR=minR;
    this->thetas=thetaNum; 
    myMap=NULL; 
}
void hbf::setNewDemand(tpose start,tpose end)
{
    cout<<"OK1"<<endl;
    //this->start=start;
    //this->end=end;
    this->cell_size=getRightParam(start.p,end.p);
    cout<<cell_size<<" "<<
    start.p.x<<" "<<start.p.y<<" "
    <<end.p.x<<" "<<end.p.y<<" "
    <<endl;
    this->LENGTH=cell_size;

    double da=asinf(LENGTH/minR/2);
    
    this->minDtheta[0]=-da;
    this->maxDtheta[0]=da;
    this->minDtheta[1]=M_PI-da;
    this->maxDtheta[1]=M_PI+da;
    all_theta=360.0/da;



    point2d dist=end.p-start.p;
    
    trans=start.p-dist;
    start.p=start.p-trans;

    //cout<<"**"<<start.p.x<<" "<<start.p.y<<" ";
    end.p=end.p-trans;
    roll_theta=atan2(dist.y,dist.x);
    if(roll_theta<0)
        roll_theta+=M_PI*2;
    roll_theta=roll_theta-M_PI/4;
    //cout<<roll_theta<<endl;
    cout<<(start.yaw-=roll_theta)<<" ";
    cout<<(end.yaw-=roll_theta)<<endl;
    rotatePoint(start.p,-roll_theta);
    rotatePoint(end.p,-roll_theta);
    
    

    this->start=start;
    this->end=end;
    int x=start.p.x;
    int y=start.p.y;

    x=abs(x)*3;
    y=abs(y)*3;
    x*=1.0/cell_size;
    y*=1.0/cell_size;
    LENGTH/=cell_size;

    cout<<x<<" "<<y<<" "<<cell_size<<endl;
    if(myMap!=NULL)
   { 
       cout<<"ptr"<<myMap<<endl;
       //delete myMap;
   }
    myMap=new map(y,vector<int>(x,1));
    this->end.p.x*=1.0/cell_size;
    this->end.p.y*=1.0/cell_size;

    this->start.p.x*=1.0/cell_size;
    this->start.p.y*=1.0/cell_size;
    cout<<"OK2"<<endl;
}

hbf::~hbf()
{
    delete myMap;
}

bool hbf::isValid(int x,int y)
{
    if(x<0 || y<0 || x>=(*myMap)[0].size() || y>=(*myMap).size())return false;
    if((*myMap)[y][x]==-1)return false;
    return true;
}
double hbf::getPredictCost(double x,double y,double th)
{
    double dis=sqrt((end.p.x-x)*(end.p.x-x)+(end.p.y-y)*(end.p.y-y));
    return dis+1.0/dis*absf(th-end.yaw);
}
vector<maze_s> hbf::expand(maze_s cur)
{
    vector<maze_s> res;
    double g=cur.g;
    double x=cur.x;
    double y=cur.y;
    double theta=cur.theta;
    if(cur.dir==-1)
        theta=cur.theta-M_PI;//转为正向的角度
    double g2=g+(*myMap)[idx(y)][idx(x)];
    double asternG2=g+(*myMap)[idx(y)][idx(x)]*2;
    double asternTimes=cur.asternTimes;
    int dir=cur.dir;
    for(double i=minDtheta[0];i<=maxDtheta[0];i+=maxDtheta[0]*2/thetas)
    {
        double delta=i;
        double theta2=theta+i;
        double x2=x+cos(theta)*LENGTH;
        double y2=y+sin(theta)*LENGTH;
        maze_s state2;
        legalizeAngle(theta2);
        state2.f=g2+getPredictCost(x2,y2,theta2);
        state2.g=g2;
        state2.x=x2;
        state2.y=y2;
        state2.theta=theta2;
        state2.dir=1;
        state2.asternTimes=asternTimes;
        res.push_back(state2);
    }
    if(astern && asternTimes<2)
    {
        for(double i=minDtheta[0];i<=maxDtheta[0];i+=maxDtheta[0]*2/thetas)
    {
        double delta=i;
        double theta2=theta+i;
        double x2=x-cos(theta)*LENGTH;
        double y2=y-sin(theta)*LENGTH;
        maze_s state2;
        legalizeAngle(theta2);
        state2.f=g2+getPredictCost(x2,y2,theta2);
        state2.g=asternG2;//倒車代價是正向開車的雙倍代價
        state2.x=x2;
        state2.y=y2;
        state2.theta=theta2;
        state2.dir=-1;
        if(dir==1)
            state2.asternTimes=asternTimes+1;
        else
            state2.asternTimes=asternTimes;
        res.push_back(state2);
    }
    }
    return res;
}
tpath hbf::getPath()
{
    vector< vector < vector < int> > > 
    closed(all_theta,vector< vector<int> >((*myMap).size(),vector<int>((*myMap)[0].size(),0)));
    vector< vector <vector<maze_s> > >
    came_from(all_theta, vector<vector<maze_s>>((*myMap).size(), vector<maze_s>((*myMap)[0].size())));
    double theta=start.yaw;


    int stack = theta_to_stack_number(theta,all_theta);
    double g = 0;
    maze_s state;

    state.g = g;
    state.x = start.p.x;
    state.y = start.p.y;
    state.f = g + getPredictCost(state.x, state.y,theta);
    state.theta = theta;
    state.dir=1;
    state.asternTimes=0;

    closed[stack][idx(state.y)][idx(state.x)] = 1;
    came_from[stack][idx(state.y)][idx(state.x)] = state;
    
    int total_closed = 1;
    std::list<maze_s> opened={state};
    bool finished = false;
    long indexS=0;
    while(!opened.empty())
    {
        //sort(opened.begin(), opened.end(), compare_maze_s);
        indexS++;
        maze_s current = opened.front(); //grab first elment
        opened.erase(opened.begin()); //pop first element

        int x = current.x;
        int y = current.y;
        double thetaC=current.theta;

        if(idx(x) == idx(end.p.x) && idx(y) == idx(end.p.y) 
        &&  dAngle(thetaC,end.yaw)<degree2rad(5) )
        {
        cout << "found path to goal in " << total_closed << " expansions" << endl;
        tpath res;
        //***********
        point2d rotaTempPoint;
        rotaTempPoint=point2d(end.p.x,end.p.y);
        rotatePoint(rotaTempPoint,roll_theta);
        rotaTempPoint.x=rotaTempPoint.x*cell_size;
        rotaTempPoint.y=rotaTempPoint.y*cell_size;
        rotaTempPoint=rotaTempPoint+trans;

        res.pointS.insert(res.pointS.begin(),rotaTempPoint);
        res.yaw.insert(res.yaw.begin(),end.yaw+roll_theta);


        int temp_stack=theta_to_stack_number(end.yaw,all_theta);
        maze_s temp=current;
        do{
            rotaTempPoint=point2d(temp.x,temp.y);
            rotatePoint(rotaTempPoint,roll_theta);
            rotaTempPoint.x=rotaTempPoint.x*cell_size;
            rotaTempPoint.y=rotaTempPoint.y*cell_size;
            rotaTempPoint=rotaTempPoint+trans;

            res.pointS.insert(res.pointS.begin(),rotaTempPoint);
            res.yaw.insert(res.yaw.begin(),temp.theta+roll_theta);

            temp_stack=theta_to_stack_number(temp.theta,all_theta);
            temp=came_from[temp_stack][idx(temp.y)][idx(temp.x)];
            //cout<<temp.x<<" "<<temp.y<<" "<<temp.theta<<endl;
        }while(idx(temp.x)!=idx(start.p.x) || idx(temp.y)!=idx(start.p.y) 
        || temp_stack==theta_to_stack_number(start.yaw,all_theta));
        cout<<"done"<<endl;

        rotaTempPoint=point2d(temp.x,temp.y);
        rotatePoint(rotaTempPoint,roll_theta);
        rotaTempPoint.x=rotaTempPoint.x*cell_size;
        rotaTempPoint.y=rotaTempPoint.y*cell_size;
        rotaTempPoint=rotaTempPoint+trans;        
        res.pointS.insert(res.pointS.begin(),rotaTempPoint);

        res.yaw.insert(res.yaw.begin(),temp.theta+roll_theta);

        return res;
        }
        vector<maze_s> next_state = expand(current);
        
        for(int i = 0; i < next_state.size(); i++)
        {
        double g2 = next_state[i].g;
        double x2 = next_state[i].x;
        double y2 = next_state[i].y;
        double theta2 = next_state[i].theta;
        //cout<<x2<<" "<<y2<<" "<<g2<<" "<<theta2<<endl;
        if(!isValid(idx(x2),idx(y2)))
        {
            //invalid cell
            continue;
        }
        int stack2 = theta_to_stack_number(theta2,all_theta);
        if(closed[stack2][idx(y2)][idx(x2)] == 0)
        {
            std::list<maze_s>::iterator begin;
            for(begin=opened.begin();begin!=opened.end();begin++)
            {
                if(begin->f >=next_state[i].f)
                    break;
            }
            //opened.push_back(next_state[i]);
            opened.insert(begin,next_state[i]);
            closed[stack2][idx(y2)][idx(x2)] = 1;
            came_from[stack2][idx(y2)][idx(x2)] = current;
            total_closed += 1;
        }
        }

    }
    tpath tempRes;
    cout<<"error done "<<total_closed<<endl;
    return tempRes;
}
double PT::getRightParam(point2d a,point2d b)//神奇的函數，還是憑感覺
{
    #define KeyParam 13.3333f//神奇的參數
    double xy=(a.x-b.x)*(a.x-b.x)+(a.y-b.y)*(a.y-b.y);
    xy/=2.0;
    xy*=3;
    cout<<sqrt(xy)/KeyParam<<endl;
    return sqrt(xy)/KeyParam;
}
