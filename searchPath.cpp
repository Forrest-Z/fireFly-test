#include<iostream>
#include<list>
#include<vector>
#include<stack>
using namespace std;
struct point{
    int x=0;
    int y=0;
    int g=0;//最终
    int k=0;//k代表的是到当前的真实代价
    list<point>::iterator pre;
    point(int _x=0,int _y=0,int _g=0)
    {
        x=_x;
        y=_y;
        g=_g;
    }
    friend ostream& operator<<(ostream&os,point p)
    {
        os<<"<"<<p.x<<","<<p.y<<","<<p.g<<">";
        return os;
    }
};
#define M 10
#define N 10
char map[M][N]={{0,0,0,-1,0,0,0,0,0,0},
                {0,100,0,-1,0,0,0,0,0,0},
                {0,100,0,-1,0,0,0,0,0,0},
                {0,100,0,0,0,0,0,0,0,0},
                {0,100,0,-1,0,0,0,0,0,0},
                {0,100,0,-1,0,0,0,0,0,0},
               {0,100,0,-1,0,0,0,0,0,0},
               {0,100,0,-1,0,0,0,0,0,0},
               {0,100,0,-1,0,0,0,0,0,0},
               {0,100,0,-1,0,0,0,0,0,0}};
list<point> openlist;
list<point> closelist;
bool isDeal(point p)
{
    if(p.x<0 || p.y<0 || p.x>=N || p.y>=M)
        return false;
    if(map[p.y][p.x]==-1)
        return false;
    for(auto it=closelist.begin();it!=closelist.end();it++)
    {
        if((*it).x==p.x &&(*it).y==p.y)
            return false;
    }
    for(auto it=openlist.begin();it!=openlist.end();it++)
    {
        if((*it).x==p.x &&(*it).y==p.y)
            return false;
    }
    return true;
}
bool isInOpen(point p,list<point>::iterator &re)
{
    for(auto it=openlist.begin();it!=openlist.end();it++)
    {
        if((*it).x==p.x &&(*it).y==p.y)
        {
            re=it;
            return true;
        }
    }
    return false;

}

int dirr[4]={-1,1,0,0};
int dirc[4]={0,0,-1,1};
stack<point> getPath(point begin,point end)//bfs
{
    int costTime=0;
    openlist.clear();
    closelist.clear();
    stack<point> re;
    if(begin.x==end.x && begin.y==end.y)
    {
        re.push(begin);
        return re;
    }
    begin.g=0;
    openlist.push_back(begin);
    while(!openlist.empty())
    {
        point nowP=openlist.front();
        openlist.pop_front();
        closelist.push_back(nowP);
        list<point>::iterator nowIt=closelist.end();
        --nowIt;
        for(int i=0;i<4;i++)
        {
            point temp;
            temp.x=nowP.x+dirc[i];
            temp.y=nowP.y+dirr[i];
            temp.g=nowP.g+1;
            temp.pre=nowIt;
            if(isDeal(temp))
            {
                costTime++;
                list<point>::iterator tempIt;
                for(tempIt=openlist.begin();tempIt!=openlist.end();tempIt++)
                {
                    if((*tempIt).g>temp.g)
                        break;
                }
                openlist.insert(tempIt,temp);
            }
            if(temp.x==end.x && temp.y==end.y)
            {

                list<point>::iterator reIt=temp.pre;
                re.push(temp);
                while(!((*reIt).x==begin.x && (*reIt).y==begin.y))
                {
                    re.push(*reIt);
                    reIt=(*reIt).pre;
                }
                re.push(*reIt);
                cout<<"costTime:"<<costTime<<endl;
                return re;
            }
        }
    }
    cout<<"costTime:"<<costTime<<endl;
    return re;
}
stack<point> getPath2(point begin,point end)//djikstra
{
    int costTime=0;
    openlist.clear();
    closelist.clear();
    stack<point> re;
    if(begin.x==end.x && begin.y==end.y)
    {
        re.push(begin);
        return re;
    }
    begin.g=0;
    openlist.push_back(begin);
    while(!openlist.empty())
    {
        point nowP=openlist.front();
        openlist.pop_front();
        closelist.push_back(nowP);
        list<point>::iterator nowIt=closelist.end();
        --nowIt;
        for(int i=0;i<4;i++)
        {
            point temp;
            temp.x=nowP.x+dirc[i];
            temp.y=nowP.y+dirr[i];
            temp.g=nowP.g+map[temp.y][temp.x];
            temp.pre=nowIt;
            list<point>::iterator openlistTempIt;
            if(isDeal(temp))
            {
                costTime++;
                list<point>::iterator tempIt;
                for(tempIt=openlist.begin();tempIt!=openlist.end();tempIt++)
                {
                    if((*tempIt).g>temp.g)
                        break;
                }
                openlist.insert(tempIt,temp);
            }
            else if(isInOpen(temp,openlistTempIt))
            {
                if(temp.g<(*openlistTempIt).g)
                {
                    costTime++;
                    openlist.erase(openlistTempIt);
                    list<point>::iterator tempIt;
                    for(tempIt=openlist.begin();tempIt!=openlist.end();tempIt++)
                    {
                        if((*tempIt).g>temp.g)
                            break;
                    }
                    openlist.insert(tempIt,temp);
                }
            }

            if(temp.x==end.x && temp.y==end.y)
            {

                list<point>::iterator reIt=temp.pre;
                re.push(temp);
                while(!((*reIt).x==begin.x && (*reIt).y==begin.y))
                {
                    re.push(*reIt);
                    reIt=(*reIt).pre;
                }
                re.push(*reIt);
                cout<<"costTime:"<<costTime<<endl;
                return re;
            }
        }
    }
    cout<<"costTime:"<<costTime<<endl;
    return re;
}

stack<point> getPath3(point begin,point end)//djikstra
{
    int costTime=0;
    openlist.clear();
    closelist.clear();
    stack<point> re;
    if(begin.x==end.x && begin.y==end.y)
    {
        re.push(begin);
        return re;
    }
    begin.g=abs(begin.x-end.x)+abs(begin.y-end.y);
    begin.k=0;
    openlist.push_back(begin);
    while(!openlist.empty())
    {
        point nowP=openlist.front();
        openlist.pop_front();
        closelist.push_back(nowP);
        list<point>::iterator nowIt=closelist.end();
        --nowIt;
        for(int i=0;i<4;i++)
        {
            point temp;
            temp.x=nowP.x+dirc[i];
            temp.y=nowP.y+dirr[i];
            temp.g=nowP.k+map[temp.y][temp.x]+abs(temp.x-end.x)+abs(temp.y-end.y);
            temp.k=nowP.k+map[temp.y][temp.x];
            temp.pre=nowIt;
            list<point>::iterator openlistTempIt;
            if(isDeal(temp))
            {
                costTime++;
                list<point>::iterator tempIt;
                for(tempIt=openlist.begin();tempIt!=openlist.end();tempIt++)
                {
                    if((*tempIt).g>temp.g)
                        break;
                }
                openlist.insert(tempIt,temp);
            }
            else if(isInOpen(temp,openlistTempIt))
            {
                if(temp.g<(*openlistTempIt).g)
                {
                    costTime++;
                    openlist.erase(openlistTempIt);
                    list<point>::iterator tempIt;
                    for(tempIt=openlist.begin();tempIt!=openlist.end();tempIt++)
                    {
                        if((*tempIt).g>temp.g)
                            break;
                    }
                    openlist.insert(tempIt,temp);
                }
            }

            if(temp.x==end.x && temp.y==end.y)
            {

                list<point>::iterator reIt=temp.pre;
                re.push(temp);
                while(!((*reIt).x==begin.x && (*reIt).y==begin.y))
                {
                    re.push(*reIt);
                    reIt=(*reIt).pre;
                }
                re.push(*reIt);
                cout<<"costTime:"<<costTime<<endl;
                return re;
            }
        }
    }
    cout<<"costTime:"<<costTime<<endl;
    return re;
}

void showMap(char m[M][N],stack<point> path)
{
    char tempM[M][N];
    for(int i=0;i<M;i++)
    {
        for(int j=0;j<N;j++)
        {
            tempM[i][j]=m[i][j]==-1?'1':'0';
        }
    }
    while(!path.empty())
    {
        point p=path.top();
        tempM[p.y][p.x]='*';
        path.pop();
    }
    for(int i=0;i<M;i++)
    {
        for(int j=0;j<N;j++)
            cout<<tempM[i][j]<<" ";
        cout<<endl;
    }
}

int main()
{
    point begin(0,0,0);
    point end(N-1,M-1,0);
    showMap(map,getPath(begin,end));
    cout<<endl;
    showMap(map,getPath2(begin,end));
    cout<<endl;
    showMap(map,getPath3(begin,end));
    return 0;
}
