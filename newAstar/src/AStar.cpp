#include "AStar.h"
using namespace PT;

AStar::AStar()
{
    minR = -1;
    start.pose = -1;
    aim.pose = -1;
    length = 1;
    maxda = 2 * asin(length / (2 * minR));
    perThetas = 2;
    astern = 0;
}
void AStar::setMap(map _map)
{
    this->_map = _map;
    map_r = _map.size();
    map_c = _map[0].size();
}
void AStar::setMinR(float minR)
{
    this->minR = minR;
    maxda = 2 * asin(length / (2 * minR));
}
void AStar::setNewAim(pxy start, pxy aim)
{
    this->start = start;
    this->aim = aim;
}
bool AStar::isInit()
{
    if (minR = -1)
        return false;
    if (start.pose = -1)
        return false;
    if (_map.empty())
        return false;
    cout << "there have some data without init" << endl;
    return true;
}
void PT::legalizeAngle(float& angle)
{
    while (angle < 0 || angle >= M_PI * 2) {
        if (angle < 0)
            angle += M_PI * 2;
        if (angle >= M_PI * 2)
            angle -= M_PI * 2;
    }
}
float PT::absf(float s)
{
    if (s < 0)
        return -s;
    return s;
}
float AStar::getPredictCost(float x, float y, float th, float th_l)
{
    float dis = sqrtf((aim.xy.x - x) * (aim.xy.x - x) + (aim.xy.y - y) * (aim.xy.y - y));
    return dis + absAngle(th, th_l) * 0.001;
}

vector<maze_s> AStar::expand(maze_s cur)
{
    vector<maze_s> res;
    float g = cur.g;
    float x = cur.x;
    float y = cur.y;
    float theta = cur.theta;

    float g2 = g + _map[idx(y)][idx(x)]; //正向开车代价系数为1
    float asternG2 = g + _map[idx(y)][idx(x)] * 2; //反向开车代价系数为2
    float asternTimes = cur.asternTimes;
    int dir = cur.dir;
    length = _planfunc(pxy(point(x, y), theta))[0];
    maxda = 2 * asin(length / (2 * minR));
    /* cout << "step " << length << "maxda " << maxda << endl
         << "sin " << length / (2 * minR) << endl;*/
    float minA = theta - maxda;
    float maxA = theta + maxda;
    float step = maxda / (2 * perThetas);
    //cout << "theta" << theta << "*" << endl;
    for (float i = minA; i < maxA; i += step) {
        float x2 = x + cos(theta) * length;
        float y2 = y + sin(theta) * length; //theta指的是在当前格子的当前速度，不过换成i相差一步，哪个好像都可以
        float theta2 = i;
        maze_s state2;
        legalizeAngle(theta2);
        state2.asternTimes = asternTimes;
        state2.dir = 1;
        state2.f = g2 + getPredictCost(x2, y2, theta2, theta);
        state2.g = g2;
        state2.theta = theta2;
        state2.x = x2;
        state2.y = y2;
        res.push_back(state2);
        //cout << theta2 << endl;
        if (astern && asternTimes < 2) {
            state2.x = x - cos(theta) * length;
            state2.y = y - sin(theta) * length;
            if (dir == 1)
                state2.asternTimes = asternTimes + 1;
            else
                state2.asternTimes = asternTimes;
            res.push_back(state2);
        }
    }

    return res;
}
float PT::degree2rad(float s)
{
    return s / 180.0 * M_PI;
}
float PT::rad2degree(float s)
{
    return s / M_PI * 180;
}
float PT::absAngle(float a1, float a2)
{
    return min<float>(2 * M_PI - absf(a1 - a2), absf(a1 - a2));
}
bool AStar::CloseInsert(vector<vector<list<close_s>>>& closed, int x, int y, float angle, float come_x, float come_y, float come_angle)
{
    close_s temp;
    temp.angle = angle;
    temp.cx = come_x;
    temp.cy = come_y;
    temp.cangle = come_angle;
    for (list<close_s>::iterator it = closed[y][x].begin(); it != closed[y][x].end(); it++) {
        if (absAngle(it->angle, angle) < minDifferAngle) {
            return false;
        }
        if (it->angle > angle) {
            closed[y][x].insert(it, temp);
            return true;
        }
    }
    closed[y][x].insert(closed[y][x].end(), temp);
    return true;
}
pxy AStar::getPxyFromClosed(vector<vector<list<close_s>>>& closed, pxy p)
{
    pxy res;
    for (list<close_s>::iterator it = closed[idx(p.xy.y)][idx(p.xy.x)].begin();
         it != closed[idx(p.xy.y)][idx(p.xy.x)].end(); it++) {
        if (absAngle(p.pose, it->angle) < minDifferAngle) {
            res.pose = it->cangle;
            res.xy.x = it->cx;
            res.xy.y = it->cy;
            return res;
        }
    }
    return res;
}
bool AStar::isValid(int x, int y)
{
    if (x < 0 || y < 0)
        return false;
    if (x >= map_c || y >= map_r)
        return false;
    if (_map[y][x] == -1)
        return false;
    return true;
}
path AStar::getPath()
{
    vector<vector<list<close_s>>> closed(
        map_r,
        vector<list<close_s>>(map_c, list<close_s>()));
    CloseInsert(closed, start.xy.x, start.xy.y, start.pose, 0, 0, 0);
    maze_s state;

    state.g = 0;
    state.x = start.xy.x;
    state.y = start.xy.y;
    state.f = 0 + getPredictCost(state.x, state.y, start.pose, start.pose);
    state.theta = start.pose;
    state.dir = 1;
    state.asternTimes = 0;
    int total_closed = 1;
    std::list<maze_s> opened = { state };
    bool finshed = false;
    while (!opened.empty()) {
        maze_s current = opened.front(); // grab first elment
        opened.erase(opened.begin()); // pop first element

        float x = current.x;
        float y = current.y;
        double thetaC = current.theta;
        if (total_closed > 20000) {
            cout << "can not find the path in 20000 times" << endl;
            finshed = true;
        }
        if (finshed)
            break;
        if (idx(x) == idx(aim.xy.x) && idx(y) == idx(aim.xy.y)
            && absAngle(thetaC, aim.pose) < degree2rad(5)) {
            cout << "found path to goal in " << total_closed << " expansions" << endl;
            path res;
            //***********
            pxy reTemp;
            reTemp.xy.x = x;
            reTemp.xy.y = y;
            reTemp.pose = thetaC;
            res.insert(res.begin(), reTemp);
            do {
                reTemp = getPxyFromClosed(closed, reTemp);
                res.insert(res.begin(), reTemp);
                if (reTemp.xy.x == 0 && reTemp.xy.y == 0)
                    break;
            } while (
                !(reTemp.xy.x == start.xy.x && reTemp.xy.y == start.xy.y && reTemp.pose == start.pose));
            cout << "done" << endl;
            return res;
        }
        vector<maze_s> next_state = expand(current);

        for (int i = 0; i < next_state.size(); i++) {
            double g2 = next_state[i].g;
            double x2 = next_state[i].x;
            double y2 = next_state[i].y;
            double theta2 = next_state[i].theta;
            if (!isValid(idx(x2), idx(y2))) {
                continue;
            }

            if (CloseInsert(closed, x2, y2, theta2, current.x, current.y, current.theta)) {
                std::list<maze_s>::iterator begin;
                for (begin = opened.begin(); begin != opened.end(); begin++) {
                    if (begin->f >= next_state[i].f)
                        break;
                }
                // opened.push_back(next_state[i]);
                opened.insert(begin, next_state[i]);
                total_closed += 1;
            }
        }
    }
    cout << "error done " << total_closed << endl;
    return path();
}

float PT::getDis(point s)
{
    return sqrtf(s.x * s.x + s.y * s.y);
}
float PT::getDis(float x, float y)
{
    return sqrt(x * x + y * y);
}
float PT::getDis(point p1, point p2)
{
    return PT::getDis(p2 - p1);
}

void AStar::setPlanFunc(planFunc t)
{
    _planfunc = t;
}