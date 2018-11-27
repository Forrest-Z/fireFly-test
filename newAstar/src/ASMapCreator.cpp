#include "AStar.h"
using namespace PT;
pxy pstart;
pxy paim;
vector<float> planFunc1(pxy now)
{
    return { 1.5 };
    /*float ndis = getDis(now.xy, paim.xy);
    float adis = getDis(pstart.xy, paim.xy);
    if (ndis > adis)
        ndis -= adis;
    if (ndis < 1 || ndis > adis - 1)
        return { 1.5f };
    else if (ndis <= adis / 2.0)
        return { ndis * 8.0f / adis * 5.0f + 1.5f };
    else
        return { (adis - ndis) * 8.0f / adis * 5.0f + 1.5f };*/
}
void ASMapCreator::setNewAim(pxy start, pxy aim, float minR) //必须满足R > 0.035dis(start, aim)
{
    this->start = start;
    this->aim = aim;
    this->minR = minR;
}

void ASMapCreator::setNewMap(map _map)
{
    this->_map = _map;
}
bool ASMapCreator::update()
{
    point v = aim.xy - start.xy;
    dTheta = M_PI / 4.0 - atan2f(v.y, v.x);
    Eigen::AngleAxisf rotation_vector(dTheta, Eigen::Vector3f(0, 0, 1));
    T = Eigen::Matrix3f::Identity();
    T.rotate(rotation_vector);
    float externX = getDis(v) * 7.0 / (sqrtf(2.0) * 15);
    T.pretranslate(
        Eigen::Vector3f(externX, externX, 0) - Eigen::Vector3f(start.xy.x, start.xy.y, 0));
    Eigen::Vector3f v1(start.xy.x, start.xy.y, 0);
    Eigen::Vector3f v2(aim.xy.x, aim.xy.y, 0);
    /*cout << T.matrix() << endl;
    cout << v1 << endl
         << v2 << endl;*/
    Eigen::Vector3f mv1 = T * v1;
    Eigen::Vector3f mv2 = T * v2;
    /*cout << mv1 << endl
         << mv2 << endl;*/
    MapStart.xy.x = mv1[0];
    MapStart.xy.y = mv1[1];
    MapStart.pose = start.pose + dTheta;
    legalizeAngle(MapStart.pose);

    MapAim.xy.x = mv2[0];
    MapAim.xy.y = mv2[1];
    MapAim.pose = aim.pose + dTheta;
    legalizeAngle(MapAim.pose);

    float dis1 = getDis(v);
    s = ((sqrtCellSize - externDis * 2) / (dis1)*sqrt(2));
    cout << dis1 << endl
         << s << endl;
    /*cout << MapAim.xy << endl
         << MapStart.xy << endl;*/

    MapAim.xy *= s;
    MapStart.xy *= s; //缩放因子
    newMinR = minR * s;
    /*cout << MapAim.xy << endl
         << MapStart.xy << endl;*/
    pstart = MapStart;
    paim = MapAim;
    //以上是更改姿态到新地图的起始位置
    ASMap = map(sqrtCellSize,
        vector<float>(sqrtCellSize, 1.0));
    cout << paim.xy << endl
         << pstart.xy << endl
         << rad2degree(paim.pose) << endl
         << rad2degree(pstart.pose) << endl
         << ASMap.size() << endl
         << ASMap[0].size() << endl;
    return true;
}
planFunc ASMapCreator::getPlanFunc()
{
    return planFunc1;
}

map ASMapCreator::getASMap()
{
    return ASMap;
}
path ASMapCreator::getOriginalPath(path p)
{
    for (int i = 0; i < p.size(); i++) {
        p[i].xy /= s;
        p[i].pose -= dTheta;
        legalizeAngle(p[i].pose);
        Eigen::Vector3f temp = T.inverse() * Eigen::Vector3f(p[i].xy.x, p[i].xy.y, 0);
        p[i].xy.x = temp[0];
        p[i].xy.y = temp[1];
    }
    return p;
}
vector<pxy> ASMapCreator::getMapAim()
{
    return { MapStart, MapAim };
}
float ASMapCreator::getMinR()
{
    return newMinR;
}