
#include "TwoLinkKinematic.h"
void TLKinematic::PoseMatrixToTCP(float PoseMatrix[4][4], std::array<float, 6> &tcp)
{
    float rot[3][3] = {0};
    float RPY[3] = {0};
    float sy = sqrt(rot[0][0] * rot[0][0] + rot[1][0] * rot[1][0]);
    for (int i = 0; i++; i < 3)
    {
        for (int j = 0; j++; j < 3)
        {
            rot[i][j] = PoseMatrix[i][j];
        }
    }
    if (sy > 1e-6)
    {
        RPY[0] = atan2(rot[2][1], rot[2][2]);
        RPY[1] = atan2(-rot[2][0], sy);
        RPY[2] = atan2(rot[1][0], rot[0][0]);
    }
    else
    {
        //        RPY[0] = atan2(-rot[1][2], rot[1][1]);
        //        RPY[1] = atan2(-rot[2][0], sy);
        //        RPY[2] = 0;

        // edited by xuzhenyu 20190509
        if (rot[2][0] < 0) // beta=90
        {
            RPY[0] = atan2(rot[0][1], rot[1][1]);
            RPY[1] = PI / 2;
            RPY[2] = 0;
        }
        else
        {
            RPY[0] = -atan2(rot[0][1], rot[1][1]);
            RPY[1] = -PI / 2;
            RPY[2] = 0;
        }
    }
    tcp[0] = PoseMatrix[0][3];
    tcp[1] = PoseMatrix[1][3];
    tcp[2] = PoseMatrix[2][3];
    tcp[3] = RPY[0];
    tcp[4] = RPY[1];
    tcp[5] = RPY[2];
}

bool TLKinematic::TLForward(const float *q, std::array<float, 6> &tcp)
{
    float PoseMatrix[4][4] = {0};
    float rotZYX[3][3] = {0};
    if (q[1] > 170 || q[1] < -170)
    {
        return false;
    }
    else
    {
        PoseMatrix[0][0] = cos(q[1] + q[2]);
        PoseMatrix[0][1] = -sin(q[1] + q[2]);
        PoseMatrix[0][2] = 0;
        PoseMatrix[0][3] = L2 * cos(q[1] + q[2]) + L1 * cos(q[1]);

        PoseMatrix[1][0] = sin(q[1] + q[2]);
        PoseMatrix[1][1] = cos(q[1] + q[2]);
        PoseMatrix[1][2] = 0;
        PoseMatrix[1][3] = L2 * sin(q[1] + q[2]) + L1 * sin(q[1]);

        PoseMatrix[2][0] = 0;
        PoseMatrix[2][1] = 0;
        PoseMatrix[2][2] = 1;
        PoseMatrix[2][3] = 0;

        PoseMatrix[3][0] = 0;
        PoseMatrix[3][1] = 0;
        PoseMatrix[3][2] = 0;
        PoseMatrix[3][3] = 1;

        PoseMatrixToTCP(PoseMatrix, tcp);
        return true;
    }
}

bool TLKinematic::TLInverse(std::array<float, 6> &tcp, float Targetq[2], float LastQ[2])
{

    float DistanceToBase = 0;
    float thea3, thea2, thea1;
    float j11, j12, j21, j22;
    DistanceToBase = sqrt(pow(tcp[0], 2) + pow(tcp[1], 2));

    if (DistanceToBase > L1 + L2)
    {
        return false;
    }
    else
    {
        thea3 = acos((pow(L1, 2) + pow(L2, 2) - pow(DistanceToBase, 2)) / (2 * L1 * L2));
        thea2 = (PI - thea3) / 2;
        if (tcp[0] >= 0)
        {
            if (tcp[1] == 0)
            {
                thea1 = PI / 2;
            }
            else
            {
                thea1 = atan(tcp[1] / tcp[0]);
            }
        }
        else
        {
            if (tcp[1] == 0)
            {
                thea1 = -PI / 2;
            }
            else
            {
                thea1 = PI + atan(tcp[1] / tcp[0]);
            }
        }
        j11 = thea1 - thea2;
        j12 = thea1 + thea2;
        j21 = PI - thea3;
        j22 = thea3 - PI;
        if ((j11 - LastQ[0]) < (j21 - LastQ[0]))
        {
            Targetq[0] = j11;
            Targetq[1] = j21;
        }
        else
        {
            Targetq[0] = j12;
            Targetq[1] = j22;
        }
        return true;
    }
}

void TLKinematic::TLJacobi(float *q, float *v, float J[2][2])
{
}
//采用梯形轨迹规划的方式来进行运动
bool TLKinematic::MoveJ(float Aimq[2], float speed, float acc, m_servo ms)
{
    float Nowq[2] = {ms.Para.cur_angle, ms.Para.cur_angle + 3.8};
    static float T = 0.0f;   //控制时间
    static float tc = 0.0f;  //加速变成匀速时间
    static float tf = 0.0f;  //总运动时间
    static float tm = 0.0f;  //匀速运动绝对时间
    static float th = 0.0f; //匀速变减速时间
    static float a[2] = {0}; //关节加速度
    static float v[2] = {0}; //关节速度
    float xc = 0;           //加速段位移
    float xm=0;           //中间段位移
    float maxacc = 0;
    std::array<float, 2> Jerror{0};
    float fabJerror[2]={0};
    std::array<float, 2> accall{0};
    std::array<float, 2> speedall{0};
    float jointmove[2] = {0};
    float jointmoveSpeed[2] = {0};


    std::array<float, 2> outPS{0};
    static bool runi = false;
    static float LastAimq[2] = {0};
    static float MaxMoveJoint = 0;
    static int OtherMoveJoint = 0;
    bool Moverunflag = false;
    static float StartQ[2] = {0};
    if (runi == false || LastAimq[0] != Aimq[0] || LastAimq[1] != Aimq[1])
    {
        for (int i = 0; i < 2; i++)
        {
            Jerror[i] = Aimq[i] - Nowq[i];
            fabJerror[i]=abs(Jerror[i]);
        }
        fabJerror[0]>fabJerror[1]?MaxMoveJoint=fabJerror[0]:MaxMoveJoint=fabJerror[1];
        //运动时间计算
        tc = speed / acc; //加速段时间
        xc = 0.5* acc * pow(tc,2);
        xm = MaxMoveJoint - 2 * xc;
        tm = xm / speed;
        th=tc+tm;
        tf=th+tc;
       //计算其他轴的运动速度和加速度
        for (int i = 0; i < 2; i++)
        {
            accall[i] = Jerror[i]/(tc+tm);
            speedall[i]=accall[i]/tc;
        }
    }
    else
    {
        if (LastAimq[0] == Aimq[0] && LastAimq[1] == Aimq[1])
        {
            T = T + 0.015;
        }
        else
        {
            T = 0.0;
        }
    }
    runi = true;
    //找到需要运动最大的规划角度    以此为基准    保证每个关节同时运动到位置
    if (T <= tc)
    {
        for (int i=0;i<2;i++)
        {
            jointmove[i]=Nowq[2]+0.5*accall[i]*pow(T,2);
        }
    }
    else if (T > tc && T <= (tf - tc))
    {
         for (int i=0;i<2;i++)
        {
            jointmove[i]= Nowq[2]+0.5*accall[i]*pow(tc,2)+speedall[i]*(T-tc); 
        }
    }
    else if (T > (tf - tc) && T <= tf)
    {
         for (int i=0;i<2;i++)
        {
            jointmove[i]= Nowq[2]+0.5*accall[i]*pow(tc,2)+speedall[i]*(tf-tc)+
            speedall[i]*(T-th)+0.5*(-accall[i])*pow((T-th),2); 
        }
    }
    else
    {
         for (int i=0;i<2;i++)
        {
            jointmove[i]= Aimq[i];
        }
    }
    if (Moverunflag == false)
    {
       
    }

    return Moverunflag;
}

bool TLKinematic::MoveL(float Aimpose[2], float speed, float acc)
{
    std::array<float, 6> tcp{0};
    float DistanceLine = 0.0;
    float Nowpose[2] = {0};
    Nowpose[0] = Para.cur_angle;
    float tc = 0.0;   //加速时间点
    float tf = 0.0;   //结束运动时间点
    float tfc = 0.0;  //减速运动时间点
    float Ptc = 0.0;  //加速运动位移
    float Ptf = 0.0;  //总运动位移
    float Ptfc = 0.0; //匀速运动位移
    static float T = 0;
    static float LastAimpose[2] = {0};
    static bool runi = false;
    float InterPoint[2] = {0};
    float InterPointQ[2] = {0};
    static float LastInterPointQ[2] = {0};
    static float StartQ[2] = {0};
    bool Moverunflag = false;
    if (runi == false || LastAimpose[0] != Aimpose[0] || LastAimpose[1] != Aimpose[1])
    {
        T = 0.01;
        StartQ[0] = Nowpose[0];
        StartQ[1] = Nowpose[1];
        DistanceLine = sqrt(pow(Aimpose[0] - Nowpose[0], 2) + pow(Aimpose[1] - Nowpose[1], 2));
        tc = speed / acc;
        Ptc = 0.5 * acc * pow(tc, 2);
        Ptfc = DistanceLine - 2 * Ptc;
        tfc = Ptfc / speed + tc;
        tf = tfc + tc;
    }
    else
    {
        if (LastAimpose[0] == Aimpose[0] && LastAimpose[1] == Aimpose[1])
        {
            T = T + 0.01;
        }
        else
        {
            T = 0.0;
        }
    }
    runi = true;
    if (T <= tc)
    {
        InterPoint[0] = StartQ[0] + (0.5 * acc * pow(T, 2) / DistanceLine) * (Aimpose[0] - Nowpose[0]);
        InterPoint[1] = StartQ[1] + (0.5 * acc * pow(T, 2) / DistanceLine) * (Aimpose[1] - Nowpose[1]);
    }
    else if (T > tc && T <= (tfc))
    {
        InterPoint[0] = StartQ[0] + (Ptc + ((T - tc) * speed) / DistanceLine) * (Aimpose[0] - Nowpose[0]);
        InterPoint[1] = StartQ[1] + (Ptc + ((T - tc) * speed) / DistanceLine) * (Aimpose[1] - Nowpose[1]);
    }
    else if (T > (tfc) && T <= tf)
    {
        InterPoint[0] = StartQ[0] + (DistanceLine - Ptc + speed * T - 0.5 * acc * pow(T, 2) / DistanceLine) * (Aimpose[0] - Nowpose[0]);
        InterPoint[1] = StartQ[1] + (DistanceLine - Ptc + speed * T - 0.5 * acc * pow(T, 2) / DistanceLine) * (Aimpose[1] - Nowpose[1]);
    }
    else
    {
        InterPoint[0] = Aimpose[0];
        InterPoint[1] = Aimpose[0];
        Moverunflag = true;
    }
    if (Moverunflag == false)
    {
        tcp[0] = InterPoint[0];
        tcp[1] = InterPoint[1];
        TLInverse(tcp, InterPointQ, LastInterPointQ);
        set_angle(1, InterPointQ[0], 10);
        LastInterPointQ[0] = InterPointQ[0];
        LastInterPointQ[1] = InterPointQ[1];
    }
    return Moverunflag;
}

bool TLKinematic::MoveC(float Aimq[2], float speed, float acc)
{
}