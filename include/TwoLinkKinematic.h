#ifndef TWOLINKKINEMATIC_H_
#define TWOLINKKINEMATIC_H_
#include <Arduino.h>
#include "array"
#include "math.h"
#include "m_servo.h"
#include "mathAL.h"

class TLKinematic: public m_servo
{
   public:
     TLKinematic():m_servo(0){}
     void PoseMatrixToTCP(float PoseMatrix[4][4], std::array<float, 6> & tcp);
     bool TLForward(const float *q,std::array<float, 6> & tcp);
     bool TLInverse(std::array<float, 6> & tcp,float Targetq[2],float LastQ[2]);
     void TLJacobi(float *q,float *v,float J[2][2]);
     void GetMoveLq();
     bool MoveJ(float Aimq[2],float speed,float acc,m_servo ms);
     bool MoveL(float Aimpose[2],float speed,float acc);
     bool MoveC(float Aimq[2],float speed,float acc);
     bool Bezier();//贝塞尔曲线运动
     bool LineACircle(float Pose[][6],float speed,float acc );//直线圆弧过度 输入任意多个坐标点
   private:
   const float L1=90;
   const float L2=90;
};





#endif