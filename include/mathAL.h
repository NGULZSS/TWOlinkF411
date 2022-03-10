#ifndef MATHAL_H_
#define MATHAL_H_
#include "array"
#include "math.h"
#include <Arduino.h>
//#include <iostream>
template<typename T>
struct FVector2
{
	T X;
	T Y;
};
typedef FVector2<float> FVector2F;
typedef FVector2<double> FVector2D;
template<typename T>
FVector2F AnyToF(T t)
{
	return { t.x(),t.y() };
}
template<typename T>
FVector2F AnyToF(T x, T y)
{
	return { x,y };
}
template<typename T>
struct FVector3
{
	T X;
	T Y;
	T Z;
};
typedef FVector3<float> FVector3F;
template<typename T>
FVector3F AnyTo3F(T t)
{
	return { t.x(),t.y(),t.z() };
}
namespace MathCalculate
{
   template<typename T>
   int Sign(T &x)
   {
       int a;
       x>=0?a=1:a=-1;
       return a;
   }
}




#endif