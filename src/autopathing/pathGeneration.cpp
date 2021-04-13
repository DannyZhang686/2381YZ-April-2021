#include "main.h"
#include "opcontrol.h"
#include "motors.h"
#include "utilities.h"
#include "control/motor_controller.hpp"

#include "autonomous.h"
#include "utilities.h"   
#include "pid.h"
#include "autonomous/auto_task.hpp"

#include "control/pid.hpp"
#include "globals.hpp"
#include <math.h> /* pow */
#include <numeric>
#include <complex>
#include <array>
#include <cmath>
#include <vector>

#include "pathing.hpp"


using namespace std;
using namespace pros;
using namespace std::complex_literals;


PointList DefinePath(Point startPoint, Point endPoint, double startAngle)
{
    // disp between the 2 points
    Point disp = endPoint - startPoint;
    double distance = abs(disp) / 4;
    Point midwayPoint = startPoint + distance * Point(cos(startAngle), sin(startAngle));

    return {startPoint, midwayPoint, endPoint};
}

PointList InjectPoints(PointList path, double spacing)
{
    // spacing is space between 2 points that r injected
    PointList newPointList = {path[0]};
    // additional points for line 1: solve for linear equation between both points 1 and 2
    // from there, you will get both the slope and y-intercept (which will be common for all points on this line)
    auto Disp = path[1] - path[0];
    auto distance1 = abs(Disp);

    for (double i = 0; i < distance1; i += spacing)
    {
        Point newPoint = path[0] + Disp * i / distance1;
        newPointList.emplace_back(newPoint);
    }
    // additional points for line 1: solve for linear equation between both points 1 and 2
    // from there, you will get both the slope and y-intercept (which will be common for all points on this line)
    auto Disp2 = path[2] - path[1];
    auto distance2 = abs(Disp2);

    for (double i = 0; i < distance2; i += spacing)
    {
        Point newPoint = path[1] + Disp2 * i / distance2;
        newPointList.emplace_back(newPoint);
    }
    newPointList.emplace_back(path[2]);

    return newPointList;
}

PointList smoother(PointList path, double weight_data, double weight_smooth, double tolerance)
{
    //copy array

    PointList newPath = PointList(path);
    double change = tolerance;

    while (change >= tolerance)
    {
        change = 0.0;

        for (int i = 1; i < path.size() - 1; i++)
        {
            double auxR = newPath[i].real();
            newPath[i] += Point((weight_data * (path[i] - newPath[i]) + weight_smooth * (newPath[i - 1] + newPath[i + 1] - (2.0 * newPath[i]))).real(), 0);
            change += abs(auxR - newPath[i].real());

            double auxI = newPath[i].imag();
            newPath[i] += Point(0, (weight_data * (path[i] - newPath[i]) + weight_smooth * (newPath[i - 1] + newPath[i + 1] - (2.0 * newPath[i]))).imag());
            change += abs(auxI - newPath[i].imag());
        }
    }

    return newPath;
}


const double Curvature(Point currentPos, Point targetPos, double currentOrientation)
{
    Point disp = targetPos - currentPos;
    double arg = std::arg(disp);

    double dist = abs(targetPos - currentPos);

    if ((arg == currentOrientation))
    {
        return NAN;
    }

    double angleTargetCurrentCenter = arg;
    double angleToTravel = remainder(currentOrientation - angleTargetCurrentCenter, 2 * M_PI);

    //sin(angle) = dist / radius, radius = dist / sin(angle)
    return dist / (2 * sin(angleToTravel));
}

long GetClosest(PointList path, Point currentPoint, long previousIndex)
{
    long double min = -1;
    long minIndex = previousIndex;
    previousIndex = previousIndex - 3;
    if (previousIndex < 0)
    {
        previousIndex = 0;
    }

    for (int i = previousIndex; i < previousIndex + 10; i++)
    {
        if (i == path.size() - 1)
        {
            break;
        }
        if (abs(path[i] - currentPoint) < min || min == -1)
        {
            min = abs(path[i] - currentPoint);
            minIndex = i;
        }
    }
    return minIndex;
}


PointList GeneratePathCirc(Point startpoint, Point endpoint, double startAngle, double spacing) 
{
    startAngle = NormalizeAngle(startAngle);
    // # a0 = startAngle * math.pi
    double a0 = NormalizeAngle(startAngle-0.5* M_PI);
    double distSq = abs((startpoint - endpoint)) * abs((startpoint - endpoint));
    double r = (distSq / (2*(startpoint.real() - endpoint.real())*cos(a0) + 2*(startpoint.imag() - endpoint.imag())*sin(a0)));
    double a = atan2((distSq)*sin(a0) - 2*(startpoint.real() - endpoint.real())*(startpoint.imag()-endpoint.imag())*cos(a0), (distSq)*cos(a0) - 2*(startpoint.real() - endpoint.real())*(startpoint.imag()-endpoint.imag())*sin(a0)) - a0;

    auto center = startpoint - r * exp<double>(1i * a0);
    PointList newPointList = {};

    auto i = 0.0;
    auto angleDiff = NormalizeAngle(a,2);

    while(abs(i/r) < abs(angleDiff))
    {
        auto newPoint = center + r * exp<double>(1i * (a0 + i/r));
        newPointList.emplace_back(newPoint);
        i+=  spacing;
    }
    return newPointList;
}
   

PointList GeneratePath(Point startpoint, Point endpoint, double startAngle, double spacing)
{
    // PointList path = DefinePath(startpoint, endpoint, startAngle);
    // path = InjectPoints(path, spacing);
    // // Second Picture
    // path = smoother(path, 0.1, 0.9, 1);
    PointList path = GeneratePathCirc(startpoint, endpoint, startAngle, spacing);
    return path;
}



Point CheckIntersection(Point circleCenter, Point startPoint, Point endPoint, double radius)
{
    auto lineSegmentVector = endPoint - startPoint;
    auto circleToStartVector = startPoint - circleCenter;

    double a = abs(lineSegmentVector) * abs(lineSegmentVector);
    double b = 2 * (circleToStartVector.real() * lineSegmentVector.real() + circleToStartVector.imag() * lineSegmentVector.imag());
    double c = abs(circleToStartVector) * abs(circleToStartVector) - radius * radius;
    bool root1Intersect = false, root2Intersect = false;

    double discriminant = b * b - 4 * a * c;

    if (discriminant >= 0)
    {
        discriminant = sqrt(discriminant);

        double root1 = (-b - discriminant) / (2 * a);
        double root2 = (-b + discriminant) / (2 * a);
        
        if ((root2 >= 0) && (root2 <= 1))
        {
            //Return immediately; root2 is further along the vector
            //and so is always a preferable return to root1
            auto a = startPoint + root2 * lineSegmentVector;
            auto dist = abs(a - circleCenter);
            s__t(3, "root2 found: " + t__s(root2) + " dist:" + t__s(dist));
            return startPoint + root2 * lineSegmentVector;
        }
        else if ((root1 >= 0) && (root1 <= 1))
        {
            auto a = startPoint + root1 * lineSegmentVector;
            auto dist = abs(a - circleCenter);
            s__t(3, "root1 found: " + t__s(root2) + " dist:" + t__s(dist));
            return startPoint + root1 * lineSegmentVector;
        }
    }
    return PointNotFound;
}
