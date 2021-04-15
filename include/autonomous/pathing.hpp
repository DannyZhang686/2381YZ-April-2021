#ifndef __PATHING__H__
#define __PATHING__H__
#include <math.h> /* pow */
#include <numeric>
#include <complex>
#include <array>
#include <cmath>
#include <vector>
#include <tuple>

typedef std::complex<double> Point;
typedef std::vector<std::complex<double>> PointList;

PointList DefinePath(Point startPoint, Point endPoint, double startAngle);
PointList InjectPoints(PointList path, double spacing);
PointList smoother(PointList path, double weight_data, double weight_smooth, double tolerance);
const double Curvature(Point currentPos, Point targetPos, double currentOrientation);
long GetClosest(PointList path, Point currentPoint, long previousIndex = 0);
PointList GeneratePath(Point startpoint, Point endpoint, double startAngle, double spacing);
const static Point PointNotFound = Point(-100000, 100000);
std::tuple<long, Point> FindLookAhead(Point currentPos, PointList path, double radius, long maxLookahead, long previousIndex = 0);

Point CheckIntersection(Point circleCenter, Point startPoint, Point endPoint, double radius);


#endif  //!__PATHING__H__