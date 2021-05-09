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
const long GetClosest(const PointList path, const Point currentPoint, const long previousIndex = 0);
PointList GeneratePath(Point startpoint, Point endpoint, double startAngle, double spacing);
const static Point PointNotFound = Point(-100000, 100000);
const std::tuple<long, Point> FindLookAhead(const Point currentPos, const PointList path, const double radius, const long maxLookahead, const long previousIndex = 0);

const Point CheckIntersection(const Point circleCenter, const Point startPoint, const Point endPoint, const double radius);
extern const double Inner_Product(const Point A, const Point B);


#endif  //!__PATHING__H__