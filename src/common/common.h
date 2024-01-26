#pragma once
#include <cmath>
#include <map>

#include "prediction_interface.h"
#include "sglog/sglog.h"
#include "struct.h"
namespace jarvis {
namespace prediction_lib {
namespace common {
bool IsInQuadrilateral(const Point2d point_e, const Point2d point_a,
                       const Point2d point_b, const Point2d point_c,
                       const Point2d point_d);
Point2d Vector(const Point2d point_1, const Point2d point_2);
double CrossProduct(const Point2d vector_1, const Point2d vector_2);
bool IsInRoadSection(const MonospacedLaneMsg &lane_msg, const double obs_x,
                     const double obs_y);
double DistancePoint2Line(const Point2d point_a, const Point2d point_b,
                          const Point2d point_p);
Point2d PointVertical(const Point2d point_a, const Point2d point_b,
                      const Point2d point_p);
bool IsIntersectedByLineSection(const Point2d point_a, const Point2d point_b,
                                const Point2d point_c, const Point2d point_d);
bool IsIntersectedSectionAndStraightLine(const Point2d point_a,
                                         const Point2d point_b,
                                         const Point2d section_point_c,
                                         const Point2d section_point_d);
void VehCoord2EnuCoord(const double ego_x, const double ego_y,
                       const double ego_yaw, double &x, double &y);
Point2d PointIntersect(const Point2d point_a, const Point2d point_b,
                       const Point2d point_c, const Point2d point_d);
Point2d PointDivide(const Point2d point_a, const Point2d point_b, int sum,
                    int sub);
Point2d PointExtend(const Point2d point_a, const double length,
                    const double yaw);
double Length(const Point2d point_a, const Point2d point_b);
bool CaculateRadius(const std::vector<Point3d> &points, double &r, double &ox,
                    double &oy);
}  // namespace common
}  // namespace prediction_lib
}  // namespace jarvis