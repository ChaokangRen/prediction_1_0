#include "common.h"
namespace jarvis {
namespace prediction_lib {
namespace common {

bool IsInQuadrilateral(const Point2d point_e, const Point2d point_a,
                       const Point2d point_b, const Point2d point_c,
                       const Point2d point_d) {
    /*
    A----------B
    |          |
    |     E    |
    |          |
    D----------C
    (AB X AE ) * (CD X CE)  >= 0 && (DA X DE ) * (BC X BE) >= 0
    */
    Point2d AB = Vector(point_a, point_b);
    Point2d AE = Vector(point_a, point_e);
    Point2d CD = Vector(point_c, point_d);
    Point2d CE = Vector(point_c, point_e);
    Point2d DA = Vector(point_d, point_a);
    Point2d DE = Vector(point_d, point_e);
    Point2d BC = Vector(point_b, point_c);
    Point2d BE = Vector(point_b, point_e);
    double cross1 = CrossProduct(AB, AE);
    double cross2 = CrossProduct(CD, CE);
    double cross3 = CrossProduct(DA, DE);
    double cross4 = CrossProduct(BC, BE);
    return cross1 * cross2 >= 0 && cross3 * cross4 >= 0;
}

Point2d Vector(const Point2d point_1, const Point2d point_2) {
    Point2d vector;
    vector.x = point_2.x - point_1.x;
    vector.y = point_2.y - point_1.y;
    return vector;
}

double CrossProduct(const Point2d vector_1, const Point2d vector_2) {
    return vector_1.x * vector_2.y - vector_1.y * vector_2.x;
}

bool IsInRoadSection(const MonospacedLaneMsg &lane_msg, const double obs_x,
                     const double obs_y) {
    Point2d point_obs;
    point_obs.x = obs_x;
    point_obs.y = obs_y;

    Point2d point_section_fl;
    point_section_fl.x =
        lane_msg.monospaced_section[0].left_lane_line.lane_line.point_end.x;
    point_section_fl.y =
        lane_msg.monospaced_section[0].left_lane_line.lane_line.point_end.y;
    Point2d point_section_rl;
    point_section_rl.x =
        lane_msg.monospaced_section[0].left_lane_line.lane_line.point_begin.x;
    point_section_rl.y =
        lane_msg.monospaced_section[0].left_lane_line.lane_line.point_begin.y;
    Point2d point_section_fr;
    point_section_fr.x = lane_msg.monospaced_section.back()
                             .right_lane_line.lane_line.point_end.x;
    point_section_fr.y = lane_msg.monospaced_section.back()
                             .right_lane_line.lane_line.point_end.y;
    Point2d point_section_rr;
    point_section_rr.x = lane_msg.monospaced_section.back()
                             .right_lane_line.lane_line.point_begin.x;
    point_section_rr.y = lane_msg.monospaced_section.back()
                             .right_lane_line.lane_line.point_begin.y;
    return IsInQuadrilateral(point_obs, point_section_fl, point_section_fr,
                             point_section_rr, point_section_rl);
}

double DistancePoint2Line(const Point2d point_a, const Point2d point_b,
                          const Point2d point_p) {
    double dy = point_a.y - point_b.y;
    double dx = point_b.x - point_a.x;
    double cross = CrossProduct(point_a, point_b);
    double distance = 0.0;
    // left- right+
    if (point_p.x > (-(dx * point_p.y + cross) / dy)) {
        distance = (std::fabs(dy * point_p.x + dx * point_p.y + cross)) /
                   (std::hypot(dy, dx));
    } else {
        distance = -(std::fabs(dy * point_p.x + dx * point_p.y + cross)) /
                   (std::hypot(dy, dx));
    }
    return distance;
}

Point2d PointVertical(const Point2d point_a, const Point2d point_b,
                      const Point2d point_p) {
    Point2d point_vertical;
    double k = 0.0;
    if (point_a.x == point_b.x) {
        point_vertical.x = point_a.x;
        point_vertical.y = point_p.y;
    } else {
        k = (point_b.y - point_a.y) / (point_b.x - point_a.x);
        double A = k;
        double B = -1.0;
        double C = point_a.y - k * point_a.x;
        point_vertical.x =
            (B * B * point_p.x - A * B * point_p.y - A * C) / (A * A + B * B);
        point_vertical.y =
            (A * A * point_p.y - A * B * point_p.x - B * C) / (A * A + B * B);
    }
    return point_vertical;
}

bool IsIntersectedByLineSection(const Point2d point_a, const Point2d point_b,
                                const Point2d point_c, const Point2d point_d) {
    /*
            A
            |
    C-------|----D
            |
            |
            B
    */

    Point2d AB = Vector(point_a, point_b);
    Point2d AC = Vector(point_a, point_c);
    Point2d AD = Vector(point_a, point_d);
    Point2d CD = Vector(point_c, point_d);
    Point2d CA = Vector(point_c, point_a);
    Point2d CB = Vector(point_c, point_b);

    double ABxAC = CrossProduct(AB, AC);
    double ABxAD = CrossProduct(AB, AD);
    double CDxCA = CrossProduct(CD, CA);
    double CDxCB = CrossProduct(CD, CB);
    if (ABxAC * ABxAD < 0 && CDxCA * CDxCB < 0) {
        return true;
    } else {
        return false;
    }
    // if (ABxAC * ABxAD < 0) {
    //     return true;
    // } else {
    //     return false;
    // }
}

bool IsIntersectedSectionAndStraightLine(const Point2d point_a,
                                         const Point2d point_b,
                                         const Point2d section_point_c,
                                         const Point2d section_point_d) {
    /*
            A
            |
            |
            |
            B

    C-----------D
    */

    Point2d AB = Vector(point_a, point_b);
    Point2d AC = Vector(point_a, section_point_c);
    Point2d AD = Vector(point_a, section_point_d);
    Point2d CD = Vector(section_point_c, section_point_d);
    Point2d CA = Vector(section_point_c, point_a);
    Point2d CB = Vector(section_point_c, point_b);

    double ABxAC = CrossProduct(AB, AC);
    double ABxAD = CrossProduct(AB, AD);
    double CDxCA = CrossProduct(CD, CA);
    double CDxCB = CrossProduct(CD, CB);

    if (ABxAC * ABxAD < 0) {
        return true;
    } else {
        return false;
    }
}

void VehCoord2EnuCoord(const double ego_x, const double ego_y,
                       const double ego_yaw, double &x, double &y) {
    double x_veh = x;
    double y_veh = y;
    double rotation = (M_PI / 2 - ego_yaw);
    x = x_veh * std::cos(rotation) + y_veh * std::sin(rotation) + ego_x;
    y = y_veh * std::cos(rotation) - x_veh * std::sin(rotation) + ego_y;
}

Point2d PointIntersect(const Point2d point_a, const Point2d point_b,
                       const Point2d point_c, const Point2d point_d) {
    /*
            A
            |
    C-------P----D
            |
            |
            B
    P = A + AB * t
    */
    Point2d AB = Vector(point_a, point_b);
    Point2d CD = Vector(point_c, point_d);
    Point2d AC = Vector(point_a, point_c);
    double cross1 = CrossProduct(AC, CD);
    double cross2 = CrossProduct(AB, CD);
    double t = cross1 / cross2;
    Point2d point_p;
    point_p.x = point_a.x + t * AB.x;
    point_p.y = point_a.y + t * AB.y;
    return point_p;
}

Point2d PointDivide(const Point2d point_a, const Point2d point_b, int sum,
                    int sub) {
    Point2d AB = Vector(point_a, point_b);
    double dx = AB.x / sum;
    double dy = AB.y / sum;
    Point2d point_p;
    point_p.x = point_a.x + dx * sub;
    point_p.y = point_a.y + dy * sub;
    return point_p;
}

Point2d PointExtend(const Point2d point_a, const double length,
                    const double yaw) {
    Point2d point_p;
    point_p.x = point_a.x + length * std::cos(yaw);
    point_p.y = point_a.y + length * std::sin(yaw);
    return point_p;
}

double Length(const Point2d point_a, const Point2d point_b) {
    Point2d vector = Vector(point_a, point_b);
    return std::hypot(vector.x, vector.y);
}

bool CaculateRadius(const std::vector<Point3d> &points, double &r, double &ox,
                    double &oy) {
    if (points.size() < 3) {
        return false;
    }
    // for (int i = 0; i < points.size(); ++i) {
    //     SG_INFO("------------------ x=%lf", points[i].x);
    // }
    // for (int i = 0; i < points.size(); ++i) {
    //     SG_INFO("------------------ y=%lf", points[i].y);
    // }
    double sumX = 0.0;
    double sumY = 0.0;
    double sumX2 = 0.0;
    double sumY2 = 0.0;
    double sumX3 = 0.0;
    double sumY3 = 0.0;
    double sumXY = 0.0;
    double sumXY2 = 0.0;
    double sumX2Y = 0.0;

    int N = points.size();
    for (int pId = 0; pId < N; ++pId) {
        // SG_INFO("%dx=%lf,y=%lf", pId, points[pId].x, points[pId].y);
        sumX += points[pId].x;
        sumY += points[pId].y;

        double x2 = points[pId].x * points[pId].x;
        double y2 = points[pId].y * points[pId].y;
        sumX2 += x2;
        sumY2 += y2;

        sumX3 += x2 * points[pId].x;
        sumY3 += y2 * points[pId].y;
        sumXY += points[pId].x * points[pId].y;
        sumXY2 += points[pId].x * y2;
        sumX2Y += x2 * points[pId].y;
    }

    double C, D, E, G, H;
    double a, b, c;

    C = N * sumX2 - sumX * sumX;
    D = N * sumXY - sumX * sumY;
    E = N * sumX3 + N * sumXY2 - (sumX2 + sumY2) * sumX;
    G = N * sumY2 - sumY * sumY;
    H = N * sumX2Y + N * sumY3 - (sumX2 + sumY2) * sumY;

    a = (H * D - E * G) / (C * G - D * D);
    b = (H * C - E * D) / (D * D - G * C);
    c = -(a * sumX + b * sumY + sumX2 + sumY2) / N;

    ox = -a / 2.0;
    oy = -b / 2.0;
    r = sqrt(a * a + b * b - 4 * c) / 2.0;
    // SG_INFO("r = %lf,ox = %lf,oy = %lf", r, ox, oy);

    return true;
}

}  // namespace common
}  // namespace prediction_lib
}  // namespace jarvis