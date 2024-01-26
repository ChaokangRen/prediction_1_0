#ifndef OPTOBJTRAJ_H_
#define OPTOBJTRAJ_H_
#include <gtsam/geometry/Pose2.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

using namespace std;
using namespace gtsam;
using symbol_shorthand::X;

#define RAD1 (double)(1 / 57.3)
#define RAD2 (double)(2 / 57.3)
#define RAD3 (double)(3 / 57.3)
#define RAD4 (double)(4 / 57.3)
#define RAD5 (double)(5 / 57.3)

class optObjTraj {
public:
    optObjTraj(){};
    ~optObjTraj(){};

    /** \brief Optimize the 2D trajectory by gtsam.
     * \param[in] input The input vector of origin 2D trajectory
     * \param[out] output The result of optimized 2D trajectory
     * \param[in] flag The sensor type. False represent lidar-detected
     * trajectory. True represent camera-detected trajectory. \param[in]
     * algorithem The algorithem type. 0 represent GaussNewton. 1 represent
     * ISAM2. 2 represent LevenbergMarquardt.
     */
    bool optTraj(const std::vector<tuple<double, double, double>> &input,
                 std::vector<tuple<double, double, double>> &output,
                 const bool &flag, const int &algorithem = 0);

    std::vector<gtsam::Pose2> tuple2gtsam(
        const std::vector<tuple<double, double, double>> &input);
    std::vector<tuple<double, double, double>> gtsam2tuple(
        const std::vector<gtsam::Pose2> &input);

private:
    NonlinearFactorGraph m_graph;
    Values m_initials;
    Values m_estimate;

    ISAM2 *m_isam;

    ISAM2Params m_isamParam;
    GaussNewtonParams m_gaussParam;
    LevenbergMarquardtParams m_levenParam;
};
#endif