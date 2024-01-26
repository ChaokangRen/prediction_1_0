#pragma once

#include <Eigen/Core>
#include <cmath>
#include <limits>
#include <list>
#include <map>
#include <string>
#include <unordered_set>
#include <vector>

#include "common.h"
#include "lane_sequence_predictor.h"
#include "prediction/prediction_interface.h"
#include "sglog/sglog.h"
#include "struct.h"
namespace jarvis {
namespace prediction_lib {

class JunctionPredictor {
public:
    void Init(const PredictionConf &prediction_conf);
    void Predict(ObstaclesInfo &obs_info, const ObstaclesInfo &obs_info_enu,
                 ObsTrafficType &traffic_type, const HistoryMap &obs_his_msg,
                 const ObsDeque &obstacles_deque,
                 const MonospacedLaneMsg &lane_msg, std::string &debug_info);

private:
    void GetInPredict(ObstaclesInfo &obs_info,
                      const ObstaclesInfo &obs_info_enu,

                      const MonospacedLaneMsg &lane_msg,
                      ObsTrafficType &obs_traffic_type);
    void GetOutPredict(ObstaclesInfo &obs_info,
                       const ObstaclesInfo &obs_info_enu,
                       const HistoryMap &obs_his_msg,
                       const ObsDeque &obstacles_deque,
                       const MonospacedLaneMsg &lane_msg,
                       ObsTrafficType &obs_traffic_type);
    void UpdateTrafficTypeMap(const MonospacedLaneMsg &lane_msg,
                              ObsTrafficType &traffic_type,
                              const std::string id, const int sub_section,
                              const Point2d point_obs);
    void GetIntersectSub(const ObsTrajectory &trajectory,
                         const std::vector<Point2d> &points_lane,
                         const double lane_yaw_enu, int &section_id);
    ObsTrajectory GetNewLaneTrajectory(const LinePoint center_line,
                                       const ObstacleMsg &obstacle,
                                       const double ego_x, const double ego_y,
                                       const double ego_yaw);

    double GetBestTime(const std::array<double, 2> &start_x,
                       const std::array<double, 2> &end_x,
                       const std::array<double, 2> &start_y,
                       const std::array<double, 2> &end_y);
    std::vector<double> GenerateCandidateTimes();

    template <std::size_t N>
    std::array<double, 2 * N - 2> ComputePolynomial(
        const std::array<double, N - 1> &start_state,
        const std::array<double, N - 1> &end_state, const double param);

    inline std::array<double, 4> ComputePolynomial(
        const std::array<double, 2> &start_state,
        const std::array<double, 2> &end_state, const double param) {
        std::array<double, 4> coefs;
        coefs[0] = start_state[0];
        coefs[1] = start_state[1];

        auto m0 = end_state[0] - start_state[0] - start_state[1] * param;
        auto m1 = end_state[1] - start_state[1];

        auto param_p3 = param * param * param;
        coefs[3] = (m1 * param - 2.0 * m0) / param_p3;

        coefs[2] = (m1 - 3.0 * coefs[3] * param * param) / param * 0.5;
        return coefs;
    }

    double EvaluateCubicPolynomial(const std::array<double, 4> &coefs,
                                   const double t, const uint32_t order);

    double CostFunction(const std::array<double, 4> &x_coeffs,
                        const std::array<double, 4> &y_coeffs,
                        const double time_to_exit);

    void JunctionTurn(ObstaclesInfo &obs_info,
                      const ObstaclesInfo &obs_info_enu,
                      const HistoryMap &obs_his_msg,
                      const ObsTrafficType &obs_traffic_type);

    void FilterBySolidLane(ObstaclesInfo &obs_info,
                           const ObsTrafficType &traffic_type,
                           const MonospacedLaneMsg &lane_msg);

    void DrawFreeMoveTrajectoryPoints(const ObsDeque &obstacles_deque,
                                      const double vel, const std::string id,
                                      const double ox, const double oy,
                                      const double r, const double total_time,
                                      const double period,
                                      ObsTrajectory *trajectory);

    void DrawFreeMoveTrajectoryPoints(
        const Eigen::Vector2d &position, const Eigen::Vector2d &velocity,
        const Eigen::Vector2d &acceleration, const double theta,
        const double dtheta, const double start_time, const double total_time,
        const double period, ObsTrajectory *trajectory);

    void GenerateFreeMoveTrajectoryPoints(
        Eigen::Matrix<double, 6, 1> *state,
        const Eigen::Matrix<double, 6, 6> &transition, double theta,
        const double dtheta, const double start_time, const int num,
        const double period, ObsTrajectory *trajectory);
    double prediction_total_time_ = 4.0;
    double prediction_period_ = 0.1;
    double yaw_limit_junctionturn_ = 80 * M_PI / 180;
};
}  // namespace prediction_lib
}  // namespace jarvis