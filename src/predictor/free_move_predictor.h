#pragma once

#include <Eigen/Core>
#include <algorithm>
#include <cmath>
#include <map>
#include <string>
#include <unordered_set>
#include <vector>

#include "common.h"
#include "optObjTraj.h"
#include "prediction/prediction_interface.h"
#include "sglog/sglog.h"
#include "struct.h"
namespace jarvis {
namespace prediction_lib {

class FreeMovePredictor {
public:
    void Init(const PredictionConf &prediction_conf);
    void Predict(ObstaclesInfo &obs_info, const ObstaclesInfo &obs_info_enu,
                 ObsTrafficType &traffic_type, const HistoryMap &obs_his_msg,
                 const ObsDeque &obstacles_deque,
                 const MonospacedLaneMsg &lane_msg, std::string &debug_info);

private:
    void DrawFreeMoveTrajectoryPoints(
        const Eigen::Vector2d &position, const Eigen::Vector2d &velocity,
        const Eigen::Vector2d &acceleration, const double theta,
        const double dtheta, const double start_time, const double total_time,
        const double period, ObsTrajectory *points);

    void GenerateFreeMoveTrajectoryPoints(
        Eigen::Matrix<double, 6, 1> *state,
        const Eigen::Matrix<double, 6, 6> &transition, double theta,
        const double dtheta, const double start_time, const int num,
        const double period, ObsTrajectory *points);

    void OptimizeHistoryMsg(const HistoryMap &obs_his_msg, ObsMap &opt_map);
    void DrawFreeMoveTrajectoryPoints(const ObsDeque &obstacles_deque,
                                      const double vel, const std::string id,
                                      const double ox, const double oy,
                                      const double r, const double total_time,
                                      const double period,
                                      ObsTrajectory *trajectory);
    void UpdateTrafficTypeMap(const MonospacedLaneMsg &lane_msg,
                              ObsTrafficType &traffic_type,
                              const std::string id, const double obs_x,
                              const double obs_y);
    optObjTraj optObj_traj_;
    ObsMap obstacles_map_;

    double prediction_total_time_ = 4.0;
    double prediction_period_ = 0.1;
    int obs_list_len_ = 20;
};

}  // namespace prediction_lib
}  // namespace jarvis
