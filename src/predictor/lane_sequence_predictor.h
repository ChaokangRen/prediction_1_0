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
#include "prediction/prediction_interface.h"
#include "sglog/sglog.h"
#include "struct.h"
namespace jarvis {
namespace prediction_lib {

class LaneSequencePredictor {
public:
    void Init(const PredictionConf &prediction_conf);
    void Predict(ObstaclesInfo &obs_info, const MonospacedLaneMsg &lane_msg,
                 const ObsTrafficType &traffic_type,
                 const HistoryMap &obs_his_msg, std::string &debug_info);

private:
    void FirstGetIntension(const HistoryMap &obs_his_msg,
                           const std::string obs_id,
                           const LinePoint &center_line, const double distance,
                           INTENSION &intension);
    void SecondGetIntension(const int32_t sub_section,
                            const MonospacedLaneMsg &lane_msg,
                            const double obs_vel, INTENSION &intension,
                            int32_t &section_id_target);
    CenterLineMsg GetCenterLine(const int32_t id,
                                const LaneCenterLine &lane_center_line);
    void GetTargetPoint(const LinePoint &center_line, const double vel,
                        const double obs_x, const double obs_y,
                        const double yaw_obs2lane, const INTENSION intension,
                        double &x_target, double &y_target, double &yaw_target);

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
    void Rotation(const double end_x, const double end_y, const double yaw,
                  double &x, double &y);
    std::map<std::string, std::pair<INTENSION, int>> obs_history_intension_;
    ObsMap obstacles_history_pos_;
    double dtheta_limit_ = 0.08;
    int his_msg_limit_ = 5;
    double prediction_total_time_ = 4.0;
    double prediction_period_ = 0.1;
    double his_period_ = 0.1;
    double dx_change_lane_limit_ = 0.8;
};
}  // namespace prediction_lib
}  // namespace jarvis