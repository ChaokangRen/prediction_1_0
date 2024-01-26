#pragma once

#include <cmath>
#include <deque>
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

class ObstacleContainer {
public:
    void Process(const ObstaclesInfo &obs_info,
                 const MonospacedLaneMsg &lane_msg);
    void PostProcess(const ObstaclesInfo &obs_info,
                     ObsTrafficType &traffic_type, std::string &debug_info);
    ObsTrafficType GetTrafficType();
    ObstaclesInfo GetObsInfoEnu();
    HistoryMap GetHisMsgEnu();
    HistoryMap GetHisMsgVeh();
    ObsDeque GetObsDeque();
    void SetTrafficType(const ObsTrafficType &obs_traffic_type);

    void VehCoord2EnuCoord(const double ego_x, const double ego_y,
                           const double ego_yaw, double &x, double &y,
                           double &yaw, double &velx, double &vely);
    void VehCoord2EnuCoord(const double ego_x, const double ego_y,
                           const double ego_yaw, double &x, double &y);
    void EnuCoord2VehCoord(const double ego_x, const double ego_y,
                           const double ego_yaw, double &x, double &y,
                           double &yaw, double &velx, double &vely);
    void EnuCoord2VehCoord(const double ego_x, const double ego_y,
                           const double ego_yaw, double &x, double &y);
    void UpdateDebugInfo(const ObstaclesInfo &obs_info,
                         std::string &debug_info);
    void UpdateMap(const std::string &obs_id, const ObsHistoryMsg &his_msg);
    void CleanMap(const ObstaclesInfo &obs_info);

    void Enu2VehCoordMap(const ObstaclesInfo &obs_info);

    void UpdateTrajectory(const ObstaclesInfo &obs_info,
                          const ObsTrafficType &obs_traffic_type);
    void UpdateDistanceTrajectory(const ObstaclesInfo &obs_info);

private:
    void UpdateHistoryMsg(const ObstaclesInfo &obs_info);
    void ProcessTrafficType(const ObstaclesInfo &obs_info,
                            const MonospacedLaneMsg &lane_msg);
    void CalculateEvaluationIndicator();
    void TrafficTypeClear();
    void UpdateObsSectionMap(const MonospacedLaneMsg &lane_msg,
                             const double obs_x, const double obs_y,
                             const std::string obs_id);
    int his_list_len_ = 60;
    int his_deque_len_ = 30;
    ObsTrafficType traffic_type_;
    ObstaclesInfo obstacles_enu_info_;
    HistoryMap obstacles_history_msg_enu_;
    HistoryMap obstacles_history_msg_veh_;
    ObsDeque obstacles_deque_;
};
}  // namespace prediction_lib
}  // namespace jarvis