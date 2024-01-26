#pragma once

#include <algorithm>
#include <cmath>

#include "common.h"
#include "prediction/prediction_interface.h"
#include "sglog/sglog.h"
#include "struct.h"
namespace jarvis {
namespace prediction_lib {
class LaneContainer {
public:
    void Process(LaneInfo &lane_info,
                 const TrafficLightFrameResult &traffic_lights_results);
    MonospacedLaneMsg GetLaneMsg();
    void UpdateDubugInfo(std::string &debug_info);

private:
    void Clear();
    void UpdateLaneByHistory(Lanes &lane_lines, Lanes &stop_lines);
    void FilterLane(const LaneInfo &lane_info,
                    const TrafficLightFrameResult &traffic_lights_results);
    static bool Left2RightSortLaneLineMsg(const LaneLineMsg &lane_line_msg_1,
                                          const LaneLineMsg &lane_line_msg_2);
    static bool Left2RightSortMonospacedLane(
        const MonospacedLane &monospaced_lane_1,
        const MonospacedLane &monospaced_lane_2);
    static bool Close2FarSortStopLineMsg(const LaneLineMsg &lane_line_msg_1,
                                         const LaneLineMsg &lane_line_msg_2);
    void InitializeLane(const Lanes &lane_lines);
    double CalculateLaneYaw(const Lanes &lane_lines);
    void AddLostLane(MonospacedLaneMsg &lane_msg);
    std::string TypeToString(const LaneLineType type);
    std::string ColorToString(const LaneLineColor color);
    void UpdateMsg(MonospacedLaneMsg &lane_msg);
    void SetID(MonospacedLaneMsg &lane_msg);
    void SetIDByYellowLane(MonospacedLaneMsg &lane_msg, int sub_yellow_lane);
    void PreProcess(const LaneInfo &lane_info,
                    const TrafficLightFrameResult &traffic_lights_results);
    void ProcessLane();
    void ProcessSection();
    void GenerateSection(MonospacedLaneMsg &lane_msg);
    void GenerateStopLine(MonospacedLaneMsg &lane_msg, const Lanes &stop_lines);
    void GenerateCenterLine(MonospacedSection &section);
    void UpdateSectionState(MonospacedSection &section);
    void AddLaneOnLeft(std::vector<MonospacedLane> &monospaced_lane,
                       const int sub, const double lane_width);
    void AddLaneOnBack(std::vector<MonospacedLane> &monospaced_lane,
                       const double lane_width);
    void MatchLaneInfo(MonospacedLaneMsg &lane_msg);
    void UpdateCenterLine(LaneInfo &lane_info,
                          const MonospacedLaneMsg &lane_msg);

    MonospacedLaneMsg lane_msg_;
    Lanes lane_lines_;
    Lanes stop_lines_;
    Lanes lane_lines_pre_;
    Lanes stop_lines_pre_;

    int count_no_lane_limit_ = 3;
    int count_lane_limit_ = 3;
    int count_no_stop_line_limit_ = 3;
    int count_stop_line_limit_ = 3;
    int lane_length_limit_ = 5;
    double distance_match_limit_ = 1;
    int count_no_lane_ = 0;
    int count_lane_ = 0;
    int count_stop_line_ = 0;
    int count_no_stop_line_ = 0;

    double point_begin_y_ = 0.0;
    double point_end_y_ = 60.0;
    double lane_width_ = 3.5;

    double center_line_interval_ = 0.5;

    bool has_lane_line_pre_ = false;
    bool has_stop_line_pre_ = false;
    bool has_lane_line_ = false;
    bool has_stop_line_ = false;
    bool has_traffic_light_ = false;
};
}  // namespace prediction_lib
}  // namespace jarvis