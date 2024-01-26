#pragma once
#include <deque>
#include <list>
#include <map>

#include "prediction_interface.h"
namespace jarvis {
namespace prediction_lib {

struct Point2d {
    double x;
    double y;
};

struct Position {
    Point3d pos;
    Point3d rotation;
    double s;
};

struct PredictionConf {
    double prediction_total_time;
    double prediction_period;
};

struct ObsInLine {
    int32_t id;
    double distance;
    double yaw;
    int s;  // wuzihan
};

enum INTENSION { STRAIGHT = 0, LEFT = 1, RIGHT = 2, TURN = 3, STOP = 4 };

enum PREDICTOR { LANE_SEQUENCE_PREDICTOR = 0, FREE_MOVE_PREDICTOR = 1 };

struct CenterLineMsg {
    std::vector<Position> center_line;
    int32_t id;
};

struct LaneCenterLine {
    std::vector<CenterLineMsg> lane_center_line_;
    std::map<std::string, ObsInLine> obs_in_line;
    double interval = 1;  // wzh
    int32_t id;           // wzh
};

struct ObsTrafficType {
    std::vector<int> interactive_obs;
    std::vector<int> non_interactive_obs;
    std::vector<int> on_lane_obs;
    std::vector<int> off_lane_obs;
    std::vector<int> get_in_lane_obs;
    std::vector<int> get_out_lane_obs;
    // pair first:section sub; pair second:distance to center line
    std::map<std::string, std::pair<int, double>> map_obs_in_section;
};

struct ObsHistoryMsg {
    Point3d pos;
    Point3d rotation;
    Point3d velocity;
    ObsTrajectories trajectories;
    PREDICTOR predictor;
    INTENSION intension;
    double mean_error;
    double end_error;
};

typedef std::map<std::string, std::vector<ObsHistoryMsg>> HistoryMap;
typedef std::map<std::string, std::deque<std::tuple<double, double, double>>>
    ObsDeque;
typedef std::map<std::string, std::list<std::tuple<double, double, double>>>
    ObsMap;
typedef std::vector<LaneLineMsg> Lanes;

struct LinePoint {
    Point2d point_begin;
    Point2d point_end;
    double yaw_veh_rad;
    int id;
};

struct MonospacedLane {
    LinePoint lane_line;
    LaneLineType lane_line_type;
    LaneLineColor lane_line_color;
};

struct MonospacedSection {
    MonospacedLane left_lane_line;
    MonospacedLane right_lane_line;
    LinePoint stop_lane_line;
    LinePoint center_line;
    bool can_go_straight = true;
    bool can_turn_left = true;
    bool can_turn_right = true;
    bool can_turn_around = true;
    bool can_cutted_left = true;
    bool can_cutted_right = true;
    bool is_opposite = false;
    bool has_stop_line = false;
};

struct MonospacedLaneMsg {
    std::vector<MonospacedLane> monospaced_lane;
    std::vector<MonospacedSection> monospaced_section;
    int sub_ego_in_section;
    double yaw_veh_rad;
    bool has_lane = true;
};

}  // namespace prediction_lib
}  // namespace jarvis