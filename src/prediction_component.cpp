#include "prediction_component.h"

#include <sglog/sglog.h>
#include <sgtime/sgtime.h>

#include "prediction/prediction_interface.h"

namespace jarvis {
namespace prediction_lib {

PredictionInterface *PredictionInterface::create_instance() {
    return new PredictionComponent();
}

PredictionComponent::PredictionComponent() {
    SG_INFO("PredictionComponent construct");
}

PredictionComponent::~PredictionComponent() {
    SG_INFO("PredictionComponent destruct");
}

std::string PredictionComponent::get_version() {
#ifdef PKG_VERSION
    return PKG_VERSION;
#else
    return "UNKNOWN";
#endif
}

bool PredictionComponent::init(
    const PredictionConfigPath &prediction_config_path) {
    SG_INFO("PredictionComponent init");
    PredictionConf prediction_conf;
    prediction_conf.prediction_total_time = 4.0;
    prediction_conf.prediction_period = 0.1;
    lane_sequence_predictor_.Init(prediction_conf);
    free_move_predictor_.Init(prediction_conf);
    junction_predictor_.Init(prediction_conf);
    return true;
}

bool PredictionComponent::execute(ObstaclesInfo *obstacles_info,
                                  LaneInfo *lane_info,
                                  std::string &debug_info) {
    auto begin = SgTimeUtil::now_nsecs();
    // SG_INFO("left=%d,dis=%lf", traffic_lights_results.semantic_left.color,
    //         traffic_lights_results.semantic_left.distance);
    // SG_INFO("right=%d,dis=%lf", traffic_lights_results.semantic_right.color,
    //         traffic_lights_results.semantic_right.distance);
    // SG_INFO("straight=%d,dis=%lf",
    //         traffic_lights_results.semantic_straight.color,
    //         traffic_lights_results.semantic_straight.distance);
    // SG_INFO("uturn=%d,dis=%lf", traffic_lights_results.semantic_uturn.color,
    //         traffic_lights_results.semantic_uturn.distance);
    // SG_INFO("digit=%d,num=%s", traffic_lights_results.digit.color,
    //         traffic_lights_results.digit.digit_num.c_str());

    // for (int i = 0; i < lane_info.lane_lines_msg.size(); ++i) {
    //     if (lane_info.lane_lines_msg[i].type == LaneLineMsg::Type::STOP_LINE)
    //     {
    //         SG_INFO("stop_line%d=%lf", i,
    //                 lane_info.lane_lines_msg[i].lane_line[0].pos.y);
    //     }
    // }

    lane_container_.Process(*lane_info, traffic_light_frame_result_);
    MonospacedLaneMsg lane_msg = lane_container_.GetLaneMsg();

    obstacle_container_.Process(*obstacles_info, lane_msg);

    ObsTrafficType traffic_type = obstacle_container_.GetTrafficType();

    ObstaclesInfo obstacles_info_enu = obstacle_container_.GetObsInfoEnu();
    HistoryMap obstacles_history_msg_enu = obstacle_container_.GetHisMsgEnu();
    ObsDeque obstacles_deque = obstacle_container_.GetObsDeque();

    free_move_predictor_.Predict(*obstacles_info, obstacles_info_enu,
                                 traffic_type, obstacles_history_msg_enu,
                                 obstacles_deque, lane_msg, debug_info);

    HistoryMap obstacles_history_msg_veh = obstacle_container_.GetHisMsgVeh();

    lane_sequence_predictor_.Predict(*obstacles_info, lane_msg, traffic_type,
                                     obstacles_history_msg_veh, debug_info);

    junction_predictor_.Predict(*obstacles_info, obstacles_info_enu,
                                traffic_type, obstacles_history_msg_enu,
                                obstacles_deque, lane_msg, debug_info);

    obstacle_container_.PostProcess(*obstacles_info, traffic_type, debug_info);

    for (int i = 0; i < obstacles_info_enu.obstacles.size(); ++i) {
        obstacles_info_enu.obstacles[i].obs_trajectories =
            obstacles_info->obstacles[i].obs_trajectories;
    }
    *obstacles_info = obstacles_info_enu;

    lane_container_.UpdateDubugInfo(debug_info);

    auto after = SgTimeUtil::now_nsecs();
    auto delta_summary = SgTimeUtil::time_delta_record(after, begin, "Process");
    // SG_INFO("Process Prediction obs_cnt = %d %s",
    //         obstacles_info->obstacles.size(),
    //         delta_summary->to_str().c_str());
    return true;
}

bool PredictionComponent::set_pos_from_ins(const PosFromIns &pos_ins) {
    pos_ins_ = pos_ins;
    return true;
}

bool PredictionComponent::set_speed_from_veh(const SpeedFromVeh &speed_veh) {
    speed_veh_ = speed_veh;
    return true;
}

bool PredictionComponent::set_wheel_speed_from_veh(
    const WheelSpeedFromVeh &wheel_speed_veh) {
    wheel_speed_veh_ = wheel_speed_veh;
    return true;
}

bool PredictionComponent::set_traffic_light_frame_result(
    const TrafficLightFrameResult &traffic_light_frame_result) {
    traffic_light_frame_result_ = traffic_light_frame_result;
    return true;
}

}  // namespace prediction_lib
}  // namespace jarvis
