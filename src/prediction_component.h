#pragma once

#include <memory>

#include "common/struct.h"
#include "container/obstacle_container.h"
#include "lane_container.h"
#include "optimizer/optObjTraj.h"
#include "prediction/prediction_interface.h"
#include "predictor/free_move_predictor.h"
#include "predictor/junction_predictor.h"
#include "predictor/lane_sequence_predictor.h"
#include "predictor/predictor_interface.h"

namespace jarvis {
namespace prediction_lib {

typedef std::shared_ptr<jarvis::prediction_lib::PredictorInterface>
    PredictorInterfacePtr;

class PredictionComponent : public PredictionInterface {
public:
    PredictionComponent();
    virtual ~PredictionComponent();

    std::string get_version() override;

    bool init(const PredictionConfigPath &prediction_config_path) override;
    bool execute(ObstaclesInfo *obstacles_info, LaneInfo *lane_info,
                 std::string &debug_info) override;
    bool set_pos_from_ins(const PosFromIns &pos_ins) override;
    bool set_speed_from_veh(const SpeedFromVeh &speed_veh) override;
    bool set_wheel_speed_from_veh(
        const WheelSpeedFromVeh &wheel_speed_veh) override;
    bool set_traffic_light_frame_result(
        const TrafficLightFrameResult &traffic_light_frame_result) override;

private:
    LaneSequencePredictor lane_sequence_predictor_;
    FreeMovePredictor free_move_predictor_;
    JunctionPredictor junction_predictor_;
    ObstacleContainer obstacle_container_;
    LaneContainer lane_container_;
    PredictorInterfacePtr predictor_ptr_;

    PosFromIns pos_ins_;
    SpeedFromVeh speed_veh_;
    WheelSpeedFromVeh wheel_speed_veh_;
    TrafficLightFrameResult traffic_light_frame_result_;
};

}  // namespace prediction_lib
}  // namespace jarvis
