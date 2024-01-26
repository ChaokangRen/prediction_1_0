#include "obstacle_container.h"

namespace jarvis {
namespace prediction_lib {

void ObstacleContainer::Process(const ObstaclesInfo &obs_info,
                                const MonospacedLaneMsg &lane_msg) {
    UpdateHistoryMsg(obs_info);
    CalculateEvaluationIndicator();
    Enu2VehCoordMap(obs_info);
    UpdateDistanceTrajectory(obstacles_enu_info_);
    ProcessTrafficType(obs_info, lane_msg);
}

void ObstacleContainer::PostProcess(const ObstaclesInfo &obs_info,
                                    ObsTrafficType &obs_traffic_type,
                                    std::string &debug_info) {
    SetTrafficType(obs_traffic_type);
    UpdateTrajectory(obs_info, obs_traffic_type);
    UpdateDebugInfo(obs_info, debug_info);
}

void ObstacleContainer::TrafficTypeClear() {
    traffic_type_.interactive_obs.clear();
    traffic_type_.interactive_obs.clear();
    traffic_type_.non_interactive_obs.clear();
    traffic_type_.on_lane_obs.clear();
    traffic_type_.off_lane_obs.clear();
    traffic_type_.get_in_lane_obs.clear();
    traffic_type_.get_out_lane_obs.clear();
    traffic_type_.map_obs_in_section.clear();
}

void ObstacleContainer::ProcessTrafficType(const ObstaclesInfo &obs_info,
                                           const MonospacedLaneMsg &lane_msg) {
    TrafficTypeClear();
    double lane_yaw = lane_msg.yaw_veh_rad;

    for (int i = 0; i < obs_info.obstacles.size(); ++i) {
        std::string obs_id = obs_info.obstacles[i].track_id;
        ObsType obs_type = obs_info.obstacles[i].type;
        double obs_x = obs_info.obstacles[i].state.position.x;
        double obs_y = obs_info.obstacles[i].state.position.y;
        double obs_velx = obs_info.obstacles[i].state.linear_velocity.x;
        double obs_vely = obs_info.obstacles[i].state.linear_velocity.y;
        double obs_vel = std::hypot(obs_velx, obs_vely);
        double obs_yaw = obs_info.obstacles[i].state.angular_deg.z * M_PI / 180;
        while (obs_yaw < 0) {
            obs_yaw += 2 * M_PI;
        }
        while (obs_yaw > 2 * M_PI) {
            obs_yaw -= 2 * M_PI;
        }

        double yaw_diff = obs_yaw - lane_yaw;
        while (yaw_diff < -M_PI) {
            yaw_diff += 2 * M_PI;
        }
        while (yaw_diff > M_PI) {
            yaw_diff -= 2 * M_PI;
        }

        ObsType type = obs_info.obstacles[i].type;

        if ((obs_x < -5 || obs_x > 5 || obs_y < -10 || obs_y > 30) &&
            obs_vel == 0) {
            traffic_type_.non_interactive_obs.emplace_back(i);
            continue;
        }

        if (type != ObsType::VEHICLE) {
            traffic_type_.interactive_obs.emplace_back(i);
            traffic_type_.off_lane_obs.emplace_back(i);
            continue;
        }

        if (obs_vel <= 1) {
            traffic_type_.interactive_obs.emplace_back(i);
            traffic_type_.off_lane_obs.emplace_back(i);
            continue;
        }

        if ((yaw_diff > -3 * M_PI / 4 && yaw_diff < -M_PI / 4) ||
            (yaw_diff > M_PI / 4 && yaw_diff < 3 * M_PI / 4)) {
            traffic_type_.interactive_obs.emplace_back(i);
            traffic_type_.off_lane_obs.emplace_back(i);
            continue;
        }

        // for demo
        std::vector<double> stop_line_x = {4132.641755};
        std::vector<double> stop_line_y = {2467.181396};
        double obs_x_enu = obstacles_enu_info_.obstacles[i].state.position.x;
        double obs_y_enu = obstacles_enu_info_.obstacles[i].state.position.y;
        double dis_to_stop_line = std::numeric_limits<double>::max();
        for (int j = 0; j < stop_line_x.size(); ++j) {
            double dis_now = std::hypot(obs_x_enu - stop_line_x[j],
                                        obs_y_enu - stop_line_y[j]);
            dis_to_stop_line = std::min(dis_now, dis_to_stop_line);
        }
        if (dis_to_stop_line < 100 && obs_vel < 0.5) {
            traffic_type_.non_interactive_obs.emplace_back(i);
            continue;
        }

        if (!lane_msg.monospaced_section.size()) {
            traffic_type_.interactive_obs.emplace_back(i);
            traffic_type_.off_lane_obs.emplace_back(i);
        } else {
            if (common::IsInRoadSection(lane_msg, obs_x, obs_y)) {
                UpdateObsSectionMap(lane_msg, obs_x, obs_y, obs_id);
                traffic_type_.interactive_obs.emplace_back(i);
                traffic_type_.on_lane_obs.emplace_back(i);
            } else {
                traffic_type_.interactive_obs.emplace_back(i);
                traffic_type_.off_lane_obs.emplace_back(i);
            }
        }
    }
}

ObsTrafficType ObstacleContainer::GetTrafficType() {
    return traffic_type_;
}

ObstaclesInfo ObstacleContainer::GetObsInfoEnu() {
    return obstacles_enu_info_;
}

HistoryMap ObstacleContainer::GetHisMsgEnu() {
    return obstacles_history_msg_enu_;
}

HistoryMap ObstacleContainer::GetHisMsgVeh() {
    return obstacles_history_msg_veh_;
}

ObsDeque ObstacleContainer::GetObsDeque() {
    return obstacles_deque_;
}

void ObstacleContainer::UpdateHistoryMsg(const ObstaclesInfo &obs_info) {
    obstacles_enu_info_ = obs_info;
    for (int i = 0; i < obs_info.obstacles.size(); ++i) {
        ObsHistoryMsg his_msg;
        std::string obs_id = obs_info.obstacles[i].track_id;
        // SG_INFO("init_obs --- %s", obs_id.c_str());
        double obs_x = obs_info.obstacles[i].state.position.x;
        double obs_y = obs_info.obstacles[i].state.position.y;
        double obs_yaw = obs_info.obstacles[i].state.angular_deg.z * M_PI / 180;
        while (obs_yaw < 0) {
            obs_yaw += 2 * M_PI;
        }
        while (obs_yaw > 2 * M_PI) {
            obs_yaw -= 2 * M_PI;
        }
        double obs_velx = obs_info.obstacles[i].state.linear_velocity.x;
        double obs_vely = obs_info.obstacles[i].state.linear_velocity.y;
        double ego_x = obs_info.location.position.x;
        double ego_y = obs_info.location.position.y;
        double ego_yaw = obs_info.location.yaw * M_PI / 180;

        ego_yaw += M_PI / 2;
        while (ego_yaw < 0) {
            ego_yaw += 2 * M_PI;
        }
        while (ego_yaw > 2 * M_PI) {
            ego_yaw -= 2 * M_PI;
        }

        VehCoord2EnuCoord(ego_x, ego_y, ego_yaw, obs_x, obs_y, obs_yaw,
                          obs_velx, obs_vely);

        obstacles_enu_info_.obstacles[i].state.position.x = obs_x;
        obstacles_enu_info_.obstacles[i].state.position.y = obs_y;
        obstacles_enu_info_.obstacles[i].state.angular_deg.z = obs_yaw;
        // SG_INFO("%s-----obs_enu_yaw=%lf", obs_id.c_str(), obs_yaw);
        obstacles_enu_info_.obstacles[i].state.linear_velocity.x = obs_velx;
        obstacles_enu_info_.obstacles[i].state.linear_velocity.y = obs_vely;
        // SG_INFO("obs_yaw = %lf", obs_yaw);

        his_msg.pos.x = obs_x;
        his_msg.pos.y = obs_y;
        his_msg.pos.z = 0;
        his_msg.rotation.x = 0;
        his_msg.rotation.y = 0;
        his_msg.rotation.z = obs_yaw;
        his_msg.velocity.x = obs_velx;
        his_msg.velocity.y = obs_vely;
        his_msg.velocity.z = 0;

        UpdateMap(obs_id, his_msg);
    }
    CleanMap(obs_info);
}

void ObstacleContainer::VehCoord2EnuCoord(const double ego_x,
                                          const double ego_y,
                                          const double ego_yaw, double &x,
                                          double &y, double &yaw, double &velx,
                                          double &vely) {
    double x_veh = x;
    double y_veh = y;
    double yaw_veh = yaw;
    double velx_veh = velx;
    double vely_veh = vely;
    // const double deg_to_rad = M_PI / 180;
    // const double rad_to_deg = 180 / M_PI;
    double rotation = (M_PI / 2 - ego_yaw);
    // double rotation = -ego_yaw;

    x = x_veh * std::cos(rotation) + y_veh * std::sin(rotation) + ego_x;
    y = y_veh * std::cos(rotation) - x_veh * std::sin(rotation) + ego_y;
    yaw = yaw_veh + ego_yaw - M_PI / 2;
    while (yaw < 0) {
        yaw += 2 * M_PI;
    }
    while (yaw > 2 * M_PI) {
        yaw -= 2 * M_PI;
    }
    velx = velx_veh * std::cos(rotation) + vely_veh * std::sin(rotation);
    vely = vely_veh * std::cos(rotation) - velx_veh * std::sin(rotation);
    // //  SG_INFO("rotation = %lf,enu_x=%lf,enu_y=%lf", rotation, x, y);
}

void ObstacleContainer::VehCoord2EnuCoord(const double ego_x,
                                          const double ego_y,
                                          const double ego_yaw, double &x,
                                          double &y) {
    // //  SG_INFO("ego_x = %lf,ego_y =%lf,ego_yaw=%lf,x=%lf,y=%lf", ego_x,
    // ego_y,
    //         ego_yaw, x, y);
    double x_veh = x;
    double y_veh = y;

    // const double deg_to_rad = M_PI / 180;
    // const double rad_to_deg = 180 / M_PI;
    double rotation = (M_PI / 2 - ego_yaw);
    // double rotation = -ego_yaw;

    x = x_veh * std::cos(rotation) + y_veh * std::sin(rotation) + ego_x;
    y = y_veh * std::cos(rotation) - x_veh * std::sin(rotation) + ego_y;
}

void ObstacleContainer::UpdateMap(const std::string &obs_id,
                                  const ObsHistoryMsg &his_msg) {
    if (obstacles_history_msg_enu_.find(obs_id) ==
        obstacles_history_msg_enu_.end()) {
        std::vector<ObsHistoryMsg> his_msg_vec;
        his_msg_vec.emplace_back(his_msg);
        obstacles_history_msg_enu_[obs_id] = his_msg_vec;
    } else {
        obstacles_history_msg_enu_[obs_id].emplace_back(his_msg);
        if (obstacles_history_msg_enu_[obs_id].size() > his_list_len_) {
            auto it = obstacles_history_msg_enu_[obs_id].begin();
            obstacles_history_msg_enu_[obs_id].erase(it);
        }
    }
}

void ObstacleContainer::CleanMap(const ObstaclesInfo &obs_info) {
    std::unordered_set<std::string> obs_set;
    for (const ObstacleMsg &obstacle : obs_info.obstacles) {
        obs_set.insert(obstacle.track_id);
    }
    std::vector<std::string> wait_for_del_obs;
    for (auto &obs_map_ite : obstacles_history_msg_enu_) {
        if (obs_set.find(obs_map_ite.first) == obs_set.end()) {
            wait_for_del_obs.emplace_back(obs_map_ite.first);
        }
    }
    for (const std::string &str : wait_for_del_obs) {
        obstacles_history_msg_enu_.erase(str);
    }
}

void ObstacleContainer::Enu2VehCoordMap(const ObstaclesInfo &obs_info) {
    obstacles_history_msg_veh_.clear();
    double ego_x = obs_info.location.position.x;
    double ego_y = obs_info.location.position.y;
    double ego_yaw = obs_info.location.yaw * M_PI / 180;
    ego_yaw += M_PI / 2;
    while (ego_yaw < 0) {
        ego_yaw += 2 * M_PI;
    }
    while (ego_yaw > 2 * M_PI) {
        ego_yaw -= 2 * M_PI;
    }

    for (auto iter = obstacles_history_msg_enu_.begin();
         iter != obstacles_history_msg_enu_.end(); ++iter) {
        std::vector<ObsHistoryMsg> obs_veh_his_msg;
        std::string obs_id = iter->first;

        for (int i = 0; i < iter->second.size(); ++i) {
            double veh_x = iter->second[i].pos.x;
            double veh_y = iter->second[i].pos.y;
            double veh_velx = iter->second[i].velocity.x;
            double veh_vely = iter->second[i].velocity.y;
            double veh_yaw = iter->second[i].rotation.z;

            EnuCoord2VehCoord(ego_x, ego_y, ego_yaw, veh_x, veh_y, veh_yaw,
                              veh_velx, veh_vely);

            ObsHistoryMsg veh_his_msg;
            veh_his_msg.pos.x = veh_x;
            veh_his_msg.pos.y = veh_y;
            veh_his_msg.pos.z = 0;
            veh_his_msg.rotation.x = 0;
            veh_his_msg.rotation.y = 0;
            veh_his_msg.rotation.z = veh_yaw;
            veh_his_msg.velocity.x = veh_velx;
            veh_his_msg.velocity.x = veh_vely;
            veh_his_msg.velocity.x = 0;
            obs_veh_his_msg.emplace_back(std::move(veh_his_msg));
        }

        obstacles_history_msg_veh_[obs_id] = obs_veh_his_msg;
        // for (int i = 0; i < obs_veh_his_msg.size(); ++i) {
        //     //  SG_INFO("%s------------------------veh_x=%lf,veh_y=%lf",
        //             obs_id.c_str(), obs_veh_his_msg[i].pos.x,
        //             obs_veh_his_msg[i].pos.y);
        // }
    }
}

void ObstacleContainer::EnuCoord2VehCoord(const double ego_x,
                                          const double ego_y,
                                          const double ego_yaw, double &x,
                                          double &y, double &yaw, double &velx,
                                          double &vely) {
    double x_enu = x - ego_x;
    double y_enu = y - ego_y;
    double yaw_enu = yaw;
    double velx_enu = velx;
    double vely_enu = vely;
    // const double deg_to_rad = M_PI / 180;
    // const double rad_to_deg = 180 / M_PI;
    double rotation = -(M_PI / 2 - ego_yaw);
    // double rotation = -ego_yaw;

    x = x_enu * std::cos(rotation) + y_enu * std::sin(rotation);
    y = y_enu * std::cos(rotation) - x_enu * std::sin(rotation);
    yaw = yaw_enu - ego_yaw + M_PI / 2;
    while (yaw < 0) {
        yaw += 2 * M_PI;
    }
    while (yaw > 2 * M_PI) {
        yaw -= 2 * M_PI;
    }
    velx = velx_enu * std::cos(rotation) + vely_enu * std::sin(rotation);
    vely = vely_enu * std::cos(rotation) - velx_enu * std::sin(rotation);
}

void ObstacleContainer::EnuCoord2VehCoord(const double ego_x,
                                          const double ego_y,
                                          const double ego_yaw, double &x,
                                          double &y) {
    double x_enu = x - ego_x;
    double y_enu = y - ego_y;

    // const double deg_to_rad = M_PI / 180;
    // const double rad_to_deg = 180 / M_PI;
    double rotation = -(M_PI / 2 - ego_yaw);
    // double rotation = -ego_yaw;

    x = x_enu * std::cos(rotation) + y_enu * std::sin(rotation);
    y = y_enu * std::cos(rotation) - x_enu * std::sin(rotation);
}

void ObstacleContainer::UpdateTrajectory(
    const ObstaclesInfo &obs_info, const ObsTrafficType &obs_traffic_type) {
    for (int j = 0; j < obs_traffic_type.off_lane_obs.size(); ++j) {
        ObsTrajectories trajectories;
        int i = obs_traffic_type.off_lane_obs[j];
        std::string obs_id = obs_info.obstacles[i].track_id;
        trajectories = obs_info.obstacles[i].obs_trajectories;
        if (obstacles_history_msg_enu_.find(obs_id) !=
            obstacles_history_msg_enu_.end()) {
            obstacles_history_msg_enu_[obs_id].back().trajectories =
                trajectories;
            obstacles_history_msg_enu_[obs_id].back().predictor =
                FREE_MOVE_PREDICTOR;
        }
        if (obstacles_history_msg_veh_.find(obs_id) !=
            obstacles_history_msg_veh_.end()) {
            obstacles_history_msg_veh_[obs_id].back().trajectories =
                trajectories;
            obstacles_history_msg_veh_[obs_id].back().predictor =
                FREE_MOVE_PREDICTOR;
        }
    }
    for (int j = 0; j < obs_traffic_type.on_lane_obs.size(); ++j) {
        ObsTrajectories trajectories;
        int i = obs_traffic_type.on_lane_obs[j];
        std::string obs_id = obs_info.obstacles[i].track_id;
        trajectories = obs_info.obstacles[i].obs_trajectories;
        if (obstacles_history_msg_enu_.find(obs_id) !=
            obstacles_history_msg_enu_.end()) {
            obstacles_history_msg_enu_[obs_id].back().trajectories =
                trajectories;
            obstacles_history_msg_enu_[obs_id].back().predictor =
                LANE_SEQUENCE_PREDICTOR;
        }
        if (obstacles_history_msg_veh_.find(obs_id) !=
            obstacles_history_msg_veh_.end()) {
            obstacles_history_msg_veh_[obs_id].back().trajectories =
                trajectories;
            obstacles_history_msg_veh_[obs_id].back().predictor =
                LANE_SEQUENCE_PREDICTOR;
        }
    }
}

void ObstacleContainer::UpdateDebugInfo(const ObstaclesInfo &obs_info,
                                        std::string &debug_info) {
    debug_info += "{obstacles:";
    double ego_x = obs_info.location.position.x;
    double ego_y = obs_info.location.position.y;
    double ego_yaw = obs_info.location.yaw * M_PI / 180;
    ego_yaw += M_PI / 2;
    while (ego_yaw < 0) {
        ego_yaw += 2 * M_PI;
    }
    while (ego_yaw > 2 * M_PI) {
        ego_yaw -= 2 * M_PI;
    }
    for (auto iter = obs_info.obstacles.begin();
         iter != obs_info.obstacles.end(); ++iter) {
        double obs_x = iter->state.position.x;
        double obs_y = iter->state.position.y;
        double obs_yaw = iter->state.angular_deg.z * M_PI / 180;
        while (obs_yaw < 0) {
            obs_yaw += 2 * M_PI;
        }
        while (obs_yaw > 2 * M_PI) {
            obs_yaw -= 2 * M_PI;
        }
        double obs_length = iter->length;
        double obs_width = iter->width;
        // SG_INFO("obs_length = %lf, obs_width = %lf", obs_length,
        // obs_width);
        if (iter->obs_trajectories.size()) {
            debug_info += "state:";
            debug_info += "[" + std::to_string(obs_x) + "," +
                          std::to_string(obs_y) + "," +
                          std::to_string(obs_yaw) + "," +
                          std::to_string(obs_length) + "," +
                          std::to_string(obs_width) + "]";
            debug_info += "trajectory:";
            debug_info += "[";
            for (auto iter2 = iter->obs_trajectories[0].points.begin();
                 iter2 != iter->obs_trajectories[0].points.end(); ++iter2) {
                double traj_x = iter2->path_point.position_m.x;
                double traj_y = iter2->path_point.position_m.y;
                EnuCoord2VehCoord(ego_x, ego_y, ego_yaw, traj_x, traj_y);
                debug_info += "(" + std::to_string(traj_x) + "," +
                              std::to_string(traj_y) + ")";
            }
            debug_info += "]";
        }
    }
    debug_info += "obstacles_end}";
    debug_info += "{indicator:";
    debug_info += "ego:";
    debug_info += "(" + std::to_string(ego_x) + "," + std::to_string(ego_y) +
                  ")" + "mean_Error:";
    for (auto iter = obstacles_history_msg_enu_.begin();
         iter != obstacles_history_msg_enu_.end(); ++iter) {
        if (iter->second.size() == his_list_len_) {
            double obs_x = iter->second[0].pos.x;
            double obs_y = iter->second[0].pos.y;
            double mean_error = iter->second[0].mean_error;
            PREDICTOR predictor = iter->second[0].predictor;
            debug_info += "(" + std::to_string(obs_x) + "," +
                          std::to_string(obs_y) + "," +
                          std::to_string(mean_error) + "," +
                          std::to_string(predictor) + ")";
        }
    }
    debug_info += "indicator_end}";
}

void ObstacleContainer::UpdateDistanceTrajectory(
    const ObstaclesInfo &obs_info) {
    std::unordered_set<std::string> obs_set;
    std::vector<std::string> wait_for_del_obs;
    for (int i = 0; i < obs_info.obstacles.size(); ++i) {
        std::string obs_id = obs_info.obstacles[i].track_id;
        obs_set.insert(obs_id);
        double obs_x = obs_info.obstacles[i].state.position.x;
        double obs_y = obs_info.obstacles[i].state.position.y;
        double obs_velx = obs_info.obstacles[i].state.linear_velocity.x;
        double obs_vely = obs_info.obstacles[i].state.linear_velocity.y;
        double obs_vel = std::hypot(obs_velx, obs_vely);
        double obs_yaw = obs_info.obstacles[i].state.angular_deg.z;
        while (obs_yaw < 0) {
            obs_yaw += 2 * M_PI;
        }
        while (obs_yaw > 2 * M_PI) {
            obs_yaw -= 2 * M_PI;
        }

        if (obstacles_deque_.find(obs_id) == obstacles_deque_.end()) {
            std::tuple<double, double, double> pos =
                std::make_tuple(obs_x, obs_y, obs_yaw);

            std::deque<std::tuple<double, double, double>> pos_deque;
            pos_deque.emplace_back(pos);
            obstacles_deque_[obs_id] = pos_deque;
        } else {
            double x_final = std::get<0>(obstacles_deque_[obs_id].back());
            double y_final = std::get<1>(obstacles_deque_[obs_id].back());
            double distance = std::hypot(obs_x - x_final, obs_y - y_final);
            if (distance > 0.5) {
                std::tuple<double, double, double> pos =
                    std::make_tuple(obs_x, obs_y, obs_yaw);
                obstacles_deque_[obs_id].emplace_back(pos);

                if (obstacles_deque_[obs_id].size() > his_deque_len_) {
                    obstacles_deque_[obs_id].pop_front();
                }
            }

            if (obs_vel < 0.5) {
                std::tuple<double, double, double> pos =
                    std::make_tuple(obs_x, obs_y, obs_yaw);
                std::deque<std::tuple<double, double, double>> pos_deque;
                pos_deque.emplace_back(pos);
                obstacles_deque_[obs_id] = pos_deque;
            }
        }
    }

    for (auto &obs_deque_ite : obstacles_deque_) {
        if (obs_set.find(obs_deque_ite.first) == obs_set.end()) {
            wait_for_del_obs.emplace_back(obs_deque_ite.first);
        }
    }
    for (const std::string &str : wait_for_del_obs) {
        // SG_INFO("erase --- %s", str.c_str());
        obstacles_deque_.erase(str);
    }
}

void ObstacleContainer::CalculateEvaluationIndicator() {
    for (auto iter = obstacles_history_msg_enu_.begin();
         iter != obstacles_history_msg_enu_.end(); ++iter) {
        double msg_count = iter->second.size();
        for (int i = 0; i < msg_count - 1; ++i) {
            if (!iter->second[i].trajectories.size()) {
                continue;
            }
            double end_error;
            double end_pred_x = iter->second[i]
                                    .trajectories[0]
                                    .points[msg_count - i]
                                    .path_point.position_m.x;
            double end_pred_y = iter->second[i]
                                    .trajectories[0]
                                    .points[msg_count - i]
                                    .path_point.position_m.y;
            double end_real_x = iter->second.back().pos.x;
            double end_real_y = iter->second.back().pos.y;

            end_error =
                std::hypot(end_real_x - end_pred_x, end_real_y - end_pred_y);
            iter->second[i].end_error = end_error;
            // SG_INFO("end_error=%lf", end_error);
            int point_count = iter->second[i].trajectories[0].points.size();
            double mean_error = 0;
            double sum_error = 0;
            int num = msg_count - i;
            for (int j = 0; j < num; ++j) {
                double pred_x = iter->second[i]
                                    .trajectories[0]
                                    .points[j]
                                    .path_point.position_m.x;
                double pred_y = iter->second[i]
                                    .trajectories[0]
                                    .points[j]
                                    .path_point.position_m.y;
                double real_x = iter->second[i + j].pos.x;
                double real_y = iter->second[i + j].pos.y;
                double error = std::hypot(real_x - pred_x, real_y - pred_y);
                sum_error += error;
                // SG_INFO("error=%lf,sum_error=%lf", error, sum_error);
            }
            mean_error = sum_error / num;
            iter->second[i].mean_error = mean_error;
            // SG_INFO("sum_error=%lf,mean_error=%lf,num=%d", sum_error,
            //         mean_error, num);
        }
    }
}

void ObstacleContainer::UpdateObsSectionMap(const MonospacedLaneMsg &lane_msg,
                                            const double obs_x,
                                            const double obs_y,
                                            const std::string obs_id) {
    Point2d point_obs;
    point_obs.x = obs_x;
    point_obs.y = obs_y;

    for (int i = 0; i < lane_msg.monospaced_section.size(); ++i) {
        Point2d point_section_fl =
            lane_msg.monospaced_section[i].left_lane_line.lane_line.point_end;
        Point2d point_section_rl =
            lane_msg.monospaced_section[i].left_lane_line.lane_line.point_begin;
        Point2d point_section_fr =
            lane_msg.monospaced_section[i].right_lane_line.lane_line.point_end;

        Point2d point_section_rr = lane_msg.monospaced_section[i]
                                       .right_lane_line.lane_line.point_begin;

        if (common::IsInQuadrilateral(point_obs, point_section_fl,
                                      point_section_fr, point_section_rr,
                                      point_section_rl)) {
            Point2d point_center_line_begin =
                lane_msg.monospaced_section[i].center_line.point_begin;
            Point2d point_center_line_end =
                lane_msg.monospaced_section[i].center_line.point_end;
            double distance = common::DistancePoint2Line(
                point_center_line_begin, point_center_line_end, point_obs);
            auto pair = std::make_pair(i, distance);
            traffic_type_.map_obs_in_section[obs_id] = pair;
            return;
        }
    }
}

void ObstacleContainer::SetTrafficType(const ObsTrafficType &obs_traffic_type) {
    traffic_type_ = obs_traffic_type;
}

}  // namespace prediction_lib
}  // namespace jarvis