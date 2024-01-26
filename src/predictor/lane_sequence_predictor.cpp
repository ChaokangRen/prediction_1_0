#include "lane_sequence_predictor.h"
namespace jarvis {
namespace prediction_lib {

void LaneSequencePredictor::Init(const PredictionConf &prediction_conf) {
    prediction_total_time_ = prediction_conf.prediction_total_time;
    prediction_period_ = prediction_conf.prediction_period;
}

void LaneSequencePredictor::Predict(ObstaclesInfo &obs_info,
                                    const MonospacedLaneMsg &lane_msg,
                                    const ObsTrafficType &traffic_type,
                                    const HistoryMap &his_msg_veh,
                                    std::string &debug_info) {
    if (!lane_msg.monospaced_section.size()) {
        //  SG_INFO("lane_center_line not exist!");
        return;
    }

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

    int section_id_min = lane_msg.monospaced_section[0].center_line.id;
    int section_id_max = lane_msg.monospaced_section.back().center_line.id;

    for (int j = 0; j < traffic_type.on_lane_obs.size(); ++j) {
        int i = traffic_type.on_lane_obs[j];
        double obs_x = obs_info.obstacles[i].state.position.x;
        double obs_y = obs_info.obstacles[i].state.position.y;
        double obs_yaw = obs_info.obstacles[i].state.angular_deg.z * M_PI / 180;
        ObsTrajectory obs_trajectory;

        while (obs_yaw < 0) {
            obs_yaw += 2 * M_PI;
        }

        while (obs_yaw > 2 * M_PI) {
            obs_yaw -= 2 * M_PI;
        }
        // SG_INFO("origin_yaw=%lf,111obs_yaw=%lf",
        //         obs_info.obstacles[i].state.angular_deg.z, obs_yaw);
        double obs_enu_yaw = obs_yaw + ego_yaw - M_PI / 2;
        while (obs_enu_yaw > 2 * M_PI) {
            obs_enu_yaw -= 2 * M_PI;
        }

        //  SG_INFO("obs_x=%lf,obs_y=%lf,obs_enu_yaw=%lf", obs_x, obs_y,
        // obs_enu_yaw);
        double obs_velx = obs_info.obstacles[i].state.linear_velocity.x;
        double obs_vely = obs_info.obstacles[i].state.linear_velocity.y;

        double obs_vel = std::hypot(obs_velx, obs_vely);

        std::string obs_id = obs_info.obstacles[i].track_id;

        auto obs_map_iter = traffic_type.map_obs_in_section.find(obs_id);

        int sub_section = obs_map_iter->second.first;
        LinePoint origin_center_line =
            lane_msg.monospaced_section[sub_section].center_line;
        double distance = obs_map_iter->second.second;

        int s = 0;

        double lane_yaw = lane_msg.yaw_veh_rad;

        double lane_yaw_enu = lane_yaw - M_PI / 2 + ego_yaw;
        while (lane_yaw_enu > 2 * M_PI) {
            lane_yaw_enu -= 2 * M_PI;
        }

        lane_yaw = M_PI / 2 + 2 * (lane_yaw - M_PI / 2);

        double yaw_obs2lane = obs_yaw - lane_yaw;

        while (yaw_obs2lane < -M_PI) {
            yaw_obs2lane += 2 * M_PI;
        }

        while (yaw_obs2lane > M_PI) {
            yaw_obs2lane -= 2 * M_PI;
        }

        INTENSION intension;
        int section_id_target;
        FirstGetIntension(his_msg_veh, obs_id, origin_center_line, distance,
                          intension);
        SecondGetIntension(sub_section, lane_msg, obs_vel, intension,
                           section_id_target);

        int section_sub_target = section_id_target - section_id_min;
        LinePoint center_line =
            lane_msg.monospaced_section[section_sub_target].center_line;

        // if (intension == INTENSION::RIGHT) {
        //     SG_WARN("%s ------ RIGHT", obs_id.c_str());
        // }
        // if (intension == INTENSION::LEFT) {
        //     SG_WARN("%s ------ LEFT", obs_id.c_str());
        // }

        double x_target;
        double y_target;
        double yaw_target;
        GetTargetPoint(center_line, obs_vel, obs_x, obs_y, yaw_obs2lane,
                       intension, x_target, y_target, yaw_target);

        // SG_INFO("%s---x=%lf,y=%lf", obs_id.c_str(), x_target, y_target);

        const std::array<double, 2> start_x = {obs_x, obs_velx};
        const std::array<double, 2> end_x = {x_target,
                                             obs_vel * std::cos(yaw_target)};
        const std::array<double, 2> start_y = {obs_y, obs_vely};
        const std::array<double, 2> end_y = {y_target,
                                             obs_vel * std::sin(yaw_target)};

        double exit_time = GetBestTime(start_x, end_x, start_y, end_y);
        std::array<double, 4> x_coeffs =
            ComputePolynomial(start_x, end_x, exit_time);
        std::array<double, 4> y_coeffs =
            ComputePolynomial(start_y, end_y, exit_time);

        double time_sequence = 0.0;

        int num = prediction_total_time_ / prediction_period_ + 1;

        int num1 = 0;

        double time_to_change = std::min(exit_time, prediction_total_time_);
        int num_first = time_to_change / prediction_period_ + 1;
        for (; num1 < num_first; ++num1) {
            TrajectoryPoint trajectory_point;
            trajectory_point.path_point.position_m.x =
                (EvaluateCubicPolynomial(x_coeffs, time_sequence, 0));
            trajectory_point.path_point.position_m.y =
                (EvaluateCubicPolynomial(y_coeffs, time_sequence, 0));

            common::VehCoord2EnuCoord(ego_x, ego_y, ego_yaw,
                                      trajectory_point.path_point.position_m.x,
                                      trajectory_point.path_point.position_m.y);

            trajectory_point.path_point.position_m.z = 0;
            trajectory_point.path_point.theta_rad =
                std::atan2(
                    EvaluateCubicPolynomial(y_coeffs, time_sequence, 1),
                    EvaluateCubicPolynomial(x_coeffs, time_sequence, 1)) +
                ego_yaw - M_PI / 2;
            // SG_INFO(
            //     "num1=%d,traj_theta=%lf,ego_yaw=%lf,enu_theta=%lf", num1,
            //     std::atan2(EvaluateCubicPolynomial(y_coeffs, time_sequence,
            //     1),
            //                EvaluateCubicPolynomial(x_coeffs, time_sequence,
            //                1)),
            //     ego_yaw, trajectory_point.theta_rad);
            // SG_INFO("obs_yaw=%lf", obs_yaw);
            trajectory_point.path_point.kappa = 0;
            trajectory_point.path_point.dkappa = 0;
            trajectory_point.velocity_mps =
                std::hypot(EvaluateCubicPolynomial(x_coeffs, time_sequence, 1),
                           EvaluateCubicPolynomial(y_coeffs, time_sequence, 1));

            // SG_INFO("%s -- -- -- traj_vel=%lf", obs_id.c_str(),
            //         trajectory_point.velocity_mps);
            trajectory_point.relative_time_s = time_sequence;
            obs_trajectory.points.emplace_back(std::move(trajectory_point));
            time_sequence += prediction_period_;
        }

        for (; num1 < num; ++num1) {
            TrajectoryPoint trajectory_point;
            int traj_size = obs_trajectory.points.size();
            double vel_pre = obs_trajectory.points[traj_size - 1].velocity_mps;
            double theta_pre =
                obs_trajectory.points[traj_size - 1].path_point.theta_rad;
            double x_pre =
                obs_trajectory.points[traj_size - 1].path_point.position_m.x;
            double x_pre_pre =
                obs_trajectory.points[traj_size - 2].path_point.position_m.x;
            double y_pre =
                obs_trajectory.points[traj_size - 1].path_point.position_m.y;
            double y_pre_pre =
                obs_trajectory.points[traj_size - 2].path_point.position_m.y;
            double dx = x_pre - x_pre_pre;
            double dy = y_pre - y_pre_pre;
            trajectory_point.path_point.position_m.x = x_pre + dx;
            trajectory_point.path_point.position_m.y = y_pre + dy;
            trajectory_point.path_point.position_m.z = 0;
            trajectory_point.path_point.theta_rad = theta_pre;
            trajectory_point.path_point.kappa = 0;
            trajectory_point.path_point.dkappa = 0;
            trajectory_point.velocity_mps = vel_pre;
            trajectory_point.relative_time_s = time_sequence;
            obs_trajectory.points.emplace_back(std::move(trajectory_point));
            time_sequence += prediction_period_;
        }

        // // print history trajectory
        // for (int a = 0; a < his_msg_vec.size(); ++a) {
        //     ObsTrajectoryPoint trajectory_point;
        //     double x = his_msg_vec[a].pos.x;
        //     double y = his_msg_vec[a].pos.y;

        //     VehCoord2EnuCoord(ego_x, ego_y, ego_yaw, x, y);
        //     trajectory_point.position_m.x = x;
        //     trajectory_point.position_m.y = y;
        //     obs_trajectory.points.emplace_back(std::move(trajectory_point));
        // }

        obs_info.obstacles[i].obs_trajectories.emplace_back(obs_trajectory);
        // SG_INFO("obs_id=%s", obs_info.obstacles[i].track_id.c_str());
        // for (int j = 0;
        //      j < obs_info.obstacles[i].obs_trajectories[0].points.size();
        //      ++j) {
        //     SG_INFO("traj_x = %lf,traj_y = %lf",
        //             obs_info.obstacles[i]
        //                 .obs_trajectories[0]
        //                 .points[j]
        //                 .position_m.x,
        //             obs_info.obstacles[i]
        //                 .obs_trajectories[0]
        //                 .points[j]
        //                 .position_m.y);
        // }
    }
}

void LaneSequencePredictor::FirstGetIntension(const HistoryMap &his_msg_veh,
                                              const std::string obs_id,
                                              const LinePoint &center_line,
                                              const double distance,
                                              INTENSION &intension) {
    std::vector<ObsHistoryMsg> his_msg_vec;
    his_msg_vec = his_msg_veh.find(obs_id)->second;
    if (his_msg_vec.size() <= 5) {
        intension = INTENSION::STRAIGHT;
    } else {
        Point2d center_line_end = center_line.point_end;
        Point2d center_line_begin = center_line.point_begin;

        int count_his = his_msg_vec.size();
        int count_max = 10;
        int count_real = count_his < count_max ? count_his : count_max;
        std::vector<double> dis;

        for (int i = 0; i < count_real; ++i) {
            Point2d his;
            his.x = his_msg_vec[count_his - 1 - i].pos.x;
            his.y = his_msg_vec[count_his - 1 - i].pos.y;
            double distance2Lane = common::DistancePoint2Line(
                center_line_begin, center_line_end, his);
            dis.emplace_back(distance2Lane);
        }
        int count_right = 0;
        for (int i = 0; i < dis.size() - 1; ++i) {
            if (dis[i + 1] - dis[i] < 0) {
                ++count_right;
            }
        }
        Point2d his_begin;
        his_begin.x = his_msg_vec[count_his - count_max].pos.x;
        his_begin.y = his_msg_vec[count_his - count_max].pos.y;
        Point2d his_end;
        his_end.x = his_msg_vec.back().pos.x;
        his_end.y = his_msg_vec.back().pos.y;

        Point2d foot_begin = common::PointVertical(center_line_begin,
                                                   center_line_end, his_begin);
        Point2d foot_end =
            common::PointVertical(center_line_begin, center_line_end, his_end);

        double dy = common::Length(foot_begin, foot_end);
        double dx = dis[0] - dis.back();

        double yaw_vel = std::atan2(dy, dx);

        bool trend_left = false;
        bool trend_right = false;

        // SG_INFO(
        //     "%s --- "
        //     "dx=%lf,dy=%lf,count_right=%d,count_real=%d,yaw_vel=%lf,distance=%"
        //     "lf",
        //     obs_id.c_str(), dx, dy, count_right, count_real, yaw_vel,
        //     distance);

        if ((count_right >= ((count_real - 1) * 2 / 3)) &&
            (yaw_vel < (88.5 / 180 * M_PI))) {
            trend_right = true;
        }

        if ((count_right <= ((count_real - 1) / 3)) &&
            (yaw_vel > (91.5 / 180 * M_PI))) {
            trend_left = true;
        }

        if (trend_right && distance > 0.4) {
            intension = INTENSION::RIGHT;
        } else if (trend_left && distance < -0.4) {
            intension = INTENSION::LEFT;
        } else {
            intension = INTENSION::STRAIGHT;
        }
        // SG_INFO("intension=%d", intension);
    }

    // obs_his_msg[obs_id].back().intension = intension;
    // int count_s = 0;
    // int count_l = 0;
    // int count_r = 0;
    // int count_t = 0;
    // auto new_vec = obs_his_msg.find(obs_id)->second;
    // auto iter = new_vec.end() - 1;
    // for (int i = 0; i < 20 || iter == new_vec.begin(); ++i) {
    //     switch (iter->intension) {
    //         case INTENSION::STRAIGHT:
    //             ++count_s;
    //             --iter;
    //             continue;
    //         case INTENSION::LEFT:
    //             ++count_l;
    //             --iter;
    //             continue;
    //         case INTENSION::RIGHT:
    //             ++count_r;
    //             --iter;
    //             continue;
    //         case INTENSION::TURN:
    //             ++count_t;
    //             --iter;
    //             continue;
    //         default:
    //             ++count_s;
    //             --iter;
    //             continue;
    //     }
    // }
    // // SG_INFO("%d %d %d %d", count_s, count_l, count_r, count_t);

    // if (count_l > 4 && distance < 0) {
    //     intension = INTENSION::LEFT;
    //     return;
    // }
    // if (count_r > 4 && distance > 0) {
    //     intension = INTENSION::RIGHT;
    //     return;
    // }
    // if (count_t > 4) {
    //     intension = INTENSION::TURN;
    //     return;
    // }
    // // intension = INTENSION::STRAIGHT;
}

void LaneSequencePredictor::SecondGetIntension(
    const int32_t sub_section, const MonospacedLaneMsg &lane_msg,
    const double obs_vel, INTENSION &intension, int32_t &section_id_target) {
    int section_id = lane_msg.monospaced_section[sub_section].center_line.id;
    int section_id_max = lane_msg.monospaced_section.back().center_line.id;
    int section_id_min = lane_msg.monospaced_section[0].center_line.id;

    if (obs_vel < 6) {
        intension = INTENSION::STRAIGHT;
        section_id_target = section_id;
        return;
    }

    if (intension == INTENSION::RIGHT) {
        if (section_id >= 0) {
            if (section_id == section_id_max) {
                intension = INTENSION::STRAIGHT;
                section_id_target = section_id;
            } else {
                intension = INTENSION::RIGHT;
                section_id_target = section_id + 1;
            }

        } else {
            if (section_id == section_id_min) {
                intension = INTENSION::STRAIGHT;
                section_id_target = section_id;
            } else {
                intension = INTENSION::RIGHT;
                section_id_target = section_id - 1;
            }
        }
    } else if (intension == INTENSION::LEFT) {
        if (section_id == 0 && section_id_min < 0) {
            intension = INTENSION::TURN;
            section_id_target = section_id - 1;
        } else if (section_id == 0 && section_id_min == 0) {
            intension = INTENSION::STRAIGHT;
            section_id_target = section_id;
        } else if (section_id == -1 && section_id_max > -1) {
            intension = INTENSION::TURN;
            section_id_target = section_id + 1;
        } else if (section_id == -1 && section_id_max == -1) {
            intension = INTENSION::STRAIGHT;
            section_id_target = section_id;
        } else if (section_id > 0) {
            intension = INTENSION::LEFT;
            section_id_target = section_id - 1;
        } else {
            intension = INTENSION::LEFT;
            section_id_target = section_id + 1;
        }
    } else if (intension == INTENSION::TURN) {
        if (section_id == 0 && section_id_min < 0) {
            intension = INTENSION::TURN;
            section_id_target = section_id - 1;
        } else if (section_id == -1 && section_id_max > -1) {
            intension = INTENSION::TURN;
            section_id_target = section_id + 1;
        } else {
            intension = INTENSION::STRAIGHT;
            section_id_target = section_id;
        }
    } else {
        intension = INTENSION::STRAIGHT;
        section_id_target = section_id;
    }

    if (intension == INTENSION::LEFT) {
        LaneLineType type_left = lane_msg.monospaced_section[sub_section]
                                     .left_lane_line.lane_line_type;
        if (type_left == LaneLineType::SOLID ||
            type_left == LaneLineType::BROKEN_SOLID ||
            type_left == LaneLineType::SOLID_SOLID) {
            intension == INTENSION::STRAIGHT;
            section_id_target = section_id;
        }
    }
    if (intension == INTENSION::RIGHT) {
        LaneLineType type_left = lane_msg.monospaced_section[sub_section]
                                     .right_lane_line.lane_line_type;
        if (type_left == LaneLineType::SOLID ||
            type_left == LaneLineType::SOLID_BROKEN ||
            type_left == LaneLineType::SOLID_SOLID) {
            intension == INTENSION::STRAIGHT;
            section_id_target = section_id;
        }
    }
}

CenterLineMsg LaneSequencePredictor::GetCenterLine(
    const int32_t id, const LaneCenterLine &lane_center_line) {
    CenterLineMsg center_line;
    for (auto lane_iter = lane_center_line.lane_center_line_.begin();
         lane_iter != lane_center_line.lane_center_line_.end(); ++lane_iter) {
        // //  SG_INFO("iter.id = %d,id=%d", lane_iter->id, id);
        if (lane_iter->id == id) {
            // //  SG_INFO("1111111111111111");
            int sub = std::distance(lane_center_line.lane_center_line_.begin(),
                                    lane_iter);
            // //  SG_INFO("sub= %d", sub);
            center_line = lane_center_line.lane_center_line_[sub];
            break;
        }
    }

    return center_line;
}

void LaneSequencePredictor::GetTargetPoint(
    const LinePoint &center_line, const double vel, const double obs_x,
    const double obs_y, const double yaw_obs2lane, const INTENSION intension,
    double &x_target, double &y_target, double &yaw_target) {
    if (intension != INTENSION::TURN) {
        Point2d point_center_line_begin = center_line.point_begin;
        Point2d point_center_line_end = center_line.point_end;
        Point2d point_obs;
        point_obs.x = obs_x;
        point_obs.y = obs_y;
        Point2d point_vertical = common::PointVertical(
            point_center_line_begin, point_center_line_end, point_obs);
        double length = vel * prediction_total_time_;
        double lane_yaw = center_line.yaw_veh_rad;
        if (yaw_obs2lane < M_PI / 2 && yaw_obs2lane > -M_PI / 2) {
            x_target = point_vertical.x + length * std::cos(lane_yaw);
            y_target = point_vertical.y + length * std::sin(lane_yaw);
            yaw_target = lane_yaw;
        } else {
            x_target = point_vertical.x - length * std::cos(lane_yaw);
            y_target = point_vertical.y - length * std::sin(lane_yaw);
            yaw_target = lane_yaw - M_PI;
        }
    }
}

double LaneSequencePredictor::GetBestTime(const std::array<double, 2> &start_x,
                                          const std::array<double, 2> &end_x,
                                          const std::array<double, 2> &start_y,
                                          const std::array<double, 2> &end_y) {
    // Generate candidate finish times.
    std::vector<double> candidate_times = GenerateCandidateTimes();
    double best_time = 0.0;
    double best_cost = std::numeric_limits<double>::infinity();
    for (size_t i = 0; i < candidate_times.size(); ++i) {
        double time_to_exit = candidate_times[i];
        std::array<double, 4> x_coeffs =
            ComputePolynomial(start_x, end_x, time_to_exit);
        std::array<double, 4> y_coeffs =
            ComputePolynomial(start_y, end_y, time_to_exit);
        double cost_of_trajectory =
            CostFunction(x_coeffs, y_coeffs, time_to_exit);
        if (cost_of_trajectory <= best_cost) {
            best_cost = cost_of_trajectory;
            best_time = time_to_exit;
        }
    }
    return best_time;
}

std::vector<double> LaneSequencePredictor::GenerateCandidateTimes() {
    std::vector<double> candidate_times;
    double t = 1.0;
    double time_gap = 0.2;
    while (t <= 8) {
        candidate_times.push_back(t);
        t += time_gap;
    }
    return candidate_times;
}

double LaneSequencePredictor::EvaluateCubicPolynomial(
    const std::array<double, 4> &coefs, const double t, const uint32_t order) {
    switch (order) {
        case 0: {
            return ((coefs[3] * t + coefs[2]) * t + coefs[1]) * t + coefs[0];
        }
        case 1: {
            return (3.0 * coefs[3] * t + 2.0 * coefs[2]) * t + coefs[1];
        }
        case 2: {
            return 6.0 * coefs[3] * t + 2.0 * coefs[2];
        }
        default:
            return 0;
    }
}

double LaneSequencePredictor::CostFunction(
    const std::array<double, 4> &x_coeffs,
    const std::array<double, 4> &y_coeffs, const double time_to_exit) {
    double t = 0.0;
    double cost = 0.0;

    double velx_0 = EvaluateCubicPolynomial(x_coeffs, t, 2);
    double vely_0 = EvaluateCubicPolynomial(y_coeffs, t, 2);
    double vel_pre = std::hypot(velx_0, vely_0);

    while (t <= time_to_exit) {
        double x_1 = EvaluateCubicPolynomial(x_coeffs, t, 1);
        double x_2 = EvaluateCubicPolynomial(x_coeffs, t, 2);
        double y_1 = EvaluateCubicPolynomial(y_coeffs, t, 1);
        double y_2 = EvaluateCubicPolynomial(y_coeffs, t, 2);

        double vel_now = std::hypot(x_2, y_2);
        double dvel = vel_now - vel_pre;
        // cost = curvature * v^2 + time_to_exit + dvel * 10
        cost = std::max(cost,
                        std::abs(x_1 * y_2 - y_1 * x_2) / std::hypot(x_1, y_1) +
                            3 * time_to_exit + dvel * 10);

        vel_pre = vel_now;
        t += prediction_period_;
    }
    return cost;
}

void LaneSequencePredictor::Rotation(const double end_x, const double end_y,
                                     const double yaw, double &x, double &y) {
    double rotation = -yaw;
    double x1 = x - end_x;
    double y1 = y - end_y;
    x = x1 * std::cos(rotation) + y1 * std::sin(rotation) + end_x;
    y = y1 * std::cos(rotation) - x1 * std::sin(rotation) + end_y;
}

}  // namespace prediction_lib
}  // namespace jarvis