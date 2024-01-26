#include "junction_predictor.h"

namespace jarvis {
namespace prediction_lib {

void JunctionPredictor::Init(const PredictionConf &prediction_conf) {
    prediction_total_time_ = prediction_conf.prediction_total_time;
    prediction_period_ = prediction_conf.prediction_period;
}

void JunctionPredictor::Predict(ObstaclesInfo &obs_info,
                                const ObstaclesInfo &obs_info_enu,
                                ObsTrafficType &traffic_type,
                                const HistoryMap &obs_his_msg,
                                const ObsDeque &obstacles_deque,
                                const MonospacedLaneMsg &lane_msg,
                                std::string &debug_info) {
    GetInPredict(obs_info, obs_info_enu, lane_msg, traffic_type);
    FilterBySolidLane(obs_info, traffic_type, lane_msg);
    JunctionTurn(obs_info, obs_info_enu, obs_his_msg, traffic_type);
    GetOutPredict(obs_info, obs_info_enu, obs_his_msg, obstacles_deque,
                  lane_msg, traffic_type);
}

void JunctionPredictor::GetInPredict(ObstaclesInfo &obs_info,
                                     const ObstaclesInfo &obs_info_enu,
                                     const MonospacedLaneMsg &lane_msg,
                                     ObsTrafficType &obs_traffic_type) {
    if (!lane_msg.monospaced_section.size()) {
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

    std::vector<Point2d> points_lane;
    for (int i = 0; i < lane_msg.monospaced_lane.size(); ++i) {
        Point2d point_lane = lane_msg.monospaced_lane[i].lane_line.point_begin;
        common::VehCoord2EnuCoord(ego_x, ego_y, ego_yaw, point_lane.x,
                                  point_lane.y);
        points_lane.emplace_back(std::move(point_lane));
    }

    std::vector<int> obs_erase;

    for (int j = 0; j < obs_traffic_type.off_lane_obs.size(); ++j) {
        int i = obs_traffic_type.off_lane_obs[j];
        std::string id = obs_info.obstacles[i].track_id;
        double obs_velx = obs_info.obstacles[i].state.linear_velocity.x;
        double obs_vely = obs_info.obstacles[i].state.linear_velocity.y;
        double obs_vel = std::hypot(obs_velx, obs_vely);
        double pos_x = obs_info_enu.obstacles[i].state.position.x;
        double pos_y = obs_info_enu.obstacles[i].state.position.y;
        ObsType type = obs_info.obstacles[i].type;
        if (type != ObsType::VEHICLE && type != ObsType::CYCLIST) {
            continue;
        }
        double obs_yaw = obs_info.obstacles[i].state.angular_deg.z * M_PI / 180;
        while (obs_yaw < 0) {
            obs_yaw += 2 * M_PI;
        }
        while (obs_yaw > 2 * M_PI) {
            obs_yaw -= 2 * M_PI;
        }
        double lane_yaw = lane_msg.yaw_veh_rad;

        double theta = obs_yaw - lane_yaw;
        while (theta < -2 * M_PI) {
            theta += 2 * M_PI;
        }
        while (theta > 2 * M_PI) {
            obs_yaw -= 2 * M_PI;
        }

        if (fabs(theta) > M_PI / 3 || obs_vel < 3) {
            continue;
        }

        double lane_yaw_enu = lane_yaw - M_PI / 2 + ego_yaw;

        ObsTrajectory trajectory = obs_info.obstacles[i].obs_trajectories[0];

        int sub_section = -1;
        GetIntersectSub(trajectory, points_lane, lane_yaw_enu, sub_section);

        if (sub_section != -1) {
            double obs_x = obs_info.obstacles[i].state.position.x;
            double obs_y = obs_info.obstacles[i].state.position.y;
            Point2d point_obs;
            point_obs.x = obs_x;
            point_obs.y = obs_y;
            UpdateTrafficTypeMap(lane_msg, obs_traffic_type, id, sub_section,
                                 point_obs);
            LinePoint center_line =
                lane_msg.monospaced_section[sub_section].center_line;
            ObsTrajectory obs_trajectory = GetNewLaneTrajectory(
                center_line, obs_info.obstacles[i], ego_x, ego_y, ego_yaw);
            obs_info.obstacles[i].obs_trajectories.clear();
            obs_info.obstacles[i].obs_trajectories.emplace_back(
                std::move(obs_trajectory));
            obs_traffic_type.get_in_lane_obs.emplace_back(i);
            obs_erase.emplace_back(i);
        }
    }
    for (int i = 0; i < obs_erase.size(); ++i) {
        auto iter = find(obs_traffic_type.off_lane_obs.begin(),
                         obs_traffic_type.off_lane_obs.end(), obs_erase[i]);
        if (iter != obs_traffic_type.off_lane_obs.end()) {
            obs_traffic_type.off_lane_obs.erase(iter);
        }
    }
}

void JunctionPredictor::GetOutPredict(ObstaclesInfo &obs_info,
                                      const ObstaclesInfo &obs_info_enu,
                                      const HistoryMap &obs_his_msg,
                                      const ObsDeque &obstacles_deque,
                                      const MonospacedLaneMsg &lane_msg,
                                      ObsTrafficType &obs_traffic_type) {
    if (!lane_msg.monospaced_section.size()) {
        return;
    }

    bool has_stop_line = false;
    for (int i = 0; i < lane_msg.monospaced_section.size(); ++i) {
        if (lane_msg.monospaced_section[i].has_stop_line == true) {
            has_stop_line = true;
            break;
        }
    }

    if (!has_stop_line) {
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
    for (int j = 0; j < obs_traffic_type.on_lane_obs.size(); ++j) {
        int i = obs_traffic_type.on_lane_obs[j];

        Point2d traj_end;
        traj_end.x = obs_info.obstacles[i]
                         .obs_trajectories[0]
                         .points.back()
                         .path_point.position_m.x;
        traj_end.y = obs_info.obstacles[i]
                         .obs_trajectories[0]
                         .points.back()
                         .path_point.position_m.y;

        Point2d lane_lf =
            lane_msg.monospaced_section[0].left_lane_line.lane_line.point_end;
        Point2d lane_lr =
            lane_msg.monospaced_section[0].left_lane_line.lane_line.point_begin;
        Point2d lane_rf = lane_msg.monospaced_section.back()
                              .left_lane_line.lane_line.point_end;
        Point2d lane_rr = lane_msg.monospaced_section.back()
                              .left_lane_line.lane_line.point_begin;

        common::VehCoord2EnuCoord(ego_x, ego_y, ego_yaw, lane_lf.x, lane_lf.y);
        common::VehCoord2EnuCoord(ego_x, ego_y, ego_yaw, lane_lr.x, lane_lr.y);
        common::VehCoord2EnuCoord(ego_x, ego_y, ego_yaw, lane_rf.x, lane_rf.y);
        common::VehCoord2EnuCoord(ego_x, ego_y, ego_yaw, lane_rr.x, lane_rr.y);

        if (common::IsInQuadrilateral(traj_end, lane_lf, lane_rf, lane_rr,
                                      lane_lr)) {
            continue;
        }
        obs_info.obstacles[i].obs_trajectories.clear();

        ObsTrajectory obs_trajectory;
        double pos_x = obs_info_enu.obstacles[i].state.position.x;
        double pos_y = obs_info_enu.obstacles[i].state.position.y;
        double vel_x = obs_info_enu.obstacles[i].state.linear_velocity.x;
        double vel_y = obs_info_enu.obstacles[i].state.linear_velocity.y;
        double vel = std::hypot(vel_x, vel_y);
        double acc_x = 0;
        double acc_y = 0;
        std::string id = obs_info_enu.obstacles[i].track_id;

        const auto his_msg = obs_his_msg.find(id)->second;

        double vx0 = his_msg.front().velocity.x;
        double vy0 = his_msg.front().velocity.y;
        double vx1 = his_msg.back().velocity.x;
        double vy1 = his_msg.back().velocity.y;
        double sx = fabs(his_msg.back().pos.x - his_msg.front().pos.x);
        double sy = fabs(his_msg.back().pos.y - his_msg.front().pos.y);
        double v0 = std::hypot(vx0, vy0);
        double v1 = std::hypot(vx1, vy1);
        double s = std::hypot(sx, sy);
        double acc = (v1 * v1 - v0 * v0) / 2 / s;
        if (his_msg.size() > 5 && acc < -3 && acc > -10) {
            double time_to_stop = v1 / fabs(acc);
            if (time_to_stop != 0) {
                acc_x = -vx1 / time_to_stop;
                acc_y = -vy1 / time_to_stop;
            }
        }
        double dtheta = 0;

        Eigen::Vector2d position(pos_x, pos_y);
        Eigen::Vector2d velocity(vel_x, vel_y);
        Eigen::Vector2d acceleration(acc_x, acc_y);
        double theta = obs_info_enu.obstacles[i].state.angular_deg.z;

        std::vector<Point3d> points;
        const auto obs_his_deque = obstacles_deque.find(id)->second;

        int deque_size = obs_his_deque.size();
        if (deque_size > 3) {
            Point3d point1;
            point1.x = std::get<0>(obs_his_deque.front());
            point1.y = std::get<1>(obs_his_deque.front());
            points.emplace_back(point1);

            Point3d point2;
            point2.x =
                std::get<0>(obs_his_deque[floor(obs_his_deque.size() / 2) - 1]);
            point2.y =
                std::get<1>(obs_his_deque[floor(obs_his_deque.size() / 2) - 1]);

            points.emplace_back(point2);

            Point3d point3;
            point3.x = std::get<0>(obs_his_deque.back());
            point3.y = std::get<1>(obs_his_deque.back());
            points.emplace_back(point3);
        }

        double ox;
        double oy;
        double r;

        if (common::CaculateRadius(points, r, ox, oy) &&
            obs_his_deque.size() > 10 && acc_x == 0 && acc_y == 0) {
            // r = std::max(r, 20.0);
            DrawFreeMoveTrajectoryPoints(obstacles_deque, vel, id, ox, oy, r,
                                         prediction_total_time_,
                                         prediction_period_, &obs_trajectory);
        } else {
            dtheta = 0;
            DrawFreeMoveTrajectoryPoints(
                position, velocity, acceleration, theta, dtheta, 0.0,
                prediction_total_time_, prediction_period_, &obs_trajectory);
        }
        obs_info.obstacles[i].obs_trajectories.emplace_back(obs_trajectory);
    }
}

void JunctionPredictor::UpdateTrafficTypeMap(const MonospacedLaneMsg &lane_msg,
                                             ObsTrafficType &traffic_type,
                                             const std::string id,
                                             const int sub_section,
                                             const Point2d point_obs) {
    Point2d point_center_line_begin =
        lane_msg.monospaced_section[sub_section].center_line.point_begin;
    Point2d point_center_line_end =
        lane_msg.monospaced_section[sub_section].center_line.point_end;
    double distance = common::DistancePoint2Line(
        point_center_line_begin, point_center_line_end, point_obs);
    traffic_type.map_obs_in_section[id] = std::make_pair(
        lane_msg.monospaced_section[sub_section].center_line.id, distance);
}

void JunctionPredictor::GetIntersectSub(const ObsTrajectory &trajectory,
                                        const std::vector<Point2d> &points_lane,
                                        const double lane_yaw_enu,
                                        int &section_id) {
    Point2d point_traj_1;
    Point2d point_traj_2;
    Point2d section_begin = points_lane[0];
    Point2d section_end = points_lane.back();

    bool is_intersect = false;
    double diff_min = std::numeric_limits<double>::max();
    Point2d traj_parallel;
    Point2d traj_intersect;

    for (int i = 0; i < (trajectory.points.size() - 1); ++i) {
        point_traj_1.x = trajectory.points[i].path_point.position_m.x;
        point_traj_1.y = trajectory.points[i].path_point.position_m.y;
        point_traj_2.x = trajectory.points[i + 1].path_point.position_m.x;
        point_traj_2.y = trajectory.points[i + 1].path_point.position_m.y;
        double point_yaw = trajectory.points[i].path_point.theta_rad;
        double diff_yaw = point_yaw - lane_yaw_enu;
        while (diff_yaw < -M_PI) {
            diff_yaw += 2 * M_PI;
        }
        while (diff_yaw > M_PI) {
            diff_yaw -= 2 * M_PI;
        }
        diff_yaw = std::fabs(diff_yaw);
        if (common::IsIntersectedByLineSection(section_begin, section_end,
                                               point_traj_1, point_traj_2) &&
            (!is_intersect)) {
            traj_intersect = point_traj_1;
            is_intersect = true;
        }
        if (diff_yaw < diff_min) {
            diff_min = diff_yaw;
            traj_parallel = point_traj_1;
        }

        // SG_INFO("point_yaw=%lf,lane_yaw_enu=%lf,diff_yaw=%lf,diff_min=%lf",
        //         point_yaw, lane_yaw_enu, diff_yaw, diff_min);
    }

    if (is_intersect) {
        if (diff_min > 0.1) {
            traj_parallel = traj_intersect;
        }
        Point2d traj_foot =
            common::PointExtend(traj_parallel, 10, lane_yaw_enu);

        Point2d point_section_l;
        Point2d point_section_r;
        for (int i = 0; i < points_lane.size() - 1; ++i) {
            point_section_l = points_lane[i];
            point_section_r = points_lane[i + 1];
            if (common::IsIntersectedSectionAndStraightLine(
                    traj_foot, traj_parallel, point_section_l,
                    point_section_r)) {
                section_id = i;
                break;
            }
        }
    }
}

ObsTrajectory JunctionPredictor::GetNewLaneTrajectory(
    const LinePoint center_line, const ObstacleMsg &obstacle,
    const double ego_x, const double ego_y, const double ego_yaw) {
    ObsTrajectory obs_trajectory;
    double x_target;
    double y_target;
    double yaw_target;
    Point2d point_center_line_begin = center_line.point_begin;
    Point2d point_center_line_end = center_line.point_end;

    double obs_x = obstacle.state.position.x;
    double obs_y = obstacle.state.position.y;
    double obs_velx = obstacle.state.linear_velocity.x;
    double obs_vely = obstacle.state.linear_velocity.y;
    double obs_vel = std::hypot(obs_velx, obs_vely);

    Point2d point_obs;
    point_obs.x = obs_x;
    point_obs.y = obs_y;
    Point2d point_vertical = common::PointVertical(
        point_center_line_begin, point_center_line_end, point_obs);

    double vel = std::hypot(obstacle.state.linear_velocity.x,
                            obstacle.state.linear_velocity.y);

    double length = vel * prediction_total_time_;
    double lane_yaw = center_line.yaw_veh_rad;

    x_target = point_vertical.x + length * std::cos(lane_yaw);
    y_target = point_vertical.y + length * std::sin(lane_yaw);
    yaw_target = lane_yaw;
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
            std::atan2(EvaluateCubicPolynomial(y_coeffs, time_sequence, 1),
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
    return obs_trajectory;
}

double JunctionPredictor::GetBestTime(const std::array<double, 2> &start_x,
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

std::vector<double> JunctionPredictor::GenerateCandidateTimes() {
    std::vector<double> candidate_times;
    double t = 1.0;
    double time_gap = 0.2;
    while (t <= 8) {
        candidate_times.push_back(t);
        t += time_gap;
    }
    return candidate_times;
}

double JunctionPredictor::EvaluateCubicPolynomial(
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

double JunctionPredictor::CostFunction(const std::array<double, 4> &x_coeffs,
                                       const std::array<double, 4> &y_coeffs,
                                       const double time_to_exit) {
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

void JunctionPredictor::JunctionTurn(ObstaclesInfo &obs_info,
                                     const ObstaclesInfo &obs_info_enu,
                                     const HistoryMap &obs_his_msg,
                                     const ObsTrafficType &obs_traffic_type) {
    for (int j = 0; j < obs_traffic_type.off_lane_obs.size(); ++j) {
        int i = obs_traffic_type.off_lane_obs[j];

        ObsType type = obs_info.obstacles[i].type;
        if (type != ObsType::VEHICLE) {
            continue;
        }

        double traj_yaw_end = obs_info.obstacles[i]
                                  .obs_trajectories[0]
                                  .points.back()
                                  .path_point.theta_rad;
        double traj_yaw_begin = obs_info.obstacles[i]
                                    .obs_trajectories[0]
                                    .points[0]
                                    .path_point.theta_rad;

        // if (fabs(traj_yaw_end - traj_yaw_begin) > M_PI / 6) {
        //     continue;
        // }

        std::string id = obs_info_enu.obstacles[i].track_id;

        const auto his_msg = obs_his_msg.find(id)->second;
        double his_yaw_0 = his_msg[0].rotation.z;
        // SG_INFO("%s --- his_yaw_0=%lf,traj_yaw_end=%lf", id.c_str(),
        // his_yaw_0,
        //         traj_yaw_end);
        if (fabs(his_yaw_0 - traj_yaw_end) > yaw_limit_junctionturn_) {
            for (int a = 1;
                 a < obs_info.obstacles[i].obs_trajectories[0].points.size();
                 ++a) {
                double traj_yaw = obs_info.obstacles[i]
                                      .obs_trajectories[0]
                                      .points[a]
                                      .path_point.theta_rad;
                if (fabs(his_yaw_0 - traj_yaw) > yaw_limit_junctionturn_) {
                    TrajectoryPoint traj_point;
                    double x = obs_info.obstacles[i]
                                   .obs_trajectories[0]
                                   .points[a - 1]
                                   .path_point.position_m.x;
                    double y = obs_info.obstacles[i]
                                   .obs_trajectories[0]
                                   .points[a - 1]
                                   .path_point.position_m.y;
                    double vel = obs_info.obstacles[i]
                                     .obs_trajectories[0]
                                     .points[a - 1]
                                     .velocity_mps;
                    double theta = obs_info.obstacles[i]
                                       .obs_trajectories[0]
                                       .points[a - 1]
                                       .path_point.theta_rad;
                    traj_point.path_point.position_m.x =
                        x + vel * cos(theta) * prediction_period_;
                    traj_point.path_point.position_m.y =
                        y + vel * sin(theta) * prediction_period_;
                    // SG_INFO("%s -- x =%lf,y=%lf,vel=%lf,theta=%lf",
                    // id.c_str(),
                    //         x, y, vel, theta);
                    traj_point.path_point.position_m.z = 0.0;
                    traj_point.path_point.s_m = 0.0;
                    traj_point.path_point.theta_rad = theta;
                    traj_point.path_point.kappa = 0.0;
                    traj_point.path_point.dkappa = 0.0;
                    traj_point.velocity_mps = vel;
                    traj_point.relative_time_s = obs_info.obstacles[i]
                                                     .obs_trajectories[0]
                                                     .points[a - 1]
                                                     .relative_time_s +
                                                 prediction_period_;

                    obs_info.obstacles[i].obs_trajectories[0].points[a] =
                        traj_point;
                }
            }
        }
    }
}

void JunctionPredictor::DrawFreeMoveTrajectoryPoints(
    const ObsDeque &obstacles_deque, const double vel, const std::string id,
    const double ox, const double oy, const double r, const double total_time,
    const double period, ObsTrajectory *trajectory) {
    const auto obs_pos_list = obstacles_deque.find(id)->second;

    TrajectoryPoint trajectory_point;

    int list_size = obs_pos_list.size();
    int sub_mid = floor(list_size / 2) - 1;
    double y_end = std::get<1>(obs_pos_list.back());
    double x_end = std::get<0>(obs_pos_list.back());
    double y_mid = std::get<1>(obs_pos_list[sub_mid]);
    double x_mid = std::get<0>(obs_pos_list[sub_mid]);
    double y_begin = std::get<1>(obs_pos_list.front());
    double x_begin = std::get<0>(obs_pos_list.front());
    double theta_enu_end = std::get<2>(obs_pos_list.back());

    double dx_end = x_end - ox;
    double dy_end = y_end - oy;
    double dx_mid = x_mid - ox;
    double dy_mid = y_mid - oy;
    double dx_begin = x_begin - ox;
    double dy_begin = y_begin - oy;

    double theta_end = std::atan2(dy_end, dx_end);
    double theta_mid = std::atan2(dy_mid, dx_mid);
    double theta_begin = std::atan2(dy_begin, dx_begin);

    double deltatheta1 = theta_mid - theta_begin;
    double deltatheta2 = theta_end - theta_mid;
    if (deltatheta1 > 0 && deltatheta2 < 0) {
        if (fabs(deltatheta1) > fabs(deltatheta2)) {
            theta_begin += 2 * M_PI;
        }
        if (fabs(deltatheta1) < fabs(deltatheta2)) {
            theta_end += 2 * M_PI;
        }
    }
    if (deltatheta1 < 0 && deltatheta2 > 0) {
        if (fabs(deltatheta1) > fabs(deltatheta2)) {
            theta_begin -= 2 * M_PI;
        }
        if (fabs(deltatheta1) < fabs(deltatheta2)) {
            theta_end -= 2 * M_PI;
        }
    }

    double deltatheta = theta_end - theta_begin;

    // while (deltatheta > M_PI) {
    //     deltatheta -= 2 * M_PI;
    // }

    // while (deltatheta < -M_PI) {
    //     deltatheta += 2 * M_PI;
    // }

    double dtheta;
    if (theta_end - theta_begin == 0) {
        dtheta = vel * period / r;
    } else {
        dtheta = vel * period / r * (deltatheta) / fabs(deltatheta);
    }
    // SG_INFO("%s --- dtheta=%lf,r=%lf,theta_begin=%lf,theta_end=%lf",
    // id.c_str(),
    //         dtheta, r, theta_begin, theta_end);

    if (dtheta > 0.1) {
        dtheta = 0.1;
    }

    if (dtheta < -0.1) {
        dtheta = -0.1;
    }

    double theta = theta_end;
    int num = static_cast<int>(total_time / period) + 1;

    double dx = x_end - (ox + r * std::cos(theta));
    double dy = y_end - (oy + r * std::sin(theta));

    for (int i = 0; i < num; ++i) {
        theta += dtheta;
        trajectory_point.path_point.position_m.x =
            ox + r * std::cos(theta) + dx;
        trajectory_point.path_point.position_m.y =
            oy + r * std::sin(theta) + dy;
        trajectory_point.path_point.position_m.z = 0;
        trajectory_point.path_point.theta_rad = theta_enu_end + dtheta * i;
        trajectory_point.path_point.kappa = 0;
        trajectory_point.path_point.dkappa = 0;
        trajectory_point.velocity_mps = vel;
        trajectory_point.relative_time_s = i * period;
        trajectory->points.emplace_back(std::move(trajectory_point));
    }
}

void JunctionPredictor::DrawFreeMoveTrajectoryPoints(
    const Eigen::Vector2d &position, const Eigen::Vector2d &velocity,
    const Eigen::Vector2d &acceleration, const double theta,
    const double dtheta, const double start_time, const double total_time,
    const double period, ObsTrajectory *trajectory) {
    Eigen::Matrix<double, 6, 1> state;
    state.setZero();
    state(0, 0) = position(0);
    state(1, 0) = position(1);
    state(2, 0) = velocity(0);
    state(3, 0) = velocity(1);
    state(4, 0) = acceleration(0);
    state(5, 0) = acceleration(1);

    Eigen::Matrix<double, 6, 6> transition;
    transition.setIdentity();
    transition(0, 2) = period;
    transition(0, 4) = 0.5 * period * period;
    transition(1, 3) = period;
    transition(1, 5) = 0.5 * period * period;
    transition(2, 4) = period;
    transition(3, 5) = period;

    int num = static_cast<int>(total_time / period) + 1;
    GenerateFreeMoveTrajectoryPoints(&state, transition, theta, dtheta,
                                     start_time, num, period, trajectory);
    // SG_INFO("traj.size = %d", trajectory->points.size());
}

void JunctionPredictor::GenerateFreeMoveTrajectoryPoints(
    Eigen::Matrix<double, 6, 1> *state,
    const Eigen::Matrix<double, 6, 6> &transition, double theta,
    const double dtheta, const double start_time, const int num,
    const double period, ObsTrajectory *trajectory) {
    TrajectoryPoint trajectory_point;
    double x = (*state)(0, 0);
    double y = (*state)(1, 0);
    double v_x = (*state)(2, 0);
    double v_y = (*state)(3, 0);
    double acc_x = (*state)(4, 0);
    double acc_y = (*state)(5, 0);
    double param = 1;
    for (int i = 0; i < num; ++i) {
        double speed = std::hypot(v_x, v_y);
        double acc = 0.0;
        double theta_now = theta + std::pow(param, i) * i * dtheta;
        // double theta_now = theta + i * dtheta;
        if (speed <= 0.5) {
            speed = 0.0;
            v_x = 0.0;
            v_y = 0.0;
            acc_x = 0.0;
            acc_y = 0.0;
            acc = 0.0;
        }

        // // update theta and acc
        // else {
        //     if (trajectory->points.size() != 0) {
        //         theta = std::atan2(y -
        //         trajectory->points.back().position_m.y,
        //                            x -
        //                            trajectory->points.back().position_m.x);
        //         if (theta < 0) {
        //             theta += 2 * M_PI;
        //         }
        //     }
        // }

        // update state
        (*state)(0, 0) = x;
        (*state)(1, 0) = y;
        (*state)(2, 0) = v_x;
        (*state)(3, 0) = v_y;
        (*state)(4, 0) = acc_x;
        (*state)(5, 0) = acc_y;

        // Generate trajectory point
        trajectory_point.path_point.position_m.x = (*state)(0, 0);
        trajectory_point.path_point.position_m.y = (*state)(1, 0);
        trajectory_point.path_point.position_m.z = 0;
        trajectory_point.path_point.theta_rad = theta_now;
        // SG_INFO("theta_now=%lf", theta_now);
        trajectory_point.path_point.kappa = 0;
        trajectory_point.path_point.dkappa = 0;
        trajectory_point.velocity_mps = speed;
        trajectory_point.relative_time_s = start_time + i * period;
        trajectory->points.emplace_back(std::move(trajectory_point));

        // Update position, velocity and acceleration
        (*state) = transition * (*state);
        x = (*state)(0, 0) * std::cos(std::pow(param, i) * i * dtheta) -
            (*state)(1, 0) * std::sin(std::pow(param, i) * i * dtheta);
        y = (*state)(0, 0) * std::sin(std::pow(param, i) * i * dtheta) +
            (*state)(1, 0) * std::cos(std::pow(param, i) * i * dtheta);
        // x = (*state)(0, 0) * std::cos(i * dtheta) -
        //     (*state)(1, 0) * std::sin(i * dtheta);
        // y = (*state)(0, 0) * std::sin(i * dtheta) +
        //     (*state)(1, 0) * std::cos(i * dtheta);
        v_x = (*state)(2, 0);
        v_y = (*state)(3, 0);
        acc_x = (*state)(4, 0);
        acc_y = (*state)(5, 0);
    }
    // SG_INFO("traj_matrix.size = %d", trajectory->points.size());
}

void JunctionPredictor::FilterBySolidLane(ObstaclesInfo &obs_info,
                                          const ObsTrafficType &traffic_type,
                                          const MonospacedLaneMsg &lane_msg) {
    if (lane_msg.monospaced_section.empty()) {
        return;
    }

    Point2d solid_begin;
    Point2d solid_end;
    bool has_left_solid_line = false;

    for (int i = lane_msg.sub_ego_in_section; i >= 0; --i) {
        if (!lane_msg.monospaced_section[i].can_cutted_left) {
            has_left_solid_line = true;
            solid_begin = lane_msg.monospaced_section[i]
                              .left_lane_line.lane_line.point_begin;
            solid_end = lane_msg.monospaced_section[i]
                            .left_lane_line.lane_line.point_end;
            break;
        }
    }
    if (!has_left_solid_line) {
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

    common::VehCoord2EnuCoord(ego_x, ego_y, ego_yaw, solid_begin.x,
                              solid_begin.y);
    common::VehCoord2EnuCoord(ego_x, ego_y, ego_yaw, solid_end.x, solid_end.y);

    for (int j = 0; j < traffic_type.off_lane_obs.size(); ++j) {
        int i = traffic_type.off_lane_obs[j];
        Point2d traj_end;
        traj_end.x = obs_info.obstacles[i]
                         .obs_trajectories[0]
                         .points.back()
                         .path_point.position_m.x;
        traj_end.y = obs_info.obstacles[i]
                         .obs_trajectories[0]
                         .points.back()
                         .path_point.position_m.y;
        Point2d traj_begin;
        traj_begin.x = obs_info.obstacles[i]
                           .obs_trajectories[0]
                           .points[0]
                           .path_point.position_m.x;
        traj_begin.y = obs_info.obstacles[i]
                           .obs_trajectories[0]
                           .points[0]
                           .path_point.position_m.y;
        if (common::IsIntersectedByLineSection(traj_begin, traj_end,
                                               solid_begin, solid_end)) {
            int size = obs_info.obstacles[i].obs_trajectories[0].points.size();
            int sub = size - 2;
            for (; sub >= 0; --sub) {
                Point2d point_traj;
                point_traj.x = obs_info.obstacles[i]
                                   .obs_trajectories[0]
                                   .points[sub]
                                   .path_point.position_m.x;
                point_traj.y = obs_info.obstacles[i]
                                   .obs_trajectories[0]
                                   .points[sub]
                                   .path_point.position_m.y;

                if (!common::IsIntersectedByLineSection(
                        traj_begin, point_traj, solid_begin, solid_end)) {
                    break;
                }
            }
            for (; sub < size - 1; ++sub) {
                obs_info.obstacles[i]
                    .obs_trajectories[0]
                    .points[sub + 1]
                    .path_point.position_m.x = obs_info.obstacles[i]
                                                   .obs_trajectories[0]
                                                   .points[sub]
                                                   .path_point.position_m.x;
                obs_info.obstacles[i]
                    .obs_trajectories[0]
                    .points[sub + 1]
                    .path_point.position_m.y = obs_info.obstacles[i]
                                                   .obs_trajectories[0]
                                                   .points[sub]
                                                   .path_point.position_m.y;
                obs_info.obstacles[i]
                    .obs_trajectories[0]
                    .points[sub + 1]
                    .path_point.theta_rad = obs_info.obstacles[i]
                                                .obs_trajectories[0]
                                                .points[sub]
                                                .path_point.theta_rad;
            }
        }
    }
}

}  // namespace prediction_lib
}  // namespace jarvis
