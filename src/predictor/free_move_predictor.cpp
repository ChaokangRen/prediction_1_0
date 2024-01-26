#include "free_move_predictor.h"

namespace jarvis {
namespace prediction_lib {

void FreeMovePredictor::Init(const PredictionConf &prediction_conf) {
    prediction_total_time_ = prediction_conf.prediction_total_time;
    prediction_period_ = prediction_conf.prediction_period;
}

void FreeMovePredictor::Predict(ObstaclesInfo &obs_info,
                                const ObstaclesInfo &obs_info_enu,
                                ObsTrafficType &traffic_type,
                                const HistoryMap &obs_his_msg,
                                const ObsDeque &obstacles_deque,
                                const MonospacedLaneMsg &lane_msg,
                                std::string &debug_info) {
    // ObsMap opt_map;

    // for (int i = 0; i < obs_info.obstacles.size(); ++i) {
    //     SG_INFO("%s --- %lf", obs_info.obstacles[i].track_id.c_str(),
    //             obs_info.obstacles[i].state.angular_deg.z);
    // }

    // OptimizeHistoryMsg(obs_his_msg, opt_map);
    for (int j = 0; j < traffic_type.off_lane_obs.size(); ++j) {
        int i = traffic_type.off_lane_obs[j];
        ObsTrajectory obs_trajectory;
        double pos_x = obs_info_enu.obstacles[i].state.position.x;
        double pos_y = obs_info_enu.obstacles[i].state.position.y;
        double vel_x = obs_info_enu.obstacles[i].state.linear_velocity.x;
        double vel_y = obs_info_enu.obstacles[i].state.linear_velocity.y;
        double vel = std::hypot(vel_x, vel_y);
        double acc_x = 0;
        double acc_y = 0;
        // double acc_x =
        // obs_info_enu.obstacles[i].state.linear_acceleration.x; double
        // acc_y = obs_info_enu.obstacles[i].state.linear_acceleration.y;

        // SG_INFO("pos_x = %lf,pos_y = %lf,vel_x = %lf,vel_y = %lf", pos_x,
        // pos_y,
        //         vel_x, vel_y);
        std::string id = obs_info_enu.obstacles[i].track_id;
        // SG_INFO("%s --- pos_x=%lf,pos_y=%lf", id.c_str(), pos_x, pos_y);

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

        // SG_INFO("%s !!! vel_x=%lf,vel_y=%lf,acc_x=%lf,acc_y=%lf",
        // id.c_str(),
        //         vel_x, vel_y, acc_x, acc_y);

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

void FreeMovePredictor::DrawFreeMoveTrajectoryPoints(
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

void FreeMovePredictor::GenerateFreeMoveTrajectoryPoints(
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

void FreeMovePredictor::OptimizeHistoryMsg(const HistoryMap &obs_his_msg,
                                           ObsMap &opt_map) {
    for (auto ite : obs_his_msg) {
        std::vector<std::tuple<double, double, double>> input;
        std::vector<std::tuple<double, double, double>> output;
        std::string obs_id = ite.first;

        for (auto ite2 : ite.second) {
            input.emplace_back(
                std::make_tuple(ite2.pos.x, ite2.pos.y, ite2.rotation.z));
            // SG_INFO("%s->->->->->->->->->%lf,%lf,%lf", obs_id.c_str(),
            //         ite2.pos.x, ite2.pos.y, ite2.rotation.z);
        }
        optObj_traj_.optTraj(input, output, 0);
        // SG_INFO("input_size=%d,output_size=%d", input.size(), output.size());
        std::list<std::tuple<double, double, double>> output_list;
        for (int i = 0; i < output.size(); ++i) {
            output_list.push_back(output[i]);
            // SG_INFO("%s-<-<-<-<-<-<-<-<-<%lf,%lf,%lf", obs_id.c_str(),
            //         std::get<0>(output[i]), std::get<1>(output[i]),
            //         std::get<2>(output[i]));
        }
        opt_map[obs_id] = output_list;
        // SG_INFO("%s-------output_list_size=%d", obs_id.c_str(),
        //         opt_map[obs_id].size());
    }
}

void FreeMovePredictor::DrawFreeMoveTrajectoryPoints(
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

void FreeMovePredictor::UpdateTrafficTypeMap(const MonospacedLaneMsg &lane_msg,
                                             ObsTrafficType &traffic_type,
                                             const std::string id,
                                             const double obs_x,
                                             const double obs_y) {
    int sub_section = 0;
    for (int i = 0; i < lane_msg.monospaced_section.size(); ++i) {
        sub_section = lane_msg.monospaced_section[i].center_line.id;
        if (obs_x < lane_msg.monospaced_section[i]
                        .right_lane_line.lane_line.point_begin.x) {
            break;
        }
    }
    Point2d point_center_line_begin =
        lane_msg.monospaced_section[sub_section].center_line.point_begin;
    Point2d point_center_line_end =
        lane_msg.monospaced_section[sub_section].center_line.point_end;
    Point2d point_obs;
    point_obs.x = obs_x;
    point_obs.y = obs_y;
    double distance = common::DistancePoint2Line(
        point_center_line_begin, point_center_line_end, point_obs);
    auto pair = std::make_pair(sub_section, distance);
    traffic_type.map_obs_in_section[id] = pair;
}

}  // namespace prediction_lib
}  // namespace jarvis
