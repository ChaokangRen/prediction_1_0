#include "lane_container.h"

namespace jarvis {
namespace prediction_lib {
void LaneContainer::Process(
    LaneInfo &lane_info,
    const TrafficLightFrameResult &traffic_lights_results) {
    PreProcess(lane_info, traffic_lights_results);
    ProcessLane();
    ProcessSection();
    SetID(lane_msg_);

    UpdateMsg(lane_msg_);
    UpdateCenterLine(lane_info, lane_msg_);
    lane_lines_.clear();
    stop_lines_.clear();
}

MonospacedLaneMsg LaneContainer::GetLaneMsg() {
    return lane_msg_;
}

void LaneContainer::Clear() {
    lane_msg_.has_lane = false;
    lane_msg_.monospaced_lane.clear();
    lane_msg_.monospaced_section.clear();
    lane_msg_.yaw_veh_rad = 0.0;
}

void LaneContainer::UpdateLaneByHistory(Lanes &lane_lines, Lanes &stop_lines) {
    if (has_lane_line_) {
        lane_lines_pre_ = lane_lines_;
        if (has_lane_line_pre_) {
            ++count_lane_;
        } else {
            count_no_lane_ = 0;
            ++count_lane_;
        }
        has_lane_line_pre_ = true;
    } else {
        if (has_lane_line_pre_) {
            count_lane_ = 0;
            ++count_no_lane_;
        } else {
            ++count_no_lane_;
        }
        has_lane_line_pre_ = false;
    }

    if (has_stop_line_) {
        stop_lines_pre_ = stop_lines_;
        if (has_stop_line_pre_) {
            ++count_stop_line_;
        } else {
            count_no_stop_line_ = 0;
            ++count_stop_line_;
        }
        has_stop_line_pre_ = true;
    } else {
        if (has_stop_line_pre_) {
            count_stop_line_ = 0;
            ++count_no_stop_line_;
        } else {
            ++count_no_stop_line_;
        }
        has_stop_line_pre_ = false;
    }

    if (count_no_stop_line_ < count_stop_line_limit_) {
        stop_lines_ = stop_lines_pre_;
    }

    if (count_no_lane_ < count_no_lane_limit_) {
        lane_lines_ = lane_lines_pre_;
    }

    // SG_INFO(
    //     "count_no_lane_=%d,count_lane_=%d,count_no_stop_line_=%d,count_stop_"
    //     "line_=%d",
    //     count_no_lane_, count_lane_, count_no_stop_line_, count_stop_line_);
}

void LaneContainer::FilterLane(
    const LaneInfo &lane_info,
    const TrafficLightFrameResult &traffic_lights_results) {
    if (traffic_lights_results.semantic_left.color ==
            TRAFFICLIGHT_COLOR::TRAFFICLIGHT_COLOR_UNKNOWN &&
        traffic_lights_results.semantic_right.color ==
            TRAFFICLIGHT_COLOR::TRAFFICLIGHT_COLOR_UNKNOWN &&
        traffic_lights_results.semantic_straight.color ==
            TRAFFICLIGHT_COLOR::TRAFFICLIGHT_COLOR_UNKNOWN &&
        traffic_lights_results.semantic_uturn.color ==
            TRAFFICLIGHT_COLOR::TRAFFICLIGHT_COLOR_UNKNOWN) {
        has_traffic_light_ = false;
    } else {
        has_traffic_light_ = true;
    }

    for (int i = 0; i < lane_info.lane_lines_msg.size(); ++i) {
        if (lane_info.lane_lines_msg[i].type == LaneLineMsg::Type::LANE_LANE) {
            double lane_x_begin =
                lane_info.lane_lines_msg[i].lane_line.back().point.position_m.x;
            double lane_y_begin =
                lane_info.lane_lines_msg[i].lane_line.back().point.position_m.y;
            double lane_x_end =
                lane_info.lane_lines_msg[i].lane_line[0].point.position_m.x;
            double lane_y_end =
                lane_info.lane_lines_msg[i].lane_line[0].point.position_m.y;
            double lane_dx = lane_x_end - lane_x_begin;
            double lane_dy = lane_y_end - lane_y_begin;
            double lane_length = std::hypot(lane_dx, lane_dy);
            if (lane_length > lane_length_limit_) {
                lane_lines_.emplace_back(lane_info.lane_lines_msg[i]);
            }
        }
        if (has_traffic_light_ &&
            lane_info.lane_lines_msg[i].type == LaneLineMsg::Type::STOP_LINE) {
            double stop_x1 =
                lane_info.lane_lines_msg[i].lane_line[0].point.position_m.x;
            double stop_y1 =
                lane_info.lane_lines_msg[i].lane_line[0].point.position_m.y;
            double stop_x2 =
                lane_info.lane_lines_msg[i].lane_line.back().point.position_m.x;
            double stop_y2 =
                lane_info.lane_lines_msg[i].lane_line.back().point.position_m.y;
            if (stop_x1 < 0 && stop_x2 > 0 && stop_y1 < point_end_y_ &&
                stop_y2 < point_end_y_) {
                stop_lines_.emplace_back(lane_info.lane_lines_msg[i]);
            }
        }
    }

    // filter lane
    if (lane_lines_.size() < 1) {
        lane_lines_.clear();
    } else {
        std::sort(lane_lines_.begin(), lane_lines_.end(),
                  Left2RightSortLaneLineMsg);

        // todo:filter too close lane
    }

    // filter stop line
    if (stop_lines_.size() > 1) {
        std::sort(stop_lines_.begin(), stop_lines_.end(),
                  Close2FarSortStopLineMsg);
        for (; stop_lines_.size() > 1;) {
            stop_lines_.pop_back();
        }
    }

    if (lane_lines_.size()) {
        has_lane_line_ = true;
    } else {
        has_lane_line_ = false;
    }

    if (stop_lines_.size()) {
        has_stop_line_ = true;
    } else {
        has_stop_line_ = false;
    }
}

bool LaneContainer::Left2RightSortLaneLineMsg(
    const LaneLineMsg &lane_line_msg_1, const LaneLineMsg &lane_line_msg_2) {
    return lane_line_msg_1.lane_line.back().point.position_m.x <
           lane_line_msg_2.lane_line.back().point.position_m.x;
}

bool LaneContainer::Left2RightSortMonospacedLane(
    const MonospacedLane &monospaced_lane_1,
    const MonospacedLane &monospaced_lane_2) {
    return monospaced_lane_1.lane_line.point_begin.x <
           monospaced_lane_2.lane_line.point_begin.x;
}

bool LaneContainer::Close2FarSortStopLineMsg(
    const LaneLineMsg &lane_line_msg_1, const LaneLineMsg &lane_line_msg_2) {
    return lane_line_msg_1.lane_line[0].point.position_m.y <
           lane_line_msg_2.lane_line[0].point.position_m.y;
}

void LaneContainer::InitializeLane(const Lanes &lane_lines) {
    double lane_yaw = CalculateLaneYaw(lane_lines);
    lane_msg_.yaw_veh_rad = lane_yaw;
    lane_msg_.monospaced_lane.clear();
    for (int i = 0; i < lane_lines.size(); ++i) {
        MonospacedLane lane;
        lane.lane_line_color = lane_lines[i].lane_line.back().color_type;
        lane.lane_line_type = lane_lines[i].lane_line.back().type;
        double lane_x_origin =
            lane_lines[i].lane_line.back().point.position_m.x;
        double lane_y_origin =
            lane_lines[i].lane_line.back().point.position_m.y;
        lane.lane_line.point_begin.y = point_begin_y_;
        lane.lane_line.point_end.y = point_end_y_;
        lane.lane_line.yaw_veh_rad = lane_yaw;
        if (lane_yaw == M_PI / 2 || lane_yaw == -M_PI / 2) {
            lane.lane_line.point_begin.x = lane_x_origin;
            lane.lane_line.point_end.x = lane_x_origin;
        } else {
            double slope = std::tan(lane_yaw);
            lane.lane_line.point_begin.x =
                (point_begin_y_ - lane_y_origin) / slope + lane_x_origin;
            lane.lane_line.point_end.x =
                (point_end_y_ - lane_y_origin) / slope + lane_x_origin;
        }
        lane_msg_.monospaced_lane.emplace_back(std::move(lane));
    }
}

double LaneContainer::CalculateLaneYaw(const Lanes &lane_lines) {
    int sub_nearest_1 = 0;
    int sub_nearest_2 = 0;
    double dx_nearest_1 = std::numeric_limits<double>::max();
    double dx_nearest_2 = std::numeric_limits<double>::max();
    for (int i = 0; i < lane_lines.size(); ++i) {
        double lane_x = lane_lines[i].lane_line.back().point.position_m.x;
        if (std::fabs(lane_x) < dx_nearest_1) {
            dx_nearest_2 = dx_nearest_1;
            dx_nearest_1 = std::fabs(lane_x);
            sub_nearest_2 = sub_nearest_1;
            sub_nearest_1 = i;
        } else if (std::fabs(lane_x) < dx_nearest_2) {
            dx_nearest_2 = std::fabs(lane_x);
            sub_nearest_2 = i;
        }
    }
    std::vector<int> sub;
    sub.emplace_back(sub_nearest_1);
    sub.emplace_back(sub_nearest_2);

    double sum_yaw = 0.0;
    int sub_size = sub.size();
    for (int i = 0; i < sub_size; ++i) {
        int count_lane_point = lane_lines[sub[i]].lane_line.size();
        double lane_x_begin =
            lane_lines[sub[i]].lane_line.back().point.position_m.x;
        double lane_y_begin =
            lane_lines[sub[i]].lane_line.back().point.position_m.y;
        double lane_x_end = lane_lines[sub[i]].lane_line[0].point.position_m.x;
        double lane_y_end = lane_lines[sub[i]].lane_line[0].point.position_m.y;
        if (lane_y_end > lane_y_begin) {
            sum_yaw += std::atan2(lane_y_end - lane_y_begin,
                                  lane_x_end - lane_x_begin);
        } else {
            sum_yaw += std::atan2(lane_y_begin - lane_y_end,
                                  lane_x_begin - lane_x_end);
        }
    }

    double lane_yaw = sum_yaw / sub_size;
    return lane_yaw;
}

void LaneContainer::AddLostLane(MonospacedLaneMsg &lane_msg) {
    if (!lane_msg.monospaced_lane.size()) {
        return;
    }
    int lane_size_origin = lane_msg.monospaced_lane.size();
    double lane_yaw = lane_msg.monospaced_lane[0].lane_line.yaw_veh_rad;
    std::vector<MonospacedLane> monospaced_lane_left;
    std::vector<MonospacedLane> monospaced_lane_right;
    for (int i = 0; i < lane_msg.monospaced_lane.size(); ++i) {
        if (lane_msg.monospaced_lane[i].lane_line.point_begin.x < 0) {
            monospaced_lane_left.emplace_back(lane_msg.monospaced_lane[i]);
        } else {
            monospaced_lane_right.emplace_back(lane_msg.monospaced_lane[i]);
        }
    }

    // SG_INFO("lane_left=%d,lane_right=%d", monospaced_lane_left.size(),
    //         monospaced_lane_right.size());

    lane_msg.monospaced_lane.clear();
    std::vector<MonospacedLane> monospaced_lane_new;
    if (monospaced_lane_left.size() == 0) {
        if (monospaced_lane_right.size() == 0) {
            return;
        } else if (monospaced_lane_right.size() == 1) {
            monospaced_lane_new.emplace_back(monospaced_lane_right[0]);
            for (; monospaced_lane_new[0].lane_line.point_begin.x > 0;) {
                AddLaneOnLeft(monospaced_lane_new, 0, lane_width_);
            }
            int new_lane_size = monospaced_lane_new.size();
            int count = new_lane_size <= 3 ? new_lane_size : 3;
            for (int i = 0; i < count; ++i) {
                lane_msg.monospaced_lane.emplace_back(monospaced_lane_new[i]);
            }
        } else {
            Point2d point_p = monospaced_lane_right[0].lane_line.point_begin;
            Point2d point_a = monospaced_lane_right[1].lane_line.point_begin;
            Point2d point_b = monospaced_lane_right[1].lane_line.point_end;
            double distance = std::fabs(
                common::DistancePoint2Line(point_a, point_b, point_p));
            if (distance > 4) {
                distance = lane_width_;
                monospaced_lane_new.emplace_back(monospaced_lane_right[0]);
                AddLaneOnBack(monospaced_lane_new, distance);
            } else {
                monospaced_lane_new.emplace_back(monospaced_lane_right[0]);
                monospaced_lane_new.emplace_back(monospaced_lane_right[1]);
            }
            for (; monospaced_lane_new[0].lane_line.point_begin.x > 0;) {
                AddLaneOnLeft(monospaced_lane_new, 0, distance);
            }
            int new_lane_size = monospaced_lane_new.size();
            int count = new_lane_size <= 3 ? new_lane_size : 3;
            for (int i = 0; i < count; ++i) {
                lane_msg.monospaced_lane.emplace_back(monospaced_lane_new[i]);
            }
        }
    } else if (monospaced_lane_left.size() == 1) {
        if (monospaced_lane_right.size() == 0) {
            monospaced_lane_new.emplace_back(monospaced_lane_left.back());
            for (; monospaced_lane_new.back().lane_line.point_begin.x < 0;) {
                AddLaneOnBack(monospaced_lane_new, lane_width_);
            }
            int new_lane_size = monospaced_lane_new.size();
            int count = new_lane_size <= 3 ? new_lane_size : 3;
            for (int i = 0; i < count; ++i) {
                lane_msg.monospaced_lane.emplace_back(
                    monospaced_lane_new[new_lane_size - 1 - i]);
            }
        } else {
            Point2d point_p = monospaced_lane_left.back().lane_line.point_begin;
            Point2d point_a = monospaced_lane_right[0].lane_line.point_begin;
            Point2d point_b = monospaced_lane_right[0].lane_line.point_end;
            double distance = std::fabs(
                common::DistancePoint2Line(point_a, point_b, point_p));
            if (distance > 4) {
                distance = lane_width_;
                double dis_left = std::fabs(
                    monospaced_lane_left.back().lane_line.point_begin.x);
                double dis_right =
                    std::fabs(monospaced_lane_right[0].lane_line.point_begin.x);
                if (dis_left > dis_right) {
                    monospaced_lane_new.emplace_back(monospaced_lane_right[0]);
                    if (monospaced_lane_right.size() > 1) {
                        AddLaneOnBack(monospaced_lane_new, distance);
                    }
                    for (;
                         monospaced_lane_new[0].lane_line.point_begin.x > 0;) {
                        AddLaneOnLeft(monospaced_lane_new, 0, distance);
                    }
                    AddLaneOnLeft(monospaced_lane_new, 0, distance);
                    int new_lane_size = monospaced_lane_new.size();
                    int count = new_lane_size <= 4 ? new_lane_size : 4;
                    for (int i = 0; i < count; ++i) {
                        lane_msg.monospaced_lane.emplace_back(
                            monospaced_lane_new[i]);
                    }
                } else {
                    monospaced_lane_new.emplace_back(
                        monospaced_lane_left.back());
                    for (; monospaced_lane_new.back().lane_line.point_begin.x <
                           0;) {
                        AddLaneOnBack(monospaced_lane_new, distance);
                    }
                    AddLaneOnBack(monospaced_lane_new, distance);
                    int new_lane_size = monospaced_lane_new.size();
                    int count = new_lane_size <= 4 ? new_lane_size : 4;
                    for (int i = 0; i < count; ++i) {
                        lane_msg.monospaced_lane.emplace_back(
                            monospaced_lane_new[new_lane_size - 1 - i]);
                    }
                }
            } else {
                monospaced_lane_new.emplace_back(monospaced_lane_left.back());
                monospaced_lane_new.emplace_back(monospaced_lane_right[0]);
                if (monospaced_lane_right.size() > 1) {
                    AddLaneOnBack(monospaced_lane_new, lane_width_);
                }
                lane_msg.monospaced_lane = monospaced_lane_new;
            }
        }
    } else {
        if (monospaced_lane_right.size() == 0) {
            int size = monospaced_lane_left.size();
            Point2d point_p =
                monospaced_lane_left[size - 1].lane_line.point_begin;
            Point2d point_a =
                monospaced_lane_left[size - 2].lane_line.point_begin;
            Point2d point_b =
                monospaced_lane_left[size - 2].lane_line.point_end;
            double distance = std::fabs(
                common::DistancePoint2Line(point_a, point_b, point_p));
            if (distance > 4) {
                distance = lane_width_;
                monospaced_lane_new.emplace_back(
                    monospaced_lane_left[size - 1]);
                AddLaneOnLeft(monospaced_lane_new, 0, distance);
                for (;
                     monospaced_lane_new.back().lane_line.point_begin.x < 0;) {
                    AddLaneOnBack(monospaced_lane_new, distance);
                }
                int new_lane_size = monospaced_lane_new.size();
                int count = new_lane_size <= 3 ? new_lane_size : 3;
                for (int i = 0; i < count; ++i) {
                    lane_msg.monospaced_lane.emplace_back(
                        monospaced_lane_new[new_lane_size - 1 - i]);
                }
            } else {
                monospaced_lane_new.emplace_back(
                    monospaced_lane_left[size - 2]);
                monospaced_lane_new.emplace_back(
                    monospaced_lane_left[size - 1]);
                for (;
                     monospaced_lane_new.back().lane_line.point_begin.x < 0;) {
                    AddLaneOnBack(monospaced_lane_new, distance);
                }
                int new_lane_size = monospaced_lane_new.size();
                int count = new_lane_size <= 3 ? new_lane_size : 3;
                for (int i = 0; i < count; ++i) {
                    lane_msg.monospaced_lane.emplace_back(
                        monospaced_lane_new[new_lane_size - 1 - i]);
                }
            }
        } else {
            Point2d point_p = monospaced_lane_left.back().lane_line.point_begin;
            Point2d point_a = monospaced_lane_right[0].lane_line.point_begin;
            Point2d point_b = monospaced_lane_right[0].lane_line.point_end;
            double distance = std::fabs(
                common::DistancePoint2Line(point_a, point_b, point_p));
            if (distance > 4) {
                distance = lane_width_;
                double dis_left = std::fabs(
                    monospaced_lane_left.back().lane_line.point_begin.x);
                double dis_right =
                    std::fabs(monospaced_lane_right[0].lane_line.point_begin.x);
                if (dis_left > dis_right) {
                    monospaced_lane_new.emplace_back(monospaced_lane_right[0]);
                    if (monospaced_lane_right.size() > 1) {
                        AddLaneOnBack(monospaced_lane_new, distance);
                    }
                    for (;
                         monospaced_lane_new[0].lane_line.point_begin.x > 0;) {
                        AddLaneOnLeft(monospaced_lane_new, 0, distance);
                    }
                    AddLaneOnLeft(monospaced_lane_new, 0, distance);
                    int new_lane_size = monospaced_lane_new.size();
                    int count = new_lane_size <= 4 ? new_lane_size : 4;
                    for (int i = 0; i < count; ++i) {
                        lane_msg.monospaced_lane.emplace_back(
                            monospaced_lane_new[i]);
                    }
                } else {
                    monospaced_lane_new.emplace_back(
                        monospaced_lane_left.back());
                    AddLaneOnLeft(monospaced_lane_new, 0, distance);
                    for (; monospaced_lane_new.back().lane_line.point_begin.x <
                           0;) {
                        AddLaneOnBack(monospaced_lane_new, distance);
                    }
                    AddLaneOnBack(monospaced_lane_new, distance);
                    int new_lane_size = monospaced_lane_new.size();
                    int count = new_lane_size <= 4 ? new_lane_size : 4;
                    for (int i = 0; i < count; ++i) {
                        lane_msg.monospaced_lane.emplace_back(
                            monospaced_lane_new[new_lane_size - 1 - i]);
                    }
                }
            } else {
                monospaced_lane_new.emplace_back(monospaced_lane_left.back());
                monospaced_lane_new.emplace_back(monospaced_lane_right[0]);
                AddLaneOnLeft(monospaced_lane_new, 0, lane_width_);
                if (monospaced_lane_right.size() > 1) {
                    AddLaneOnBack(monospaced_lane_new, lane_width_);
                }
                lane_msg.monospaced_lane = monospaced_lane_new;
            }
        }
    }
    std::sort(lane_msg.monospaced_lane.begin(), lane_msg.monospaced_lane.end(),
              Left2RightSortMonospacedLane);
}

void LaneContainer::AddLaneOnLeft(std::vector<MonospacedLane> &monospaced_lane,
                                  const int sub, const double lane_width) {
    MonospacedLane lane;
    lane.lane_line_color = LaneLineColor::COLOR_WHITE;
    lane.lane_line_type = LaneLineType::BROKEN;
    double yaw = monospaced_lane[sub].lane_line.yaw_veh_rad;
    double dx = std::fabs(lane_width / std::sin(yaw));
    lane.lane_line.point_begin.x =
        monospaced_lane[sub].lane_line.point_begin.x - dx;
    lane.lane_line.point_begin.y = point_begin_y_;
    lane.lane_line.point_end.x =
        monospaced_lane[sub].lane_line.point_end.x - dx;
    lane.lane_line.point_end.y = point_end_y_;
    lane.lane_line.yaw_veh_rad = monospaced_lane[sub].lane_line.yaw_veh_rad;
    auto iter = monospaced_lane.begin();
    for (int i = 0; i < sub; ++i) {
        ++iter;
    }
    monospaced_lane.emplace(iter, std::move(lane));
}

void LaneContainer::AddLaneOnBack(std::vector<MonospacedLane> &monospaced_lane,
                                  const double lane_width) {
    MonospacedLane lane;
    lane.lane_line_color = LaneLineColor::COLOR_WHITE;
    lane.lane_line_type = LaneLineType::BROKEN;
    double yaw = monospaced_lane.back().lane_line.yaw_veh_rad;
    double dx = std::fabs(lane_width / std::sin(yaw));
    lane.lane_line.point_begin.x =
        monospaced_lane.back().lane_line.point_begin.x + dx;
    lane.lane_line.point_begin.y = point_begin_y_;
    lane.lane_line.point_end.x =
        monospaced_lane.back().lane_line.point_end.x + dx;
    lane.lane_line.point_end.y = point_end_y_;
    lane.lane_line.yaw_veh_rad = monospaced_lane.back().lane_line.yaw_veh_rad;
    monospaced_lane.emplace_back(std::move(lane));
}

void LaneContainer::UpdateDubugInfo(std::string &debug_info) {
    debug_info += "{section:";
    for (int i = 0; i < lane_msg_.monospaced_section.size(); ++i) {
        debug_info += "[";
        double x_begin = lane_msg_.monospaced_section[i]
                             .left_lane_line.lane_line.point_begin.x;
        double y_begin = lane_msg_.monospaced_section[i]
                             .left_lane_line.lane_line.point_begin.y;
        double x_end = lane_msg_.monospaced_section[i]
                           .left_lane_line.lane_line.point_end.x;
        double y_end = lane_msg_.monospaced_section[i]
                           .left_lane_line.lane_line.point_end.y;
        LaneLineType type =
            lane_msg_.monospaced_section[i].left_lane_line.lane_line_type;
        LaneLineColor color =
            lane_msg_.monospaced_section[i].left_lane_line.lane_line_color;
        debug_info += "(" + std::to_string(x_begin) + "," +
                      std::to_string(y_begin) + "),(" + std::to_string(x_end) +
                      "," + std::to_string(y_end) + ")" + "," +
                      TypeToString(type) + "," + ColorToString(color);
        if (lane_msg_.monospaced_section[i].has_stop_line) {
            double stop_x_begin =
                lane_msg_.monospaced_section[i].stop_lane_line.point_begin.x;
            double stop_y_begin =
                lane_msg_.monospaced_section[i].stop_lane_line.point_begin.y;
            double stop_x_end =
                lane_msg_.monospaced_section[i].stop_lane_line.point_end.x;
            double stop_y_end =
                lane_msg_.monospaced_section[i].stop_lane_line.point_end.y;
            debug_info += "(" + std::to_string(stop_x_begin) + "," +
                          std::to_string(stop_y_begin) + "),(" +
                          std::to_string(stop_x_end) + "," +
                          std::to_string(stop_y_end) + ")";
        }
        debug_info += "]";
    }
    if (lane_msg_.monospaced_section.size()) {
        debug_info += "[";
        double x_end_begin = lane_msg_.monospaced_section.back()
                                 .right_lane_line.lane_line.point_begin.x;
        double y_end_begin = lane_msg_.monospaced_section.back()
                                 .right_lane_line.lane_line.point_begin.y;
        double x_end_end = lane_msg_.monospaced_section.back()
                               .right_lane_line.lane_line.point_end.x;
        double y_end_end = lane_msg_.monospaced_section.back()
                               .right_lane_line.lane_line.point_end.y;
        LaneLineType end_type =
            lane_msg_.monospaced_section.back().right_lane_line.lane_line_type;
        LaneLineColor end_color =
            lane_msg_.monospaced_section.back().right_lane_line.lane_line_color;
        debug_info += "(" + std::to_string(x_end_begin) + "," +
                      std::to_string(y_end_begin) + "),(" +
                      std::to_string(x_end_end) + "," +
                      std::to_string(y_end_end) + ")" + "," +
                      TypeToString(end_type) + "," + ColorToString(end_color);
        debug_info += "]";
    }

    debug_info += "section_end}";
}

std::string LaneContainer::TypeToString(const LaneLineType type) {
    static std::map<LaneLineType, std::string> type_map = {
        {LaneLineType::SOLID, "solid"},
        {LaneLineType::BROKEN, "broken"},
        {LaneLineType::SOLID_SOLID, "solid_solid"},
        {LaneLineType::SOLID_BROKEN, "solid_broken"},
        {LaneLineType::BROKEN_SOLID, "broken_solid"},
        {LaneLineType::BROKEN_BROKEN, "broken_broken"},
        {LaneLineType::SPECIAL, "special"},
    };
    return type_map[type];
}

std::string LaneContainer::ColorToString(const LaneLineColor color) {
    static std::map<LaneLineColor, std::string> color_map = {
        {LaneLineColor::COLOR_WHITE, "white"},
        {LaneLineColor::COLOR_YELLOW, "yellow"},
        {LaneLineColor::COLOR_RED, "red"},
        {LaneLineColor::COLOR_GREEN, "green"},
        {LaneLineColor::COLOR_BLUE, "blue"},
        {LaneLineColor::COLOR_ORANGE, "orange"},
        {LaneLineColor::COLOR_VIOLET, "violet"},
    };
    return color_map[color];
}

void LaneContainer::UpdateMsg(MonospacedLaneMsg &lane_msg) {
    if (lane_msg.monospaced_lane.size()) {
        lane_msg.has_lane = false;
    } else {
        lane_msg.has_lane = true;
    }
}

void LaneContainer::SetID(MonospacedLaneMsg &lane_msg) {
    int sub = 0;
    for (; sub < lane_msg.monospaced_lane.size(); ++sub) {
        if (lane_msg.monospaced_lane[sub].lane_line_color ==
            LaneLineColor::COLOR_YELLOW) {
            SetIDByYellowLane(lane_msg, sub);
            break;
        } else {
            lane_msg.monospaced_lane[sub].lane_line.id = sub;
            if (sub < lane_msg.monospaced_lane.size() - 1) {
                lane_msg.monospaced_section[sub].center_line.id = sub;
                if (lane_msg.monospaced_section[sub].has_stop_line) {
                    lane_msg.monospaced_section[sub].stop_lane_line.id = sub;
                }
            }
        }
    }
}

void LaneContainer::SetIDByYellowLane(MonospacedLaneMsg &lane_msg,
                                      const int sub_yellow_lane) {
    int sub_begin = 0;
    int sub_end = lane_msg.monospaced_lane.size() - 1;
    for (int i = 0; sub_yellow_lane + i <= sub_end; ++i) {
        lane_msg.monospaced_lane[sub_yellow_lane + i].lane_line.id = i;
        if (sub_yellow_lane + i < sub_end) {
            lane_msg.monospaced_section[sub_yellow_lane + i].center_line.id = i;
            if (lane_msg.monospaced_section[sub_yellow_lane + i]
                    .has_stop_line) {
                lane_msg.monospaced_section[sub_yellow_lane + i]
                    .stop_lane_line.id = i;
            }
        }
    }
    for (int i = 1; sub_yellow_lane - i >= sub_begin; ++i) {
        lane_msg.monospaced_lane[sub_yellow_lane - i].lane_line.id = -i;
        lane_msg.monospaced_section[sub_yellow_lane - i].center_line.id = -i;
        if (lane_msg.monospaced_section[sub_yellow_lane - i].has_stop_line) {
            lane_msg.monospaced_section[sub_yellow_lane - i].stop_lane_line.id =
                -i;
        }
    }
}

void LaneContainer::PreProcess(
    const LaneInfo &lane_info,
    const TrafficLightFrameResult &traffic_lights_results) {
    FilterLane(lane_info, traffic_lights_results);
    UpdateLaneByHistory(lane_lines_, stop_lines_);
}

void LaneContainer::ProcessLane() {
    if (!(lane_lines_.size()) && !(stop_lines_.size())) {
        Clear();
    } else if (!(lane_lines_.size()) && (stop_lines_.size())) {
        lane_msg_.monospaced_section.clear();
    } else {
        Clear();
        InitializeLane(lane_lines_);
        AddLostLane(lane_msg_);
        MatchLaneInfo(lane_msg_);
    }
}

void LaneContainer::MatchLaneInfo(MonospacedLaneMsg &lane_msg) {
    for (int i = 0; i < lane_msg.monospaced_lane.size(); ++i) {
        Point2d lane_begin = lane_msg.monospaced_lane[i].lane_line.point_begin;
        double distance_min = std::numeric_limits<double>::max();
        int sub_dis_min = -1;
        for (int j = 0; j < lane_lines_.size(); ++j) {
            Point2d origin_begin;
            origin_begin.x = lane_lines_[j].lane_line.back().point.position_m.x;
            origin_begin.y = lane_lines_[j].lane_line.back().point.position_m.y;
            Point2d origin_end;
            origin_end.x = lane_lines_[j].lane_line[0].point.position_m.x;
            origin_end.y = lane_lines_[j].lane_line[0].point.position_m.y;
            double distance = std::fabs(common::DistancePoint2Line(
                origin_begin, origin_end, lane_begin));
            if (distance < distance_min) {
                distance_min = distance;
                sub_dis_min = j;
            }
        }

        if (distance_min < distance_match_limit_) {
            lane_msg.monospaced_lane[i].lane_line_color =
                lane_lines_[sub_dis_min].lane_line[0].color_type;
            lane_msg.monospaced_lane[i].lane_line_type =
                lane_lines_[sub_dis_min].lane_line_type;
        }
    }
}

void LaneContainer::ProcessSection() {
    if (lane_msg_.monospaced_lane.size() < 2) {
        return;
    } else {
        GenerateSection(lane_msg_);
    }
}

void LaneContainer::GenerateSection(MonospacedLaneMsg &lane_msg) {
    lane_msg.monospaced_section.clear();
    int sum_section = lane_msg.monospaced_lane.size() - 1;
    for (int i = 0; i < sum_section; ++i) {
        MonospacedSection section;
        section.left_lane_line = lane_msg.monospaced_lane[i];
        section.right_lane_line = lane_msg.monospaced_lane[i + 1];
        GenerateCenterLine(section);
        UpdateSectionState(section);
        lane_msg.monospaced_section.emplace_back(std::move(section));
    }

    int sub_ego = -1;
    for (int i = 0; i < lane_msg.monospaced_section.size(); ++i) {
        if (lane_msg.monospaced_section[i]
                .left_lane_line.lane_line.point_begin.x < 0) {
            sub_ego = i;
        } else {
            break;
        }
    }
    lane_msg.sub_ego_in_section = sub_ego;

    if (stop_lines_.size()) {
        GenerateStopLine(lane_msg, stop_lines_);
    }
}
void LaneContainer::GenerateStopLine(MonospacedLaneMsg &lane_msg,
                                     const Lanes &stop_lines) {
    if (!stop_lines_.size()) {
        return;
    }

    Point2d stop_begin;
    stop_begin.x = stop_lines[0].lane_line[0].point.position_m.x;
    stop_begin.y = stop_lines[0].lane_line[0].point.position_m.y;
    Point2d stop_end;
    stop_end.x = stop_lines[0].lane_line.back().point.position_m.x;
    stop_end.y = stop_lines[0].lane_line.back().point.position_m.y;
    int sub_intersect_begin = -1;
    int sub_intersect_end = -1;
    int sum_lane = lane_msg.monospaced_lane.size();
    for (int i = 0; i < sum_lane; ++i) {
        Point2d lane_begin = lane_msg.monospaced_lane[i].lane_line.point_begin;
        Point2d lane_end = lane_msg.monospaced_lane[i].lane_line.point_end;
        if (common::IsIntersectedByLineSection(stop_begin, stop_end, lane_begin,
                                               lane_end)) {
            sub_intersect_begin = i;
            break;
        };
    }

    if (sub_intersect_begin == -1) {
        // not intersected
        return;
    } else {
        for (int i = 0; i < sum_lane; ++i) {
            Point2d lane_begin = lane_msg.monospaced_lane[sum_lane - 1 - i]
                                     .lane_line.point_begin;
            Point2d lane_end =
                lane_msg.monospaced_lane[sum_lane - 1 - i].lane_line.point_end;
            if (common::IsIntersectedByLineSection(stop_begin, stop_end,
                                                   lane_begin, lane_end)) {
                sub_intersect_end = sum_lane - 1 - i;
                break;
            };
        }
    }
    if (sub_intersect_begin != 0) {
        Point2d lane_stop1_begin =
            lane_msg.monospaced_lane[sub_intersect_begin].lane_line.point_begin;
        Point2d lane_stop1_end =
            lane_msg.monospaced_lane[sub_intersect_begin].lane_line.point_end;
        double distance2begin = std::fabs(common::DistancePoint2Line(
            lane_stop1_begin, lane_stop1_end, stop_begin));
        if (distance2begin > lane_width_ / 2) {
            --sub_intersect_begin;
        }
    }

    if (sub_intersect_end != sum_lane - 1) {
        Point2d lane_stop2_begin =
            lane_msg.monospaced_lane[sub_intersect_end].lane_line.point_begin;
        Point2d lane_stop2_end =
            lane_msg.monospaced_lane[sub_intersect_end].lane_line.point_end;
        double distance2end = std::fabs(common::DistancePoint2Line(
            lane_stop2_begin, lane_stop2_end, stop_end));
        if (distance2end > lane_width_ / 2) {
            ++sub_intersect_end;
        }
    }

    if (sub_intersect_begin == sub_intersect_end) {
        return;
    }

    Point2d lane_begin_begin =
        lane_msg.monospaced_lane[0].lane_line.point_begin;
    Point2d lane_begin_end = lane_msg.monospaced_lane[0].lane_line.point_end;
    Point2d lane_cut_begin = common::PointIntersect(
        lane_begin_begin, lane_begin_end, stop_begin, stop_end);
    Point2d lane_end_begin =
        lane_msg.monospaced_lane.back().lane_line.point_begin;
    Point2d lane_end_end = lane_msg.monospaced_lane.back().lane_line.point_end;
    Point2d lane_cut_end = common::PointIntersect(lane_end_begin, lane_end_end,
                                                  stop_begin, stop_end);

    int sum_section = lane_msg.monospaced_section.size();
    double dx_cut = lane_cut_end.x - lane_cut_begin.x;
    double dy_cut = lane_cut_end.y - lane_cut_begin.y;
    double dx = dx_cut / (2 * sum_section);
    double dy = dy_cut / (2 * sum_section);
    // SG_INFO("----------------");
    for (int i = 0; i < sum_section; ++i) {
        Point2d left_point_end;
        left_point_end.x = lane_cut_begin.x + dx * i * 2;
        left_point_end.y = lane_cut_begin.y + dy * i * 2;
        Point2d center_point_end;
        center_point_end.x = lane_cut_begin.x + dx * (i * 2 + 1);
        center_point_end.y = lane_cut_begin.y + dy * (i * 2 + 1);
        Point2d right_point_end;
        right_point_end.x = lane_cut_begin.x + dx * (i * 2 + 2);
        right_point_end.y = lane_cut_begin.y + dy * (i * 2 + 2);

        lane_msg.monospaced_section[i].left_lane_line.lane_line.point_end =
            left_point_end;
        lane_msg.monospaced_section[i].center_line.point_end = center_point_end;
        lane_msg.monospaced_section[i].right_lane_line.lane_line.point_end =
            right_point_end;
        // SG_INFO(
        //     "lane_x=%lf,lane_y=%lf",
        //     lane_msg.monospaced_section[i].left_lane_line.lane_line.point_end.x,
        //     lane_msg.monospaced_section[i]
        //         .left_lane_line.lane_line.point_end.y);
        // SG_INFO("center_x=%lf,center_y=%lf",
        //         lane_msg.monospaced_section[i].center_line.point_end.x,
        //         lane_msg.monospaced_section[i].center_line.point_end.y);
    }
    // SG_INFO("lane_x=%lf,lane_y=%lf",
    //         lane_msg.monospaced_section.back()
    //             .right_lane_line.lane_line.point_end.x,
    //         lane_msg.monospaced_section.back()
    //             .right_lane_line.lane_line.point_end.y);

    // for (int i = 0; i < sum_section; ++i) {
    //     lane_msg.monospaced_section[i].left_lane_line.lane_line.point_end =
    //         common::PointDivide(lane_cut_begin, lane_cut_end, 2 *
    //         sum_section,
    //                             2 * i);
    //     lane_msg.monospaced_section[i].center_line.point_end =
    //         common::PointDivide(lane_cut_begin, lane_cut_end, sum_section,
    //                             (2 * i + 1));
    //     SG_INFO("-------------");
    //     SG_INFO(
    //         "lane_x=%lf,lane_y=%lf",
    //         lane_msg.monospaced_section[i].left_lane_line.lane_line.point_end.x,
    //         lane_msg.monospaced_section[i]
    //             .left_lane_line.lane_line.point_end.y);
    //     SG_INFO("center_x=%lf,center_y=%lf",
    //             lane_msg.monospaced_section[i].center_line.point_end.x,
    //             lane_msg.monospaced_section[i].center_line.point_end.y);
    //     lane_msg.monospaced_section[i].right_lane_line.lane_line.point_end =
    //         common::PointDivide(lane_cut_begin, lane_cut_end, sum_section,
    //                             (2 * i + 2));
    // }
    // SG_INFO("lane_x=%lf,lane_y=%lf",
    //         lane_msg.monospaced_section.back()
    //             .right_lane_line.lane_line.point_end.x,
    //         lane_msg.monospaced_section.back()
    //             .right_lane_line.lane_line.point_end.y);

    double dx_stop_line = stop_end.x - stop_begin.x;
    double dy_stop_line = stop_end.y - stop_begin.y;
    double stop_line_yaw = std::atan2(dy_stop_line, dx_stop_line);

    for (int i = sub_intersect_begin; i < sub_intersect_end; ++i) {
        lane_msg.monospaced_section[i].stop_lane_line.point_begin =
            lane_msg.monospaced_section[i].left_lane_line.lane_line.point_end;
        lane_msg.monospaced_section[i].stop_lane_line.point_end =
            lane_msg.monospaced_section[i].right_lane_line.lane_line.point_end;
        lane_msg.monospaced_section[i].stop_lane_line.yaw_veh_rad =
            stop_line_yaw;
        lane_msg.monospaced_section[i].has_stop_line = true;
    }
}

void LaneContainer::GenerateCenterLine(MonospacedSection &section) {
    double Lane_begin_lx = section.left_lane_line.lane_line.point_begin.x;
    double Lane_begin_ly = section.left_lane_line.lane_line.point_begin.y;
    double Lane_begin_rx = section.right_lane_line.lane_line.point_begin.x;
    double Lane_begin_ry = section.right_lane_line.lane_line.point_begin.y;
    double Lane_end_lx = section.left_lane_line.lane_line.point_end.x;
    double Lane_end_ly = section.left_lane_line.lane_line.point_end.y;
    double Lane_end_rx = section.right_lane_line.lane_line.point_end.x;
    double Lane_end_ry = section.right_lane_line.lane_line.point_end.y;
    double lane_yaw = section.left_lane_line.lane_line.yaw_veh_rad;
    section.center_line.point_begin.x = (Lane_begin_lx + Lane_begin_rx) / 2;
    section.center_line.point_end.x = (Lane_end_lx + Lane_end_rx) / 2;
    section.center_line.point_begin.y = (Lane_begin_ly + Lane_begin_ry) / 2;
    section.center_line.point_end.y = (Lane_end_ly + Lane_end_ry) / 2;
    section.center_line.yaw_veh_rad = lane_yaw;
}

void LaneContainer::UpdateSectionState(MonospacedSection &section) {
    if (section.left_lane_line.lane_line_type == LaneLineType::SOLID ||
        section.left_lane_line.lane_line_type == LaneLineType::SOLID_BROKEN ||
        section.left_lane_line.lane_line_type == LaneLineType::SOLID_SOLID) {
        section.can_cutted_left = false;
    }

    if (section.right_lane_line.lane_line_type == LaneLineType::SOLID ||
        section.right_lane_line.lane_line_type == LaneLineType::BROKEN_SOLID ||
        section.right_lane_line.lane_line_type == LaneLineType::SOLID_SOLID) {
        section.can_cutted_right = false;
    }
}

void LaneContainer::UpdateCenterLine(LaneInfo &lane_info,
                                     const MonospacedLaneMsg &lane_msg) {
    if (lane_msg.monospaced_section.empty()) {
        return;
    }
    for (int i = 0; i < lane_msg.monospaced_section.size(); ++i) {
        CenterLine center_line;
        PathPoint path_point;
        double point_x_begin =
            lane_msg.monospaced_section[i].center_line.point_begin.x;
        double point_y_begin =
            lane_msg.monospaced_section[i].center_line.point_begin.y;
        double point_x_end =
            lane_msg.monospaced_section[i].center_line.point_end.x;
        double point_y_end =
            lane_msg.monospaced_section[i].center_line.point_end.y;
        double theta = lane_msg.monospaced_section[i].center_line.yaw_veh_rad;
        path_point.position_m.x = point_x_begin;
        path_point.position_m.y = point_y_begin;
        path_point.theta_rad = theta;
        for (double s = 0.0; path_point.position_m.y < point_y_end;
             s += center_line_interval_) {
            path_point.position_m.x = point_x_begin + s * std::cos(theta);
            path_point.position_m.y = point_y_begin + s * std::sin(theta);
            path_point.s_m = s;
            center_line.points.emplace_back(path_point);
        }
        center_line.id = lane_msg.monospaced_section[i].center_line.id;
        lane_info.center_line.emplace_back(center_line);
    }
}

}  // namespace prediction_lib
}  // namespace jarvis