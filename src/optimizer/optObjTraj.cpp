#include "optObjTraj.h"

std::vector<gtsam::Pose2> optObjTraj::tuple2gtsam(
    const std::vector<tuple<double, double, double>> &input) {
    std::vector<gtsam::Pose2> res;
    for (int i = 0; i < input.size(); i++) {
        gtsam::Pose2 temp(get<0>(input[i]), get<1>(input[i]), get<2>(input[i]));
        res.emplace_back(temp);
    }
    return res;
}

std::vector<tuple<double, double, double>> optObjTraj::gtsam2tuple(
    const std::vector<gtsam::Pose2> &input) {
    std::vector<tuple<double, double, double>> res;
    for (int i = 0; i < input.size(); i++) {
        tuple<double, double, double> temp(input[i].x(), input[i].y(),
                                           input[i].theta());
        res.emplace_back(temp);
    }
    return res;
}

bool optObjTraj::optTraj(
    const std::vector<tuple<double, double, double>> &input,
    std::vector<tuple<double, double, double>> &output, const bool &flag,
    const int &algorithem) {
    vector<Pose2> pose = tuple2gtsam(input);

    int size = pose.size();
    double rad_delta = (RAD3 - RAD2) / size;
    double xy_delta = 0.2 / size;

    if (5 > size) {
        // cout << "Input vector size too small!" << endl;
        return false;
    }

    for (int i = 0; i < size; i++) {
        if (0 == i) {
            noiseModel::Diagonal::shared_ptr priorNoise =
                noiseModel::Diagonal::Sigmas(
                    Vector3(5e-1, 5e-1, RAD3));  // meter,meter,rad
            m_graph.add(PriorFactor<Pose2>(0, pose[i], priorNoise));
            m_initials.insert(0, pose[i]);

        }
        //角度误差由3度线性递减到2度
        // XY距离误差由0.5m线性递减到0.3m
        //后观测的置信度高
        else {
            Vector3 sigmas_delta, temp;
            Vector3 const_delta(5e-1, 5e-1, RAD3);
            gtsam::Pose2 poseFrom = pose[i - 1];
            //视觉深度约束-测试中
            if (flag) {
                Vector2 pose1(pose[i].x() - pose[i - 1].x(),
                              pose[i].y() - pose[i - 1].y());
                double dis = sqrt(pow(pose[i].x() - pose[i - 1].x(), 2) +
                                  pow(pose[i].y() - pose[i - 1].y(), 2));
                double standard = (pose[i - 1].theta() + pose[i].theta()) / 2;
                Vector2 pose2(
                    dis * cos((pose[i - 1].theta() + pose[i].theta()) / 2),
                    dis * sin((pose[i - 1].theta() + pose[i].theta()) / 2));
                double theta = fabs(atan2(pose1.y(), pose1.x()) -
                                    atan2(pose2.y(), pose2.x()));
                if (theta > standard) {
                    temp << pose[i - 1].x() + dis * cos((pose[i - 1].theta() +
                                                         pose[i].theta()) /
                                                        2),
                        pose[i - 1].y() +
                            dis * sin((pose[i - 1].theta() + pose[i].theta()) /
                                      2),
                        (pose[i - 1].theta() + pose[i].theta()) / 2;
                } else {
                    temp << pose[i].x(), pose[i].y(), pose[i].theta();
                }
            } else {
                temp << pose[i].x(), pose[i].y(), pose[i].theta();
            }
            gtsam::Pose2 poseTo(temp[0], temp[1], temp[2]);
            sigmas_delta << 5e-1 - i * xy_delta, 5e-1 - i * xy_delta,
                RAD3 - i * rad_delta;
            noiseModel::Base::shared_ptr hubernoise =
                noiseModel::Robust::Create(
                    noiseModel::mEstimator::Huber::Create(0.1),
                    gtsam::noiseModel::Diagonal::Variances(sigmas_delta));
            gtsam::Pose2 relPose = poseFrom.between(poseTo);
            m_graph.add(BetweenFactor<Pose2>(i - 1, i, relPose, hubernoise));
            m_initials.insert(i, poseTo);
        }
    }

    for (int i = 2; i < size; i++) {
        Pose2 poseprep = pose[i - 2];
        Pose2 posepre = pose[i - 1];
        Pose2 posecru = pose[i];
        double delta_x = posepre.x() - poseprep.x();
        double delta_y = posepre.y() - poseprep.y();

        double delta_x2 = posecru.x() - posepre.x();
        double delta_y2 = posecru.y() - posepre.y();
        double delta_theta2 = posecru.theta() - posepre.theta();
        double delta_theta = posepre.theta() - poseprep.theta();

        Vector3 error;
        if (fabs(delta_x2 - delta_x) >= 1)
            error[0] = 1e-2 / fabs(delta_x2 - delta_x);
        else if (fabs(delta_x2 - delta_x) >= 1e-1)
            error[0] = 1 - fabs(delta_x2 - delta_x);
        else
            error[0] = 1;

        if (fabs(delta_y2 - delta_y) >= 1)
            error[1] = 1e-2 / fabs(delta_y2 - delta_y);
        else if (fabs(delta_y2 - delta_y) >= 1e-1)
            error[1] = 1 - fabs(delta_y2 - delta_y);
        else
            error[1] = 1;

        if (fabs(delta_theta2 - delta_theta) <= RAD1)
            error[2] = RAD5;
        else if (fabs(delta_theta2 - delta_theta) <= RAD5)
            error[2] = RAD5 + RAD1 - fabs(delta_theta2 - delta_theta);
        else
            error[2] = RAD1 / 2;

        noiseModel::Diagonal::shared_ptr veloNoise =
            noiseModel::Diagonal::Sigmas(error);
        // noiseModel::Base::shared_ptr robustveloNoise =
        // noiseModel::Robust::Create
        // (noiseModel::mEstimator::Cauchy::Create(1),gtsam::noiseModel::Diagonal::Variances(error));
        // noiseModel::Base::shared_ptr hubernoise = noiseModel::Robust::Create
        // (noiseModel::mEstimator::Huber::Create(0.1),gtsam::noiseModel::Diagonal::Variances(error));
        double dis = sqrt(pow(delta_x, 2) + pow(delta_y, 2));
        Pose2 predict(
            pose[i - 1].x() + dis * cos(pose[i - 1].theta() + delta_theta2 / 2),
            pose[i - 1].y() + dis * sin(pose[i - 1].theta() + delta_theta2 / 2),
            pose[i - 1].theta() + delta_theta2 / 2);
        Pose2 trans = pose[i - 1].between(predict);
        m_graph.add(BetweenFactor<Pose2>(i - 1, i, trans, veloNoise));
    }

    switch (algorithem) {
        case 0: {
            // m_gaussParam.setVerbosity("ERROR");
            m_gaussParam.setMaxIterations(20);
            m_gaussParam.setLinearSolverType("MULTIFRONTAL_QR");
            GaussNewtonOptimizer optimizer(m_graph, m_initials, m_gaussParam);
            m_estimate = optimizer.optimize();
            break;
        }
        case 1: {
            m_isamParam.relinearizeThreshold = 0.1;
            m_isamParam.relinearizeSkip = 1;
            m_isam = new ISAM2(m_isamParam);
            m_isam->update(m_graph, m_initials);
            m_isam->update();
            m_estimate = m_isam->calculateEstimate();
            break;
        }
        case 2: {
            m_levenParam.relativeErrorTol = 1e-4;
            m_levenParam.maxIterations = 20;
            gtsam::LevenbergMarquardtOptimizer optimizer(m_graph, m_initials,
                                                         m_levenParam);
            m_estimate = optimizer.optimize();
            break;
        }
        default:
            // std::cout << "Algorithem param input wrong!" << std::endl;
            return false;
    }

    // m_initials.print("Initial Estimate:\n");
    // m_estimate.print("Final Result:\n");

    int count = 0;
    gtsam::Values::ConstFiltered<gtsam::Pose2> viewPose =
        m_estimate.filter<gtsam::Pose2>();
    for (const gtsam::Values::ConstFiltered<gtsam::Pose2>::KeyValuePair
             &key_value : viewPose) {
        tuple<double, double, double> temp(
            key_value.value.x(), key_value.value.y(), key_value.value.theta());
        output.emplace_back(temp);
    }

    m_graph.resize(0);
    m_initials.clear();
    m_estimate.clear();

    if (1 == algorithem) {
        delete m_isam;
        m_isam = NULL;
    }

    return true;
}