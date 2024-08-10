//|
//|    Copyright (c) 2024 Ioannis Tsikelis
//|    Copyright (c) 2024 Konstantinos Chatzilygeroudis
//|    Copyright (c) 2024 Laboratory of Automation and Robotics, University of Patras, Greece
//|    Copyright (c) 2024 Computational Intelligence Lab, University of Patras, Greece
//|
//|    Authors:  Ioannis Tsikelis, Konstantinos Chatzilygeroudis
//|    emails:    tsikelis.i@proton.me, costashatz@gmail.com
//|    website:  https://nosalro.github.io/cregopt
//|
//|    This file is part of cregopt.
//|
//|    All rights reserved.
//|
//|    Redistribution and use in source and binary forms, with or without
//|    modification, are permitted provided that the following conditions are met:
//|
//|    1. Redistributions of source code must retain the above copyright notice, this
//|       list of conditions and the following disclaimer.
//|
//|    2. Redistributions in binary form must reproduce the above copyright notice,
//|       this list of conditions and the following disclaimer in the documentation
//|       and/or other materials provided with the distribution.
//|
//|    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
//|    AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
//|    IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
//|    DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
//|    FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
//|    DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
//|    SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
//|    CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
//|    OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
//|    OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//|
#include <cem_trajopt/viz/viz.hpp>

#include <iostream>
#include <string>

#include <ifopt/problem.h>

#if VIZ

#include <robot_dart/gui/magnum/graphics.hpp>
#include <robot_dart/robot_dart_simu.hpp>

#endif // VIZ

namespace cem_trajopt {
    void visualise(const towr::SplineHolder& solution, unsigned int ee_count, double dt, const std::string vid_name)
    {
#if VIZ
        robot_dart::RobotDARTSimu simu(dt);
        auto graphics = std::make_shared<robot_dart::gui::magnum::Graphics>();
        simu.set_graphics(graphics);
        Eigen::Vector3d cam_pos = {0., 6., 4.};
        graphics->look_at(cam_pos);
        // simu.set_graphics_freq(500);
        if (vid_name != "") {
            graphics->record_video(std::string(SRCPATH) + "/" + vid_name);
        }

        simu.add_checkerboard_floor();
        // auto gap = robot_dart::Robot::create_box(Eigen::Vector3d(0.2, 10., 0.01), Eigen::Isometry3d::Identity(), "free", 0.1, dart::Color::Blue(1.0));
        // simu.add_visual_robot(gap);
        // simu.robot(1)->set_positions(robot_dart::make_vector({0., 0., 0., 2., 0., 0.}));
        // auto terrain = std::make_shared<robot_dart::Robot>(std::string(SRCPATH) + "/step-terrain.urdf");
        // simu.add_visual_robot(terrain);

        auto base = robot_dart::Robot::create_box(Eigen::Vector3d(0.6, 0.2, 0.15), Eigen::Isometry3d::Identity(), "free");

        switch (ee_count) {
        case 2:
            base = robot_dart::Robot::create_box(Eigen::Vector3d(0.3, 0.2, 0.15), Eigen::Isometry3d::Identity(), "free");
            break;
        case 4:
            base = robot_dart::Robot::create_box(Eigen::Vector3d(0.6, 0.2, 0.15), Eigen::Isometry3d::Identity(), "free");
            break;
        case 6:
            base = robot_dart::Robot::create_box(Eigen::Vector3d(0.24, 0.2, 0.04), Eigen::Isometry3d::Identity(), "free");
        default:
            std::cerr << "Invalid ee count!" << std::endl;
            assert(false);
            break;
        }

        simu.add_visual_robot(base);

        size_t num_feet = solution.ee_motion_.size();
        for (size_t k = 0; k < num_feet; ++k) {
            auto foot = robot_dart::Robot::create_ellipsoid(Eigen::Vector3d(0.05, 0.05, 0.05), Eigen::Isometry3d::Identity(), "free", 0.1, dart::Color::Red(1.0), "foot" + std::to_string(k));
            simu.add_visual_robot(foot);
        }
        // Visualise trajectory.
        double total_time = solution.base_linear_->GetTotalTime();
        size_t iters = total_time / dt + 1;

        double t = 0.;
        for (size_t i = 0; i < iters; ++i) {
            // std::cout << solution.base_linear_->GetPoint(t).p() << "\t[m]" << std::endl;
            Eigen::Vector3d body_pos = solution.base_linear_->GetPoint(t).p();
            Eigen::Vector3d body_rot = solution.base_angular_->GetPoint(t).p(); // Get body roll, pitch, yaw (XYZ)

            Eigen::AngleAxisd x_rot(body_rot[0], Eigen::Vector3d::UnitX());
            Eigen::AngleAxisd y_rot(body_rot[1], Eigen::Vector3d::UnitY());
            Eigen::AngleAxisd z_rot(body_rot[2], Eigen::Vector3d::UnitZ());
            auto tf = Eigen::Isometry3d::Identity();
            tf.rotate(x_rot);
            tf.rotate(y_rot);
            tf.rotate(z_rot);

            auto rot = dart::math::logMap(tf);
            simu.robot(1)->set_positions(robot_dart::make_vector({rot[0], rot[1], rot[2], body_pos[0], body_pos[1], body_pos[2]}));

            for (size_t k = 0; k < num_feet; ++k) {
                Eigen::Vector3d foot_pos = solution.ee_motion_.at(k)->GetPoint(t).p();
                simu.robot(k + 2)->set_positions(robot_dart::make_vector({0., 0., 0., foot_pos[0], foot_pos[1], foot_pos[2] + 0.025}));
            }

            simu.step_world();

            t += dt;
        }
        simu.run(1.);
#endif // VIZ
    }

} // namespace cem_trajopt
