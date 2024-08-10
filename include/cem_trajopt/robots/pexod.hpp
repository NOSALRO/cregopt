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
#pragma once

#include <towr/models/endeffector_mappings.h>
#include <towr/models/kinematic_model.h>
#include <towr/models/robot_model.h>
#include <towr/models/single_rigid_body_dynamics.h>

namespace towr {

    /**
     * @brief The Kinematics of the pexod hexapod robot.
     */
    class PexodKinematicModel : public KinematicModel {
    public:
        PexodKinematicModel() : KinematicModel(6)
        {
            const double x_nominal_b = 0.11;
            const double y_nominal_b = 0.185;
            const double z_nominal_b = -0.14;

            nominal_stance_.at(0) << x_nominal_b, y_nominal_b, z_nominal_b;
            nominal_stance_.at(1) << x_nominal_b, -y_nominal_b, z_nominal_b;

            nominal_stance_.at(2) << 0., y_nominal_b, z_nominal_b;
            nominal_stance_.at(3) << 0., -y_nominal_b, z_nominal_b;

            nominal_stance_.at(4) << -x_nominal_b, y_nominal_b, z_nominal_b;
            nominal_stance_.at(5) << -x_nominal_b, -y_nominal_b, z_nominal_b;

            max_dev_from_nominal_ << 0.15, 0.1, 0.05;
        }
    };

    /**
     * @brief The Dynamics of the pexod hexapod robot.
     */
    class PexodDynamicModel : public SingleRigidBodyDynamics {
    public:
        PexodDynamicModel()
            : SingleRigidBodyDynamics(1.031, 0.00357413333333, 0., 0., 0.00508626666667, 0.0, 0.00838546666667, 6) {}
    };

    class PexodRobot : public RobotModel {
    public:
        PexodRobot() : RobotModel()
        {
            kinematic_model_ = std::make_shared<PexodKinematicModel>();
            dynamic_model_ = std::make_shared<PexodDynamicModel>();
        }
    };

} // namespace towr
