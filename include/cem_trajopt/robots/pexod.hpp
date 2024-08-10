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
