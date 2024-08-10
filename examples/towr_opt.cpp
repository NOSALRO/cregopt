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
// Experiment with max number of steps and towr gait optimisations

#include <algevo/algo/cem_discrete.hpp>

#include <iostream>

#include <ifopt/composite.h>
#include <ifopt/ipopt_solver.h>
#include <ifopt/problem.h>

#include <cem_trajopt/robots/pexod.hpp>

#include <cem_trajopt/terrain/gap.hpp>
#include <cem_trajopt/terrain/terrain_grid.hpp>

#include <cem_trajopt/utils/utils.hpp>
#include <cem_trajopt/viz/viz.hpp>

// Experiment parameters
namespace params {
#if defined(BIPED)
    static constexpr unsigned int ee_count = 2;
    static const Eigen::Vector3d init_base_pos = Eigen::Vector3d(0., 0., 0.65);
    static const Eigen::Vector3d target_base_pos = Eigen::Vector3d(6., 0., 0.65);
#elif defined(HEXAPOD)
    static constexpr unsigned int ee_count = 6;
    static const Eigen::Vector3d init_base_pos = Eigen::Vector3d(0., 0., 0.14);
    static const Eigen::Vector3d target_base_pos = Eigen::Vector3d(6., 0., 0.14);
#else
    static constexpr unsigned int ee_count = 4;
    static const Eigen::Vector3d init_base_pos = Eigen::Vector3d(0., 0., 0.5);
    static const Eigen::Vector3d target_base_pos = Eigen::Vector3d(6., 0., 0.5);
#endif

    static constexpr unsigned int min_stance_phases = 5;
    static constexpr unsigned int num_values = 5; // number of total steps will thus be = num_values + min_stance_phases - 1

    // Towr parameters
    static constexpr int ee_polynomials_per_swing_phase = 2;
    static constexpr double min_phase_duration = 0.08;
    static constexpr double max_phase_duration = 0.36;
    static constexpr double stance_duration = 0.2;
    static constexpr double swing_duration = 0.1;
    static constexpr bool optimise_time = true;

    // IPOPT parameters
    static constexpr int max_iter = 200;
    static constexpr int print_level = 0;
    static constexpr double max_wall_time = 40.;
    static const std::string jacobian_approximation = "exact";
    static const std::string nlp_scaling_method = "gradient-based"; // none
    static const std::string linear_solver = "ma97"; // ma27

    // Terrain parameters
    static constexpr size_t rows = 50;
    static constexpr size_t cols = 50;
    static constexpr double friction_coeff = 1.;
    static constexpr double min_x = -10.;
    static constexpr double min_y = -10.;
    static constexpr double max_x = 10.;
    static constexpr double max_y = 10.;

#if defined(USE_STEP_TERRAIN)
    static const std::string grid_csv_filename = "./../../../../../../tools/step.csv";
#endif

    // Misc
    static constexpr bool print_eval_wall_time = false; // Print wall time elapsed in towr optimisation.
    static constexpr bool export_to_csv = true; // Save cem solution to csv file.
    static const std::string results_filename = "towr_opt.txt";
} // namespace params

int main(int argc, char* argv[])
{
    if (argc != 2) {
        std::cerr << "Usage: " << argv[0] << " <index>" << std::endl;
        return 1;
    }

    int index = std::atoi(argv[1]);
    if (index < 1 || index > 20) {
        std::cerr << "Error: Index out of range ( " << index << ". It must be between 1 and 20." << std::endl;
        return 1;
    }

    const double start_angle = -M_PI / 4;
    const double end_angle = M_PI / 4;
    const int num_points = 20;
    double step_size = (end_angle - start_angle) / static_cast<double>(num_points - 1);
    double angle = start_angle + (index - 1) * step_size;

    ifopt::Problem nlp;
    towr::NlpFormulation formulation;

    auto terrain = std::make_shared<cem_trajopt::TerrainGrid>(params::rows, params::cols, params::friction_coeff, params::min_x, params::min_y, params::max_x, params::max_y);
#if defined(USE_STEP_TERRAIN)
    terrain->ReadFromCsv(params::grid_csv_filename);
#else
    terrain->SetZero();
#endif

    Eigen::Matrix<unsigned int, 1, params::ee_count> num_stance_phases;
    for (size_t i = 0; i < static_cast<size_t>(num_stance_phases.cols()); ++i)
        num_stance_phases(0, i) = params::num_values;
    num_stance_phases = num_stance_phases.array() + params::min_stance_phases;

    std::vector<bool> standing_at_start(params::ee_count, true);

    formulation.terrain_ = terrain;

    formulation.initial_base_.lin.at(towr::kPos) = params::init_base_pos;
    formulation.final_base_.lin.at(towr::kPos) = Eigen::Vector3d(std::cos(angle) * params::target_base_pos[0], std::cos(angle) * params::target_base_pos[1], params::target_base_pos[2]);

#if defined(ROTATE)
    formulation.final_base_.ang.at(towr::kPos).z() = M_PI;
#endif

#if defined(BIPED)
    formulation.model_ = towr::RobotModel(towr::RobotModel::Biped);
    cem_trajopt::AddBipedInitialEePositions(formulation);
#elif defined(HEXAPOD)
    formulation.model_ = towr::PexodRobot();
    cem_trajopt::AddHexapodInitialEePositions(formulation);
#else
    formulation.model_ = towr::RobotModel(towr::RobotModel::Anymal);
    cem_trajopt::AddAnymalInitialEePositions(formulation);
#endif

    for (size_t i = 0; i < params::ee_count; ++i) {
        std::vector<double> durations;
        size_t steps = num_stance_phases[i];

        bool is_stance = standing_at_start[i];
        size_t step_counter = 0;
        while (step_counter < steps) {
            if (is_stance) {
                step_counter++;
                durations.push_back(params::stance_duration);
            }
            else {
                durations.push_back(params::swing_duration);
            }
            is_stance = !is_stance;
        }

        formulation.params_.ee_phase_durations_.push_back(durations);

        formulation.params_.ee_in_contact_at_start_.push_back(true);
    }

    cem_trajopt::FixDurations(formulation.params_.ee_phase_durations_);

    if (params::optimise_time) {
        // Find foot with maximum step number
        unsigned int max = 0.;
        for (auto& s : num_stance_phases) {
            if (s > max) {
                max = s;
            }
        }
        // Bound the phase duration according to the largest step count
        formulation.params_.bound_phase_duration_ = std::make_pair(params::min_phase_duration, params::max_phase_duration);
        formulation.params_.OptimizePhaseDurations();
    }

    // bounds on final 6DoF base state
    formulation.params_.bounds_final_lin_pos_ = {towr::X, towr::Y, towr::Z};
    formulation.params_.bounds_final_lin_vel_ = {}; // {towr::X, towr::Y, towr::Z};
    formulation.params_.bounds_final_ang_pos_ = {towr::X, towr::Y, towr::Z};
    formulation.params_.bounds_final_ang_vel_ = {}; // {towr::X, towr::Y, towr::Z};

    // splines in each swing phase of motion ee spline
    formulation.params_.ee_polynomials_per_swing_phase_ = params::ee_polynomials_per_swing_phase;

    auto solution = towr::SplineHolder();
    for (auto c : formulation.GetVariableSets(solution))
        nlp.AddVariableSet(c);
    for (auto c : formulation.GetConstraints(solution))
        nlp.AddConstraintSet(c);
    for (auto c : formulation.GetCosts())
        nlp.AddCostSet(c);

    // // Set middle body node at 0.7.
    // double t_total = solution.base_linear_->GetTotalTime();
    // auto var = std::static_pointer_cast<towr::NodesVariables>(nlp.GetOptVariables()->GetComponent("base-lin"));
    // int mid_node_idx = cem_trajopt::FindClosestNode(var, t_total / 2., t_total);
    //
    // Eigen::VectorXd bounds(1);
    // bounds << 0.7;
    //
    // var->AddBounds(mid_node_idx, towr::kPos, {towr::Z}, bounds);
    // ///////////

    auto solver = std::make_shared<ifopt::IpoptSolver>();
    solver->SetOption("jacobian_approximation", params::jacobian_approximation);
    solver->SetOption("max_wall_time", params::max_wall_time);
    solver->SetOption("max_iter", params::max_iter);
    solver->SetOption("print_level", params::print_level);
    solver->SetOption("nlp_scaling_method", params::nlp_scaling_method);
    solver->SetOption("linear_solver", params::linear_solver);

    solver->Solve(nlp);

    if (params::export_to_csv) {
        std::ofstream file(params::results_filename);
        if (!file.is_open()) {
            std::cerr << "Error opening file!" << std::endl;
            return 1;
        }
        file << solver->GetTotalWallclockTime() << "\n";
        file << (cem_trajopt::GetNumViolations(nlp) == 0) << "\n";
        file.close();
    }

    // double t = 0.0;
    // while (t <= solution.base_linear_->GetTotalTime() + 1e-5) {
    //     std::cout << "t=" << t << "\n";
    //     std::cout << "Base linear position x,y,z:   \t";
    //     std::cout << solution.base_linear_->GetPoint(t).p().transpose() << "\t[m]" << std::endl;
    //
    //     // std::cout << "Base Euler roll, pitch, yaw:  \t";
    //     // Eigen::Vector3d rad = solution.base_angular_->GetPoint(t).p();
    //     // std::cout << (rad / M_PI * 180).transpose() << "\t[deg]" << std::endl;
    //
    //     for (size_t k = 0; k < params::ee_count; k++) {
    //         std::cout << "Foot " << k << " position x,y,z:          \t";
    //         std::cout << solution.ee_motion_.at(k)->GetPoint(t).p().transpose() << "\t[m]" << std::endl;
    //
    //         std::cout << "Contact force x,y,z:          \t";
    //         std::cout << solution.ee_force_.at(k)->GetPoint(t).p().transpose() << "\t[N]" << std::endl;
    //
    //         bool contact = solution.phase_durations_.at(k)->IsContactPhase(t);
    //         std::string foot_in_contact = contact ? "yes" : "no";
    //         std::cout << "Foot in contact:              \t" + foot_in_contact << std::endl;
    //     }
    //
    //     std::cout << std::endl;
    //
    //     t += 0.2;
    // }

    return 0;
}
