//|
//|    Copyright (c) 2024 Ioannis Tsikelis
//|    Copyright (c) 2024 Konstantinos Chatzilygeroudis
//|    Copyright (c) 2024 Laboratory of Automation and Robotics, University of Patras, Greece
//|    Copyright (c) 2024 Computational Intelligence Lab, University of Patras, Greece
//|
//|    Authors:  Ioannis Tsikelis, Konstantinos Chatzilygeroudis
//|    emails:   tsikelis.i@protonmail.com, costashatz@gmail.com
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
// Experiment with CEM-MD algorithm with heuristic

#include <iostream>

#include <algevo/algo/cem_discrete.hpp>

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
    static const Eigen::Vector3d init_base_pos = Eigen::Vector3d(0., 0., 0.5);
    static const Eigen::Vector3d target_base_pos = Eigen::Vector3d(4., 0., 0.5);

    static constexpr unsigned int min_stance_phases = 5;
    static constexpr unsigned int num_values = 3; // number of total steps will thus be = num_values + min_stance_phases

    static constexpr unsigned int num_triplets = 3; // HEURISTIC SPECIFIC: num of triplets
    static const std::vector<unsigned int> triplet_start = {0, 2, 4}; // triplet start
    static constexpr unsigned int max_triplet_value = 4 + 2; // triplet_start.back() + 2

    // Towr parameters
    static constexpr int ee_polynomials_per_swing_phase = 2;

    // IPOPT parameters
    static constexpr int max_iter = 50;
    static constexpr int print_level = 0;
    static constexpr double max_wall_time = 2.;
    static const std::string jacobian_approximation = "exact";
    static const std::string nlp_scaling_method = "gradient-based"; // none
    static const std::string linear_solver = "ma27"; // ma27

    // Terrain parameters
    static constexpr size_t rows = 10;
    static constexpr size_t cols = 20;
    static constexpr double friction_coeff = 1.;
    static constexpr double min_x = -5.;
    static constexpr double min_y = -5.;
    static constexpr double max_x = 5.;
    static constexpr double max_y = 5.;

    static constexpr bool read_from_csv = false;
    static const std::string grid_csv_filename = "../tools/gapped_stairs_10_20.csv";

    // Misc
    static constexpr bool print_eval_wall_time = false; // Print wall time elapsed in towr optimisation.
    // static constexpr bool export_to_csv = false; // Save cem-md solution to csv file.
    static const std::string discrete_csv_filename = "../tools/results/experiment_1/scenario_1/cregopt/run_17/cregopt_best_discrete.csv";
    static const std::string continuous_csv_filename = "../tools/results/experiment_1/scenario_1/cregopt/run_17/cregopt_best_continuous.csv";

} // namespace params

int main(int argc, char **argv)
{
    if (argc != 5){
        std::cout << "Usage: \n ee_count use_heuristic save_data_for_viz best_discrete best_continuous" << std::endl;
        return 1;
    }

    size_t ee_count = std::stoi(argv[1]);
    bool heuristic = !std::strcmp(argv[2], "true");
    std::string best_solution_discrete = argv[3];
    std::string best_solution_continuous = argv[4];

    ifopt::Problem nlp;
    towr::NlpFormulation formulation;

    // Create terrain
    auto terrain = std::make_shared<cem_trajopt::TerrainGrid>(params::rows, params::cols, params::friction_coeff, params::min_x, params::min_y, params::max_x, params::max_y);
    if (params::read_from_csv)
        terrain->ReadFromCsv(params::grid_csv_filename);
    else
        terrain->SetZero();

    std::vector<unsigned int> xd = cem_trajopt::ImportCemDataFromCSV<unsigned int>(best_solution_discrete);
    std::vector<double> xc = cem_trajopt::ImportCemDataFromCSV<double>(best_solution_continuous);

    std::vector<unsigned int> num_stance_phases(ee_count);

    if (heuristic) {
        // If using heuristic
        unsigned int offset = params::triplet_start.at(xd.at(0));
        for (size_t i = 0; i < ee_count; ++i) {
            num_stance_phases.at(i) = xd.at(i + 1) + params::min_stance_phases + offset;
        }
    }
    else {
        // If using simple cemmd
        for (auto& item : num_stance_phases) {
            item = item + params::min_stance_phases;
        }
    }


    std::vector<bool> standing_at_start(ee_count, true);

    formulation.terrain_ = terrain;

    formulation.initial_base_.lin.at(towr::kPos) = params::init_base_pos;
    formulation.final_base_.lin.at(towr::kPos) = params::target_base_pos;

    // formulation.final_base_.ang.at(towr::kPos).z() = M_PI;

    switch (ee_count) {
    case 2:
        formulation.model_ = towr::RobotModel(towr::RobotModel::Biped);
        cem_trajopt::AddBipedInitialEePositions(formulation);
        break;
    case 4:
        formulation.model_ = towr::RobotModel(towr::RobotModel::Anymal);
        cem_trajopt::AddAnymalInitialEePositions(formulation);
        break;
    case 6:
        formulation.model_ = towr::PexodRobot();
        cem_trajopt::AddHexapodInitialEePositions(formulation);
        break;
    default:
        std::cerr << "Invalid ee count!" << std::endl;
        break;
    }

    for (size_t k = 0; k < ee_count; ++k) {
        std::vector<double> durations;
        size_t steps = num_stance_phases[k];

        bool is_stance = standing_at_start[k];
        size_t step_counter = 0;
        size_t duration_idx = 0;
        // Always ends in stance phase
        while (step_counter < steps) {
            const size_t idx = duration_idx * ee_count + k;
            double phase_time = std::exp(xc[idx]);
            if (is_stance) {
                durations.push_back(phase_time);
                ++duration_idx;
                ++step_counter;
            }
            else {
                durations.push_back(phase_time);
                ++duration_idx;
            }
            is_stance = !is_stance;
        }

        formulation.params_.ee_phase_durations_.push_back(durations);

        formulation.params_.ee_in_contact_at_start_.push_back(true);
    }

    cem_trajopt::FixDurations(formulation.params_.ee_phase_durations_);

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

    auto solver = std::make_shared<ifopt::IpoptSolver>();
    solver->SetOption("jacobian_approximation", params::jacobian_approximation);
    // solver->SetOption("max_wall_time", params::max_wall_time);
    // solver->SetOption("max_iter", params::max_iter);
    // solver->SetOption("print_level", params::print_level);
    // solver->SetOption("nlp_scaling_method", params::nlp_scaling_method);
    // solver->SetOption("linear_solver", params::linear_solver);

    solver->Solve(nlp);

    std::string filename = "trajectory.txt";
    std::ofstream file(filename);
    if (!file.is_open()) {
        std::cerr << "failed to open file: " << filename << std::endl;
        return 1;
    }

    double dt = 0.05;
    double total_time = solution.base_linear_->GetTotalTime();
    size_t iters = total_time / dt + 1;
    double t = 0.;
    for (size_t i = 0; i < iters; ++i) {
        Eigen::Vector3d body_pos = solution.base_linear_->GetPoint(t).p();
        file << body_pos.transpose() << " ";
        Eigen::Vector3d body_rot = solution.base_angular_->GetPoint(t).p(); // Get body roll, pitch, yaw (XYZ)
        file << body_rot.transpose() << " ";

        for (size_t k = 0; k < ee_count; ++k) {
            Eigen::Vector3d foot_pos = solution.ee_motion_.at(k)->GetPoint(t).p();
            file << foot_pos.transpose() << " ";
        }

        t += dt;
        file << std::endl;
    }

    file.close();

    if (params::print_eval_wall_time)
        std::cout << "Wall Time: " << solver->GetTotalWallclockTime() << std::endl;

    return 0;
}
