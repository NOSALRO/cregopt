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
#include <iostream>

#include <algevo/algo/cem_discrete.hpp>

#include <ifopt/composite.h>
#include <ifopt/ipopt_solver.h>
#include <ifopt/problem.h>

#include <cem_trajopt/terrain/gap.hpp>
#include <cem_trajopt/terrain/terrain_grid.hpp>
#include <cem_trajopt/utils/utils.hpp>
#include <cem_trajopt/viz/viz.hpp>

// Experiment parameters
namespace params {
    static const Eigen::Vector3d init_base_pos = Eigen::Vector3d(0., 0., 0.5);
    static const Eigen::Vector3d target_base_pos = Eigen::Vector3d(1., 0., 0.5);

    // CEM parameters
    static constexpr unsigned int dim_discrete = 5; // equal to the number of end-effectors (+1 for indicator in heuristic)

    static constexpr unsigned int min_stance_phases = 5;
    static constexpr unsigned int num_values = 3; // number of total steps will thus be = num_values + min_stance_phases

    static constexpr unsigned int num_triplets = 3; // HEURISTIC SPECIFIC: num of triplets
    static const std::vector<unsigned int> triplet_start = {0, 2, 4}; // triplet start
    static constexpr unsigned int max_triplet_value = 4 + 2; // triplet_start.back() + 2

    static constexpr unsigned int ee_count = 4;
    static constexpr double violation_penalty = 100.; // constraint violation penalty on evaluation

    static constexpr unsigned int dim_continuous = ee_count * (2 * ((max_triplet_value + 1) + min_stance_phases - 1) - 1); // set to the maximum possible size of steps

    static constexpr unsigned int pop_size = 16;
    static constexpr unsigned int num_elites = pop_size * 0.8;
    static constexpr double min_prob = 0.05; // Enforce a minimum probability

    static constexpr double max_value_continuous = -1.; // maximum value returned from continuous CEM
    static constexpr double min_value_continuous = -2.5; // maximum value returned from continuous CEM
    static constexpr double init_mu_continuous = -1.75; // initial mean for continuous CEM
    static constexpr double init_std_continuous = 1.; // initial standard deviation for continuous CEM
    static constexpr double min_std_continuous = 1e-3; // minimum standard deviation for continuous CEM

    static constexpr unsigned int max_steps = 50; // max steps for algorithm to converge

    // Towr parameters
    static constexpr double min_phase_duration = 0.08;
    static constexpr double max_phase_duration = 0.36;
    static constexpr double stance_duration = 0.2;
    static constexpr double swing_duration = 0.1;
    static constexpr bool optimise_time = true;

    // IPOPT parameters
    static constexpr int max_iter = 50;
    static constexpr int print_level = 0;
    static constexpr double max_wall_time = 2.;
    static const std::string jacobian_approximation = "exact";
    static const std::string nlp_scaling_method = "gradient-based"; // none
    static const std::string linear_solver = "ma97"; // ma27

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
    static constexpr bool export_to_csv = false; // Save cem solution to csv file.
    static const std::string discrete_csv_filename = "cem_md_heuristic_best_discrete.csv";
    static const std::string continuous_csv_filename = "cem_md_heuristic_best_continuous.csv";

} // namespace params

int main()
{
    ifopt::Problem nlp;

    // Create terrain
    auto terrain = std::make_shared<cem_trajopt::TerrainGrid>(params::rows, params::cols, params::friction_coeff, params::min_x, params::min_y, params::max_x, params::max_y);
    terrain->ReadFromCsv(params::grid_csv_filename);

    std::vector<unsigned int> num_steps = cem_trajopt::ImportCemDataFromCSV<unsigned int>(params::discrete_csv_filename);

    for (auto& item : num_steps) {
        item = item + params::min_stance_phases;
    }

    std::vector<bool> standing_at_start(params::ee_count, true);

    towr::NlpFormulation formulation;

    formulation.terrain_ = terrain;
    formulation.model_ = towr::RobotModel(towr::RobotModel::Anymal);

    formulation.initial_base_.lin.at(towr::kPos) = params::init_base_pos;
    formulation.final_base_.lin.at(towr::kPos) = params::target_base_pos;

    // formulation.final_base_.ang.at(towr::kPos).z() = M_PI;

    if (params::optimise_time) {
        // Find foot with maximum step number
        unsigned int max = 0.;
        for (auto& s : num_steps) {
            if (s > max) {
                max = s;
            }
        }
        // Bound the phase duration according to the largest step count
        formulation.params_.bound_phase_duration_ = std::make_pair(params::min_phase_duration, params::max_phase_duration);
        formulation.params_.OptimizePhaseDurations();
    }

    cem_trajopt::AddAnymalInitialEePositions(formulation);

    for (size_t i = 0; i < params::ee_count; ++i) {
        std::vector<double> durations;
        size_t steps = num_steps[i];

        bool is_stance = standing_at_start[i];
        size_t step_counter = 0;
        while (step_counter < steps) {
            if (is_stance) {
                step_counter++;
                durations.push_back(params::max_phase_duration);
            }
            else {
                durations.push_back(params::max_phase_duration);
            }
            is_stance = !is_stance;
        }

        formulation.params_.ee_phase_durations_.push_back(durations);

        formulation.params_.ee_in_contact_at_start_.push_back(true);
    }

    cem_trajopt::FixDurations(formulation.params_.ee_phase_durations_);

    auto solution = towr::SplineHolder();
    for (auto c : formulation.GetVariableSets(solution))
        nlp.AddVariableSet(c);
    for (auto c : formulation.GetConstraints(solution))
        nlp.AddConstraintSet(c);
    for (auto c : formulation.GetCosts())
        nlp.AddCostSet(c);

    nlp.PrintCurrent();

    // // Set middle body node at 0.7.
    // double t_total = solution.base_linear_->GetTotalTime();
    // auto var = std::static_pointer_cast<towr::NodesVariables>(nlp.GetOptVariables()->GetComponent("base-lin"));
    // int mid_node_idx = cem_trajopt::FindClosestNode(var, t_total / 2., t_total);
    //
    // Eigen::VectorXd bounds(3);
    // bounds << 1.75, 0., 0.8;
    //
    // var->AddBounds(mid_node_idx, towr::kPos, {towr::X, towr::Y, towr::Z}, bounds);
    // ///////////

    auto solver = std::make_shared<ifopt::IpoptSolver>();
    solver->SetOption("jacobian_approximation", params::jacobian_approximation);
    solver->SetOption("max_cpu_time", params::max_wall_time);
    solver->SetOption("max_iter", params::max_iter);
    solver->SetOption("print_level", params::print_level);

    solver->Solve(nlp);

    if (params::print_eval_wall_time)
        std::cout << "Wall Time: " << solver->GetTotalWallclockTime() << std::endl;

    cem_trajopt::visualise(solution, formulation.params_.dt_constraint_dynamic_, "anymal.mp4");

    return 0;
}
