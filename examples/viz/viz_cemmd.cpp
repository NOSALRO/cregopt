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

    // CEM parameters
    static constexpr unsigned int ee_count = 2;
    static constexpr unsigned int dim_discrete = ee_count + 1; // equal to the number of end-effectors (+1 for indicator in heuristic)

    static constexpr unsigned int min_stance_phases = 5;
    static constexpr unsigned int num_values = 3; // number of total steps will thus be = num_values + min_stance_phases

    static constexpr unsigned int num_triplets = 3; // HEURISTIC SPECIFIC: num of triplets
    static const std::vector<unsigned int> triplet_start = {0, 2, 4}; // triplet start
    static constexpr unsigned int max_triplet_value = 4 + 2; // triplet_start.back() + 2

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
    static constexpr int ee_polynomials_per_swing_phase = 1;
    static constexpr double min_phase_duration = 0.08;
    static constexpr double max_phase_duration = 0.36;
    static constexpr double stance_duration = 0.2;
    static constexpr double swing_duration = 0.1;
    static constexpr bool optimise_time = false;

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
    static const std::string discrete_csv_filename = "../cem_md_heuristic_best_discrete.csv";
    static const std::string continuous_csv_filename = "../cem_md_heuristic_best_continuous.csv";

    // Viz
    static constexpr bool save_video = true;
    static const std::string video_name = "biped_heuristic.mp4";

} // namespace params

int main()
{
    ifopt::Problem nlp;
    towr::NlpFormulation formulation;

    // Create terrain
    auto terrain = std::make_shared<cem_trajopt::TerrainGrid>(params::rows, params::cols, params::friction_coeff, params::min_x, params::min_y, params::max_x, params::max_y);
    if (params::read_from_csv)
        terrain->ReadFromCsv(params::grid_csv_filename);
    else
        terrain->SetZero();

    std::vector<unsigned int> xd = cem_trajopt::ImportCemDataFromCSV<unsigned int>(params::discrete_csv_filename);
    std::vector<double> xc = cem_trajopt::ImportCemDataFromCSV<double>(params::continuous_csv_filename);

    std::vector<unsigned int> num_stance_phases(params::ee_count);

    if (params::dim_discrete > params::ee_count) {
        // If using heuristic
        unsigned int offset = params::triplet_start.at(xd.at(0));
        for (size_t i = 0; i < params::ee_count; ++i) {
            num_stance_phases.at(i) = xd.at(i + 1) + params::min_stance_phases + offset;
        }
    }
    else {
        // If using simple cemmd
        for (auto& item : num_stance_phases) {
            item = item + params::min_stance_phases;
        }
    }

    std::vector<bool> standing_at_start(params::ee_count, true);

    formulation.terrain_ = terrain;

    formulation.initial_base_.lin.at(towr::kPos) = params::init_base_pos;
    formulation.final_base_.lin.at(towr::kPos) = params::target_base_pos;

    // formulation.final_base_.ang.at(towr::kPos).z() = M_PI;

    switch (params::ee_count) {
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

    for (size_t k = 0; k < params::ee_count; ++k) {
        std::vector<double> durations;
        size_t steps = num_stance_phases[k];

        bool is_stance = standing_at_start[k];
        size_t step_counter = 0;
        size_t duration_idx = 0;
        // Always ends in stance phase
        while (step_counter < steps) {
            const size_t idx = duration_idx * params::ee_count + k;
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

    // double total_time = std::accumulate(formulation.params_.ee_phase_durations_[0].begin(), formulation.params_.ee_phase_durations_[0].end(), 0.);
    // formulation.params_.dt_constraint_dynamic_ = 0.1;
    // formulation.params_.dt_constraint_range_of_motion_ = 0.1;
    // formulation.params_.dt_constraint_base_motion_ = 0.05;
    // formulation.params_.duration_base_polynomial_ = 0.1;

    auto solution = towr::SplineHolder();
    for (auto c : formulation.GetVariableSets(solution))
        nlp.AddVariableSet(c);
    for (auto c : formulation.GetConstraints(solution))
        nlp.AddConstraintSet(c);
    for (auto c : formulation.GetCosts())
        nlp.AddCostSet(c);

    // nlp.PrintCurrent();

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
    // solver->SetOption("max_wall_time", params::max_wall_time);
    // solver->SetOption("max_iter", params::max_iter);
    // solver->SetOption("print_level", params::print_level);
    // solver->SetOption("nlp_scaling_method", params::nlp_scaling_method);
    // solver->SetOption("linear_solver", params::linear_solver);

    solver->Solve(nlp);

    if (params::print_eval_wall_time)
        std::cout << "Wall Time: " << solver->GetTotalWallclockTime() << std::endl;

    std::cout << "dt: " << formulation.params_.dt_constraint_range_of_motion_ << std::endl;

    if (params::save_video)
        cem_trajopt::visualise(solution, params::ee_count, 0.05, params::video_name);
    else
        cem_trajopt::visualise(solution, formulation.params_.dt_constraint_dynamic_);

    return 0;
}
