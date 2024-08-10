#include <iostream>

#include <algevo/algo/cem_md.hpp>

#include <ifopt/composite.h>
#include <ifopt/ipopt_solver.h>
#include <ifopt/problem.h>

#include <cem_trajopt/robots/pexod.hpp>

#include <cem_trajopt/terrain/gap.hpp>
#include <cem_trajopt/terrain/terrain_grid.hpp>

#include <cem_trajopt/utils/utils.hpp>
#include <cem_trajopt/viz/viz.hpp>

#if (defined(STANCE_OPT) || defined(FORCE_OPT))
#define OPT 1
#else
#define OPT 0
#endif // (defined(STANCE_OPT) || defined(TIME_OPT))

namespace params {

    static double _angle = 0.;
    static double angle() { return _angle; }
    static void set_angle(double v) { _angle = v; }

    //////////////////////////////////////////////////////////////////
    // Determine ee count and initial and final targets             //
    // For gait gen and scaling experiments: target (x, y) = (6, 0) //
    // For optimisation experiment: target (x, y) = (3, 3)          //
    // ///////////////////////////////////////////////////////////////
#if defined(BIPED)
    static constexpr unsigned int ee_count = 2;

    static const Eigen::Vector3d init_base_pos = Eigen::Vector3d(0., 0., 0.65);

#if (OPT)
    static const Eigen::Vector3d target_base_pos = Eigen::Vector3d(3., 3., 0.65);
#else
    static const Eigen::Vector3d target_base_pos = Eigen::Vector3d(6., 0., 0.65);
#endif // OPT

#elif defined(HEXAPOD)
    static constexpr unsigned int ee_count = 6;

    static const Eigen::Vector3d init_base_pos = Eigen::Vector3d(0., 0., 0.14);

#if (OPT)
    static const Eigen::Vector3d target_base_pos = Eigen::Vector3d(3., 3., 0.14);
#else
    static const Eigen::Vector3d target_base_pos = Eigen::Vector3d(6., 0., 0.14);
#endif // OPT

#else
    static constexpr unsigned int ee_count = 4;

    static const Eigen::Vector3d init_base_pos = Eigen::Vector3d(0., 0., 0.5);
#if (OPT)
    static const Eigen::Vector3d target_base_pos = Eigen::Vector3d(3., 3., 0.5);
#else
    static const Eigen::Vector3d target_base_pos = Eigen::Vector3d(6., 0., 0.5);
#endif // OPT

#endif

#if (OPT)
    static constexpr unsigned int min_stance_phases = 3;
    static constexpr unsigned int max_steps = 50; // max steps for algorithm to converge
#else
    static constexpr unsigned int min_stance_phases = 5;
    static constexpr unsigned int max_steps = 20; // max steps for algorithm to converge
#endif

#if defined(USE_HEURISTIC)
    static constexpr unsigned int dim_discrete = ee_count + 1; // equal to the number of end-effectors (+1 for indicator in heuristic)

    static constexpr unsigned int num_triplets = 3; // HEURISTIC SPECIFIC: num of triplets
    static const std::vector<unsigned int> triplet_start = {2, 4, 6}; // triplet start
    static constexpr unsigned int max_triplet_value = 6 + 2; // triplet_start.back() + 2

    static constexpr unsigned int num_values = 3; // number of total steps will thus be = num_values + min_stance_phases.
    static constexpr unsigned int dim_continuous = ee_count * (2 * ((max_triplet_value + 1) + min_stance_phases - 1) - 1); // set to the maximum possible size of steps
#else
    static constexpr unsigned int dim_discrete = ee_count; // equal to the number of end-effectors
    static constexpr unsigned int num_values = 8; // number of total steps will thus be = num_values + min_stance_phases
    static constexpr unsigned int dim_continuous = ee_count * (2 * (num_values + min_stance_phases - 1) - 1); // set to the maximum possible size of steps
#endif

    static constexpr double violation_penalty = 100.; // constraint violation penalty on evaluation

    static constexpr unsigned int pop_size = 16;
    static constexpr unsigned int num_elites = pop_size * 0.8;
    static constexpr double min_prob = 0.05; // Enforce a minimum probability

    static constexpr double max_value_continuous = -1.; // maximum value returned from continuous CEM
    static constexpr double min_value_continuous = -2.5; // maximum value returned from continuous CEM
    static constexpr double init_mu_continuous = -1.75; // initial mean for continuous CEM
    static constexpr double init_std_continuous = 1.; // initial standard deviation for continuous CEM
    static constexpr double min_std_continuous = 1e-3; // minimum standard deviation for continuous CEM

    // Towr parameters
    static constexpr int ee_polynomials_per_swing_phase = 2;

    // IPOPT parameters
    static constexpr int max_iter = 50;
    static constexpr int print_level = 0;
    static constexpr double max_wall_time = 5.;
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
    static const std::string grid_csv_filename = "./../../tools/step.csv";
#endif

    // Misc
    static constexpr bool print_eval_wall_time = false; // Print wall time elapsed in towr optimisation.
    static constexpr bool export_to_csv = true; // Save cregopt solution to csv file
    static const std::string results_filename = "cregopt_results.txt";
    static const std::string discrete_csv_filename = "cregopt_best_discrete.csv";
    static const std::string continuous_csv_filename = "cregopt_best_continuous.csv";

} // namespace params

template <typename Scalar = double>
struct FitMixed {

    using xd_t = Eigen::Matrix<unsigned int, 1, params::dim_discrete>;
    using xc_t = Eigen::Matrix<double, 1, params::dim_continuous>;

    FitMixed()
    {
        terrain_ = std::make_shared<cem_trajopt::TerrainGrid>(params::rows, params::cols, params::friction_coeff, params::min_x, params::min_y, params::max_x, params::max_y);
#if defined(USE_STEP_TERRAIN)
        terrain_->ReadFromCsv(params::grid_csv_filename);
#else
        terrain_->SetZero();
#endif
    }

#if defined(USE_HEURISTIC)
    static Eigen::Matrix<unsigned int, 1, params::ee_count> compute_num_stances(const xd_t& xd)
    {
        unsigned int offset = params::triplet_start[xd[0]];
        Eigen::Matrix<unsigned int, 1, params::ee_count> num_stance_phases = Eigen::Matrix<unsigned int, 1, params::ee_count>::Zero();
        for (size_t i = 0; i < params::ee_count; ++i) {
            num_stance_phases[i] = xd[i + 1] + params::min_stance_phases + offset;
        }
        return num_stance_phases;
    }
#else
    static Eigen::Matrix<unsigned int, 1, params::ee_count> compute_num_stances(const xd_t& xd)
    {
        Eigen::Matrix<unsigned int, 1, params::ee_count> num_stance_phases = xd.array() + params::min_stance_phases;
        return num_stance_phases;
    }
#endif

    // xd: discrete num of steps for each foot
    // xc: continuous phase durations for each phase of each foot
    Scalar eval(const xd_t& xd, const xc_t& xc)
    {
        ifopt::Problem nlp;
        towr::NlpFormulation formulation;

        Eigen::Matrix<unsigned int, 1, params::ee_count> num_stance_phases = compute_num_stances(xd);
        std::vector<bool> standing_at_start(params::ee_count, true);

        formulation.terrain_ = terrain_;

        formulation.initial_base_.lin.at(towr::kPos) = params::init_base_pos;

        const double angle = params::angle();
        formulation.final_base_.lin.at(towr::kPos) = Eigen::Vector3d(std::cos(angle) * params::target_base_pos[0], std::sin(angle) * params::target_base_pos[0], params::target_base_pos[2]);
        formulation.final_base_.lin.at(towr::kPos)[2] += terrain_->GetHeight(formulation.final_base_.lin.at(towr::kPos)[0], formulation.final_base_.lin.at(towr::kPos)[1]);

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

        solution_ = towr::SplineHolder();
        for (auto c : formulation.GetVariableSets(solution_))
            nlp.AddVariableSet(c);
        for (auto c : formulation.GetConstraints(solution_))
            nlp.AddConstraintSet(c);
        for (auto c : formulation.GetCosts())
            nlp.AddCostSet(c);

        auto solver = std::make_shared<ifopt::IpoptSolver>();
        solver->SetOption("jacobian_approximation", params::jacobian_approximation);
        solver->SetOption("max_wall_time", params::max_wall_time);
        solver->SetOption("max_cpu_time", 1e50);
        solver->SetOption("max_iter", params::max_iter);
        solver->SetOption("print_level", params::print_level);
        solver->SetOption("nlp_scaling_method", params::nlp_scaling_method);
        solver->SetOption("linear_solver", params::linear_solver);

        solver->Solve(nlp);

        if (params::print_eval_wall_time)
            std::cout << " Evaluation wall time: " << solver->GetTotalWallclockTime() << std::endl;

#if defined(FORCE_OPT)
        double sum = 0.;
        size_t cnt = 0;
        for (size_t k = 0; k < params::ee_count; ++k) {
            auto var = std::static_pointer_cast<towr::NodesVariables>(nlp.GetOptVariables()->GetComponent("ee-force_" + std::to_string(k)));
            for (auto node : var->GetNodes()) {
                sum += std::abs(node.p()[2]);
                cnt++;
            }
        }
        double force_cost = sum / static_cast<double>(cnt);

        double cost = static_cast<double>(force_cost + params::violation_penalty * static_cast<double>(cem_trajopt::GetNumViolations(nlp)));
#else
        double cost = static_cast<double>(num_stance_phases.sum()) + params::violation_penalty * static_cast<double>(cem_trajopt::GetNumViolations(nlp));
#endif

        return -cost;
    }

public:
    std::shared_ptr<cem_trajopt::TerrainGrid> terrain_;
    towr::SplineHolder solution_;
};

// Typedefs
using FitD = FitMixed<>;
using Algo = algevo::algo::CrossEntropyMethodMixed<FitD>;

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

    params::set_angle(angle);

    // algevo::tools::parallel_init(1);
    Algo::Params params;
    params.dim_discrete = params::dim_discrete;
    params.dim_continuous = params::dim_continuous;
    params.pop_size = params::pop_size;
    params.num_elites = params::num_elites;
    params.num_values = std::vector(params::dim_discrete, params::num_values);

#if defined(USE_HEURISTIC)
    params.num_values.at(0) = params::num_triplets;
    params.init_probs = Algo::p_t::Ones(params::dim_discrete, params::num_values) / static_cast<double>(params::num_values);
    // params.init_probs(0, 0) = 1. / static_cast<double>(params::num_triplets);
    for (unsigned int i = 0; i < params::num_triplets; i++)
        params.init_probs(0, i) = 1. / static_cast<double>(params::num_triplets);
#else
    params.init_probs = Algo::p_t::Ones(params::dim_discrete, params::num_values) / static_cast<double>(params::num_values);
#endif

    params.min_prob = params::min_prob;

    params.max_value_continuous = Algo::xc_t::Constant(params.dim_continuous, params::max_value_continuous);
    params.min_value_continuous = Algo::xc_t::Constant(params.dim_continuous, params::min_value_continuous);
    params.init_mu_continuous = Algo::xc_t::Constant(params.dim_continuous, params::init_mu_continuous);
    params.init_std_continuous = Algo::xc_t::Constant(params.dim_continuous, params::init_std_continuous);
    params.min_std_continuous = Algo::xc_t::Constant(params.dim_continuous, params::min_std_continuous);

    Algo cem(params);

    std::ofstream results_file(params::results_filename);
    if (!results_file.is_open()) {
        std::cerr << "Error opening file!" << std::endl;
        return 1;
    }

    std::ofstream best_discrete_file(params::discrete_csv_filename);
    if (!best_discrete_file.is_open()) {
        std::cerr << "Error opening file!" << std::endl;
        return 1;
    }

    std::ofstream best_continuous_file(params::continuous_csv_filename);
    if (!best_continuous_file.is_open()) {
        std::cerr << "Error opening file!" << std::endl;
        return 1;
    }

    const auto t_start = std::chrono::high_resolution_clock::now();
#if !(OPT)
    bool success = false;
#endif
    unsigned int i = 0;
    for (; i < params::max_steps; ++i) {
        auto log = cem.step();

        std::cout << log.iterations << "(" << log.func_evals << "): " << log.best_value << std::endl;

#if (OPT)
        if (params::export_to_csv) {
            results_file << log.best_value << "\n";
            best_discrete_file << log.best_discrete.transpose() << "\n";
            best_continuous_file << log.best_continuous.transpose() << "\n";
        }
#else
        if (static_cast<int>(FitD::compute_num_stances(log.best_discrete).sum()) == -static_cast<int>(log.best_value)) {
            success = true;
            break;
        }
#endif //(OPT)
    }

    const auto t_end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration<double, std::milli>(t_end - t_start);
    if (params::print_eval_wall_time)
        std::cout << "Wall clock time: " << duration.count() / 1000. << " seconds." << std::endl;

    // Export total wall time, success and total iterations at the end of the results file
    if (params::export_to_csv) {
        results_file << duration.count() / 1000. << "\n";
#if !(OPT)
        results_file << success << "\n";
        results_file << (i + 1) << "\n";
        best_discrete_file << cem.best_discrete().transpose() << "\n";
        best_continuous_file << cem.best_continuous().transpose() << "\n";
#endif
    }

    results_file.close();
    best_discrete_file.close();
    best_continuous_file.close();

    return 0;
}
