#include <iostream>

#include <algevo/algo/cem_discrete.hpp>

#include <ifopt/composite.h>
#include <ifopt/ipopt_solver.h>
#include <ifopt/problem.h>

#include <cem_trajopt/terrain/terrain_grid.hpp>
#include <cem_trajopt/utils/utils.hpp>
#include <cem_trajopt/viz/viz.hpp>

int main()
{
    ifopt::Problem nlp;

    // Create terrain
    auto terrain = std::make_shared<towr::Gap>();

    Eigen::Vector3d init_body_pos = Eigen::Vector3d::Zero();
    Eigen::Vector3d target_body_pos = Eigen::Vector3d::Zero();
    init_body_pos << 0., 0., 0.5;
    target_body_pos << 2.5, 0., 0.5;

    size_t num_steps = 6;
    double swing_duration = 0.1;
    double stance_duration = 0.2;
    bool standing_at_start = true;
    bool optimise_time = true;
    auto formulation = cem_trajopt::CreateHopperNlpFormulation(terrain, init_body_pos, target_body_pos, num_steps, stance_duration, swing_duration, standing_at_start, optimise_time);

    towr::SplineHolder solution;
    for (auto c : formulation.GetVariableSets(solution))
        nlp.AddVariableSet(c);
    for (auto c : formulation.GetConstraints(solution))
        nlp.AddConstraintSet(c);
    for (auto c : formulation.GetCosts())
        nlp.AddCostSet(c);

    auto solver = std::make_shared<ifopt::IpoptSolver>();
    solver->SetOption("jacobian_approximation", "exact");
    solver->SetOption("max_wall_time", 12.);
    solver->SetOption("max_iter", 1000);
    solver->SetOption("print_level", 0);

    solver->Solve(nlp);
    std::cout << "Wall Time: " << solver->GetTotalWallclockTime() << std::endl;

    cem_trajopt::visualise(solution, 1, 0.01, "hopper.mp4");

    return 0;
}
