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
