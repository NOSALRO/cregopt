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
#include <algorithm>
#include <ctime>
#include <iostream>
#include <memory>
#include <random>

#include <cem_trajopt/terrain/terrain_grid.hpp>
#include <cem_trajopt/utils/utils.hpp>
#include <cem_trajopt/viz/viz.hpp>

namespace params {
    static constexpr unsigned int rows = 50; // x axis
    static constexpr unsigned int cols = 50; // y axis
    static constexpr double mu = 1.0;
    static constexpr double min_x = -10.;
    static constexpr double min_y = -10;
    static constexpr double max_x = 10.;
    static constexpr double max_y = 10.;

    static const std::string filename = "../tools/step.csv";

    static constexpr bool create_random = false;
    static constexpr double random_lower_bound = -0.3;
    static constexpr double random_upper_bound = 0.5;

} // namespace params

int main()
{
    // Create terrain
    auto terrain = std::make_shared<cem_trajopt::TerrainGrid>(params::rows, params::cols, params::mu, params::min_x, params::min_y, params::max_x, params::max_y);

    auto grid = cem_trajopt::TerrainGrid::Grid();
    grid.resize(params::rows * params::cols);

    if (params::create_random) {
        std::uniform_real_distribution<double> unif(params::random_lower_bound, params::random_upper_bound);
        std::default_random_engine re(std::time(0));

        std::generate(grid.begin(), grid.end(), [&]() { return unif(re); });
    }
    else {
        // Generate a step terrain
        for (size_t i = 0; i < params::rows; i++) {
            for (size_t j = 0; j < params::cols; ++j) {
                if (i < 35) {
                    grid[i * params::cols + j] = 0.;
                }
                else {
                    grid[i * params::cols + j] = 0.15;
                }
            }
        }
    }

    terrain->SetGrid(grid);
    std::cout << "Height at (0, 0) and (1, 1): " << terrain->GetHeight(0., 0.) << ", " << terrain->GetHeight(1., 1.) << std::endl;
    // std::cout << "Height at (5, 0) and (1, 2): " << terrain->GetHeight(0, 0.) << ", " << terrain->GetHeight(4., 1.) << std::endl;
    terrain->ExportToCsv(params::filename);

    return 0;
}
