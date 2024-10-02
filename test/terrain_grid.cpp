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

#include <cem_trajopt/terrain/terrain_grid.hpp>
#include <cem_trajopt/utils/utils.hpp>
#include <cem_trajopt/viz/viz.hpp>

int main()
{
    // Create terrain
    size_t rows = 10;
    size_t cols = 10;
    double mu = 1.;
    double min_x = -5.;
    double min_y = -5.;
    double max_x = 5.;
    double max_y = 5.;
    auto terrain = std::make_shared<cem_trajopt::TerrainGrid>(rows, cols, mu, min_x, min_y, max_x, max_y);

    double lower_bound = -0.3;
    double upper_bound = 1.7;
    std::uniform_real_distribution<double> unif(lower_bound, upper_bound);
    std::default_random_engine re;

    auto grid = cem_trajopt::TerrainGrid::Grid();
    grid.resize(rows * cols);
    std::generate(grid.begin(), grid.end(), [&]() { return unif(re); });
    terrain->SetGrid(grid);

    double x = 1.2;
    double y = 3.9;

    double eps = 1e-6;
    double xp = x + eps;
    double xm = x - eps;
    double yp = y + eps;
    double ym = y - eps;

    double hxp = terrain->GetHeight(xp, y);
    double hxm = terrain->GetHeight(xm, y);
    double dxfdiff = (hxp - hxm) / (2 * eps);

    double hyp = terrain->GetHeight(x, yp);
    double hym = terrain->GetHeight(x, ym);
    double dyfdiff = (hyp - hym) / (2 * eps);

    double dx = terrain->GetDerivativeOfHeightWrt(towr::X_, x, y);
    double dy = terrain->GetDerivativeOfHeightWrt(towr::Y_, x, y);

    std::cout << "Height at: " << x << ", " << y << ": " << terrain->GetHeight(x, y) << std::endl;

    std::cout << "dx:" << std::endl;
    std::cout << "Actual: " << dx << ", Finite diff: " << dxfdiff << std::endl;
    std::cout << "Absolute difference: " << std::abs(dx - dxfdiff) << std::endl;

    std::cout << "dy:" << std::endl;
    std::cout << "Actual: " << dy << ", Finite diff: " << dyfdiff << std::endl;
    std::cout << "Absolute difference: " << std::abs(dy - dyfdiff) << std::endl;

    // dxx
    double dxx = terrain->GetHeightDerivWrtXX(x, y);

    double dxxp = terrain->GetDerivativeOfHeightWrt(towr::X_, xp, y);
    double dxxm = terrain->GetDerivativeOfHeightWrt(towr::X_, xm, y);
    double dxxfdiff = (dxxp - dxxm) / (2 * eps);

    std::cout << "dxx:" << std::endl;
    std::cout << "Actual: " << dxx << ", Finite diff: " << dxxfdiff << std::endl;
    std::cout << "Absolute difference: " << std::abs(dxx - dxxfdiff) << std::endl;

    // dyy
    double dyy = terrain->GetHeightDerivWrtYY(x, y);

    double dyyp = terrain->GetDerivativeOfHeightWrt(towr::Y_, x, yp);
    double dyym = terrain->GetDerivativeOfHeightWrt(towr::Y_, x, ym);
    double dyyfdiff = (dyyp - dyym) / (2 * eps);

    std::cout << "dyy:" << std::endl;
    std::cout << "Actual: " << dyy << ", Finite diff: " << dyyfdiff << std::endl;
    std::cout << "Absolute difference: " << std::abs(dyy - dyyfdiff) << std::endl;

    // dxy
    double dxy = terrain->GetHeightDerivWrtXY(x, y);

    double dxyp = terrain->GetDerivativeOfHeightWrt(towr::X_, x, yp);
    double dxym = terrain->GetDerivativeOfHeightWrt(towr::X_, x, ym);
    double dxyfdiff = (dxyp - dxym) / (2 * eps);

    std::cout << "dxy:" << std::endl;
    std::cout << "Actual: " << dxy << ", Finite diff: " << dxyfdiff << std::endl;
    std::cout << "Absolute difference: " << std::abs(dxy - dxyfdiff) << std::endl;

    // dyx
    double dyx = terrain->GetHeightDerivWrtYX(x, y);

    double dyxp = terrain->GetDerivativeOfHeightWrt(towr::Y_, xp, y);
    double dyxm = terrain->GetDerivativeOfHeightWrt(towr::Y_, xm, y);
    double dyxfdiff = (dyxp - dyxm) / (2 * eps);

    std::cout << "dyx:" << std::endl;
    std::cout << "Actual: " << dyx << ", Finite diff: " << dyxfdiff << std::endl;
    std::cout << "Absolute difference: " << std::abs(dyx - dyxfdiff) << std::endl;

    return 0;
}
