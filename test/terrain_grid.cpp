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
