#include <cem_trajopt/terrain/gap.hpp>

#include <iostream>

namespace cem_trajopt {

    Gap::Gap() { friction_coeff_ = 1.; }

    double Gap::GetHeight(double x, double) const
    {
        double h = 0.0;

        // std::cout << "X: " << x << ", Gap Start: " << gap_start_ << ", Gap End: " << gap_end_x << std::endl;

        // modelled as parabola
        if (gap_start_ <= x && x <= gap_end_x)
            h = a * x * x + b * x + c;

        // std::cout << "Height: " << h << std::endl;
        return h;
    }

    double Gap::GetHeightDerivWrtX(double x, double) const
    {
        double dhdx = 0.0;

        if (gap_start_ <= x && x <= gap_end_x)
            dhdx = 2 * a * x + b;

        return dhdx;
    }

    double Gap::GetHeightDerivWrtXX(double x, double) const
    {
        double dzdxx = 0.0;

        if (gap_start_ <= x && x <= gap_end_x)
            dzdxx = 2 * a;

        return dzdxx;
    }
} // namespace cem_trajopt
