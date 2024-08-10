#pragma once

#include <towr/terrain/height_map.h>

namespace cem_trajopt {
    class Gap : public towr::HeightMap {
    public:
        Gap();

        double GetHeight(double x, double y) const override;

        double GetHeightDerivWrtX(double x, double y) const override;

        double GetHeightDerivWrtXX(double x, double y) const override;

    private:
        double gap_start_{1.5};
        double w_{0.5};
        double h_{100.};

        const double slope_ = h_ / w_;
        const double dx = w_ / 2.0;
        const double xc = gap_start_ + dx; // gap center
        const double gap_end_x = gap_start_ + w_;

        // generated with matlab
        // see matlab/gap_height_map.m
        // coefficients of 2nd order polynomial
        // h = a*x^2 + b*x + c
        const double a = (4 * h_) / (w_ * w_);
        const double b = -(8 * h_ * xc) / (w_ * w_);
        const double c = -(h_ * (w_ - 2 * xc) * (w_ + 2 * xc)) / (w_ * w_);
    };
} // namespace cem_trajopt
