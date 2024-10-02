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
