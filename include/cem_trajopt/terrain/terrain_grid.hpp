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
#pragma once

#include <cmath>
#include <string>
#include <vector>

#include <eigen3/Eigen/Dense>

#include <towr/terrain/height_map.h>

namespace cem_trajopt {
    class TerrainGrid : public towr::HeightMap {
    public:
        using Grid = std::vector<double>;

        TerrainGrid(size_t rows, size_t cols, double mu, double min_x, double min_y, double max_x, double max_y);

        double GetHeight(double x, double y) const override;

        void SetGrid(const Grid& new_grid) { grid_ = new_grid; }

        void SetZero();

        void ReadFromCsv(const std::string& filename);
        void ExportToCsv(const std::string& filename);

        // first derivatives that must be implemented by the user
        double GetHeightDerivWrtX(double x, double y) const override;
        double GetHeightDerivWrtY(double x, double y) const override;

        // second derivatives with respect to first letter, then second
        double GetHeightDerivWrtXX(double x, double y) const override;
        double GetHeightDerivWrtXY(double x, double y) const override;
        double GetHeightDerivWrtYX(double x, double y) const override;
        double GetHeightDerivWrtYY(double x, double y) const override;

    protected:
        bool OutOfBounds(double x, double y) const;

        std::vector<double> Split(const std::string& s, char delimiter) const;

    protected:
        const double eps_ = 1e-6;
        size_t rows_, cols_;
        double min_x_, min_y_, max_x_, max_y_;
        Grid grid_;
    };
} // namespace cem_trajopt
