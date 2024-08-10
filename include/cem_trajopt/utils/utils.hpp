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
#pragma once

#include <fstream>
#include <iostream>

#include <ifopt/composite.h>
#include <ifopt/problem.h>

#include <towr/nlp_formulation.h>
#include <towr/terrain/examples/height_map_examples.h>

namespace cem_trajopt {
    using Eigen::Vector3d;
    towr::NlpFormulation CreateHopperNlpFormulation(towr::HeightMap::Ptr terrain, const Vector3d& init_base_pos, const Vector3d& target_base_pos, size_t numSteps, double stanceDuration, double swingDuration, bool standingAtStart, bool optimiseTime = false);
    towr::NlpFormulation CreateAnymalNlpFormulation(towr::HeightMap::Ptr terrain, const Vector3d& init_base_pos, const Vector3d& target_base_pos, Eigen::Matrix<unsigned int, 1, 4> numSteps, double stanceDuration, double swingDuration, std::vector<bool> standingAtStart, bool optimiseTime = false);

    void AddBipedInitialEePositions(towr::NlpFormulation& formulation);
    void AddAnymalInitialEePositions(towr::NlpFormulation& formulation);
    void AddHexapodInitialEePositions(towr::NlpFormulation& formulation);

    size_t GetNumViolations(const ifopt::Composite& comp, double tol = 1e-4);
    size_t GetNumViolations(const ifopt::Problem& nlp, double tol = 1e-4);
    void FixDurations(std::vector<std::vector<double>>& phaseTimes);

    int FindClosestNode(towr::NodesVariables::Ptr var, double t, double t_total);

    template <typename Vector>
    void ExportEigenVectorToCsv(const Vector& vec, const std::string& filename)
    {
        std::ofstream file(filename);

        if (!file.is_open()) {
            std::cerr << "failed to open file: " << filename << std::endl;
            return;
        }

        for (int i = 0; i < vec.rows(); ++i) {
            file << vec[i];
            if (i < vec.rows() - 1) {
                file << ",";
            }
        }

        file.close();
    }

    template <typename T>
    std::vector<T> ImportCemDataFromCSV(const std::string& filename)
    {
        std::vector<T> data;
        std::ifstream file(filename);

        if (!file.is_open()) {
            std::cerr << "Failed to open file: " << filename << std::endl;
            return data;
        }

        std::string line;
        std::getline(file, line);

        std::stringstream line_stream(line);
        std::string cell;
        while (std::getline(line_stream, cell, ',')) {
            data.push_back(std::stod(cell));
        }

        file.close();
        return data;
    }
    void ExportTowrGaitTimesToCSV(const std::vector<std::vector<double>>& data, const std::string& filename);
    std::vector<std::vector<double>> ImportTowrGaitTimesFromCSV(const std::string& filename);
} // namespace cem_trajopt
