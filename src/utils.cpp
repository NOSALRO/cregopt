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
#include <cem_trajopt/utils/utils.hpp>

#include <fstream>
#include <iostream>
#include <numeric>

namespace cem_trajopt {
    using Eigen::Vector3d;

    towr::NlpFormulation CreateHopperNlpFormulation(towr::HeightMap::Ptr terrain, const Vector3d& init_base_pos, const Vector3d& target_base_pos, size_t numSteps, double stance_duration, double swing_duration, bool standing_at_start, bool optimiseTime)
    {
        towr::NlpFormulation formulation;

        formulation.terrain_ = terrain;
        formulation.model_ = towr::RobotModel(towr::RobotModel::Monoped);

        formulation.initial_base_.lin.at(towr::kPos) = init_base_pos;
        formulation.initial_ee_W_.push_back(Eigen::Vector3d::Zero());

        formulation.final_base_.lin.at(towr::kPos) = target_base_pos;

        if (optimiseTime) {
            formulation.params_.bound_phase_duration_ = std::make_pair(0.1, numSteps * 0.2 + (numSteps - 1) * 0.1);
            formulation.params_.OptimizePhaseDurations();
        }

        std::vector<double> durations;

        bool is_stance = standing_at_start;
        size_t step_counter = 0;
        while (step_counter < numSteps) {
            if (is_stance) {
                step_counter++;
                durations.push_back(stance_duration);
            }
            else {
                durations.push_back(swing_duration);
            }
            is_stance = !is_stance;
        }

        formulation.params_.ee_phase_durations_.push_back(durations);
        formulation.params_.ee_in_contact_at_start_.push_back(standing_at_start);

        return formulation;
    }

    towr::NlpFormulation CreateAnymalNlpFormulation(towr::HeightMap::Ptr terrain, const Vector3d& init_base_pos, const Vector3d& target_base_pos, Eigen::Matrix<unsigned int, 1, 4> num_steps, double stance_duration, double swing_duration, std::vector<bool> standing_at_start, bool optimise_time)
    {
        towr::NlpFormulation formulation;

        formulation.terrain_ = terrain;
        formulation.model_ = towr::RobotModel(towr::RobotModel::Anymal);

        formulation.initial_base_.lin.at(towr::kPos) = init_base_pos;
        formulation.final_base_.lin.at(towr::kPos) = target_base_pos;

        // formulation.final_base_.ang.at(towr::kPos).z() = M_PI;

        if (optimise_time) {
            // Find foot with maximum step number
            unsigned int max = 0.;
            for (auto& s : num_steps) {
                if (s > max) {
                    max = s;
                }
            }
            // Bound the phase duration according to the largest step count
            formulation.params_.bound_phase_duration_ = std::make_pair(0.1, max * 0.2 + (max - 1) * 0.1);
            formulation.params_.OptimizePhaseDurations();
        }

        cem_trajopt::AddAnymalInitialEePositions(formulation);

        // Maybe this should be set before optimize_time check
        for (size_t i = 0; i < 4; ++i) {
            std::vector<double> durations;
            size_t steps = num_steps[i];

            bool is_stance = standing_at_start[i];
            size_t step_counter = 0;
            while (step_counter < steps) {
                if (is_stance) {
                    step_counter++;
                    durations.push_back(stance_duration);
                }
                else {
                    durations.push_back(swing_duration);
                }
                is_stance = !is_stance;
            }

            formulation.params_.ee_phase_durations_.push_back(durations);

            formulation.params_.ee_in_contact_at_start_.push_back(true);
        }

        cem_trajopt::FixDurations(formulation.params_.ee_phase_durations_);

        return formulation;
    }

    void AddBipedInitialEePositions(towr::NlpFormulation& formulation)
    {
        for (size_t i = 0; i < 2; ++i) {
            double init_x = formulation.initial_base_.lin.at(towr::kPos).x();
            double init_y = formulation.initial_base_.lin.at(towr::kPos).y();
            double init_z = 0.;
            if (i == 0) {
                init_y += 0.20;
                init_z = formulation.terrain_->GetHeight(init_x, init_y);
            }
            else if (i == 1) {
                init_y -= 0.20;
                init_z = formulation.terrain_->GetHeight(init_x, init_y);
            }

            Eigen::Vector3d init_pos(init_x, init_y, init_z);
            formulation.initial_ee_W_.push_back(init_pos);
        }
    }

    void AddAnymalInitialEePositions(towr::NlpFormulation& formulation)
    {
        for (size_t i = 0; i < 4; ++i) {
            double init_x = formulation.initial_base_.lin.at(towr::kPos).x();
            double init_y = formulation.initial_base_.lin.at(towr::kPos).y();
            double init_z = 0.;
            if (i == 0 || i == 2) {
                if (i == 0)
                    init_x += 0.34;
                else
                    init_x -= 0.34;
                init_y += 0.19;

                init_z = formulation.terrain_->GetHeight(init_x, init_y);
            }
            else if (i == 1 || i == 3) {
                if (i == 1)
                    init_x += 0.34;
                else
                    init_x -= 0.34;
                init_y -= 0.19;

                init_z = formulation.terrain_->GetHeight(init_x, init_y);
            }

            Eigen::Vector3d init_pos(init_x, init_y, init_z);
            formulation.initial_ee_W_.push_back(init_pos);
        }
    }

    void AddHexapodInitialEePositions(towr::NlpFormulation& formulation)
    {
        for (size_t i = 0; i < 6; ++i) {
            double init_x = formulation.initial_base_.lin.at(towr::kPos).x();
            double init_y = formulation.initial_base_.lin.at(towr::kPos).y();
            double init_z = 0.;
            if (i == 0 || i == 2 || i == 4) {
                if (i == 0)
                    init_x += 0.11;
                else if (i == 2)
                    init_x += 0.;
                else
                    init_x -= 0.11;
                init_y += 0.185;

                init_z = formulation.terrain_->GetHeight(init_x, init_y);
            }
            else if (i == 1 || i == 3 || i == 5) {
                if (i == 1)
                    init_x += 0.11;
                else if (i == 3)
                    init_x += 0.0;
                else
                    init_x -= 0.11;
                init_y -= 0.185;

                init_z = formulation.terrain_->GetHeight(init_x, init_y);
            }

            Eigen::Vector3d init_pos(init_x, init_y, init_z);
            formulation.initial_ee_W_.push_back(init_pos);
        }
    }

    size_t GetNumViolations(const ifopt::Problem& nlp, double tol)
    {
        auto constraints = nlp.GetConstraints();
        size_t n = GetNumViolations(constraints, tol);
        auto vars = nlp.GetOptVariables();
        n += GetNumViolations(*vars, tol);
        return n;
    }

    size_t GetNumViolations(const ifopt::Composite& comp, double tol)
    {
        size_t n = 0;
        for (auto& c : comp.GetComponents()) {
            Eigen::VectorXd x = c->GetValues();
            ifopt::Component::VecBound bounds = c->GetBounds();
            for (std::size_t i = 0; i < bounds.size(); ++i) {
                double lower = bounds.at(i).lower_;
                double upper = bounds.at(i).upper_;
                double val = x(i);
                if (val < lower - tol || upper + tol < val)
                    n++;
            }
        }

        return n;
    }

    void FixDurations(std::vector<std::vector<double>>& phase_times)
    {
        // find largest duration
        double max = 0.;
        for (auto& vec : phase_times) {
            double duration = std::accumulate(vec.begin(), vec.end(), 0.);
            if (duration > max)
                max = duration;
        }

        for (auto& vec : phase_times) {
            double duration = std::accumulate(vec.begin(), vec.end(), 0.);
            vec.back() += max - duration;
        }
    }

    int FindClosestNode(towr::NodesVariables::Ptr var, double t, double t_total)
    {
        int num_nodes = var->GetNodes().size();

        double time_between_nodes = t_total / static_cast<double>(num_nodes - 1);
        double t_dist = time_between_nodes / 2.;
        t = t / t_total;

        // Initialise with final node id.
        int node_idx = var->GetNodeValuesInfo(var->GetRows() - 1).back().id_;

        bool found = false;
        for (int vidx = 0; vidx < var->GetRows(); ++vidx) {
            if (found)
                break;

            for (auto nvi : var->GetNodeValuesInfo(vidx)) {

                double node_time = nvi.id_ / static_cast<double>(num_nodes - 1); // time in which trajectory is at current node

                // std::cout << "Node id: " << nvi.id_ << " node time: " << node_time << " t:" << t << " t_total:" << t_total << " t_dist: " << t_dist << ", Distance: " << std::abs(node_time - t) << std::endl;
                if (std::abs(node_time - t) <= t_dist) {
                    node_idx = nvi.id_;
                    found = true;
                }
            }
        }
        // std::cout << "node idx:" << node_idx << std::endl;

        return node_idx;
    }

    void ExportTowrGaitTimesToCSV(const std::vector<std::vector<double>>& data, const std::string& filename)
    {
        std::ofstream file(filename);

        if (!file.is_open()) {
            std::cerr << "failed to open file: " << filename << std::endl;
            return;
        }

        for (const auto& row : data) {
            for (size_t i = 0; i < row.size(); ++i) {
                file << row[i];
                if (i < row.size() - 1) {
                    file << ",";
                }
            }
            file << "\n";
        }

        file.close();
    }

    std::vector<std::vector<double>> ImportTowrGaitTimesFromCSV(const std::string& filename)
    {
        std::vector<std::vector<double>> data;
        std::ifstream file(filename);

        if (!file.is_open()) {
            std::cerr << "Failed to open file: " << filename << std::endl;
            return data;
        }

        std::string line;
        while (std::getline(file, line)) {
            std::vector<double> row;
            std::stringstream line_stream(line);
            std::string cell;
            while (std::getline(line_stream, cell, ',')) {
                row.push_back(std::stod(cell));
            }
            data.push_back(row);
        }

        file.close();
        return data;
    }

} // namespace cem_trajopt
