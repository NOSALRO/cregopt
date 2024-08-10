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
