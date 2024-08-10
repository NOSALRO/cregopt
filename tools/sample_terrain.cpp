#include <algorithm>
#include <iostream>
#include <memory>
#include <random>

#include <cem_trajopt/terrain/terrain_grid.hpp>
#include <cem_trajopt/utils/utils.hpp>
#include <cem_trajopt/viz/viz.hpp>

namespace params {
    static constexpr unsigned int rows = 20;
    static constexpr unsigned int cols = 20;
    static constexpr double mu = 1.0;
    static constexpr double min_x = -5.;
    static constexpr double min_y = -5;
    static constexpr double max_x = 5.;
    static constexpr double max_y = 5.;

    static const std::string import_filename = "../tools/random_grid_20_20.csv";
    static const std::string export_filename = "../tools/sampled_random_grid_20_20.csv";

    static constexpr unsigned int num_x_samples = 100;
    static constexpr unsigned int num_y_samples = 100;

} // namespace params

int main()
{
    // Create terrain
    auto terrain = std::make_shared<cem_trajopt::TerrainGrid>(params::rows, params::cols, params::mu, params::min_x, params::min_y, params::max_x, params::max_y);
    terrain->ReadFromCsv(params::import_filename);

    std::vector<double> samples(params::num_x_samples * params::num_y_samples);

    double dx = static_cast<double>(params::max_x - params::min_x) / static_cast<double>(params::num_x_samples);
    double dy = static_cast<double>(params::max_y - params::min_y) / static_cast<double>(params::num_y_samples);
    double x = params::min_x;
    double y = params::min_y;
    for (size_t i = 0; i < params::num_y_samples; ++i) {
        // std::cout << "x: " << x << std::endl;
        // std::cout << "y: ";
        for (size_t j = 0; j < params::num_x_samples; ++j) {
            // std::cout << y << " ,";
            samples[i * params::num_x_samples + j] = terrain->GetHeight(x, y);
            x += dx;
        }
        // std::cout << std::endl;
        x = params::min_x;
        y += dy;
    }

    std::ofstream file(params::export_filename);

    if (!file.is_open()) {
        std::cerr << "Error opening file!" << std::endl;
        return 1;
    }

    for (size_t i = 0; i < samples.size(); ++i) {
        file << samples[i];
        if ((i + 1) % params::num_y_samples == 0) {
            file << "\n";
        }
        else {
            file << ",";
        }
    }

    file.close();

    return 0;
}
