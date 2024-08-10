#include <cem_trajopt/terrain/terrain_grid.hpp>

#include <fstream>
#include <iostream>

namespace cem_trajopt {
    using Dim2D = towr::Dim2D;

    TerrainGrid::TerrainGrid(size_t rows, size_t cols, double mu, double min_x, double min_y, double max_x, double max_y)
        : towr::HeightMap(), rows_(rows), cols_(cols), min_x_(min_x), min_y_(min_y), max_x_(max_x), max_y_(max_y)
    {
        friction_coeff_ = mu;
        grid_.resize(rows_ * cols_);
    }

    double TerrainGrid::GetHeight(double x, double y) const
    {
        if (OutOfBounds(x, y))
            return 0.;

        double x_norm = std::max(0., std::min(static_cast<double>(rows_) - 1., rows_ * (x - min_x_) / (max_x_ - min_x_)));
        double y_norm = std::max(0., std::min(static_cast<double>(cols_) - 1., cols_ * (y - min_y_) / (max_y_ - min_y_)));

        // std::cout << "x_norm: " << x_norm << " y_norm: " << y_norm << std::endl;

        double height = 0.;

        size_t x1 = static_cast<size_t>(std::floor(x_norm));
        size_t x2 = static_cast<size_t>(std::ceil(x_norm));
        size_t y1 = static_cast<size_t>(std::floor(y_norm));
        size_t y2 = static_cast<size_t>(std::ceil(y_norm));

        double fx1y1 = grid_.at(x1 * cols_ + y1);
        double fx1y2 = grid_.at(x1 * cols_ + y2);
        double fx2y1 = grid_.at(x2 * cols_ + y1);
        double fx2y2 = grid_.at(x2 * cols_ + y2);

        double fxy1 = 0.; // Interpolate y1 in the x direction.
        double fxy2 = 0.; // Interpolate y2 in the x direction.

        if (x1 == x2) {
            fxy1 = fx1y1;
            fxy2 = fx1y2;
        }
        else {
            fxy1 = ((x2 - x_norm) / static_cast<double>(x2 - x1)) * fx1y1 + ((x_norm - x1) / static_cast<double>(x2 - x1)) * fx2y1;
            fxy2 = ((x2 - x_norm) / static_cast<double>(x2 - x1)) * fx1y2 + ((x_norm - x1) / static_cast<double>(x2 - x1)) * fx2y2;
        }

        if (y1 == y2) {
            height = fxy1;
        }
        else {
            height = ((y2 - y_norm) / (y2 - y1)) * fxy1 + ((y_norm - y1) / (y2 - y1)) * fxy2;
        }

        // Sanity check
        if (std::isnan(height)) {
            height = 0.;
        }

        // std::cout << "X1: " << x1 << ", Y1: " << y1 << ", X2: " << x2 << ", Y2: " << y2 << ", Height: " << height << std::endl;
        return height;
    }

    double TerrainGrid::GetHeightDerivWrtX(double x, double y) const
    {
        if (OutOfBounds(x, y))
            return 0.;

        double x_norm = std::max(0., std::min(static_cast<double>(rows_) - 1., rows_ * (x - min_x_) / (max_x_ - min_x_)));
        double y_norm = std::max(0., std::min(static_cast<double>(cols_) - 1., cols_ * (y - min_y_) / (max_y_ - min_y_)));

        // std::cout << "x_norm: " << x_norm << " y_norm: " << y_norm << std::endl;

        size_t x1 = static_cast<size_t>(std::floor(x_norm));
        size_t x2 = static_cast<size_t>(std::ceil(x_norm));
        size_t y1 = static_cast<size_t>(std::floor(y_norm));
        size_t y2 = static_cast<size_t>(std::ceil(y_norm));

        double fx1y1 = grid_.at(x1 * cols_ + y1);
        double fx1y2 = grid_.at(x1 * cols_ + y2);
        double fx2y1 = grid_.at(x2 * cols_ + y1);
        double fx2y2 = grid_.at(x2 * cols_ + y2);

        double dR1dx_norm = (-fx1y1 + fx2y1) / (x2 - x1);
        if (x1 == x2) {
            dR1dx_norm = 0.;
        }
        double dR2dx_norm = (-fx1y2 + fx2y2) / (x2 - x1);
        if (x1 == x2) {
            dR2dx_norm = 0.;
        }

        double dfdx_norm = (dR1dx_norm * (y2 - y_norm) + dR2dx_norm * (y_norm - y1)) / (y2 - y1);
        if (y1 == y2) {
            dfdx_norm = dR1dx_norm;
        }

        double dx_normdx = rows_ / (max_x_ - min_x_);

        return dfdx_norm * dx_normdx;
    }

    double TerrainGrid::GetHeightDerivWrtY(double x, double y) const
    {
        if (OutOfBounds(x, y))
            return 0.;

        double x_norm = std::max(0., std::min(static_cast<double>(rows_) - 1., rows_ * (x - min_x_) / (max_x_ - min_x_)));
        double y_norm = std::max(0., std::min(static_cast<double>(cols_) - 1., cols_ * (y - min_y_) / (max_y_ - min_y_)));

        // std::cout << "x_norm: " << x_norm << " y_norm: " << y_norm << std::endl;

        size_t x1 = static_cast<size_t>(std::floor(x_norm));
        size_t x2 = static_cast<size_t>(std::ceil(x_norm));
        size_t y1 = static_cast<size_t>(std::floor(y_norm));
        size_t y2 = static_cast<size_t>(std::ceil(y_norm));

        double fx1y1 = grid_.at(x1 * cols_ + y1);
        double fx1y2 = grid_.at(x1 * cols_ + y2);
        double fx2y1 = grid_.at(x2 * cols_ + y1);
        double fx2y2 = grid_.at(x2 * cols_ + y2);

        double R1 = (fx1y1 * (x2 - x_norm) + fx2y1 * (x_norm - x1)) / (x2 - x1);
        if (x1 == x2) {
            R1 = fx1y1;
        }
        double R2 = (fx1y2 * (x2 - x_norm) + fx2y2 * (x_norm - x1)) / (x2 - x1);
        if (x1 == x2) {
            R2 = fx1y2;
        }

        double dfdy_norm = (-R1 + R2) / (y2 - y1);
        if (y1 == y2) {
            dfdy_norm = 0.;
        }

        double dy_normdy = cols_ / (max_y_ - min_y_);

        return dfdy_norm * dy_normdy;
    }

    double TerrainGrid::GetHeightDerivWrtXX(double, double) const
    {
        return 0.;
    }

    double TerrainGrid::GetHeightDerivWrtYY(double, double) const
    {
        return 0.;
    }

    double TerrainGrid::GetHeightDerivWrtXY(double x, double y) const
    {
        if (OutOfBounds(x, y))
            return 0.;

        double x_norm = std::max(0., std::min(static_cast<double>(rows_) - 1., rows_ * (x - min_x_) / (max_x_ - min_x_)));
        double y_norm = std::max(0., std::min(static_cast<double>(cols_) - 1., cols_ * (y - min_y_) / (max_y_ - min_y_)));

        size_t x1 = static_cast<size_t>(std::floor(x_norm));
        size_t x2 = static_cast<size_t>(std::ceil(x_norm));
        size_t y1 = static_cast<size_t>(std::floor(y_norm));
        size_t y2 = static_cast<size_t>(std::ceil(y_norm));

        if (x1 == x2 || y1 == y2) {
            return 0.;
        }

        double fx1y1 = grid_.at(x1 * cols_ + y1);
        double fx1y2 = grid_.at(x1 * cols_ + y2);
        double fx2y1 = grid_.at(x2 * cols_ + y1);
        double fx2y2 = grid_.at(x2 * cols_ + y2);

        double dR1dx_norm = (-fx1y1 + fx2y1) / (x2 - x1);
        double dR2dx_norm = (-fx1y2 + fx2y2) / (x2 - x1);

        double dfdx_normy_norm = (-dR1dx_norm + dR2dx_norm) / (y2 - y1);

        double dy_normdy = cols_ / (max_y_ - min_y_);
        double dx_normdx = rows_ / (max_x_ - min_x_);

        return dfdx_normy_norm * dy_normdy * dx_normdx;
    }

    double TerrainGrid::GetHeightDerivWrtYX(double x, double y) const
    {
        if (OutOfBounds(x, y))
            return 0.;

        double x_norm = std::max(0., std::min(static_cast<double>(rows_) - 1., rows_ * (x - min_x_) / (max_x_ - min_x_)));
        double y_norm = std::max(0., std::min(static_cast<double>(cols_) - 1., cols_ * (y - min_y_) / (max_y_ - min_y_)));

        size_t x1 = static_cast<size_t>(std::floor(x_norm));
        size_t x2 = static_cast<size_t>(std::ceil(x_norm));
        size_t y1 = static_cast<size_t>(std::floor(y_norm));
        size_t y2 = static_cast<size_t>(std::ceil(y_norm));

        if (x1 == x2 || y1 == y2) {
            return 0.;
        }

        double fx1y1 = grid_.at(x1 * cols_ + y1);
        double fx1y2 = grid_.at(x1 * cols_ + y2);
        double fx2y1 = grid_.at(x2 * cols_ + y1);
        double fx2y2 = grid_.at(x2 * cols_ + y2);

        double dR1dx_norm = (-fx1y1 + fx2y1) / (x2 - x1);
        double dR2dx_norm = (-fx1y2 + fx2y2) / (x2 - x1);

        double dfdx_normy_norm = (-dR1dx_norm + dR2dx_norm) / (y2 - y1);

        double dx_normdx = rows_ / (max_x_ - min_x_);
        double dy_normdy = cols_ / (max_y_ - min_y_);

        return dfdx_normy_norm * dx_normdx * dy_normdy;
    }

    void TerrainGrid::SetZero()
    {
        for (auto& item : grid_) {
            item = 0.;
        }
    }

    bool TerrainGrid::OutOfBounds(double x, double y) const
    {
        // if (x <= min_x_ || x >= max_x_ || y <= min_y_ || y >= max_y_)
        //     std::cout << x << ", " << y << " out of bounds!" << std::endl;

        return (x <= min_x_ || x >= max_x_ || y <= min_y_ || y >= max_y_);
    }

    void TerrainGrid::ReadFromCsv(const std::string& filename)
    {
        std::ifstream file(filename);

        if (!file.is_open()) {
            std::cerr << "Failed to open csv file." << std::endl;
            return;
        }

        std::string line;
        for (size_t i = 0; i < rows_; ++i) {
            std::getline(file, line);
            auto tokens = Split(line, ',');

            grid_.insert(grid_.begin() + i * cols_, tokens.begin(), tokens.end());
        }

        file.close();
    }

    void TerrainGrid::ExportToCsv(const std::string& filename)
    {
        std::ofstream file(filename);

        if (!file.is_open()) {
            std::cerr << "Error opening file!" << std::endl;
            return;
        }

        for (size_t i = 0; i < grid_.size(); ++i) {
            file << grid_[i];
            if ((i + 1) % cols_ == 0) {
                file << "\n";
            }
            else {
                file << ",";
            }
        }

        file.close();
    }

    std::vector<double> TerrainGrid::Split(const std::string& s, char delimiter) const
    {
        std::vector<double> tokens;
        tokens.resize(cols_);
        std::string token;
        std::istringstream token_stream(s);
        for (auto& item : tokens) {
            std::getline(token_stream, token, delimiter);
            item = std::stod(token);
        }
        return tokens;
    }

} // namespace cem_trajopt
