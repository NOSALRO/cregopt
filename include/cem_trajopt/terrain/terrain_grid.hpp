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
