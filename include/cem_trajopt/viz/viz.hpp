#pragma once

#include <ifopt/composite.h>
#include <ifopt/problem.h>

#include <towr/nlp_formulation.h>
#include <towr/terrain/examples/height_map_examples.h>

namespace cem_trajopt {
    void visualise(const towr::SplineHolder& solution, unsigned int ee_count, double dt = 0.01, const std::string vidName = "");
} // namespace cem_trajopt
