#|
#|    Copyright (c) 2024 Ioannis Tsikelis
#|    Copyright (c) 2024 Konstantinos Chatzilygeroudis
#|    Copyright (c) 2024 Laboratory of Automation and Robotics, University of Patras, Greece
#|    Copyright (c) 2024 Computational Intelligence Lab, University of Patras, Greece
#|
#|    Authors:  Ioannis Tsikelis, Konstantinos Chatzilygeroudis
#|    emails:    tsikelis.i@proton.me, costashatz@gmail.com
#|    website:  https://nosalro.github.io/cregopt
#|
#|    This file is part of cregopt.
#|
#|    All rights reserved.
#|
#|    Redistribution and use in source and binary forms, with or without
#|    modification, are permitted provided that the following conditions are met:
#|
#|    1. Redistributions of source code must retain the above copyright notice, this
#|       list of conditions and the following disclaimer.
#|
#|    2. Redistributions in binary form must reproduce the above copyright notice,
#|       this list of conditions and the following disclaimer in the documentation
#|       and/or other materials provided with the distribution.
#|
#|    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
#|    AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
#|    IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
#|    DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
#|    FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
#|    DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
#|    SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
#|    CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
#|    OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
#|    OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#|
cmake_minimum_required(VERSION 3.15 FATAL_ERROR)

project(cem_trajopt VERSION 1.0)

#Options
option(USE_TBB "Use TBB for parallel optimisation" ON)
option(USE_RobotDART "Use RobotDART for visualization" ON)

# Export compile commands for LSP.
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Find Eigen
find_package (Eigen3 3.3 REQUIRED NO_MODULE)

# Find ifopt
find_package(ifopt REQUIRED)

# Find Towr
find_package(towr 1.2 REQUIRED)

# Find algevo
if (USE_TBB)
    find_package(TBB REQUIRED)
endif()

# Include RobotDART with Mangum for graphics.
if (USE_RobotDART)
    find_package(RobotDART QUIET OPTIONAL_COMPONENTS Magnum)
endif()

################## 
# Create library #
##################
add_library(${PROJECT_NAME} 
    src/terrain_grid.cpp
    src/gap.cpp
    src/utils.cpp
    src/viz.cpp
)
target_include_directories(${PROJECT_NAME} PRIVATE ${PROJECT_SOURCE_DIR}/include)
target_link_libraries(${PROJECT_NAME} PUBLIC
  Eigen3::Eigen
  ifopt::ifopt_ipopt
  towr::towr
)
target_compile_options(${PROJECT_NAME} PUBLIC "-Wall;-Wextra;-O3;-std=c++20;-fPIC")

if (RobotDART_FOUND AND RobotDART_Magnum_FOUND)
  set_target_properties(RobotDART::Simu PROPERTIES INTERFACE_COMPILE_OPTIONS "")
  set_target_properties(RobotDART::Magnum PROPERTIES INTERFACE_COMPILE_OPTIONS "")

  target_compile_definitions(${PROJECT_NAME} PUBLIC "-DVIZ=1;SRCPATH=\"${CMAKE_CURRENT_SOURCE_DIR}\"")
  target_link_libraries(${PROJECT_NAME} PRIVATE 
    RobotDART::Simu
    RobotDART::Magnum
  )
endif()

if (TBB_FOUND)
    target_link_libraries(${PROJECT_NAME} PUBLIC
        TBB::tbb
    )
endif()

###################
# Create examples #
###################

add_subdirectory(examples)

################
# Create tests #
################

# Terrain test
add_executable(terrain_grid_test test/terrain_grid.cpp)
target_include_directories(terrain_grid_test PRIVATE ${PROJECT_SOURCE_DIR}/include)
target_link_libraries(terrain_grid_test PRIVATE
    ${PROJECT_NAME}
)

################
# Create tools #
################

# Generate terrain csv files
add_executable(gen_terrain tools/gen_terrain.cpp)
target_include_directories(gen_terrain PRIVATE ${PROJECT_SOURCE_DIR}/include)
target_link_libraries(gen_terrain PRIVATE
    ${PROJECT_NAME}
)

# Visualise terrain samples
add_executable(sample_terrain tools/sample_terrain.cpp)
target_include_directories(sample_terrain PRIVATE ${PROJECT_SOURCE_DIR}/include)
target_link_libraries(sample_terrain PRIVATE
    ${PROJECT_NAME}
)
