#|
#|    Copyright (c) 2024 Ioannis Tsikelis
#|    Copyright (c) 2024 Konstantinos Chatzilygeroudis
#|    Copyright (c) 2024 Laboratory of Automation and Robotics, University of Patras, Greece
#|    Copyright (c) 2024 Computational Intelligence Lab, University of Patras, Greece
#|
#|    Authors:  Ioannis Tsikelis, Konstantinos Chatzilygeroudis
#|    emails:   tsikelis.i@protonmail.com, costashatz@gmail.com
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
################
# Experiment 1 #
################

# Experiment1/Scenario1/CreGOpt
add_executable(exp1_scen1_cregopt cregopt.cpp)
target_include_directories(exp1_scen1_cregopt PRIVATE ${PROJECT_SOURCE_DIR}/include)
target_link_libraries(exp1_scen1_cregopt PRIVATE
    ${PROJECT_NAME}
)
target_include_directories(exp1_scen1_cregopt PUBLIC /usr/local/include)
target_compile_options(exp1_scen1_cregopt PUBLIC -DUSE_HEURISTIC)
if (TBB_FOUND)
    target_compile_options(exp1_scen1_cregopt PUBLIC -DUSE_TBB -DUSE_TBB_ONEAPI)
endif()

# Experiment1/Scenario1/Towr
add_executable(exp1_scen1_towr towr_opt.cpp)
target_include_directories(exp1_scen1_towr PRIVATE ${PROJECT_SOURCE_DIR}/include)
target_link_libraries(exp1_scen1_towr PRIVATE
    ${PROJECT_NAME}
)
target_include_directories(exp1_scen1_towr PUBLIC /usr/local/include)

# Experiment1/Scenario2/CreGOpt
add_executable(exp1_scen2_cregopt cregopt.cpp)
target_include_directories(exp1_scen2_cregopt PRIVATE ${PROJECT_SOURCE_DIR}/include)
target_link_libraries(exp1_scen2_cregopt PRIVATE
    ${PROJECT_NAME}
)
target_include_directories(exp1_scen2_cregopt PUBLIC /usr/local/include)
target_compile_options(exp1_scen2_cregopt PUBLIC -DUSE_HEURISTIC -DROTATE)
if (TBB_FOUND)
    target_compile_options(exp1_scen2_cregopt PUBLIC -DUSE_TBB -DUSE_TBB_ONEAPI)
endif()

# Experiment1/Scenario2/Towr
add_executable(exp1_scen2_towr towr_opt.cpp)
target_include_directories(exp1_scen2_towr PRIVATE ${PROJECT_SOURCE_DIR}/include)
target_link_libraries(exp1_scen2_towr PRIVATE
    ${PROJECT_NAME}
)
target_include_directories(exp1_scen2_towr PUBLIC /usr/local/include)
target_compile_options(exp1_scen2_towr PUBLIC -DROTATE)

# Experiment1/Scenario3/Towr
add_executable(exp1_scen3_towr towr_opt.cpp)
target_include_directories(exp1_scen3_towr PRIVATE ${PROJECT_SOURCE_DIR}/include)
target_link_libraries(exp1_scen3_towr PRIVATE
    ${PROJECT_NAME}
)
target_include_directories(exp1_scen3_towr PUBLIC /usr/local/include)
target_compile_options(exp1_scen3_towr PUBLIC -DROTATE -DUSE_STEP_TERRAIN)

# Experiment1/Scenario3/CreGOpt
add_executable(exp1_scen3_cregopt cregopt.cpp)
target_include_directories(exp1_scen3_cregopt PRIVATE ${PROJECT_SOURCE_DIR}/include)
target_link_libraries(exp1_scen3_cregopt PRIVATE
    ${PROJECT_NAME}
)
target_include_directories(exp1_scen3_cregopt PUBLIC /usr/local/include)
target_compile_options(exp1_scen3_cregopt PUBLIC -DUSE_HEURISTIC -DROTATE -DUSE_STEP_TERRAIN)
if (TBB_FOUND)
    target_compile_options(exp1_scen3_cregopt PUBLIC -DUSE_TBB -DUSE_TBB_ONEAPI)
endif()

################
# Experiment 2 #
################

# Experiment2/Biped/CEM-MD
add_executable(exp2_biped_cem_md cregopt.cpp)
target_include_directories(exp2_biped_cem_md PRIVATE ${PROJECT_SOURCE_DIR}/include)
target_link_libraries(exp2_biped_cem_md PRIVATE
    ${PROJECT_NAME}
)
target_include_directories(exp2_biped_cem_md PUBLIC /usr/local/include)
target_compile_options(exp2_biped_cem_md PUBLIC -DBIPED)
if (TBB_FOUND)
    target_compile_options(exp2_biped_cem_md PUBLIC -DUSE_TBB -DUSE_TBB_ONEAPI)
endif()

# Experiment2/Biped/CrEGOpt
add_executable(exp2_biped_cregopt cregopt.cpp)
target_include_directories(exp2_biped_cregopt PRIVATE ${PROJECT_SOURCE_DIR}/include)
target_link_libraries(exp2_biped_cregopt PRIVATE
    ${PROJECT_NAME}
)
target_include_directories(exp2_biped_cregopt PUBLIC /usr/local/include)
target_compile_options(exp2_biped_cregopt PUBLIC -DBIPED -DUSE_HEURISTIC)
if (TBB_FOUND)
    target_compile_options(exp2_biped_cregopt PUBLIC -DUSE_TBB -DUSE_TBB_ONEAPI)
endif()

# Experiment2/Quad/CEM-MD
add_executable(exp2_quad_cem_md cregopt.cpp)
target_include_directories(exp2_quad_cem_md PRIVATE ${PROJECT_SOURCE_DIR}/include)
target_link_libraries(exp2_quad_cem_md PRIVATE
    ${PROJECT_NAME}
)
target_include_directories(exp2_quad_cem_md PUBLIC /usr/local/include)
if (TBB_FOUND)
    target_compile_options(exp2_quad_cem_md PUBLIC -DUSE_TBB -DUSE_TBB_ONEAPI)
endif()

# Experiment2/Quad/CrEGOpt
add_executable(exp2_quad_cregopt cregopt.cpp)
target_include_directories(exp2_quad_cregopt PRIVATE ${PROJECT_SOURCE_DIR}/include)
target_link_libraries(exp2_quad_cregopt PRIVATE
    ${PROJECT_NAME}
)
target_include_directories(exp2_quad_cregopt PUBLIC /usr/local/include)
target_compile_options(exp2_quad_cregopt PUBLIC -DUSE_HEURISTIC)
if (TBB_FOUND)
    target_compile_options(exp2_quad_cregopt PUBLIC -DUSE_TBB -DUSE_TBB_ONEAPI)
endif()

# Experiment2/Hex/CEM-MD
add_executable(exp2_hex_cem_md cregopt.cpp)
target_include_directories(exp2_hex_cem_md PRIVATE ${PROJECT_SOURCE_DIR}/include)
target_link_libraries(exp2_hex_cem_md PRIVATE
    ${PROJECT_NAME}
)
target_include_directories(exp2_hex_cem_md PUBLIC /usr/local/include)
target_compile_options(exp2_hex_cem_md PUBLIC -DHEXAPOD)
if (TBB_FOUND)
    target_compile_options(exp2_hex_cem_md PUBLIC -DUSE_TBB -DUSE_TBB_ONEAPI)
endif()

# Experiment2/Hex/CrEGOpt
add_executable(exp2_hex_cregopt cregopt.cpp)
target_include_directories(exp2_hex_cregopt PRIVATE ${PROJECT_SOURCE_DIR}/include)
target_link_libraries(exp2_hex_cregopt PRIVATE
    ${PROJECT_NAME}
)
target_include_directories(exp2_hex_cregopt PUBLIC /usr/local/include)
target_compile_options(exp2_hex_cregopt PUBLIC -DHEXAPOD -DUSE_HEURISTIC)
if (TBB_FOUND)
    target_compile_options(exp2_hex_cregopt PUBLIC -DUSE_TBB -DUSE_TBB_ONEAPI)
endif()

################
# Experiment 3 #
################

# Experiment3/MinStance/CEM-MD
add_executable(exp3_min_stance_cem_md cregopt.cpp)
target_include_directories(exp3_min_stance_cem_md PRIVATE ${PROJECT_SOURCE_DIR}/include)
target_link_libraries(exp3_min_stance_cem_md PRIVATE
    ${PROJECT_NAME}
)
target_include_directories(exp3_min_stance_cem_md PUBLIC /usr/local/include)
target_compile_options(exp3_min_stance_cem_md PUBLIC -DROTATE -DSTANCE_OPT)
if (TBB_FOUND)
    target_compile_options(exp3_min_stance_cem_md PUBLIC -DUSE_TBB -DUSE_TBB_ONEAPI)
endif()

# Experiment3/MinStance/CrEGOpt
add_executable(exp3_min_stance_cregopt cregopt.cpp)
target_include_directories(exp3_min_stance_cregopt PRIVATE ${PROJECT_SOURCE_DIR}/include)
target_link_libraries(exp3_min_stance_cregopt PRIVATE
    ${PROJECT_NAME}
)
target_include_directories(exp3_min_stance_cregopt PUBLIC /usr/local/include)
target_compile_options(exp3_min_stance_cregopt PUBLIC -DUSE_HEURISTIC -DROTATE -DSTANCE_OPT)
if (TBB_FOUND)
    target_compile_options(exp3_min_stance_cregopt PUBLIC -DUSE_TBB -DUSE_TBB_ONEAPI)
endif()

# Experiment3/MinForce/CEM-MD
add_executable(exp3_min_force_cem_md cregopt.cpp)
target_include_directories(exp3_min_force_cem_md PRIVATE ${PROJECT_SOURCE_DIR}/include)
target_link_libraries(exp3_min_force_cem_md PRIVATE
    ${PROJECT_NAME}
)
target_include_directories(exp3_min_force_cem_md PUBLIC /usr/local/include)
target_compile_options(exp3_min_force_cem_md PUBLIC -DROTATE -DFORCE_OPT)
if (TBB_FOUND)
    target_compile_options(exp3_min_force_cem_md PUBLIC -DUSE_TBB -DUSE_TBB_ONEAPI)
endif()

# Experiment3/MinStance/CrEGOpt
add_executable(exp3_min_force_cregopt cregopt.cpp)
target_include_directories(exp3_min_force_cregopt PRIVATE ${PROJECT_SOURCE_DIR}/include)
target_link_libraries(exp3_min_force_cregopt PRIVATE
    ${PROJECT_NAME}
)
target_include_directories(exp3_min_force_cregopt PUBLIC /usr/local/include)
target_compile_options(exp3_min_force_cregopt PUBLIC -DUSE_HEURISTIC -DROTATE -DFORCE_OPT)
if (TBB_FOUND)
    target_compile_options(exp3_min_force_cregopt PUBLIC -DUSE_TBB -DUSE_TBB_ONEAPI)
endif()

