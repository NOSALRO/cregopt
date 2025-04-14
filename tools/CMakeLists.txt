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

# Generate terrain csv files
add_executable(gen_terrain gen_terrain.cpp)
target_include_directories(gen_terrain PRIVATE ${PROJECT_SOURCE_DIR}/include)
target_link_libraries(gen_terrain PRIVATE
    ${PROJECT_NAME}
)

# Visualise terrain samples
add_executable(sample_terrain sample_terrain.cpp)
target_include_directories(sample_terrain PRIVATE ${PROJECT_SOURCE_DIR}/include)
target_link_libraries(sample_terrain PRIVATE
    ${PROJECT_NAME}
)

# Save data for visualization in robot_dart
add_executable(save_data_for_viz save_data_for_viz.cpp)
target_include_directories(save_data_for_viz  PRIVATE ${PROJECT_SOURCE_DIR}/include)
target_link_libraries(save_data_for_viz  PRIVATE
    ${PROJECT_NAME}
)
