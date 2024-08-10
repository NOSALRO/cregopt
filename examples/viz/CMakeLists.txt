# # Visualisation of towr with time optimisation
# add_executable(viz_anymal_towr_opt examples/viz/viz_anymal_towr_opt.cpp)
# target_include_directories(viz_anymal_towr_opt PRIVATE ${PROJECT_SOURCE_DIR}/include)
# target_link_libraries(viz_anymal_towr_opt PRIVATE
#     ${PROJECT_NAME}
# )

# Visualisation of towr with mixed cem
add_executable(viz_cemmd viz_cemmd.cpp)
target_include_directories(viz_cemmd PRIVATE ${PROJECT_SOURCE_DIR}/include)
target_link_libraries(viz_cemmd PRIVATE
    ${PROJECT_NAME}
)
