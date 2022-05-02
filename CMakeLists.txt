find_package(Sophus REQUIRED)
include_directories({Sophus_INCLUDE_DIRS})
find_package(fmt)

add_executable(eval_trajectory eval_trajectory.cpp)
target_link_libraries(eval_trajectory fmt::fmt)