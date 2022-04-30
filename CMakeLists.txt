find_package(Sophus REQUIRED)
include_directories({Sophus_INCLUDE_DIRS})
add_executable(eval_trajectory eval_trajectory.cpp)