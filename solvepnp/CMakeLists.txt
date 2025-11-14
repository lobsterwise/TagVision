cmake_minimum_required(VERSION 3.0)
project(example_project)

add_executable(example "sqpnp.cpp")

# just for example add some compiler flags
target_compile_options(example PUBLIC -std=c++17)

# this lets me include files relative to the root source directory with a <> pair
# target_include_directories(example PUBLIC src/main)

# install(TARGETS example DESTINATION example_destination)
