cmake_minimum_required(VERSION 3.11 FATAL_ERROR)
project(p1_and_p2)

set(CMAKE_BUILD_TYPE RelWithDebInfo)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")

option(LAPTOP "Build code for the Laptop." OFF)

find_package(GTest REQUIRED)

# Library for code shared across projects 1 and 2. 
add_library(mbot_lib STATIC
  mbot_lib/mbot_lib/behaviors.cpp
  mbot_lib/mbot_lib/controllers.cpp
  mbot_lib/mbot_lib/utils.cpp
)
target_include_directories(mbot_lib PUBLIC
  mbot_lib
)

# Only build the executables for the robot if the laptop option is not provided.
if(NOT LAPTOP)  
  find_package(mbot_bridge REQUIRED)

  # Drive square executable.
  add_executable(drive_square 
    p1_wall_follower/1_drive_square.cpp
  )
  target_link_libraries(drive_square
    mbot_bridge_cpp
    mbot_lib
  )

  # Follow 1D executable.
  add_executable(follow_1d 
    p1_wall_follower/2_follow_1d.cpp
  )
  target_link_libraries(follow_1d
    mbot_bridge_cpp
    mbot_lib
  )

  # Drive star.
  add_executable(drive_star 
    p1_wall_follower/3_drive_star.cpp
  )
  target_link_libraries(drive_star
    mbot_bridge_cpp
    mbot_lib
  )

  # Follow 2D executable.
  add_executable(follow_2d 
    p1_wall_follower/4_follow_2d.cpp
  )
  target_link_libraries(follow_2d
    mbot_bridge_cpp
    mbot_lib
  )

  # Wall follower executable.
  add_executable(wall_follower 
    p1_wall_follower/5_wall_follower.cpp
  )
  target_link_libraries(wall_follower
    mbot_bridge_cpp
    mbot_lib
  )

  # Hit the spot executable.
  add_executable(hit_the_spot
    p2_bug_navigation/2_hit_the_spot.cpp
  )
  target_link_libraries(hit_the_spot
    mbot_bridge_cpp
    mbot_lib
  )

  # Bug navigation executable.
  add_executable(bug_navigation
    p2_bug_navigation/3_bug_navigation.cpp
  )
  target_link_libraries(bug_navigation
    mbot_bridge_cpp
    mbot_lib
  )
endif()

# Super Mario library to be used by demo and test targets. 
add_library(super_mario_lib STATIC
  p2_bug_navigation/1_super_mario.cpp
)
target_include_directories(super_mario_lib PUBLIC
  p2_bug_navigation/super_mario
)

# Super Mario executable.
add_executable(super_mario
  p2_bug_navigation/super_mario/run_super_mario.cpp
)
target_link_libraries(super_mario
  super_mario_lib
)

# Test cases.
enable_testing()

# Public test cases.
add_executable(test_public 
  test/test_public.cpp
)
target_link_libraries(test_public 
  GTest::gtest_main
  mbot_lib
  super_mario_lib
)
target_include_directories(test_public PRIVATE
  test
)
gtest_discover_tests(test_public)
