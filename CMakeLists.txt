cmake_minimum_required(VERSION 2.8.3)
project(mi_explorer)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11")
#set(CMAKE_CXX_FLAGS_RELEASE "-O3 -g")

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_generation
  nav_msgs
  roscpp
  simple_map_2d
  std_msgs
  visualization_msgs
  sc_mapping
  gf_traj_gen
)

find_package(Boost REQUIRED)
find_package(PCL REQUIRED COMPONENTS common)
find_package(Eigen3)

link_directories(/home/dcheng/mosek/8/tools/platform/linux64x86/bin)

catkin_package(
        INCLUDE_DIRS include
        CATKIN_DEPENDS geometry_msgs message_generation nav_msgs roscpp simple_map_2d std_msgs visualization_msgs sc_mapping gf_traj_gen
        DEPENDS PCL Boost
)

include_directories(
        include
  /home/dcheng/Software/libigl/include
        ${catkin_INCLUDE_DIRS}
        ${Boost_INCLUDE_DIRS}
        ${PCL_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIR}
)

add_executable(mi_naive_explorer src/mi_naive_explorer.cpp src/mi_computer.cpp src/naive_lattice.cpp)
target_link_libraries(mi_naive_explorer ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${PCL_LIBRARIES})
#add_dependencies(mi_naive_explorer ${PROJECT_NAME}_generate_messages_cpp ${catkin_EXPORTED_TARGETS})

add_executable(mi_sc_explorer src/mi_sc_explorer.cpp src/mi_computer.cpp src/sc_motion_primitive.cpp)
target_link_libraries(mi_sc_explorer ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${PCL_LIBRARIES} gf_traj_gen)
#add_dependencies(mi_sc_explorer ${PROJECT_NAME}_generate_messages_cpp ${catkin_EXPORTED_TARGETS})
