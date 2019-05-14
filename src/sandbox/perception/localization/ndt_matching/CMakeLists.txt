cmake_minimum_required(VERSION 3.5)

### Export headers
project(ndt_matching)

## dependencies
find_package(apex_cmake REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(apex_auto_msgs REQUIRED)
find_package(apexcpp REQUIRED)
find_package(apex_tf REQUIRED)
find_package(helper_functions REQUIRED)
find_package(nav_msgs)

find_package(PCL REQUIRED)
find_package(Eigen3 QUIET)

if (NOT EIGEN3_FOUND)
    # Fallback to cmake_modules
    find_package(cmake_modules REQUIRED)
    find_package(Eigen REQUIRED)
    set(EIGEN3_INCLUDE_DIRS ${EIGEN_INCLUDE_DIRS})
    set(EIGEN3_LIBRARIES ${EIGEN_LIBRARIES})  # Not strictly necessary as Eigen is head only
    # Possibly map additional variables to the EIGEN3_ prefix.
else ()
    set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
endif ()

add_library(${PROJECT_NAME} SHARED
  include/ndt_matching/ndt_matching.hpp
  src/ndt_matching.cpp
  include/ndt_matching/voxel_grid.hpp
  src/voxel_grid.cpp
  include/ndt_matching/ndt_base.hpp
  src/ndt_base.cpp
  include/ndt_matching/symmetric_eigen_solver.hpp
  include/ndt_matching/config.hpp
  src/config.cpp
)

# includes
target_include_directories(${PROJECT_NAME} PUBLIC
  include
  ${PCL_INCLUDE_DIRS}
  ${pcl_conversions_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
)

# apex_set_compile_options(${PROJECT_NAME})
ament_target_dependencies(${PROJECT_NAME}
  "apex_auto_msgs"
  "apexcpp"
  "apex_tf"
  "helper_functions"
  "nav_msgs"
  "PCL"
)


target_link_libraries(${PROJECT_NAME} ${PCL_LIBRARIES})

## install stuff
apex_install(
  LIBRARIES ${PROJECT_NAME}
)


# Ament Exporting
ament_export_dependencies(
  "apexcpp"
  "apex_auto_msgs"
  "apex_tf"
  "helper_functions"
  "nav_msgs"
  "PCL"
)
ament_package()
