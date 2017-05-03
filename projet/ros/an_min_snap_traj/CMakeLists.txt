cmake_minimum_required(VERSION 2.8.3)
project(an_min_snap_traj)

option(USE_GUROBI "Compile with GUROBI" ON)
option(USE_QPOASES "Compile with qpOASES" OFF)

## Add support for C++11, supported in ROS Kinetic and newer
add_definitions(-std=c++11)
if(USE_GUROBI)
  add_definitions(-DUSE_GUROBI)
endif()
if(USE_QPOASES)
  add_definitions(-DUSE_QPOASES)
endif()
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake")


## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  ooqp_eigen_interface
  roscpp
  trajectory_msgs
  geometry_msgs
)
find_package(Eigen3 REQUIRED)
find_package(eigen-quadprog REQUIRED)
if(USE_GUROBI)
  find_package(GUROBI REQUIRED)
  find_package(eigen-gurobi REQUIRED)
endif()
find_package(IPOPT REQUIRED)
if(USE_QPOASES)
  find_package(qpOASES REQUIRED)
endif()
find_package(Nlopt REQUIRED)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
INCLUDE_DIRS include ${EIGEN3_INCLUDE_DIR}
LIBRARIES an_min_snap_traj
CATKIN_DEPENDS ooqp_eigen_interface roscpp trajectory_msgs geometry_msgs
DEPENDS
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include)
include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(${catkin_INCLUDE_DIRS})
include_directories(${EIGEN_QUADPROG_INCLUDE_DIR})
if(USE_GUROBI)
  include_directories(${GUROBI_INCLUDE_DIRS})
  include_directories(${EIGEN_GUROBI_INCLUDE_DIR})
endif()
include_directories(${IPOPT_INCLUDE_DIRS})
if(USE_QPOASES)
  include_directories(${QPOASES_INCLUDE_DIR})
endif()
include_directories(${NLOPT_INCLUDE_DIRS})

## Declare a C++ library
add_library(alglib
  src/alglib/alglibinternal.cpp
  src/alglib/alglibmisc.cpp
  src/alglib/ap.cpp
  src/alglib/linalg.cpp
  src/alglib/optimization.cpp
  src/alglib/solvers.cpp
)
target_include_directories(alglib PUBLIC include/alglib)
target_compile_definitions(alglib PUBLIC -DAE_CPU=AE_INTEL)

add_library(${PROJECT_NAME}
  src/${PROJECT_NAME}/TrajectoryGenerator.cpp
  src/an_min_snap_traj/TrajectoryConstraint.cpp
  src/an_min_snap_traj/TrajectoryMath.cpp
  src/an_min_snap_traj/TrajectorySegment.cpp
  src/an_min_snap_traj/TimeAllocationOpt.cpp
  src/an_min_snap_traj/qp_solvers/IpoptAdapter.cpp
  src/an_min_snap_traj/nlp_solvers/MyNLP.cpp


)

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
add_dependencies(${PROJECT_NAME}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ooqpbase
  ooqpdense
  ooqpgensparse
  ooqpmehrotra
  ooqpbound
  ooqpgendense
  ooqpgondzio
  ooqpsparse
  blas
  ma27
  gfortran
  ${EIGEN_QUADPROG_LIBRARY}
  ${IPOPT_LIBRARIES}
  alglib
  ${NLOPT_LIBRARIES}
)

if(USE_GUROBI)
  target_link_libraries(${PROJECT_NAME}
    ${GUROBI_LIBRARIES}
    ${EIGEN_GUROBI_LIBRARY}
  )
endif()

if(USE_QPOASES)
  target_link_libraries(${PROJECT_NAME}
    ${QPOASES_LIBRARY}
  )
endif()
## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(${PROJECT_NAME}_node
  src/an_min_snap_traj_node.cpp
)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above
add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}
)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
install(TARGETS ${PROJECT_NAME} #${PROJECT_NAME}_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
catkin_add_gtest(${PROJECT_NAME}-test
  test/test_main.cpp
  test/test_an_min_snap_traj.cpp
  test/test_trajectory_math.cpp
  test/test_qp.cpp
)
if(TARGET ${PROJECT_NAME}-test)
  target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
