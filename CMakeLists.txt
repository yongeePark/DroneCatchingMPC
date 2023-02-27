#
# IMPORTANT
# THIS IS BACKUP THAT IS WORKING WELL!!!
# BUT THIS IS USING CS_SIMPLE!
#
cmake_minimum_required(VERSION 3.0.2)
project(my_mpc)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} )
#${PROJECT_SOURCE_DIR}/cmake 

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  genmsg
)

# Eigen
INCLUDE_DIRECTORIES ("/usr/include/eigen3" )

# lapacke
find_package(LAPACK REQUIRED)
set(lapacke_LIBRARIES lapacke)

# find_package(OpenBLAS REQUIRED)
# if (Openblas_FOUND)
#   message("-- Found Openblas library: ${Openblas_LIBRARIES}")
#   message("-- Found Openblas include dirs: ${Openblas_INCLUDE_DIRS}")
#   include_directories(${Openblas_INCLUDE_DIRS})
#   set(lapacke_LIBRARIES ${Openblas_LIBRARIES})
# else (Openblas_FOUND)
#   message("-- Openblas not found, trying to use lapacke")
#   find_package(LAPACK REQUIRED)
#   set(lapacke_LIBRARIES lapacke)
# endif (Openblas_FOUND)


# ACADO

#include_directories(${Openblas_INCLUDE_DIRS})
#set(lapacke_LIBRARIES ${Openblas_LIBRARIES})

# find_package( ACADO REQUIRED )
# INCLUDE_DIRECTORIES(${ACADO_INCLUDE_DIRS} )
# set(ACADO_LIBRARIES /usr/local/lib/libacado_toolkit_s.so)



#link_directories(${PROJECT_SOURCE_DIR} ${ACADO_SHARED_LIBRARIES})
## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()
add_message_files(FILES RollPitchYawrateThrust.msg)

generate_messages(DEPENDENCIES std_msgs geometry_msgs)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES my_mpc
#  CATKIN_DEPENDS roscpp std_msgs
#  DEPENDS system_lib
)


###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  # ${ACADO_INCLUDE_DIRS}
  solver/
	solver/qpoases
	solver/qpoases/INCLUDE
	solver/qpoases/SRC
)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/my_mpc.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_library(mympc
  src/mympc.cc
  solver/qpoases/SRC/Bounds.cpp
  solver/qpoases/SRC/Constraints.cpp
  solver/qpoases/SRC/CyclingManager.cpp
  solver/qpoases/SRC/Indexlist.cpp
  solver/qpoases/SRC/MessageHandling.cpp
  solver/qpoases/SRC/QProblem.cpp
  solver/qpoases/SRC/QProblemB.cpp
  solver/qpoases/SRC/SubjectTo.cpp
  solver/qpoases/SRC/Utils.cpp
  solver/qpoases/SRC/EXTRAS/SolutionAnalysis.cpp
  solver/acado_qpoases_interface.cpp
  solver/acado_integrator.c
  solver/acado_solver.c
  solver/acado_auxiliary_functions.c
  solver/acado_common.h
  )

add_dependencies(mympc ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} )
  target_link_libraries(mympc  ${catkin_LIBRARIES} ${lapacke_LIBRARIES}  )

  target_include_directories(mympc PUBLIC
  solver/
	solver/qpoases
	solver/qpoases/INCLUDE
	solver/qpoases/SRC
	)

add_executable(controller_node
  src/controller_node.cpp
)
# add_dependencies(controller_node my_mpc_gencpp)
#add_dependencies(controller_node my_mpc_generate_messages_cpp)
target_link_libraries(controller_node mympc)

# add
## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")



#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# catkin_install_python(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
# install(TARGETS ${PROJECT_NAME}_node
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark libraries for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
# install(TARGETS ${PROJECT_NAME}
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

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
# catkin_add_gtest(${PROJECT_NAME}-test test/test_my_mpc.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
