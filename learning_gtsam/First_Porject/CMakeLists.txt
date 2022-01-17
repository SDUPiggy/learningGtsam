cmake_minimum_required(VERSION 2.8)
project(gtsam)

find_package(GTSAM REQUIRED QUIET)
#find_package(GTSAMCMakeTools)
# include(GtsamBuildTypes) # Load build type flags and default to Debug mode
# include(GtsamTesting)    # Easy functions for creating unit tests and scripts
#INCLUDE_DIRECTORIES("/usr/include/eigen3")
# find_package(Boost COMPONENTS system REQUIRED)
include_directories(${GTSAM_INCLUDE_DIRS})
include_directories(
	# ${EIGEN3_INCLUCE_DIR}
	${GTSAM_INCLUDE_DIR}
)
link_directories(
	# ${EIGEN3_LIBRARY_DIR}
  	${GTSAM_LIBRARY_DIRS}
)
add_executable(use_gtsam use_gtsam.cpp)
target_link_libraries(use_gtsam gtsam)


# Install library
#install(TARGETS ${CONVENIENCE_LIB_NAME} LIBRARY DESTINATION lib ARCHIVE DESTINATION lib RUNTIME DESTINATION bin)

###################################################################################
# Build tests (CMake tracks the dependecy to link with GTSAM through our project's static library)
#gtsamAddTestsGlob("example" "tests/test*.cpp" "" "${CONVENIENCE_LIB_NAME}")

###################################################################################
# Build scripts (CMake tracks the dependecy to link with GTSAM through our project's static library)
#gtsamAddExamplesGlob("*.cpp" "" "${CONVENIENCE_LIB_NAME}")

###################################################################################
# Build MATLAB wrapper (CMake tracks the dependecy to link with GTSAM through our project's static library)
#wrap_and_install_library("example.h" "${CONVENIENCE_LIB_NAME}" "" "")
