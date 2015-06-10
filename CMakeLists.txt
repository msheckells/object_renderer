cmake_minimum_required (VERSION 2.6) 
project (object_renderer)

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake/Modules)

#ADD_DEFINITIONS(-g -O -std=c++0x -pthread)
ADD_DEFINITIONS(-std=c++0x)
#ADD_DEFINITIONS(-g -O -std=c++0x -pthread -Wl,--no-undefined  -I/usr/include/eigen3)
#ADD_DEFINITIONS(-g -O -pthread)

SET(CPACK_PACKAGE_DESCRIPTION_SUMMARY "Vision-based Optimal Control")
SET(CPACK_PACKAGE_VENDOR "Matt Sheckells")
#SET(CPACK_PACKAGE_DESCRIPTION_FILE "${CMAKE_CURRENT_SOURCE_DIR}/ReadMe.txt")
#SET(CPACK_RESOURCE_FILE_LICENSE "${CMAKE_CURRENT_SOURCE_DIR}/COPYING")
SET(CPACK_PACKAGE_VERSION_MAJOR "0")
SET(CPACK_PACKAGE_VERSION_MINOR "1")
SET(CPACK_PACKAGE_VERSION_PATCH "0")
SET(CPACK_PACKAGE_INSTALL_DIRECTORY "CMake ${CMake_VERSION_MAJOR}.${CMake_VERSION_MINOR}")

## Options for building this Project:
option(BUILD_TESTS "BUILD_TESTS" ON)
# Will add more like disabling est if not needed etc
#option(USE_VIEWER "USE_VIEWER" ON)
#option(DISABLE_EST "DISABLE_ESTIMATION" OFF)

#SET(CPACK_STRIP_FILES "bin/MyExecutable")
#SET(CPACK_SOURCE_STRIP_FILES "")

#find_package(OpenGL REQUIRED)
#find_package(GLUT REQUIRED)
#include_directories( ${OPENGL_INCLUDE_DIRS}  ${GLUT_INCLUDE_DIRS} )
#target_link_libraries( ${OpenGL_LIBRARIES} ${GLUT_LIBRARY} )

# We break this project gcop into smaller libraries as:
# gcop_systems.so gcop_views.so gcop_algos.so gcop_est.so
# Utils folder is linked into all the above libraries

# Instead of a common variable called LIBS we make separate LIBS for each the submodule library
# 	SYS_LIBS	ALGO_LIBS	VIEWER_LIBS	UTIL_LIBS EST_LIBS
# Any Libs needed specifically by testing binaries is named based on the testing bin name
# Example: Camtest bin needs ABC library then there will be a variable Cam_LIBS with the Libraries needed by the Camtest (It may be empty if nothing is needed)

find_package(PkgConfig REQUIRED)


find_package( PkgConfig )
pkg_check_modules( EIGEN3 REQUIRED eigen3 )
include_directories( ${EIGEN3_INCLUDE_DIRS} )

#find_package(Eigen REQUIRED)
#include_directories(${Eigen_INCLUDE_DIRS})

find_package( OpenCV REQUIRED)
include_directories( ${OpenCV_INCLUDE_DIRS} )

find_package(OGRE REQUIRED)
find_package(OIS REQUIRED)
include_directories( ${OIS_INCLUDE_DIRS}
	${OGRE_INCLUDE_DIRS}
)



include_directories(
  lib
)

add_subdirectory (bin)
add_subdirectory (lib)

#SET(CPACK_PACKAGE_EXECUTABLES test "An Example Trajectory Planner")

#set(CPACK_SOURCE_INSTALLED_DIRECTORIES "${CMAKE_SOURCE_DIR}/lib;./lib")
set(CPACK_SOURCE_IGNORE_FILES
"/docs/*;/logs/*;/ros_packages/;/build/;/.git/;/CSV/;/\\\\.svn/;/\\\\.gz/;/\\\\.eps/;/\\\\.pdf/;/\\\\.jpg/;/\\\\.ppm/;/\\\\.log/;/\\\\.avi/;/\\\\.zip/;/\\\\.so/;/\\\\.a/;/\\\\.o/;\\\\.swp$;~$;\\\\.\\\\#;/\\\\#")

INCLUDE(CPack)

set(headers
    lib/ogre_application.h
    lib/camera_render_application.h
    lib/virtual_image_handler.h)
#MESSAGE( STATUS "Headers: " ${headers} )
INSTALL(FILES ${headers} DESTINATION include/object_renderer)
