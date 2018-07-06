cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

# Require C++11 (or later)
# set( CMAKE_CXX_STANDARD 11 )
# set( CMAKE_CXX_STANDARD_REQUIRED ON )
# set( CMAKE_CXX_EXTENSIONS OFF )

project(ahc)

add_definitions(-DUSE_BOOST_SHARED_PTR)

find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)
find_package(realsense2 REQUIRED)

include_directories("../include")
include_directories(${realsense_INCLUDE_DIR})
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(plane_fitter plane_fitter.cpp)
target_link_libraries(plane_fitter ${PCL_LIBRARIES} ${OpenCV_LIBS} ${realsense2_LIBRARY})
install(TARGETS plane_fitter DESTINATION bin)

add_executable(plane_fitter_pcd plane_fitter_pcd.cpp)
target_link_libraries(plane_fitter_pcd ${PCL_LIBRARIES} ${OpenCV_LIBS} ${realsense2_LIBRARY})
install(TARGETS plane_fitter_pcd DESTINATION bin)
install(FILES plane_fitter_pcd.ini DESTINATION bin)

add_executable( Color_Depth realsense.h realsense.cpp main.cpp )
target_link_libraries( Color_Depth ${realsense2_LIBRARY} )
target_link_libraries( Color_Depth ${OpenCV_LIBS} )

add_executable(plane_fitter_rs realsense.h realsense.cpp plane_fitter_rs.cpp)
target_link_libraries(plane_fitter_rs ${PCL_LIBRARIES} ${OpenCV_LIBS} ${realsense2_LIBRARY})
install(TARGETS plane_fitter_rs DESTINATION bin)