cmake_minimum_required(VERSION 2.8)
project( est_rot )
find_package( OpenCV REQUIRED )
add_executable( est_rot ./src/est_rot.cpp )
target_link_libraries( est_rot ${OpenCV_LIBS} )
