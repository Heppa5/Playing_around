cmake_minimum_required(VERSION 2.8)
add_compile_options(-std=c++11)
project( test_visual_servoing )
 find_package( OpenCV REQUIRED )

# Set the RobWork root (edit in .bashrc if necessary)
SET(RW_ROOT $ENV{RW_ROOT})

MESSAGE("-- RobWork root: " ${RW_ROOT})
SET(RobWork_DIR ${RW_ROOT}/cmake)
FIND_PACKAGE(RobWork REQUIRED)
INCLUDE_DIRECTORIES( ${ROBWORK_INCLUDE_DIRS} )
LINK_DIRECTORIES( ${ROBWORK_LIBRARY_DIRS} )

add_executable( test_visual_servoing ./src/test_visual_servoing.cpp )
target_link_libraries( test_visual_servoing ${ROBWORK_LIBRARIES} ${OpenCV_LIBS} )
# target_link_libraries( change_tcpTmarker ${OpenCV_LIBS} )
