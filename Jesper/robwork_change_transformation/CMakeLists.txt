cmake_minimum_required(VERSION 2.8)
add_compile_options(-std=c++11)
project( change_tcpTmarker )
# find_package( OpenCV REQUIRED )

# Set the RobWork root (edit in .bashrc if necessary)
SET(RW_ROOT $ENV{RW_ROOT})

MESSAGE("-- RobWork root: " ${RW_ROOT})
SET(RobWork_DIR ${RW_ROOT}/cmake)
FIND_PACKAGE(RobWork REQUIRED)
INCLUDE_DIRECTORIES( ${ROBWORK_INCLUDE_DIRS} )
LINK_DIRECTORIES( ${ROBWORK_LIBRARY_DIRS} )

add_executable( change_tcpTmarker ./src/change_tcpTmarker.cpp )
target_link_libraries( change_tcpTmarker ${ROBWORK_LIBRARIES} )
# target_link_libraries( change_tcpTmarker ${OpenCV_LIBS} )
