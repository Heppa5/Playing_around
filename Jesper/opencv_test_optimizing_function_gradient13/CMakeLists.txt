cmake_minimum_required(VERSION 2.8)
add_compile_options(-std=c++11)
project( test_opt_func )
include_directories(
    include
    )
find_package( OpenCV REQUIRED )
add_executable( test_opt_func ./src/test_opt_func.cpp 
                              ./include/find_gradient.h  )
target_link_libraries( test_opt_func ${OpenCV_LIBS} )
