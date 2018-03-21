# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "testing_transformation_matrix: 0 messages, 1 services")

set(MSG_I_FLAGS "-Istd_msgs:/opt/ros/kinetic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(testing_transformation_matrix_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/jepod13/catkin_ws/src/testing_transformation_matrix/srv/brick_to_home.srv" NAME_WE)
add_custom_target(_testing_transformation_matrix_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "testing_transformation_matrix" "/home/jepod13/catkin_ws/src/testing_transformation_matrix/srv/brick_to_home.srv" ""
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages

### Generating Services
_generate_srv_cpp(testing_transformation_matrix
  "/home/jepod13/catkin_ws/src/testing_transformation_matrix/srv/brick_to_home.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/testing_transformation_matrix
)

### Generating Module File
_generate_module_cpp(testing_transformation_matrix
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/testing_transformation_matrix
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(testing_transformation_matrix_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(testing_transformation_matrix_generate_messages testing_transformation_matrix_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/jepod13/catkin_ws/src/testing_transformation_matrix/srv/brick_to_home.srv" NAME_WE)
add_dependencies(testing_transformation_matrix_generate_messages_cpp _testing_transformation_matrix_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(testing_transformation_matrix_gencpp)
add_dependencies(testing_transformation_matrix_gencpp testing_transformation_matrix_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS testing_transformation_matrix_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages

### Generating Services
_generate_srv_eus(testing_transformation_matrix
  "/home/jepod13/catkin_ws/src/testing_transformation_matrix/srv/brick_to_home.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/testing_transformation_matrix
)

### Generating Module File
_generate_module_eus(testing_transformation_matrix
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/testing_transformation_matrix
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(testing_transformation_matrix_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(testing_transformation_matrix_generate_messages testing_transformation_matrix_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/jepod13/catkin_ws/src/testing_transformation_matrix/srv/brick_to_home.srv" NAME_WE)
add_dependencies(testing_transformation_matrix_generate_messages_eus _testing_transformation_matrix_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(testing_transformation_matrix_geneus)
add_dependencies(testing_transformation_matrix_geneus testing_transformation_matrix_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS testing_transformation_matrix_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages

### Generating Services
_generate_srv_lisp(testing_transformation_matrix
  "/home/jepod13/catkin_ws/src/testing_transformation_matrix/srv/brick_to_home.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/testing_transformation_matrix
)

### Generating Module File
_generate_module_lisp(testing_transformation_matrix
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/testing_transformation_matrix
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(testing_transformation_matrix_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(testing_transformation_matrix_generate_messages testing_transformation_matrix_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/jepod13/catkin_ws/src/testing_transformation_matrix/srv/brick_to_home.srv" NAME_WE)
add_dependencies(testing_transformation_matrix_generate_messages_lisp _testing_transformation_matrix_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(testing_transformation_matrix_genlisp)
add_dependencies(testing_transformation_matrix_genlisp testing_transformation_matrix_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS testing_transformation_matrix_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages

### Generating Services
_generate_srv_nodejs(testing_transformation_matrix
  "/home/jepod13/catkin_ws/src/testing_transformation_matrix/srv/brick_to_home.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/testing_transformation_matrix
)

### Generating Module File
_generate_module_nodejs(testing_transformation_matrix
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/testing_transformation_matrix
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(testing_transformation_matrix_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(testing_transformation_matrix_generate_messages testing_transformation_matrix_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/jepod13/catkin_ws/src/testing_transformation_matrix/srv/brick_to_home.srv" NAME_WE)
add_dependencies(testing_transformation_matrix_generate_messages_nodejs _testing_transformation_matrix_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(testing_transformation_matrix_gennodejs)
add_dependencies(testing_transformation_matrix_gennodejs testing_transformation_matrix_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS testing_transformation_matrix_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages

### Generating Services
_generate_srv_py(testing_transformation_matrix
  "/home/jepod13/catkin_ws/src/testing_transformation_matrix/srv/brick_to_home.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/testing_transformation_matrix
)

### Generating Module File
_generate_module_py(testing_transformation_matrix
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/testing_transformation_matrix
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(testing_transformation_matrix_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(testing_transformation_matrix_generate_messages testing_transformation_matrix_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/jepod13/catkin_ws/src/testing_transformation_matrix/srv/brick_to_home.srv" NAME_WE)
add_dependencies(testing_transformation_matrix_generate_messages_py _testing_transformation_matrix_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(testing_transformation_matrix_genpy)
add_dependencies(testing_transformation_matrix_genpy testing_transformation_matrix_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS testing_transformation_matrix_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/testing_transformation_matrix)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/testing_transformation_matrix
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(testing_transformation_matrix_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/testing_transformation_matrix)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/testing_transformation_matrix
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(testing_transformation_matrix_generate_messages_eus std_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/testing_transformation_matrix)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/testing_transformation_matrix
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(testing_transformation_matrix_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/testing_transformation_matrix)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/testing_transformation_matrix
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(testing_transformation_matrix_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/testing_transformation_matrix)
  install(CODE "execute_process(COMMAND \"/usr/bin/python\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/testing_transformation_matrix\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/testing_transformation_matrix
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(testing_transformation_matrix_generate_messages_py std_msgs_generate_messages_py)
endif()
