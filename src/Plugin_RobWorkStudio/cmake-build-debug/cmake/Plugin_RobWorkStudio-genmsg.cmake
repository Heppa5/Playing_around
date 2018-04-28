# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "Plugin_RobWorkStudio: 0 messages, 1 services")

set(MSG_I_FLAGS "-Istd_msgs:/opt/ros/kinetic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(Plugin_RobWorkStudio_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/jepod13/catkin_ws/src/Plugin_RobWorkStudio/srv/suctionCup.srv" NAME_WE)
add_custom_target(_Plugin_RobWorkStudio_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "Plugin_RobWorkStudio" "/home/jepod13/catkin_ws/src/Plugin_RobWorkStudio/srv/suctionCup.srv" ""
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages

### Generating Services
_generate_srv_cpp(Plugin_RobWorkStudio
  "/home/jepod13/catkin_ws/src/Plugin_RobWorkStudio/srv/suctionCup.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/Plugin_RobWorkStudio
)

### Generating Module File
_generate_module_cpp(Plugin_RobWorkStudio
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/Plugin_RobWorkStudio
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(Plugin_RobWorkStudio_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(Plugin_RobWorkStudio_generate_messages Plugin_RobWorkStudio_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/jepod13/catkin_ws/src/Plugin_RobWorkStudio/srv/suctionCup.srv" NAME_WE)
add_dependencies(Plugin_RobWorkStudio_generate_messages_cpp _Plugin_RobWorkStudio_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(Plugin_RobWorkStudio_gencpp)
add_dependencies(Plugin_RobWorkStudio_gencpp Plugin_RobWorkStudio_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS Plugin_RobWorkStudio_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages

### Generating Services
_generate_srv_eus(Plugin_RobWorkStudio
  "/home/jepod13/catkin_ws/src/Plugin_RobWorkStudio/srv/suctionCup.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/Plugin_RobWorkStudio
)

### Generating Module File
_generate_module_eus(Plugin_RobWorkStudio
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/Plugin_RobWorkStudio
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(Plugin_RobWorkStudio_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(Plugin_RobWorkStudio_generate_messages Plugin_RobWorkStudio_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/jepod13/catkin_ws/src/Plugin_RobWorkStudio/srv/suctionCup.srv" NAME_WE)
add_dependencies(Plugin_RobWorkStudio_generate_messages_eus _Plugin_RobWorkStudio_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(Plugin_RobWorkStudio_geneus)
add_dependencies(Plugin_RobWorkStudio_geneus Plugin_RobWorkStudio_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS Plugin_RobWorkStudio_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages

### Generating Services
_generate_srv_lisp(Plugin_RobWorkStudio
  "/home/jepod13/catkin_ws/src/Plugin_RobWorkStudio/srv/suctionCup.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/Plugin_RobWorkStudio
)

### Generating Module File
_generate_module_lisp(Plugin_RobWorkStudio
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/Plugin_RobWorkStudio
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(Plugin_RobWorkStudio_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(Plugin_RobWorkStudio_generate_messages Plugin_RobWorkStudio_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/jepod13/catkin_ws/src/Plugin_RobWorkStudio/srv/suctionCup.srv" NAME_WE)
add_dependencies(Plugin_RobWorkStudio_generate_messages_lisp _Plugin_RobWorkStudio_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(Plugin_RobWorkStudio_genlisp)
add_dependencies(Plugin_RobWorkStudio_genlisp Plugin_RobWorkStudio_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS Plugin_RobWorkStudio_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages

### Generating Services
_generate_srv_nodejs(Plugin_RobWorkStudio
  "/home/jepod13/catkin_ws/src/Plugin_RobWorkStudio/srv/suctionCup.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/Plugin_RobWorkStudio
)

### Generating Module File
_generate_module_nodejs(Plugin_RobWorkStudio
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/Plugin_RobWorkStudio
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(Plugin_RobWorkStudio_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(Plugin_RobWorkStudio_generate_messages Plugin_RobWorkStudio_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/jepod13/catkin_ws/src/Plugin_RobWorkStudio/srv/suctionCup.srv" NAME_WE)
add_dependencies(Plugin_RobWorkStudio_generate_messages_nodejs _Plugin_RobWorkStudio_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(Plugin_RobWorkStudio_gennodejs)
add_dependencies(Plugin_RobWorkStudio_gennodejs Plugin_RobWorkStudio_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS Plugin_RobWorkStudio_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages

### Generating Services
_generate_srv_py(Plugin_RobWorkStudio
  "/home/jepod13/catkin_ws/src/Plugin_RobWorkStudio/srv/suctionCup.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/Plugin_RobWorkStudio
)

### Generating Module File
_generate_module_py(Plugin_RobWorkStudio
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/Plugin_RobWorkStudio
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(Plugin_RobWorkStudio_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(Plugin_RobWorkStudio_generate_messages Plugin_RobWorkStudio_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/jepod13/catkin_ws/src/Plugin_RobWorkStudio/srv/suctionCup.srv" NAME_WE)
add_dependencies(Plugin_RobWorkStudio_generate_messages_py _Plugin_RobWorkStudio_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(Plugin_RobWorkStudio_genpy)
add_dependencies(Plugin_RobWorkStudio_genpy Plugin_RobWorkStudio_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS Plugin_RobWorkStudio_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/Plugin_RobWorkStudio)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/Plugin_RobWorkStudio
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(Plugin_RobWorkStudio_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/Plugin_RobWorkStudio)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/Plugin_RobWorkStudio
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(Plugin_RobWorkStudio_generate_messages_eus std_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/Plugin_RobWorkStudio)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/Plugin_RobWorkStudio
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(Plugin_RobWorkStudio_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/Plugin_RobWorkStudio)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/Plugin_RobWorkStudio
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(Plugin_RobWorkStudio_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/Plugin_RobWorkStudio)
  install(CODE "execute_process(COMMAND \"/usr/bin/python\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/Plugin_RobWorkStudio\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/Plugin_RobWorkStudio
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(Plugin_RobWorkStudio_generate_messages_py std_msgs_generate_messages_py)
endif()
