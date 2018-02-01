# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "mp_mini_picker: 1 messages, 4 services")

set(MSG_I_FLAGS "-Imp_mini_picker:/home/jepod13/catkin_ws/src/mp_mini_picker/msg;-Istd_msgs:/opt/ros/kinetic/share/std_msgs/cmake/../msg;-Igeometry_msgs:/opt/ros/kinetic/share/geometry_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(mp_mini_picker_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/jepod13/catkin_ws/src/mp_mini_picker/srv/currentQ.srv" NAME_WE)
add_custom_target(_mp_mini_picker_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "mp_mini_picker" "/home/jepod13/catkin_ws/src/mp_mini_picker/srv/currentQ.srv" ""
)

get_filename_component(_filename "/home/jepod13/catkin_ws/src/mp_mini_picker/srv/moveToQ.srv" NAME_WE)
add_custom_target(_mp_mini_picker_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "mp_mini_picker" "/home/jepod13/catkin_ws/src/mp_mini_picker/srv/moveToQ.srv" ""
)

get_filename_component(_filename "/home/jepod13/catkin_ws/src/mp_mini_picker/srv/moveToPose.srv" NAME_WE)
add_custom_target(_mp_mini_picker_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "mp_mini_picker" "/home/jepod13/catkin_ws/src/mp_mini_picker/srv/moveToPose.srv" ""
)

get_filename_component(_filename "/home/jepod13/catkin_ws/src/mp_mini_picker/msg/currrentToolPosition.msg" NAME_WE)
add_custom_target(_mp_mini_picker_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "mp_mini_picker" "/home/jepod13/catkin_ws/src/mp_mini_picker/msg/currrentToolPosition.msg" "geometry_msgs/Quaternion:geometry_msgs/PoseStamped:geometry_msgs/Pose:std_msgs/Header:geometry_msgs/Point"
)

get_filename_component(_filename "/home/jepod13/catkin_ws/src/mp_mini_picker/srv/moveToPoint.srv" NAME_WE)
add_custom_target(_mp_mini_picker_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "mp_mini_picker" "/home/jepod13/catkin_ws/src/mp_mini_picker/srv/moveToPoint.srv" ""
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(mp_mini_picker
  "/home/jepod13/catkin_ws/src/mp_mini_picker/msg/currrentToolPosition.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/PoseStamped.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/mp_mini_picker
)

### Generating Services
_generate_srv_cpp(mp_mini_picker
  "/home/jepod13/catkin_ws/src/mp_mini_picker/srv/moveToQ.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/mp_mini_picker
)
_generate_srv_cpp(mp_mini_picker
  "/home/jepod13/catkin_ws/src/mp_mini_picker/srv/currentQ.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/mp_mini_picker
)
_generate_srv_cpp(mp_mini_picker
  "/home/jepod13/catkin_ws/src/mp_mini_picker/srv/moveToPose.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/mp_mini_picker
)
_generate_srv_cpp(mp_mini_picker
  "/home/jepod13/catkin_ws/src/mp_mini_picker/srv/moveToPoint.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/mp_mini_picker
)

### Generating Module File
_generate_module_cpp(mp_mini_picker
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/mp_mini_picker
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(mp_mini_picker_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(mp_mini_picker_generate_messages mp_mini_picker_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/jepod13/catkin_ws/src/mp_mini_picker/srv/currentQ.srv" NAME_WE)
add_dependencies(mp_mini_picker_generate_messages_cpp _mp_mini_picker_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/jepod13/catkin_ws/src/mp_mini_picker/srv/moveToQ.srv" NAME_WE)
add_dependencies(mp_mini_picker_generate_messages_cpp _mp_mini_picker_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/jepod13/catkin_ws/src/mp_mini_picker/srv/moveToPose.srv" NAME_WE)
add_dependencies(mp_mini_picker_generate_messages_cpp _mp_mini_picker_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/jepod13/catkin_ws/src/mp_mini_picker/msg/currrentToolPosition.msg" NAME_WE)
add_dependencies(mp_mini_picker_generate_messages_cpp _mp_mini_picker_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/jepod13/catkin_ws/src/mp_mini_picker/srv/moveToPoint.srv" NAME_WE)
add_dependencies(mp_mini_picker_generate_messages_cpp _mp_mini_picker_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(mp_mini_picker_gencpp)
add_dependencies(mp_mini_picker_gencpp mp_mini_picker_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS mp_mini_picker_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(mp_mini_picker
  "/home/jepod13/catkin_ws/src/mp_mini_picker/msg/currrentToolPosition.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/PoseStamped.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/mp_mini_picker
)

### Generating Services
_generate_srv_eus(mp_mini_picker
  "/home/jepod13/catkin_ws/src/mp_mini_picker/srv/moveToQ.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/mp_mini_picker
)
_generate_srv_eus(mp_mini_picker
  "/home/jepod13/catkin_ws/src/mp_mini_picker/srv/currentQ.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/mp_mini_picker
)
_generate_srv_eus(mp_mini_picker
  "/home/jepod13/catkin_ws/src/mp_mini_picker/srv/moveToPose.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/mp_mini_picker
)
_generate_srv_eus(mp_mini_picker
  "/home/jepod13/catkin_ws/src/mp_mini_picker/srv/moveToPoint.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/mp_mini_picker
)

### Generating Module File
_generate_module_eus(mp_mini_picker
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/mp_mini_picker
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(mp_mini_picker_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(mp_mini_picker_generate_messages mp_mini_picker_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/jepod13/catkin_ws/src/mp_mini_picker/srv/currentQ.srv" NAME_WE)
add_dependencies(mp_mini_picker_generate_messages_eus _mp_mini_picker_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/jepod13/catkin_ws/src/mp_mini_picker/srv/moveToQ.srv" NAME_WE)
add_dependencies(mp_mini_picker_generate_messages_eus _mp_mini_picker_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/jepod13/catkin_ws/src/mp_mini_picker/srv/moveToPose.srv" NAME_WE)
add_dependencies(mp_mini_picker_generate_messages_eus _mp_mini_picker_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/jepod13/catkin_ws/src/mp_mini_picker/msg/currrentToolPosition.msg" NAME_WE)
add_dependencies(mp_mini_picker_generate_messages_eus _mp_mini_picker_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/jepod13/catkin_ws/src/mp_mini_picker/srv/moveToPoint.srv" NAME_WE)
add_dependencies(mp_mini_picker_generate_messages_eus _mp_mini_picker_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(mp_mini_picker_geneus)
add_dependencies(mp_mini_picker_geneus mp_mini_picker_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS mp_mini_picker_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(mp_mini_picker
  "/home/jepod13/catkin_ws/src/mp_mini_picker/msg/currrentToolPosition.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/PoseStamped.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/mp_mini_picker
)

### Generating Services
_generate_srv_lisp(mp_mini_picker
  "/home/jepod13/catkin_ws/src/mp_mini_picker/srv/moveToQ.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/mp_mini_picker
)
_generate_srv_lisp(mp_mini_picker
  "/home/jepod13/catkin_ws/src/mp_mini_picker/srv/currentQ.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/mp_mini_picker
)
_generate_srv_lisp(mp_mini_picker
  "/home/jepod13/catkin_ws/src/mp_mini_picker/srv/moveToPose.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/mp_mini_picker
)
_generate_srv_lisp(mp_mini_picker
  "/home/jepod13/catkin_ws/src/mp_mini_picker/srv/moveToPoint.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/mp_mini_picker
)

### Generating Module File
_generate_module_lisp(mp_mini_picker
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/mp_mini_picker
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(mp_mini_picker_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(mp_mini_picker_generate_messages mp_mini_picker_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/jepod13/catkin_ws/src/mp_mini_picker/srv/currentQ.srv" NAME_WE)
add_dependencies(mp_mini_picker_generate_messages_lisp _mp_mini_picker_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/jepod13/catkin_ws/src/mp_mini_picker/srv/moveToQ.srv" NAME_WE)
add_dependencies(mp_mini_picker_generate_messages_lisp _mp_mini_picker_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/jepod13/catkin_ws/src/mp_mini_picker/srv/moveToPose.srv" NAME_WE)
add_dependencies(mp_mini_picker_generate_messages_lisp _mp_mini_picker_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/jepod13/catkin_ws/src/mp_mini_picker/msg/currrentToolPosition.msg" NAME_WE)
add_dependencies(mp_mini_picker_generate_messages_lisp _mp_mini_picker_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/jepod13/catkin_ws/src/mp_mini_picker/srv/moveToPoint.srv" NAME_WE)
add_dependencies(mp_mini_picker_generate_messages_lisp _mp_mini_picker_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(mp_mini_picker_genlisp)
add_dependencies(mp_mini_picker_genlisp mp_mini_picker_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS mp_mini_picker_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(mp_mini_picker
  "/home/jepod13/catkin_ws/src/mp_mini_picker/msg/currrentToolPosition.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/PoseStamped.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/mp_mini_picker
)

### Generating Services
_generate_srv_nodejs(mp_mini_picker
  "/home/jepod13/catkin_ws/src/mp_mini_picker/srv/moveToQ.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/mp_mini_picker
)
_generate_srv_nodejs(mp_mini_picker
  "/home/jepod13/catkin_ws/src/mp_mini_picker/srv/currentQ.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/mp_mini_picker
)
_generate_srv_nodejs(mp_mini_picker
  "/home/jepod13/catkin_ws/src/mp_mini_picker/srv/moveToPose.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/mp_mini_picker
)
_generate_srv_nodejs(mp_mini_picker
  "/home/jepod13/catkin_ws/src/mp_mini_picker/srv/moveToPoint.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/mp_mini_picker
)

### Generating Module File
_generate_module_nodejs(mp_mini_picker
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/mp_mini_picker
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(mp_mini_picker_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(mp_mini_picker_generate_messages mp_mini_picker_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/jepod13/catkin_ws/src/mp_mini_picker/srv/currentQ.srv" NAME_WE)
add_dependencies(mp_mini_picker_generate_messages_nodejs _mp_mini_picker_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/jepod13/catkin_ws/src/mp_mini_picker/srv/moveToQ.srv" NAME_WE)
add_dependencies(mp_mini_picker_generate_messages_nodejs _mp_mini_picker_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/jepod13/catkin_ws/src/mp_mini_picker/srv/moveToPose.srv" NAME_WE)
add_dependencies(mp_mini_picker_generate_messages_nodejs _mp_mini_picker_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/jepod13/catkin_ws/src/mp_mini_picker/msg/currrentToolPosition.msg" NAME_WE)
add_dependencies(mp_mini_picker_generate_messages_nodejs _mp_mini_picker_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/jepod13/catkin_ws/src/mp_mini_picker/srv/moveToPoint.srv" NAME_WE)
add_dependencies(mp_mini_picker_generate_messages_nodejs _mp_mini_picker_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(mp_mini_picker_gennodejs)
add_dependencies(mp_mini_picker_gennodejs mp_mini_picker_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS mp_mini_picker_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(mp_mini_picker
  "/home/jepod13/catkin_ws/src/mp_mini_picker/msg/currrentToolPosition.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/PoseStamped.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Point.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/mp_mini_picker
)

### Generating Services
_generate_srv_py(mp_mini_picker
  "/home/jepod13/catkin_ws/src/mp_mini_picker/srv/moveToQ.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/mp_mini_picker
)
_generate_srv_py(mp_mini_picker
  "/home/jepod13/catkin_ws/src/mp_mini_picker/srv/currentQ.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/mp_mini_picker
)
_generate_srv_py(mp_mini_picker
  "/home/jepod13/catkin_ws/src/mp_mini_picker/srv/moveToPose.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/mp_mini_picker
)
_generate_srv_py(mp_mini_picker
  "/home/jepod13/catkin_ws/src/mp_mini_picker/srv/moveToPoint.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/mp_mini_picker
)

### Generating Module File
_generate_module_py(mp_mini_picker
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/mp_mini_picker
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(mp_mini_picker_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(mp_mini_picker_generate_messages mp_mini_picker_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/jepod13/catkin_ws/src/mp_mini_picker/srv/currentQ.srv" NAME_WE)
add_dependencies(mp_mini_picker_generate_messages_py _mp_mini_picker_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/jepod13/catkin_ws/src/mp_mini_picker/srv/moveToQ.srv" NAME_WE)
add_dependencies(mp_mini_picker_generate_messages_py _mp_mini_picker_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/jepod13/catkin_ws/src/mp_mini_picker/srv/moveToPose.srv" NAME_WE)
add_dependencies(mp_mini_picker_generate_messages_py _mp_mini_picker_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/jepod13/catkin_ws/src/mp_mini_picker/msg/currrentToolPosition.msg" NAME_WE)
add_dependencies(mp_mini_picker_generate_messages_py _mp_mini_picker_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/jepod13/catkin_ws/src/mp_mini_picker/srv/moveToPoint.srv" NAME_WE)
add_dependencies(mp_mini_picker_generate_messages_py _mp_mini_picker_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(mp_mini_picker_genpy)
add_dependencies(mp_mini_picker_genpy mp_mini_picker_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS mp_mini_picker_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/mp_mini_picker)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/mp_mini_picker
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(mp_mini_picker_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()
if(TARGET geometry_msgs_generate_messages_cpp)
  add_dependencies(mp_mini_picker_generate_messages_cpp geometry_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/mp_mini_picker)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/mp_mini_picker
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(mp_mini_picker_generate_messages_eus std_msgs_generate_messages_eus)
endif()
if(TARGET geometry_msgs_generate_messages_eus)
  add_dependencies(mp_mini_picker_generate_messages_eus geometry_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/mp_mini_picker)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/mp_mini_picker
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(mp_mini_picker_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()
if(TARGET geometry_msgs_generate_messages_lisp)
  add_dependencies(mp_mini_picker_generate_messages_lisp geometry_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/mp_mini_picker)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/mp_mini_picker
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(mp_mini_picker_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()
if(TARGET geometry_msgs_generate_messages_nodejs)
  add_dependencies(mp_mini_picker_generate_messages_nodejs geometry_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/mp_mini_picker)
  install(CODE "execute_process(COMMAND \"/usr/bin/python\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/mp_mini_picker\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/mp_mini_picker
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(mp_mini_picker_generate_messages_py std_msgs_generate_messages_py)
endif()
if(TARGET geometry_msgs_generate_messages_py)
  add_dependencies(mp_mini_picker_generate_messages_py geometry_msgs_generate_messages_py)
endif()
