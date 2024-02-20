# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "mtt_msgs: 2 messages, 0 services")

set(MSG_I_FLAGS "-Imtt_msgs:/home/changwon/data_2/hanwha_projects/hanwha_sot/hanwha_sot/src/mtt_msgs/msg;-Istd_msgs:/opt/ros/melodic/share/std_msgs/cmake/../msg;-Igeometry_msgs:/opt/ros/melodic/share/geometry_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(mtt_msgs_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/changwon/data_2/hanwha_projects/hanwha_sot/hanwha_sot/src/mtt_msgs/msg/FollowTargetInfo.msg" NAME_WE)
add_custom_target(_mtt_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "mtt_msgs" "/home/changwon/data_2/hanwha_projects/hanwha_sot/hanwha_sot/src/mtt_msgs/msg/FollowTargetInfo.msg" "std_msgs/Int8:std_msgs/Int32:geometry_msgs/Pose:geometry_msgs/Quaternion:std_msgs/Header:geometry_msgs/Point:std_msgs/Float32"
)

get_filename_component(_filename "/home/changwon/data_2/hanwha_projects/hanwha_sot/hanwha_sot/src/mtt_msgs/msg/TargetCandidate.msg" NAME_WE)
add_custom_target(_mtt_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "mtt_msgs" "/home/changwon/data_2/hanwha_projects/hanwha_sot/hanwha_sot/src/mtt_msgs/msg/TargetCandidate.msg" "std_msgs/Int8:geometry_msgs/Point:std_msgs/Int32:std_msgs/Header"
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(mtt_msgs
  "/home/changwon/data_2/hanwha_projects/hanwha_sot/hanwha_sot/src/mtt_msgs/msg/FollowTargetInfo.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Int8.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Int32.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Float32.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/mtt_msgs
)
_generate_msg_cpp(mtt_msgs
  "/home/changwon/data_2/hanwha_projects/hanwha_sot/hanwha_sot/src/mtt_msgs/msg/TargetCandidate.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Int8.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Int32.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/mtt_msgs
)

### Generating Services

### Generating Module File
_generate_module_cpp(mtt_msgs
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/mtt_msgs
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(mtt_msgs_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(mtt_msgs_generate_messages mtt_msgs_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/changwon/data_2/hanwha_projects/hanwha_sot/hanwha_sot/src/mtt_msgs/msg/FollowTargetInfo.msg" NAME_WE)
add_dependencies(mtt_msgs_generate_messages_cpp _mtt_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/changwon/data_2/hanwha_projects/hanwha_sot/hanwha_sot/src/mtt_msgs/msg/TargetCandidate.msg" NAME_WE)
add_dependencies(mtt_msgs_generate_messages_cpp _mtt_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(mtt_msgs_gencpp)
add_dependencies(mtt_msgs_gencpp mtt_msgs_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS mtt_msgs_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(mtt_msgs
  "/home/changwon/data_2/hanwha_projects/hanwha_sot/hanwha_sot/src/mtt_msgs/msg/FollowTargetInfo.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Int8.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Int32.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Float32.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/mtt_msgs
)
_generate_msg_eus(mtt_msgs
  "/home/changwon/data_2/hanwha_projects/hanwha_sot/hanwha_sot/src/mtt_msgs/msg/TargetCandidate.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Int8.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Int32.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/mtt_msgs
)

### Generating Services

### Generating Module File
_generate_module_eus(mtt_msgs
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/mtt_msgs
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(mtt_msgs_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(mtt_msgs_generate_messages mtt_msgs_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/changwon/data_2/hanwha_projects/hanwha_sot/hanwha_sot/src/mtt_msgs/msg/FollowTargetInfo.msg" NAME_WE)
add_dependencies(mtt_msgs_generate_messages_eus _mtt_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/changwon/data_2/hanwha_projects/hanwha_sot/hanwha_sot/src/mtt_msgs/msg/TargetCandidate.msg" NAME_WE)
add_dependencies(mtt_msgs_generate_messages_eus _mtt_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(mtt_msgs_geneus)
add_dependencies(mtt_msgs_geneus mtt_msgs_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS mtt_msgs_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(mtt_msgs
  "/home/changwon/data_2/hanwha_projects/hanwha_sot/hanwha_sot/src/mtt_msgs/msg/FollowTargetInfo.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Int8.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Int32.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Float32.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/mtt_msgs
)
_generate_msg_lisp(mtt_msgs
  "/home/changwon/data_2/hanwha_projects/hanwha_sot/hanwha_sot/src/mtt_msgs/msg/TargetCandidate.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Int8.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Int32.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/mtt_msgs
)

### Generating Services

### Generating Module File
_generate_module_lisp(mtt_msgs
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/mtt_msgs
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(mtt_msgs_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(mtt_msgs_generate_messages mtt_msgs_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/changwon/data_2/hanwha_projects/hanwha_sot/hanwha_sot/src/mtt_msgs/msg/FollowTargetInfo.msg" NAME_WE)
add_dependencies(mtt_msgs_generate_messages_lisp _mtt_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/changwon/data_2/hanwha_projects/hanwha_sot/hanwha_sot/src/mtt_msgs/msg/TargetCandidate.msg" NAME_WE)
add_dependencies(mtt_msgs_generate_messages_lisp _mtt_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(mtt_msgs_genlisp)
add_dependencies(mtt_msgs_genlisp mtt_msgs_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS mtt_msgs_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(mtt_msgs
  "/home/changwon/data_2/hanwha_projects/hanwha_sot/hanwha_sot/src/mtt_msgs/msg/FollowTargetInfo.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Int8.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Int32.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Float32.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/mtt_msgs
)
_generate_msg_nodejs(mtt_msgs
  "/home/changwon/data_2/hanwha_projects/hanwha_sot/hanwha_sot/src/mtt_msgs/msg/TargetCandidate.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Int8.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Int32.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/mtt_msgs
)

### Generating Services

### Generating Module File
_generate_module_nodejs(mtt_msgs
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/mtt_msgs
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(mtt_msgs_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(mtt_msgs_generate_messages mtt_msgs_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/changwon/data_2/hanwha_projects/hanwha_sot/hanwha_sot/src/mtt_msgs/msg/FollowTargetInfo.msg" NAME_WE)
add_dependencies(mtt_msgs_generate_messages_nodejs _mtt_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/changwon/data_2/hanwha_projects/hanwha_sot/hanwha_sot/src/mtt_msgs/msg/TargetCandidate.msg" NAME_WE)
add_dependencies(mtt_msgs_generate_messages_nodejs _mtt_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(mtt_msgs_gennodejs)
add_dependencies(mtt_msgs_gennodejs mtt_msgs_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS mtt_msgs_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(mtt_msgs
  "/home/changwon/data_2/hanwha_projects/hanwha_sot/hanwha_sot/src/mtt_msgs/msg/FollowTargetInfo.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Int8.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Int32.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Pose.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Quaternion.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Float32.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/mtt_msgs
)
_generate_msg_py(mtt_msgs
  "/home/changwon/data_2/hanwha_projects/hanwha_sot/hanwha_sot/src/mtt_msgs/msg/TargetCandidate.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/melodic/share/std_msgs/cmake/../msg/Int8.msg;/opt/ros/melodic/share/geometry_msgs/cmake/../msg/Point.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Int32.msg;/opt/ros/melodic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/mtt_msgs
)

### Generating Services

### Generating Module File
_generate_module_py(mtt_msgs
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/mtt_msgs
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(mtt_msgs_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(mtt_msgs_generate_messages mtt_msgs_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/changwon/data_2/hanwha_projects/hanwha_sot/hanwha_sot/src/mtt_msgs/msg/FollowTargetInfo.msg" NAME_WE)
add_dependencies(mtt_msgs_generate_messages_py _mtt_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/changwon/data_2/hanwha_projects/hanwha_sot/hanwha_sot/src/mtt_msgs/msg/TargetCandidate.msg" NAME_WE)
add_dependencies(mtt_msgs_generate_messages_py _mtt_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(mtt_msgs_genpy)
add_dependencies(mtt_msgs_genpy mtt_msgs_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS mtt_msgs_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/mtt_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/mtt_msgs
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(mtt_msgs_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()
if(TARGET geometry_msgs_generate_messages_cpp)
  add_dependencies(mtt_msgs_generate_messages_cpp geometry_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/mtt_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/mtt_msgs
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(mtt_msgs_generate_messages_eus std_msgs_generate_messages_eus)
endif()
if(TARGET geometry_msgs_generate_messages_eus)
  add_dependencies(mtt_msgs_generate_messages_eus geometry_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/mtt_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/mtt_msgs
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(mtt_msgs_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()
if(TARGET geometry_msgs_generate_messages_lisp)
  add_dependencies(mtt_msgs_generate_messages_lisp geometry_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/mtt_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/mtt_msgs
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(mtt_msgs_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()
if(TARGET geometry_msgs_generate_messages_nodejs)
  add_dependencies(mtt_msgs_generate_messages_nodejs geometry_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/mtt_msgs)
  install(CODE "execute_process(COMMAND \"/usr/bin/python2\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/mtt_msgs\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/mtt_msgs
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(mtt_msgs_generate_messages_py std_msgs_generate_messages_py)
endif()
if(TARGET geometry_msgs_generate_messages_py)
  add_dependencies(mtt_msgs_generate_messages_py geometry_msgs_generate_messages_py)
endif()
