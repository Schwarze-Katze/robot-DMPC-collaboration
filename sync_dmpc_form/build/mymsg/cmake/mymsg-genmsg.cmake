# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "mymsg: 1 messages, 0 services")

set(MSG_I_FLAGS "-Imymsg:/home/uav/catkin_ws/src/Multi_Robots_DMPC/sync_dmpc_form/src/mymsg/msg;-Istd_msgs:/opt/ros/melodic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(mymsg_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/uav/catkin_ws/src/Multi_Robots_DMPC/sync_dmpc_form/src/mymsg/msg/neighborpos.msg" NAME_WE)
add_custom_target(_mymsg_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "mymsg" "/home/uav/catkin_ws/src/Multi_Robots_DMPC/sync_dmpc_form/src/mymsg/msg/neighborpos.msg" ""
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(mymsg
  "/home/uav/catkin_ws/src/Multi_Robots_DMPC/sync_dmpc_form/src/mymsg/msg/neighborpos.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/mymsg
)

### Generating Services

### Generating Module File
_generate_module_cpp(mymsg
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/mymsg
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(mymsg_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(mymsg_generate_messages mymsg_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/uav/catkin_ws/src/Multi_Robots_DMPC/sync_dmpc_form/src/mymsg/msg/neighborpos.msg" NAME_WE)
add_dependencies(mymsg_generate_messages_cpp _mymsg_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(mymsg_gencpp)
add_dependencies(mymsg_gencpp mymsg_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS mymsg_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(mymsg
  "/home/uav/catkin_ws/src/Multi_Robots_DMPC/sync_dmpc_form/src/mymsg/msg/neighborpos.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/mymsg
)

### Generating Services

### Generating Module File
_generate_module_eus(mymsg
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/mymsg
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(mymsg_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(mymsg_generate_messages mymsg_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/uav/catkin_ws/src/Multi_Robots_DMPC/sync_dmpc_form/src/mymsg/msg/neighborpos.msg" NAME_WE)
add_dependencies(mymsg_generate_messages_eus _mymsg_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(mymsg_geneus)
add_dependencies(mymsg_geneus mymsg_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS mymsg_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(mymsg
  "/home/uav/catkin_ws/src/Multi_Robots_DMPC/sync_dmpc_form/src/mymsg/msg/neighborpos.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/mymsg
)

### Generating Services

### Generating Module File
_generate_module_lisp(mymsg
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/mymsg
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(mymsg_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(mymsg_generate_messages mymsg_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/uav/catkin_ws/src/Multi_Robots_DMPC/sync_dmpc_form/src/mymsg/msg/neighborpos.msg" NAME_WE)
add_dependencies(mymsg_generate_messages_lisp _mymsg_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(mymsg_genlisp)
add_dependencies(mymsg_genlisp mymsg_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS mymsg_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(mymsg
  "/home/uav/catkin_ws/src/Multi_Robots_DMPC/sync_dmpc_form/src/mymsg/msg/neighborpos.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/mymsg
)

### Generating Services

### Generating Module File
_generate_module_nodejs(mymsg
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/mymsg
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(mymsg_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(mymsg_generate_messages mymsg_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/uav/catkin_ws/src/Multi_Robots_DMPC/sync_dmpc_form/src/mymsg/msg/neighborpos.msg" NAME_WE)
add_dependencies(mymsg_generate_messages_nodejs _mymsg_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(mymsg_gennodejs)
add_dependencies(mymsg_gennodejs mymsg_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS mymsg_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(mymsg
  "/home/uav/catkin_ws/src/Multi_Robots_DMPC/sync_dmpc_form/src/mymsg/msg/neighborpos.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/mymsg
)

### Generating Services

### Generating Module File
_generate_module_py(mymsg
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/mymsg
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(mymsg_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(mymsg_generate_messages mymsg_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/uav/catkin_ws/src/Multi_Robots_DMPC/sync_dmpc_form/src/mymsg/msg/neighborpos.msg" NAME_WE)
add_dependencies(mymsg_generate_messages_py _mymsg_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(mymsg_genpy)
add_dependencies(mymsg_genpy mymsg_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS mymsg_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/mymsg)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/mymsg
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(mymsg_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/mymsg)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/mymsg
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(mymsg_generate_messages_eus std_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/mymsg)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/mymsg
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(mymsg_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/mymsg)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/mymsg
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(mymsg_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/mymsg)
  install(CODE "execute_process(COMMAND \"/usr/bin/python2\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/mymsg\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/mymsg
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(mymsg_generate_messages_py std_msgs_generate_messages_py)
endif()
