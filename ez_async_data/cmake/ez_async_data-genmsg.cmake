# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "ez_async_data: 1 messages, 0 services")

set(MSG_I_FLAGS "-Iez_async_data:/u02/vnproglib-1.1/cpp/examples/ez_async_data/msg;-Istd_msgs:/opt/ros/lunar/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(ez_async_data_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/u02/vnproglib-1.1/cpp/examples/ez_async_data/msg/Rotation.msg" NAME_WE)
add_custom_target(_ez_async_data_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "ez_async_data" "/u02/vnproglib-1.1/cpp/examples/ez_async_data/msg/Rotation.msg" ""
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(ez_async_data
  "/u02/vnproglib-1.1/cpp/examples/ez_async_data/msg/Rotation.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/ez_async_data
)

### Generating Services

### Generating Module File
_generate_module_cpp(ez_async_data
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/ez_async_data
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(ez_async_data_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(ez_async_data_generate_messages ez_async_data_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/u02/vnproglib-1.1/cpp/examples/ez_async_data/msg/Rotation.msg" NAME_WE)
add_dependencies(ez_async_data_generate_messages_cpp _ez_async_data_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(ez_async_data_gencpp)
add_dependencies(ez_async_data_gencpp ez_async_data_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS ez_async_data_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(ez_async_data
  "/u02/vnproglib-1.1/cpp/examples/ez_async_data/msg/Rotation.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/ez_async_data
)

### Generating Services

### Generating Module File
_generate_module_eus(ez_async_data
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/ez_async_data
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(ez_async_data_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(ez_async_data_generate_messages ez_async_data_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/u02/vnproglib-1.1/cpp/examples/ez_async_data/msg/Rotation.msg" NAME_WE)
add_dependencies(ez_async_data_generate_messages_eus _ez_async_data_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(ez_async_data_geneus)
add_dependencies(ez_async_data_geneus ez_async_data_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS ez_async_data_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(ez_async_data
  "/u02/vnproglib-1.1/cpp/examples/ez_async_data/msg/Rotation.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/ez_async_data
)

### Generating Services

### Generating Module File
_generate_module_lisp(ez_async_data
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/ez_async_data
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(ez_async_data_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(ez_async_data_generate_messages ez_async_data_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/u02/vnproglib-1.1/cpp/examples/ez_async_data/msg/Rotation.msg" NAME_WE)
add_dependencies(ez_async_data_generate_messages_lisp _ez_async_data_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(ez_async_data_genlisp)
add_dependencies(ez_async_data_genlisp ez_async_data_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS ez_async_data_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(ez_async_data
  "/u02/vnproglib-1.1/cpp/examples/ez_async_data/msg/Rotation.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/ez_async_data
)

### Generating Services

### Generating Module File
_generate_module_nodejs(ez_async_data
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/ez_async_data
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(ez_async_data_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(ez_async_data_generate_messages ez_async_data_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/u02/vnproglib-1.1/cpp/examples/ez_async_data/msg/Rotation.msg" NAME_WE)
add_dependencies(ez_async_data_generate_messages_nodejs _ez_async_data_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(ez_async_data_gennodejs)
add_dependencies(ez_async_data_gennodejs ez_async_data_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS ez_async_data_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(ez_async_data
  "/u02/vnproglib-1.1/cpp/examples/ez_async_data/msg/Rotation.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/ez_async_data
)

### Generating Services

### Generating Module File
_generate_module_py(ez_async_data
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/ez_async_data
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(ez_async_data_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(ez_async_data_generate_messages ez_async_data_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/u02/vnproglib-1.1/cpp/examples/ez_async_data/msg/Rotation.msg" NAME_WE)
add_dependencies(ez_async_data_generate_messages_py _ez_async_data_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(ez_async_data_genpy)
add_dependencies(ez_async_data_genpy ez_async_data_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS ez_async_data_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/ez_async_data)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/ez_async_data
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(ez_async_data_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/ez_async_data)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/ez_async_data
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(ez_async_data_generate_messages_eus std_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/ez_async_data)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/ez_async_data
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(ez_async_data_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/ez_async_data)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/ez_async_data
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(ez_async_data_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/ez_async_data)
  install(CODE "execute_process(COMMAND \"/usr/bin/python\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/ez_async_data\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/ez_async_data
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(ez_async_data_generate_messages_py std_msgs_generate_messages_py)
endif()
