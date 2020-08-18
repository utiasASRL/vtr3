############################################################################################################
## @brief CmakeFunction: GENERATE_PROTO: Generates protobuf messages and puts them in the directory defined
##                       by ${GEN_DIR}
## @param List of generated python files to put into a custom target
## @param The output directory for generated messages
## @param A list of the protobuf messages to be converted to Python
## @param Protobuf files to compile
############################################################################################################
function(GENERATE_PROTO_PYTHON PY_FILES GEN_DIR SRC_DIR)
  if(NOT ARGN)
    message(SEND_ERROR "Error: PROTOBUF_GENERATE_PYTHON() called without any proto files")
    return()
  endif(NOT ARGN)

  # Create an include path for each file specified
  foreach(ABS_FIL ${ARGN})
    file(RELATIVE_PATH FIL ${SRC_DIR} ${ABS_FIL})
    get_filename_component(ABS_PATH ${ABS_FIL} PATH)
    get_filename_component(REL_PATH ${FIL} PATH)

    list(FIND _protobuf_include_path ${ABS_PATH} _contains_already)

    if(${_contains_already} EQUAL -1)
      list(APPEND _protobuf_include_path -I ${ABS_PATH})
      set(_protobuf_includes "${_protobuf_includes} -I ${ABS_PATH}")
      file(MAKE_DIRECTORY ${GEN_DIR}/${REL_PATH})
      file(WRITE ${GEN_DIR}/${REL_PATH}/__init__.py "")
    endif()
  endforeach()

  # # set(_protobuf_includes "${_protobuf_includes} -I /usr/local/include/robochunk/msgs/proto/")
  # # list(APPEND _protobuf_include_path -I "/usr/local/include/robochunk/msgs/proto/")
  # if (${robochunk_msgs_FOUND})
  #   list(APPEND _protobuf_includes -I "${robochunk_msgs_DEVEL_PREFIX}/include/robochunk_msgs/")
  #   list(APPEND _protobuf_include_path -I "${robochunk_msgs_DEVEL_PREFIX}/include/robochunk_msgs/")
  # endif()
  # # Add the include proto paths : babelfish_non_primitive
  # if (${babelfish_non_primitive_FOUND})
  #   list(APPEND _protobuf_includes -I "${babelfish_non_primitive_SOURCE_PREFIX}/proto/")
  #   list(APPEND _protobuf_include_path -I "${babelfish_non_primitive_SOURCE_PREFIX}/proto/")
  # endif()
  # # Add the include proto paths : babelfish_rtp
  # if (${babelfish_non_primitive_FOUND})
  #   list(APPEND _protobuf_includes -I "${babel_rtp_robochunk_SOURCE_PREFIX}/proto/")
  #   list(APPEND _protobuf_include_path -I "${babel_rtp_robochunk_SOURCE_PREFIX}/proto/")
  # endif()

  file(MAKE_DIRECTORY ${GEN_DIR})
  if(NOT EXISTS ${GEN_DIR}/__init__.py)
    file(WRITE ${GEN_DIR}/__init__.py "")
  endif()

  foreach(ABS_FIL ${ARGN})
    file(RELATIVE_PATH FIL ${SRC_DIR} ${ABS_FIL})
    get_filename_component(FIL_WE ${FIL} NAME_WE)
    get_filename_component(REL_PATH ${FIL} PATH)

    list(APPEND ${PY_FILES} "${GEN_DIR}/${REL_PATH}/${FIL_WE}_pb2.py")

    add_custom_command(
      OUTPUT "${GEN_DIR}/${REL_PATH}/${FIL_WE}_pb2.py"
      COMMAND  ${PROTOBUF_PROTOC_EXECUTABLE}
      ARGS --python_out  ${GEN_DIR}/${REL_PATH} ${_protobuf_include_path} ${ABS_FIL}
      DEPENDS ${ABS_FIL}
      COMMENT "Running Python protocol buffer compiler: ${FIL} --> ${GEN_DIR}/${REL_PATH}/${FIL_WE}_pb2.py")
  endforeach()

  set_source_files_properties(${${PY_FILES}} PROPERTIES GENERATED TRUE)
  set(${PY_FILES} ${${PY_FILES}} PARENT_SCOPE)
endfunction()
