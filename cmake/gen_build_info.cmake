# Usage:
#   cmake -DOUTPUT_FILE=... -DSOURCE_DIR=... -P cmake/gen_build_info.cmake

if(NOT DEFINED OUTPUT_FILE)
  message(FATAL_ERROR "OUTPUT_FILE is not set")
endif()
if(NOT DEFINED SOURCE_DIR)
  set(SOURCE_DIR "${CMAKE_CURRENT_LIST_DIR}/..")
endif()

if(NOT DEFINED FW_TARGET)
  set(FW_TARGET "unknown")
endif()
if(NOT DEFINED FW_BUILD_TYPE)
  set(FW_BUILD_TYPE "unknown")
endif()

string(TIMESTAMP BUILD_TIME_UTC "%Y-%m-%dT%H:%M:%SZ" UTC)

function(_run_git out_var)
  set(options)
  set(oneValueArgs)
  set(multiValueArgs ARGS)
  cmake_parse_arguments(RG "" "" "ARGS" ${ARGN})

  execute_process(
    COMMAND git ${RG_ARGS}
    WORKING_DIRECTORY "${SOURCE_DIR}"
    RESULT_VARIABLE _rv
    OUTPUT_VARIABLE _out
    ERROR_VARIABLE _err
    OUTPUT_STRIP_TRAILING_WHITESPACE
    ERROR_STRIP_TRAILING_WHITESPACE
  )

  if(NOT _rv EQUAL 0)
    set(_out "unknown")
  endif()

  set(${out_var} "${_out}" PARENT_SCOPE)
endfunction()

_run_git(GIT_DESCRIBE ARGS describe --tags --always --dirty)
_run_git(GIT_SHA ARGS rev-parse --short HEAD)
_run_git(GIT_BRANCH ARGS rev-parse --abbrev-ref HEAD)

execute_process(
  COMMAND git status --porcelain
  WORKING_DIRECTORY "${SOURCE_DIR}"
  RESULT_VARIABLE _st_rv
  OUTPUT_VARIABLE _st_out
  ERROR_VARIABLE _st_err
  OUTPUT_STRIP_TRAILING_WHITESPACE
  ERROR_STRIP_TRAILING_WHITESPACE
)
if(NOT _st_rv EQUAL 0)
  set(GIT_DIRTY 0)
else()
  if(_st_out STREQUAL "")
    set(GIT_DIRTY 0)
  else()
    set(GIT_DIRTY 1)
  endif()
endif()

set(_content "")
string(APPEND _content "#ifndef NIGHTFALL_BUILD_INFO_H_\n")
string(APPEND _content "#define NIGHTFALL_BUILD_INFO_H_\n\n")
string(APPEND _content "#define FW_TARGET \"${FW_TARGET}\"\n")
string(APPEND _content "#define FW_BUILD_TYPE \"${FW_BUILD_TYPE}\"\n")
string(APPEND _content "#define FW_BUILD_TIME_UTC \"${BUILD_TIME_UTC}\"\n\n")
string(APPEND _content "#define FW_GIT_DESCRIBE \"${GIT_DESCRIBE}\"\n")
string(APPEND _content "#define FW_GIT_SHA \"${GIT_SHA}\"\n")
string(APPEND _content "#define FW_GIT_BRANCH \"${GIT_BRANCH}\"\n")
string(APPEND _content "#define FW_GIT_DIRTY ${GIT_DIRTY}\n\n")
string(APPEND _content "#endif\n")

get_filename_component(_out_dir "${OUTPUT_FILE}" DIRECTORY)
file(MAKE_DIRECTORY "${_out_dir}")

set(_write 1)
if(EXISTS "${OUTPUT_FILE}")
  file(READ "${OUTPUT_FILE}" _old)
  if(_old STREQUAL _content)
    set(_write 0)
  endif()
endif()

if(_write)
  file(WRITE "${OUTPUT_FILE}" "${_content}")
endif()
