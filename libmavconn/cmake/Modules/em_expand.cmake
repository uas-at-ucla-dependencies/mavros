function(assert VAR)
  if(NOT ${VAR})
    message(FATAL_ERROR "\nAssertion failed: ${VAR} (value is '${${VAR}}')\n")
  endif()
  debug_message(3 "assert(${VAR}) passed (${VAR} = ${${VAR}})")
endfunction()

function(assert_unset VAR)
  if(${VAR})
    message(FATAL_ERROR "\nAssertion failed: '${VAR}' is set but should not be (value is '${${VAR}}')\n")
  endif()
  debug_message(3 "assert_unset(${VAR}) passed")
endfunction()

function(assert_file_exists FILENAME MESSAGE)
  if(NOT FILENAME)
    message(FATAL_ERROR "\nAssertion failed:  check for file existence, but filename (${FILENAME}) unset. Message: ${MESSAGE}\n")
  endif()
  if(NOT EXISTS ${FILENAME})
    message(FATAL_ERROR "\nAssertion failed:  file '${FILENAME}' does not exist.  Message: ${MESSAGE}\n")
  endif()
endfunction()

# Log levels
# 0 Normal use
# 1 Catkin developer use (Stuff being developed)
# 2 Catkin developer use (Stuff working)
# 3 Also Print True Assert Statements

macro(debug_message level)
  set(loglevel ${CATKIN_LOG})
  if(NOT loglevel)
    set(loglevel 0)
  endif()

  if(NOT ${level} GREATER ${loglevel})
    message(STATUS "  ${ARGN}")
  endif()
endmacro()

macro(safe_execute_process cmd_keyword arg1)
  set(_cmd ${arg1})
  foreach(_arg ${ARGN})
    set(_cmd "${_cmd} \"${_arg}\"")
  endforeach()

  debug_message(2 "execute_process(${_cmd})")
  execute_process(${ARGV} RESULT_VARIABLE _res)

  if(NOT _res EQUAL 0)
    message(FATAL_ERROR "execute_process(${_cmd}) returned error code ${_res}")
  endif()
endmacro()

function(find_python_module module)
  # cribbed from http://www.cmake.org/pipermail/cmake/2011-January/041666.html
  string(TOUPPER ${module} module_upper)
  if(NOT PY_${module_upper})
    if(ARGC GREATER 1 AND ARGV1 STREQUAL "REQUIRED")
      set(${module}_FIND_REQUIRED TRUE)
    endif()
    # A module's location is usually a directory, but for
    # binary modules
    # it's a .so file.
    execute_process(COMMAND "${PYTHON_EXECUTABLE}" "-c" "import re, ${module}; print(re.compile('/__init__.py.*').sub('',${module}.__file__))"
      RESULT_VARIABLE _${module}_status 
      OUTPUT_VARIABLE _${module}_location
      ERROR_QUIET OUTPUT_STRIP_TRAILING_WHITESPACE)
    if(NOT _${module}_status)
      set(PY_${module_upper} ${_${module}_location} CACHE STRING "Location of Python module ${module}")
    endif(NOT _${module}_status)
  endif(NOT PY_${module_upper})
  include(FindPackageHandleStandardArgs)
  find_package_handle_standard_args(PY_${module} DEFAULT_MSG PY_${module_upper})
endfunction(find_python_module)

find_program(EMPY_EXECUTABLE empy)
if(NOT EMPY_EXECUTABLE)
  # On OSX, there's an em.py, but not executable empy
  find_python_module(em)
  if(NOT PY_EM)
    message(FATAL_ERROR "Unable to find either executable 'empy' or Python module 'em'... try installing package 'python-empy'")
  else()
    file(TO_CMAKE_PATH "${PY_EM}" PY_EM)
    set(EMPY_EXECUTABLE "${PYTHON_EXECUTABLE};${PY_EM}" CACHE STRING "Executable string for empy" FORCE)
  endif()
else()
  # ensure to use cmake-style path separators on Windows
  file(TO_CMAKE_PATH "${EMPY_EXECUTABLE}" EMPY_EXECUTABLE)
endif()

macro(em_expand context_in context_out em_file_in file_out)
  assert_file_exists("${context_in}" "input file for context missing")
  assert_file_exists("${em_file_in}" "template file missing")
  debug_message(2 "configure_file(${context_in}, ${context_out})")
  configure_file(${context_in} ${context_out} @ONLY)
  assert_file_exists("${context_out}" "context file was not generated correctly")

  stamp(${em_file_in})

  # create directory if necessary
  get_filename_component(_folder_out ${file_out} PATH)
  if(NOT IS_DIRECTORY ${_folder_out})
    file(MAKE_DIRECTORY ${_folder_out})
  endif()

  debug_message(2 "Evaluate template '${em_file_in}' to '${file_out}' (with context from '${context_out}')")
  assert(EMPY_EXECUTABLE)
  set(command ${EMPY_EXECUTABLE})
  # prepend environment if set
  if(CATKIN_ENV)
    set(command ${CATKIN_ENV} ${command})
  endif()
  safe_execute_process(COMMAND
    ${command}
    --raw-errors
    -F ${context_out}
    -o ${file_out}
    ${em_file_in})
endmacro()
