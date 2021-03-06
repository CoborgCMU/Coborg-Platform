# generated from catkin/cmake/template/pkgConfig.cmake.in

# append elements to a list and remove existing duplicates from the list
# copied from catkin/cmake/list_append_deduplicate.cmake to keep pkgConfig
# self contained
macro(_list_append_deduplicate listname)
  if(NOT "${ARGN}" STREQUAL "")
    if(${listname})
      list(REMOVE_ITEM ${listname} ${ARGN})
    endif()
    list(APPEND ${listname} ${ARGN})
  endif()
endmacro()

# append elements to a list if they are not already in the list
# copied from catkin/cmake/list_append_unique.cmake to keep pkgConfig
# self contained
macro(_list_append_unique listname)
  foreach(_item ${ARGN})
    list(FIND ${listname} ${_item} _index)
    if(_index EQUAL -1)
      list(APPEND ${listname} ${_item})
    endif()
  endforeach()
endmacro()

# pack a list of libraries with optional build configuration keywords
# copied from catkin/cmake/catkin_libraries.cmake to keep pkgConfig
# self contained
macro(_pack_libraries_with_build_configuration VAR)
  set(${VAR} "")
  set(_argn ${ARGN})
  list(LENGTH _argn _count)
  set(_index 0)
  while(${_index} LESS ${_count})
    list(GET _argn ${_index} lib)
    if("${lib}" MATCHES "^(debug|optimized|general)$")
      math(EXPR _index "${_index} + 1")
      if(${_index} EQUAL ${_count})
        message(FATAL_ERROR "_pack_libraries_with_build_configuration() the list of libraries '${ARGN}' ends with '${lib}' which is a build configuration keyword and must be followed by a library")
      endif()
      list(GET _argn ${_index} library)
      list(APPEND ${VAR} "${lib}${CATKIN_BUILD_CONFIGURATION_KEYWORD_SEPARATOR}${library}")
    else()
      list(APPEND ${VAR} "${lib}")
    endif()
    math(EXPR _index "${_index} + 1")
  endwhile()
endmacro()

# unpack a list of libraries with optional build configuration keyword prefixes
# copied from catkin/cmake/catkin_libraries.cmake to keep pkgConfig
# self contained
macro(_unpack_libraries_with_build_configuration VAR)
  set(${VAR} "")
  foreach(lib ${ARGN})
    string(REGEX REPLACE "^(debug|optimized|general)${CATKIN_BUILD_CONFIGURATION_KEYWORD_SEPARATOR}(.+)$" "\\1;\\2" lib "${lib}")
    list(APPEND ${VAR} "${lib}")
  endforeach()
endmacro()


if(goal_getter_CONFIG_INCLUDED)
  return()
endif()
set(goal_getter_CONFIG_INCLUDED TRUE)

# set variables for source/devel/install prefixes
<<<<<<< HEAD:franka_ws/ws/robot-autonomy-labs/lab3/cv_bridge_catkin_ws/devel/share/opencv_tests/cmake/opencv_testsConfig.cmake
if("TRUE" STREQUAL "TRUE")
  set(opencv_tests_SOURCE_PREFIX /home/coborg/Coborg-Platform/franka_ws/ws/robot-autonomy-labs/lab3/cv_bridge_catkin_ws/src/vision_opencv/opencv_tests)
  set(opencv_tests_DEVEL_PREFIX /home/coborg/Coborg-Platform/franka_ws/ws/robot-autonomy-labs/lab3/cv_bridge_catkin_ws/devel)
  set(opencv_tests_INSTALL_PREFIX "")
  set(opencv_tests_PREFIX ${opencv_tests_DEVEL_PREFIX})
else()
  set(opencv_tests_SOURCE_PREFIX "")
  set(opencv_tests_DEVEL_PREFIX "")
  set(opencv_tests_INSTALL_PREFIX /home/coborg/Coborg-Platform/franka_ws/ws/robot-autonomy-labs/lab3/cv_bridge_catkin_ws/install)
  set(opencv_tests_PREFIX ${opencv_tests_INSTALL_PREFIX})
=======
if("FALSE" STREQUAL "TRUE")
  set(goal_getter_SOURCE_PREFIX /home/yuqing/Desktop/Coborg-Platform/catkin_ws/src/goal_getter)
  set(goal_getter_DEVEL_PREFIX /home/yuqing/Desktop/Coborg-Platform/catkin_ws/devel)
  set(goal_getter_INSTALL_PREFIX "")
  set(goal_getter_PREFIX ${goal_getter_DEVEL_PREFIX})
else()
  set(goal_getter_SOURCE_PREFIX "")
  set(goal_getter_DEVEL_PREFIX "")
  set(goal_getter_INSTALL_PREFIX /home/yuqing/Desktop/Coborg-Platform/catkin_ws/install)
  set(goal_getter_PREFIX ${goal_getter_INSTALL_PREFIX})
>>>>>>> 0ad6f6b835c25920092c265e79cc9ef3ac4c9b72:catkin_ws/install/share/goal_getter/cmake/goal_getterConfig.cmake
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "WARNING: package 'goal_getter' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  message("${_msg}")
endif()

# flag project as catkin-based to distinguish if a find_package()-ed project is a catkin project
set(goal_getter_FOUND_CATKIN_PROJECT TRUE)

if(NOT "include " STREQUAL " ")
  set(goal_getter_INCLUDE_DIRS "")
  set(_include_dirs "include")
  if(NOT " " STREQUAL " ")
    set(_report "Check the issue tracker '' and consider creating a ticket if the problem has not been reported yet.")
  elseif(NOT " " STREQUAL " ")
    set(_report "Check the website '' for information and consider reporting the problem.")
  else()
    set(_report "Report the problem to the maintainer 'Yuqing <yuqingq@andrew.cmu.edu>' and request to fix the problem.")
  endif()
  foreach(idir ${_include_dirs})
    if(IS_ABSOLUTE ${idir} AND IS_DIRECTORY ${idir})
      set(include ${idir})
    elseif("${idir} " STREQUAL "include ")
      get_filename_component(include "${goal_getter_DIR}/../../../include" ABSOLUTE)
      if(NOT IS_DIRECTORY ${include})
        message(FATAL_ERROR "Project 'goal_getter' specifies '${idir}' as an include dir, which is not found.  It does not exist in '${include}'.  ${_report}")
      endif()
    else()
<<<<<<< HEAD:franka_ws/ws/robot-autonomy-labs/lab3/cv_bridge_catkin_ws/devel/share/opencv_tests/cmake/opencv_testsConfig.cmake
      message(FATAL_ERROR "Project 'opencv_tests' specifies '${idir}' as an include dir, which is not found.  It does neither exist as an absolute directory nor in '/home/coborg/Coborg-Platform/franka_ws/ws/robot-autonomy-labs/lab3/cv_bridge_catkin_ws/src/vision_opencv/opencv_tests/${idir}'.  ${_report}")
=======
      message(FATAL_ERROR "Project 'goal_getter' specifies '${idir}' as an include dir, which is not found.  It does neither exist as an absolute directory nor in '\${prefix}/${idir}'.  ${_report}")
>>>>>>> 0ad6f6b835c25920092c265e79cc9ef3ac4c9b72:catkin_ws/install/share/goal_getter/cmake/goal_getterConfig.cmake
    endif()
    _list_append_unique(goal_getter_INCLUDE_DIRS ${include})
  endforeach()
endif()

set(libraries "")
foreach(library ${libraries})
  # keep build configuration keywords, target names and absolute libraries as-is
  if("${library}" MATCHES "^(debug|optimized|general)$")
    list(APPEND goal_getter_LIBRARIES ${library})
  elseif(${library} MATCHES "^-l")
    list(APPEND goal_getter_LIBRARIES ${library})
  elseif(${library} MATCHES "^-")
    # This is a linker flag/option (like -pthread)
    # There's no standard variable for these, so create an interface library to hold it
    if(NOT goal_getter_NUM_DUMMY_TARGETS)
      set(goal_getter_NUM_DUMMY_TARGETS 0)
    endif()
    # Make sure the target name is unique
    set(interface_target_name "catkin::goal_getter::wrapped-linker-option${goal_getter_NUM_DUMMY_TARGETS}")
    while(TARGET "${interface_target_name}")
      math(EXPR goal_getter_NUM_DUMMY_TARGETS "${goal_getter_NUM_DUMMY_TARGETS}+1")
      set(interface_target_name "catkin::goal_getter::wrapped-linker-option${goal_getter_NUM_DUMMY_TARGETS}")
    endwhile()
    add_library("${interface_target_name}" INTERFACE IMPORTED)
    if("${CMAKE_VERSION}" VERSION_LESS "3.13.0")
      set_property(
        TARGET
        "${interface_target_name}"
        APPEND PROPERTY
        INTERFACE_LINK_LIBRARIES "${library}")
    else()
      target_link_options("${interface_target_name}" INTERFACE "${library}")
    endif()
    list(APPEND goal_getter_LIBRARIES "${interface_target_name}")
  elseif(TARGET ${library})
    list(APPEND goal_getter_LIBRARIES ${library})
  elseif(IS_ABSOLUTE ${library})
    list(APPEND goal_getter_LIBRARIES ${library})
  else()
    set(lib_path "")
    set(lib "${library}-NOTFOUND")
    # since the path where the library is found is returned we have to iterate over the paths manually
<<<<<<< HEAD:franka_ws/ws/robot-autonomy-labs/lab3/cv_bridge_catkin_ws/devel/share/opencv_tests/cmake/opencv_testsConfig.cmake
    foreach(path /home/coborg/Coborg-Platform/franka_ws/ws/robot-autonomy-labs/lab3/cv_bridge_catkin_ws/devel/lib;/opt/ros/melodic/lib)
=======
<<<<<<< HEAD:catkin_ws/install/share/goal_getter/cmake/goal_getterConfig.cmake
    foreach(path /home/yuqing/Desktop/Coborg-Platform/catkin_ws/install/lib;/opt/ros/melodic/lib)
=======
    foreach(path /home/gerry/Coborg-Platform/franka_ws/ws/robot-autonomy-labs/lab3/cv_bridge_catkin_ws/devel/lib;/home/gerry/ProgrammingFamiliarization3/catkin_ws/devel/lib;/home/gerry/Coborg-Platform/franka_ws/ws/robot-autonomy-labs/lab3/cv_bridge_catkin_ws/devel/lib;/home/gerry/ProgrammingFamiliarization2/catkin_ws/devel/lib;/opt/ros/melodic/lib)
>>>>>>> abb0aa7f41d979e59569e0a642bc56114e9d142c:franka_ws/ws/robot-autonomy-labs/lab3/cv_bridge_catkin_ws/devel/share/cv_bridge/cmake/cv_bridgeConfig.cmake
>>>>>>> 0ad6f6b835c25920092c265e79cc9ef3ac4c9b72:catkin_ws/install/share/goal_getter/cmake/goal_getterConfig.cmake
      find_library(lib ${library}
        PATHS ${path}
        NO_DEFAULT_PATH NO_CMAKE_FIND_ROOT_PATH)
      if(lib)
        set(lib_path ${path})
        break()
      endif()
    endforeach()
    if(lib)
      _list_append_unique(goal_getter_LIBRARY_DIRS ${lib_path})
      list(APPEND goal_getter_LIBRARIES ${lib})
    else()
      # as a fall back for non-catkin libraries try to search globally
      find_library(lib ${library})
      if(NOT lib)
        message(FATAL_ERROR "Project '${PROJECT_NAME}' tried to find library '${library}'.  The library is neither a target nor built/installed properly.  Did you compile project 'goal_getter'?  Did you find_package() it before the subdirectory containing its code is included?")
      endif()
      list(APPEND goal_getter_LIBRARIES ${lib})
    endif()
  endif()
endforeach()

set(goal_getter_EXPORTED_TARGETS "goal_getter_generate_messages_cpp;goal_getter_generate_messages_eus;goal_getter_generate_messages_lisp;goal_getter_generate_messages_nodejs;goal_getter_generate_messages_py")
# create dummy targets for exported code generation targets to make life of users easier
foreach(t ${goal_getter_EXPORTED_TARGETS})
  if(NOT TARGET ${t})
    add_custom_target(${t})
  endif()
endforeach()

set(depends "message_runtime")
foreach(depend ${depends})
  string(REPLACE " " ";" depend_list ${depend})
  # the package name of the dependency must be kept in a unique variable so that it is not overwritten in recursive calls
  list(GET depend_list 0 goal_getter_dep)
  list(LENGTH depend_list count)
  if(${count} EQUAL 1)
    # simple dependencies must only be find_package()-ed once
    if(NOT ${goal_getter_dep}_FOUND)
      find_package(${goal_getter_dep} REQUIRED NO_MODULE)
    endif()
  else()
    # dependencies with components must be find_package()-ed again
    list(REMOVE_AT depend_list 0)
    find_package(${goal_getter_dep} REQUIRED NO_MODULE ${depend_list})
  endif()
  _list_append_unique(goal_getter_INCLUDE_DIRS ${${goal_getter_dep}_INCLUDE_DIRS})

  # merge build configuration keywords with library names to correctly deduplicate
  _pack_libraries_with_build_configuration(goal_getter_LIBRARIES ${goal_getter_LIBRARIES})
  _pack_libraries_with_build_configuration(_libraries ${${goal_getter_dep}_LIBRARIES})
  _list_append_deduplicate(goal_getter_LIBRARIES ${_libraries})
  # undo build configuration keyword merging after deduplication
  _unpack_libraries_with_build_configuration(goal_getter_LIBRARIES ${goal_getter_LIBRARIES})

  _list_append_unique(goal_getter_LIBRARY_DIRS ${${goal_getter_dep}_LIBRARY_DIRS})
  list(APPEND goal_getter_EXPORTED_TARGETS ${${goal_getter_dep}_EXPORTED_TARGETS})
endforeach()

set(pkg_cfg_extras "goal_getter-msg-extras.cmake")
foreach(extra ${pkg_cfg_extras})
  if(NOT IS_ABSOLUTE ${extra})
    set(extra ${goal_getter_DIR}/${extra})
  endif()
  include(${extra})
endforeach()
