set(_selfdir_CMakeGen
"${CMAKE_CURRENT_SOURCE_DIR}/cmake")

if((WIN32) AND (NOT CYGWIN))
  set(DEF_INSTALL_CMAKE_DIR CMake)
else()
  set(DEF_INSTALL_CMAKE_DIR usr/lib/cmake/${PROJECT_NAME})
endif()
set(INSTALL_CMAKE_DIR ${DEF_INSTALL_CMAKE_DIR} CACHE PATH
  "Installation directory for CMake files")
  
# Make relative paths absolute (needed later on)
foreach(p LIB BIN INCLUDE CMAKE)
  set(var INSTALL_${p}_DIR)
  if((DEFINED ${var}) AND (NOT IS_ABSOLUTE "${${var}}"))
    set(${var} "${SYSROOT}/${${var}}")
  endif()
endforeach()

# Add all targets to the build-tree export set
export(TARGETS ${PROJECT_TARGETS}
  FILE "${PROJECT_BINARY_DIR}/${PROJECT_NAME}Targets.cmake")

# Export the package for use from the build-tree
# (this registers the build-tree with a global CMake-registry)
export(PACKAGE ${PROJECT_NAME})

# Create the ${PROJECT_NAME}Config.cmake and ${PROJECT_NAME}ConfigVersion files
file(RELATIVE_PATH REL_INCLUDE_DIR "${INSTALL_CMAKE_DIR}"
   "${INSTALL_INCLUDE_DIR}")
# ... for the build tree
set(CONF_INCLUDE_DIRS "${INSTALL_INCLUDE_DIR}")
configure_file(${_selfdir_CMakeGen}/Config.cmake.in
  "${PROJECT_BINARY_DIR}/${PROJECT_NAME}Config.cmake" @ONLY)
# ... for the install tree
string(TOUPPER ${PROJECT_NAME} PROJECT_NAME_UPPER)
set(CONF_INCLUDE_DIRS "\${${PROJECT_NAME_UPPER}_CMAKE_DIR}/${REL_INCLUDE_DIR}")
configure_file(${_selfdir_CMakeGen}/Config.cmake.in
  "${PROJECT_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/${PROJECT_NAME}Config.cmake" @ONLY)

# ... for both
configure_file(${_selfdir_CMakeGen}/ConfigVersion.cmake.in
  "${PROJECT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake" @ONLY)
  
# Install the <PROJECT_NAME>.cmake and <PROJECT_NAME>.cmake
install(FILES
  "${PROJECT_BINARY_DIR}${CMAKE_FILES_DIRECTORY}/${PROJECT_NAME}Config.cmake"
  "${PROJECT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake"
  DESTINATION "${INSTALL_CMAKE_DIR}" COMPONENT dev)

# Install the export set for use with the install-tree
install(EXPORT ${PROJECT_NAME}Targets DESTINATION
  "${INSTALL_CMAKE_DIR}" COMPONENT dev)