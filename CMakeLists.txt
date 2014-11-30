project(ubxproto C)
cmake_minimum_required (VERSION 2.8)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

set(UBXPROTO_VERSION 14.0)

set(INSTALL_LIB_DIR usr/lib CACHE PATH "Installation directory for libraries")
set(INSTALL_INCLUDE_DIR usr/include CACHE PATH "Installation directory for headers")
set(INSTALL_DATA_DIR usr/share/${PROJECT_NAME} CACHE PATH "Installation directory for data files")

#Define include paths
include_directories(SYSTEM ${CMAKE_CURRENT_SOURCE_DIR})

#Set sources
set(SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/ubx.c)

#Compile object files for both shared and static libraries
add_library(ubxprotoobjects OBJECT ${SOURCES})

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fPIC")

set(PUBLIC_HEADER ${CMAKE_CURRENT_SOURCE_DIR}/ubxaid.h
                  ${CMAKE_CURRENT_SOURCE_DIR}/ubxcfg.h
                  ${CMAKE_CURRENT_SOURCE_DIR}/ubx.h
                  ${CMAKE_CURRENT_SOURCE_DIR}/ubxlog.h
                  ${CMAKE_CURRENT_SOURCE_DIR}/ubxmessage.h
                  ${CMAKE_CURRENT_SOURCE_DIR}/ubxmon.h
                  ${CMAKE_CURRENT_SOURCE_DIR}/ubxrxm.h
                  ${CMAKE_CURRENT_SOURCE_DIR}/ubxutils.h)

#Static library target
add_library(ubxprotostatic STATIC $<TARGET_OBJECTS:ubxprotoobjects>)
set_target_properties(ubxprotostatic PROPERTIES OUTPUT_NAME ubxproto)
set_target_properties(ubxprotostatic PROPERTIES LINKER_LANGUAGE C)
set_target_properties(ubxprotostatic PROPERTIES PUBLIC_HEADER "${PUBLIC_HEADER}")
set(PROJECT_TARGETS ubxprotostatic)

#Shared library target
add_library(ubxprotoshared SHARED $<TARGET_OBJECTS:ubxprotoobjects>)
set_target_properties(ubxprotoshared PROPERTIES LINKER_LANGUAGE C)
set_target_properties(ubxprotoshared PROPERTIES OUTPUT_NAME ubxproto)
set_target_properties(ubxprotoshared PROPERTIES SOVERSION ${UBXPROTO_VERSION})
set(PROJECT_TARGETS ${PROJECT_TARGETS} ubxprotoshared)

set(PROJECT_LIBS ubxproto)

install(TARGETS ${PROJECT_TARGETS}
        EXPORT ${PROJECT_NAME}Targets
        ARCHIVE DESTINATION "/${INSTALL_LIB_DIR}" COMPONENT staticlib
        LIBRARY DESTINATION "/${INSTALL_LIB_DIR}" COMPONENT sharedlib
        PUBLIC_HEADER DESTINATION "/${INSTALL_INCLUDE_DIR}/ubxproto"
        COMPONENT dev
        )

find_package(Doxygen)
if(DOXYGEN_FOUND)
  set(DOXYGEN_INPUT ${CMAKE_SOURCE_DIR})
  set(DOXYGEN_OUTPUT "${CMAKE_CURRENT_SOURCE_DIR}/doc")
  add_custom_target(doc COMMAND ${DOXYGEN_EXECUTABLE} ubxproto.doxyconf
                    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
                    COMMENT "Generating doxygen documenation")
  install(DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/doc" DESTINATION "/${INSTALL_DATA_DIR}" OPTIONAL)
else(DOXYGEN_FOUND)
  message(STATUS "WARNING: Doxygen not found - Reference manual will not be created")
endif(DOXYGEN_FOUND)

#Generate cmake find library addtion
include(CMakeGen)
