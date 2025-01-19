########################################################################
#
# Fobos SDR project
# V.T.
# LGPL
#
########################################################################
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 2 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.

########################################################################
# Project setup
########################################################################
cmake_minimum_required(VERSION 3.7.2)

if(${CMAKE_VERSION} VERSION_LESS "3.12.0")
    project(fobos)
else()
    project(fobos C)
endif()

include(GNUInstallDirs)
message(>>> "CMAKE_INSTALL_LIBDIR: " ${CMAKE_INSTALL_LIBDIR})
set(CMAKE_C_FLAGS, "${CMAKE_C_FLAGS} -std=c99")
########################################################################

########################################################################
# Select the release build type by default to get optimization flags
########################################################################
if(NOT CMAKE_BUILD_TYPE)
   set(CMAKE_BUILD_TYPE "Release")
   message(STATUS "Build type not specified: defaulting to release.")
endif(NOT CMAKE_BUILD_TYPE)
########################################################################

########################################################################
# Compiler specific setup
########################################################################
if(CMAKE_COMPILER_IS_GNUCC AND NOT WIN32)
    ADD_DEFINITIONS(-Wall)
    ADD_DEFINITIONS(-Wextra)
    ADD_DEFINITIONS(-Wno-unused-parameter)
    ADD_DEFINITIONS(-Wno-unused)
    ADD_DEFINITIONS(-Wsign-compare)
    ADD_DEFINITIONS(-ansi)
elseif(MSVC14 OR MSVC14)
    ADD_DEFINITIONS(-D_CRT_SECURE_NO_WARNINGS)
    ADD_DEFINITIONS(-D_TIMESPEC_DEFINED)
endif()
########################################################################

########################################################################
# Find build dependencies
########################################################################
find_package(PkgConfig)
if(PKG_CONFIG_FOUND)
    pkg_check_modules(LIBUSB libusb-1.0 IMPORTED_TARGET)
    if(LIBUSB_LINK_LIBRARIES)
        set(LIBUSB_LIBRARIES "${LIBUSB_LINK_LIBRARIES}")
    endif()
else()
if (CMAKE_VS_PLATFORM_NAME STREQUAL Win32)
	set(LIBUSB_LIBRARIES ${CMAKE_SOURCE_DIR}/libusb/MS32/dll)
endif()
if (CMAKE_VS_PLATFORM_NAME STREQUAL x64)
    set(LIBUSB_LIBRARIES ${CMAKE_SOURCE_DIR}/libusb/MS64/dll)
endif()
    set(LIBUSB_INCLUDE_DIRS ${CMAKE_SOURCE_DIR}/libusb/include)
endif()
if(PKG_CONFIG_FOUND AND NOT LIBUSB_FOUND)
    message(FATAL_ERROR "LibUSB 1.0 required")
endif()
########################################################################

########################################################################
# Install udev rules
########################################################################
install(
    FILES fobos-sdr.rules
    DESTINATION "/etc/udev/rules.d"
    COMPONENT "udev"
    )
########################################################################

########################################################################
# Print Summary
########################################################################
message(>>> "Build: " ${CMAKE_BUILD_TYPE})
message(>>> "OS: " ${CMAKE_SYSTEM_NAME})
message(>>> "VS Platform: " ${CMAKE_VS_PLATFORM_NAME})
message(>>> "LIBUSB_LIBRARIES: " ${LIBUSB_LIBRARIES})
message(>>> "LIBUSB_INCLUDE_DIRS: " ${LIBUSB_INCLUDE_DIRS})
message(>>> "PROJECT_BINARY_DIR: " ${PROJECT_BINARY_DIR})

########################################################################
# Fobos SDR library
########################################################################
add_library(libfobos SHARED fobos/fobos.c)

target_compile_options(libfobos PUBLIC -std=c99)

if(MSVC)
target_compile_options(libfobos PUBLIC "/MT")
ADD_CUSTOM_COMMAND(TARGET libfobos
          POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E make_directory ${PROJECT_BINARY_DIR}/Debug
          COMMAND ${CMAKE_COMMAND} -E copy ${LIBUSB_LIBRARIES}/libusb-1.0.dll ${PROJECT_BINARY_DIR}/Debug
)
ADD_CUSTOM_COMMAND(TARGET libfobos
          POST_BUILD
          COMMAND ${CMAKE_COMMAND} -E make_directory ${PROJECT_BINARY_DIR}/Release
          COMMAND ${CMAKE_COMMAND} -E copy ${LIBUSB_LIBRARIES}/libusb-1.0.dll ${PROJECT_BINARY_DIR}/Release
)
else()
endif()

if(CMAKE_COMPILER_IS_GNUCC AND NOT WIN32)
    target_link_libraries(libfobos PRIVATE ${LIBUSB_LIBRARIES})
else()
    target_link_directories(libfobos PRIVATE ${LIBUSB_LIBRARIES})
endif()

target_include_directories(libfobos PUBLIC
  $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}>
  $<INSTALL_INTERFACE:${CMAKE_SOURCE_DIR}>
  $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}>/fobos
  ${LIBUSB_INCLUDE_DIRS}
  )
  
set_target_properties(libfobos PROPERTIES DEFINE_SYMBOL "FOBOS_EXPORTS")
set_target_properties(libfobos PROPERTIES OUTPUT_NAME fobos)
########################################################################

########################################################################
# Install public header files
########################################################################
INSTALL(
    FILES fobos/fobos.h
    DESTINATION include
)
########################################################################

########################################################################
# Install built library files & utilities
########################################################################
INSTALL(
    TARGETS libfobos
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
) 
########################################################################

########################################################################
# Create Pkg Config File
########################################################################
FOREACH(inc ${LIBUSB_INCLUDEDIR})
    LIST(APPEND LIBFOBOS_PC_CFLAGS "-I${inc}")
ENDFOREACH(inc)

FOREACH(lib ${LIBUSB_LIBRARY_DIRS})
    LIST(APPEND LIBFOBOS_PC_LIBS "-L${lib}")
ENDFOREACH(lib)

# use space-separation format for the pc file
STRING(REPLACE ";" " " LIBFOBOS_PC_CFLAGS "${LIBFOBOS_PC_CFLAGS}")
STRING(REPLACE ";" " " LIBFOBOS_PC_LIBS "${LIBFOBOS_PC_LIBS}")

set(prefix "${CMAKE_INSTALL_PREFIX}")
set(exec_prefix \${prefix})
set(includedir \${prefix}/include)
set(libdir \${exec_prefix}/lib)

CONFIGURE_FILE(
    ${CMAKE_CURRENT_SOURCE_DIR}/libfobos.pc.in
    ${CMAKE_CURRENT_BINARY_DIR}/libfobos.pc
@ONLY)

INSTALL(
    FILES ${CMAKE_CURRENT_BINARY_DIR}/libfobos.pc
    DESTINATION ${CMAKE_INSTALL_LIBDIR}/pkgconfig
)
########################################################################

########################################################################
# Create uninstall target
########################################################################
configure_file(
    ${CMAKE_SOURCE_DIR}/cmake/cmake_uninstall.cmake.in
    ${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake
@ONLY)

add_custom_target(uninstall
    ${CMAKE_COMMAND} -P ${CMAKE_CURRENT_BINARY_DIR}/cmake_uninstall.cmake
)
########################################################################

########################################################################
# Create executable fobos_devinfo
########################################################################
add_executable(fobos_devinfo
    eval/fobos_devinfo_main.c)

target_compile_options(fobos_devinfo PUBLIC -std=c99)

target_link_libraries(fobos_devinfo PRIVATE libfobos)

if(CMAKE_COMPILER_IS_GNUCC AND NOT WIN32)
	target_link_libraries(fobos_devinfo PRIVATE ${LIBUSB_LIBRARIES})
else()
	target_link_directories(fobos_devinfo PRIVATE ${LIBUSB_LIBRARIES})
endif()

target_include_directories(fobos_devinfo PRIVATE ${CMAKE_SOURCE_DIR})

target_include_directories(fobos_devinfo PRIVATE ${CMAKE_SOURCE_DIR}/fobos)
########################################################################

########################################################################
# Create executable fobos_recorder
########################################################################
add_executable(fobos_recorder 
    eval/fobos_recorder_main.c 
    wav/wav_file.c)

target_compile_options(fobos_recorder PUBLIC -std=c99)

target_link_libraries(fobos_recorder PRIVATE libfobos)

if(CMAKE_COMPILER_IS_GNUCC AND NOT WIN32)
    target_link_libraries(fobos_recorder PRIVATE ${LIBUSB_LIBRARIES})
else()
    target_link_directories(fobos_recorder PRIVATE ${LIBUSB_LIBRARIES})
endif()

target_include_directories(fobos_recorder PRIVATE ${CMAKE_SOURCE_DIR})

target_include_directories(fobos_recorder PRIVATE ${CMAKE_SOURCE_DIR}/fobos)
########################################################################

########################################################################
# Create executable fobos_fwloader
########################################################################
add_executable(fobos_fwloader
    eval/fobos_fwloader_main.c)

target_compile_options(fobos_fwloader PUBLIC -std=c99)

target_link_libraries(fobos_fwloader PRIVATE libfobos)

if(CMAKE_COMPILER_IS_GNUCC AND NOT WIN32)
	target_link_libraries(fobos_fwloader PRIVATE ${LIBUSB_LIBRARIES})
else()
	target_link_directories(fobos_fwloader PRIVATE ${LIBUSB_LIBRARIES})
endif()

target_include_directories(fobos_fwloader PRIVATE ${CMAKE_SOURCE_DIR})

target_include_directories(fobos_fwloader PRIVATE ${CMAKE_SOURCE_DIR}/fobos)
########################################################################


########################################################################
# Directories
########################################################################
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
########################################################################
