# Install script for directory: /home/jharbin/academic/atlas/git/atlas-middleware/custom-moos/MOOS/MOOSCore/Core/libMOOS

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/usr/local")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "Release")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "0")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/." TYPE DIRECTORY FILES
    "/home/jharbin/academic/atlas/git/atlas-middleware/custom-moos/MOOS/MOOSCore/Core/libMOOS/include"
    "/home/jharbin/academic/atlas/git/atlas-middleware/custom-moos/MOOS/MOOSCore/Core/libMOOS/App/include"
    "/home/jharbin/academic/atlas/git/atlas-middleware/custom-moos/MOOS/MOOSCore/Core/libMOOS/Comms/include"
    "/home/jharbin/academic/atlas/git/atlas-middleware/custom-moos/MOOS/MOOSCore/Core/libMOOS/DB/include"
    "/home/jharbin/academic/atlas/git/atlas-middleware/custom-moos/MOOS/MOOSCore/Core/libMOOS/Utils/include"
    "/home/jharbin/academic/atlas/git/atlas-middleware/custom-moos/MOOS/MOOSCore/Core/libMOOS/Thirdparty/PocoBits/include"
    "/home/jharbin/academic/atlas/git/atlas-middleware/custom-moos/MOOS/MOOSCore/Core/libMOOS/Thirdparty/getpot/include"
    "/home/jharbin/academic/atlas/git/atlas-middleware/custom-moos/MOOS/MOOSCore/Core/libMOOS/Thirdparty/AppCasting/include"
    FILES_MATCHING REGEX "/[^/]*\\.h$" REGEX "/[^/]*\\.hxx$" REGEX "/[^/]*\\.hpp$")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE STATIC_LIBRARY FILES "/home/jharbin/academic/atlas/git/atlas-middleware/custom-moos/build/MOOS/MOOSCore/lib/libMOOS.a")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  include("/home/jharbin/academic/atlas/git/atlas-middleware/custom-moos/build/MOOS/MOOSCore/Core/libMOOS/testing/cmake_install.cmake")

endif()

