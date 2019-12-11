# Install script for directory: /home/jharbin/academic/atlas/git/atlas-middleware/custom-moos/MOOS/proj-5.2.0/nad

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/jharbin/academic/atlas/git/atlas-middleware/custom-moos/build/MOOS/proj-5.2.0")
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
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/proj" TYPE FILE FILES
    "/home/jharbin/academic/atlas/git/atlas-middleware/custom-moos/MOOS/proj-5.2.0/nad/epsg"
    "/home/jharbin/academic/atlas/git/atlas-middleware/custom-moos/MOOS/proj-5.2.0/nad/esri"
    "/home/jharbin/academic/atlas/git/atlas-middleware/custom-moos/MOOS/proj-5.2.0/nad/world"
    "/home/jharbin/academic/atlas/git/atlas-middleware/custom-moos/MOOS/proj-5.2.0/nad/esri.extra"
    "/home/jharbin/academic/atlas/git/atlas-middleware/custom-moos/MOOS/proj-5.2.0/nad/other.extra"
    "/home/jharbin/academic/atlas/git/atlas-middleware/custom-moos/MOOS/proj-5.2.0/nad/IGNF"
    "/home/jharbin/academic/atlas/git/atlas-middleware/custom-moos/MOOS/proj-5.2.0/nad/nad27"
    "/home/jharbin/academic/atlas/git/atlas-middleware/custom-moos/MOOS/proj-5.2.0/nad/GL27"
    "/home/jharbin/academic/atlas/git/atlas-middleware/custom-moos/MOOS/proj-5.2.0/nad/nad83"
    "/home/jharbin/academic/atlas/git/atlas-middleware/custom-moos/MOOS/proj-5.2.0/nad/nad.lst"
    "/home/jharbin/academic/atlas/git/atlas-middleware/custom-moos/MOOS/proj-5.2.0/nad/proj_def.dat"
    "/home/jharbin/academic/atlas/git/atlas-middleware/custom-moos/MOOS/proj-5.2.0/nad/CH"
    "/home/jharbin/academic/atlas/git/atlas-middleware/custom-moos/MOOS/proj-5.2.0/nad/ITRF2000"
    "/home/jharbin/academic/atlas/git/atlas-middleware/custom-moos/MOOS/proj-5.2.0/nad/ITRF2008"
    "/home/jharbin/academic/atlas/git/atlas-middleware/custom-moos/MOOS/proj-5.2.0/nad/ITRF2014"
    )
endif()

