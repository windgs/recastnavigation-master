# Install script for directory: F:/code_modul/网格寻路算法/3D寻路算法/recastnavigation/recastnavigation-master/recastnavigation-master/Detour

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "C:/Program Files (x86)/RecastNavigation")
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

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if("${CMAKE_INSTALL_CONFIG_NAME}" MATCHES "^([Dd][Ee][Bb][Uu][Gg])$")
    file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE STATIC_LIBRARY FILES "F:/code_modul/网格寻路算法/3D寻路算法/recastnavigation/recastnavigation-master/recastnavigation-master/Detour/Debug/Detour.lib")
  elseif("${CMAKE_INSTALL_CONFIG_NAME}" MATCHES "^([Rr][Ee][Ll][Ee][Aa][Ss][Ee])$")
    file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE STATIC_LIBRARY FILES "F:/code_modul/网格寻路算法/3D寻路算法/recastnavigation/recastnavigation-master/recastnavigation-master/Detour/Release/Detour.lib")
  elseif("${CMAKE_INSTALL_CONFIG_NAME}" MATCHES "^([Mm][Ii][Nn][Ss][Ii][Zz][Ee][Rr][Ee][Ll])$")
    file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE STATIC_LIBRARY FILES "F:/code_modul/网格寻路算法/3D寻路算法/recastnavigation/recastnavigation-master/recastnavigation-master/Detour/MinSizeRel/Detour.lib")
  elseif("${CMAKE_INSTALL_CONFIG_NAME}" MATCHES "^([Rr][Ee][Ll][Ww][Ii][Tt][Hh][Dd][Ee][Bb][Ii][Nn][Ff][Oo])$")
    file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE STATIC_LIBRARY FILES "F:/code_modul/网格寻路算法/3D寻路算法/recastnavigation/recastnavigation-master/recastnavigation-master/Detour/RelWithDebInfo/Detour.lib")
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE FILE FILES
    "F:/code_modul/网格寻路算法/3D寻路算法/recastnavigation/recastnavigation-master/recastnavigation-master/Detour/Include/DetourAlloc.h"
    "F:/code_modul/网格寻路算法/3D寻路算法/recastnavigation/recastnavigation-master/recastnavigation-master/Detour/Include/DetourAssert.h"
    "F:/code_modul/网格寻路算法/3D寻路算法/recastnavigation/recastnavigation-master/recastnavigation-master/Detour/Include/DetourCommon.h"
    "F:/code_modul/网格寻路算法/3D寻路算法/recastnavigation/recastnavigation-master/recastnavigation-master/Detour/Include/DetourMath.h"
    "F:/code_modul/网格寻路算法/3D寻路算法/recastnavigation/recastnavigation-master/recastnavigation-master/Detour/Include/DetourNavMesh.h"
    "F:/code_modul/网格寻路算法/3D寻路算法/recastnavigation/recastnavigation-master/recastnavigation-master/Detour/Include/DetourNavMeshBuilder.h"
    "F:/code_modul/网格寻路算法/3D寻路算法/recastnavigation/recastnavigation-master/recastnavigation-master/Detour/Include/DetourNavMeshQuery.h"
    "F:/code_modul/网格寻路算法/3D寻路算法/recastnavigation/recastnavigation-master/recastnavigation-master/Detour/Include/DetourNode.h"
    "F:/code_modul/网格寻路算法/3D寻路算法/recastnavigation/recastnavigation-master/recastnavigation-master/Detour/Include/DetourStatus.h"
    )
endif()

