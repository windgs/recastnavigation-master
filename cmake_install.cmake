# Install script for directory: F:/code_modul/网格寻路算法/3D寻路算法/recastnavigation/recastnavigation-master/recastnavigation-master

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

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  include("F:/code_modul/网格寻路算法/3D寻路算法/recastnavigation/recastnavigation-master/recastnavigation-master/DebugUtils/cmake_install.cmake")
  include("F:/code_modul/网格寻路算法/3D寻路算法/recastnavigation/recastnavigation-master/recastnavigation-master/Detour/cmake_install.cmake")
  include("F:/code_modul/网格寻路算法/3D寻路算法/recastnavigation/recastnavigation-master/recastnavigation-master/DetourCrowd/cmake_install.cmake")
  include("F:/code_modul/网格寻路算法/3D寻路算法/recastnavigation/recastnavigation-master/recastnavigation-master/DetourTileCache/cmake_install.cmake")
  include("F:/code_modul/网格寻路算法/3D寻路算法/recastnavigation/recastnavigation-master/recastnavigation-master/Recast/cmake_install.cmake")
  include("F:/code_modul/网格寻路算法/3D寻路算法/recastnavigation/recastnavigation-master/recastnavigation-master/RecastDemo/cmake_install.cmake")
  include("F:/code_modul/网格寻路算法/3D寻路算法/recastnavigation/recastnavigation-master/recastnavigation-master/Tests/cmake_install.cmake")

endif()

if(CMAKE_INSTALL_COMPONENT)
  set(CMAKE_INSTALL_MANIFEST "install_manifest_${CMAKE_INSTALL_COMPONENT}.txt")
else()
  set(CMAKE_INSTALL_MANIFEST "install_manifest.txt")
endif()

string(REPLACE ";" "\n" CMAKE_INSTALL_MANIFEST_CONTENT
       "${CMAKE_INSTALL_MANIFEST_FILES}")
file(WRITE "F:/code_modul/网格寻路算法/3D寻路算法/recastnavigation/recastnavigation-master/recastnavigation-master/${CMAKE_INSTALL_MANIFEST}"
     "${CMAKE_INSTALL_MANIFEST_CONTENT}")
