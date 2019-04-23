#
# (C) Copyright 2015-2018 Johns Hopkins University (JHU), All Rights Reserved.
#
# --- begin cisst license - do not edit ---
#
# This software is provided "as is" under an open source license, with
# no warranty.  The complete license can be found in license.txt and
# http://www.cisst.org/cisst/license.txt.
#
# --- end cisst license ---

cmake_minimum_required (VERSION 2.8)

project (micron_tracker_ros)

## find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package (catkin REQUIRED
              COMPONENTS
              cisst_ros_bridge
              geometry_msgs
              roscpp
              std_msgs
              roslib
              )

# create a list of required cisst libraries
set (REQUIRED_CISST_LIBRARIES cisstCommon
                              cisstCommonQt
                              cisstCommonXML
                              cisstVector
                              cisstNumerical
                              cisstOSAbstraction
                              cisstMultiTask
                              cisstMultiTaskQt
                              cisstParameterTypes)

# find cisst and make sure the required libraries have been compiled
find_package (cisst 1.0.11 REQUIRED ${REQUIRED_CISST_LIBRARIES})

if (cisst_FOUND)

  # load cisst configuration
  include (${CISST_USE_FILE})

  # include_directories(include)
  include_directories (${catkin_INCLUDE_DIRS})

  file (MAKE_DIRECTORY "${CATKIN_DEVEL_PREFIX}/include")

  catkin_package (INCLUDE_DIRS "${CATKIN_DEVEL_PREFIX}/include"
                  CATKIN_DEPENDS cisst_ros_bridge geometry_msgs roscpp std_msgs)

  # sawClaronMicronTracker has been compiled within cisst, we should find it automatically
  find_package (sawClaronMicronTracker 1.1.0)

  if (sawClaronMicronTracker_FOUND)

    # sawClaronMicronTracker configuration
    include_directories (${sawClaronMicronTracker_INCLUDE_DIR})
    link_directories (${sawClaronMicronTracker_LIBRARY_DIR})

    add_executable (micron_tracker micron_tracker.cpp)

    # link against non cisst libraries and saw components
    target_link_libraries (micron_tracker
                           ${sawClaronMicronTracker_LIBRARIES}
                           ${catkin_LIBRARIES}
                           )

    # link against cisst libraries (and dependencies)
    cisst_target_link_libraries (micron_tracker ${REQUIRED_CISST_LIBRARIES})

  else (sawClaronMicronTracker_FOUND)
    message ("Information: code in ${CMAKE_CURRENT_SOURCE_DIR} will not be compiled, it requires sawClaronMicronTracker")
  endif (sawClaronMicronTracker_FOUND)

endif (cisst_FOUND)
