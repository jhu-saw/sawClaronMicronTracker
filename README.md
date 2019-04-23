# sawClaronMicronTracker

This SAW component contains code for interfacing with the Claronav (aka Claron) Micron Tracker
  * Linux
  * Micron Tracker with FireWire BumbleBee2 (2 cameras)

The `ros` folder contains code for a ROS node that interfaces with the sawClaronMicronTracker component and publishes the 3D transformations of each tracked tool.

# Links
 * License: http://github.com/jhu-cisst/cisst/blob/master/license.txt
 * JHU-LCSR software: http://jhu-lcsr.github.io/software/

# Dependencies
 * Micron Tracker SDK
 * cisst libraries: https://github.com/jhu-cisst/cisst
 * Qt for user interface
 * ROS (optional)

# Notes
 * If you use PointGrey's utilities (flycap, tryclops) to test the cameras, make sure you unplug/replug the cameras before using the Micron Tracker examples or sawClaronMicronTracker.  It seems that the PointGrey's programs change the cameras configuration in a way that is not compatible with the Micron Tracker SDK.
 * On Linux 64 bits OSs, make sure `_LINUX64` is defined before including `MTC.h` (this is done in CMakeLists.txt for sawClaronMicronTracker)
 * On Linux, if you're tempted to compile and run the examples provided with the SDK and don't want to use NetBeans, you can compile using the command lines:
   * Example `NetBeansProjects/MTSimpleDemoC`: `g++ -D_LINUX64 SimpleDemoC.cpp -L. -lMTC -ldc1394 -lraw1394 -lpthread -lm -lvnl -lvnl_algo` (you will first have to copy the files MTC.h and libMTC.a in this folder)
   * Example `NetBeansProjects/MTDemoCPP`: `g++ -D_LINUX64 *.cpp -L. -lfltk -lMTC -ldc1394 -lraw1394 -lpthread -lm -lvnl -lvnl_algo -lGL -lfltk_gl` (you will first have to copy the files MTC.h and libMTC.a in this folder)
   * For both examples, you can ignore the `lib*1394.a` provided along `libMTC.a`, the OS provided shared libraries work fine (at least on Ubuntu 16.04)


# ROS/Catkin build tools

This is by far the simplest solution to compile and run the sawClaronMicronTracker examples on Linux.
See how to build cisst with ROS/Catkin tools on the cisst wiki:
https://github.com/jhu-cisst/cisst/wiki/Compiling-cisst-and-SAW-with-CMake (Make sure you go to the ROS build instructions).

When compiling the sawClaronMicronTracker code, you will need to specify where to find the Micron Tracker SDK.  Do a first `catkin build`, this build will skip the sawClaronMicronTracker because the directory containing the SDK is not defined.   To define it, use `ccmake` in a shell/terminal that has all the ROS environment variables defined (DO NOT USE `cmake-gui`, for some reasons, it ignores the environment variables) on the build directory for the SAW Micron Tracker component.  For example:
```sh
adeguet1@lcsr-qla:~/catkin_ws$ ccmake build_release/saw_claron_micron_tracker
```
In the command above, the ROS workspace is `~/catkin_ws` and the build tree is `build_release`.  You might have `devel` or `devel_debug` depending on your workspace configuration.

Once in CMake, locate `ClaronMicronTracker_INCLUDE_DIR` and make it point to the directory containing the file `MTC.h`.  For example, if you're compiling 64 bits binaries, `~/MicronTracker3.7.2.3_Linux_Release/Dist/Dist_x64`.  Hit configure once and the variable `ClaronMicronTracker_MTC_LIBRARY` should have been found automatically.

Don't forget to hit "Generate" before quitting CMake.  You should now be able to build using `catkin build --force-cmake`.   The option `--force-cmake` is required to force CMake to run for all packages that depends on the `sawClaronMicronTracker` package.


# Running the examples

## ROS example

Go to the folder where you have your XML configuration file.  For example:
```sh
roscd saw_claron_micron_tracker/../examples/
```

Then start the ROS node with the `-x` option to specify the XML configuration file:
```
rosrun micron_tracker_ros micron_tracker -x configClaronMicronTracker.xml
```

You will likely have to edit the main XML configuration file to update
the directories where your calibration files are located.

If the tracker is properly configured and connected, you can click on
the "Track" button to start tracking.  At that point, you can use
`rostopic list` to find which data is published and `rostopic echo` to
visualize it.