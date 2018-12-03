/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Ali Uneri
  Created on: 2009-11-06

  (C) Copyright 2009-2018 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/

/*!
  \file
  \brief An example interface for Claron Micron Tracker.
  \ingroup devicesTutorial
*/

#include <cisstCommon/cmnPath.h>
#include <cisstCommon/cmnUnits.h>
#include <cisstCommon/cmnCommandLineOptions.h>
#include <cisstCommon/cmnQt.h>
#include <cisstOSAbstraction/osaSleep.h>
#include <cisstMultiTask/mtsTaskManager.h>
#include <cisstMultiTask/mtsSystemQtWidget.h>
#include <sawClaronMicronTracker/mtsMicronTracker.h>
#include <sawClaronMicronTracker/mtsMicronTrackerControllerQtComponent.h>
#include <sawClaronMicronTracker/mtsMicronTrackerToolQtComponent.h>

#include <ros/ros.h>
#include <cisst_ros_bridge/mtsROSBridge.h>

#include <QApplication>
#include <QMainWindow>


int main(int argc, char * argv[])
{
    // log configuration
    cmnLogger::SetMask(CMN_LOG_ALLOW_ALL);
    cmnLogger::SetMaskFunction(CMN_LOG_ALLOW_ALL);
    cmnLogger::SetMaskDefaultLog(CMN_LOG_ALLOW_ALL);
    cmnLogger::SetMaskClassMatching("mtsMicronTracker", CMN_LOG_ALLOW_ALL);
    cmnLogger::AddChannel(std::cerr, CMN_LOG_ALLOW_ERRORS);

    cmnCommandLineOptions options;
    std::string xmlConfigFile = "";
    double rosPeriod = 10.0 * cmn_ms;

    options.AddOptionOneValue("x", "xml-config",
                              "xml configuration file",
                              cmnCommandLineOptions::REQUIRED_OPTION, &xmlConfigFile);
    options.AddOptionOneValue("p", "ros-period",
                              "period in seconds to read all tool positions (default 0.01, 10 ms, 100Hz).  There is no point to have a period higher than the tracker component",
                              cmnCommandLineOptions::OPTIONAL_OPTION, &rosPeriod);

    // check that all required options have been provided
    std::string errorMessage;
    if (!options.Parse(argc, argv, errorMessage)) {
        std::cerr << "Error: " << errorMessage << std::endl;
        options.PrintUsage(std::cerr);
        return -1;
    }
    std::string arguments;
    options.PrintParsedArguments(arguments);
    std::cout << "Options provided:" << std::endl << arguments << std::endl;

    // create a Qt user interface
    QApplication application(argc, argv);

    // create the components
    mtsMicronTracker * tracker = new mtsMicronTracker("componentMicronTracker", 50.0 * cmn_ms);
    mtsMicronTrackerControllerQtComponent * trackerWidget = new mtsMicronTrackerControllerQtComponent("componentControllerQtComponent");

    // configure the components
    cmnPath searchPath;
    searchPath.Add(cmnPath::GetWorkingDirectory());
    std::string configPath = searchPath.Find(xmlConfigFile);
    if (configPath.empty()) {
        std::cerr << "Failed to find configuration: " << configPath << std::endl;
        return 1;
    }
    tracker->Configure(configPath);

    // add the components to the component manager
    mtsManagerLocal * componentManager = mtsComponentManager::GetInstance();
    componentManager->AddComponent(tracker);
    componentManager->AddComponent(trackerWidget);

    // connect the components, e.g. RequiredInterface -> ProvidedInterface
    componentManager->Connect(trackerWidget->GetName(), "Controller",
                              tracker->GetName(), "Controller");

    // Create ROS Bridge
    mtsROSBridge * rosBridge = new mtsROSBridge("MicronTrackerBridge", rosPeriod, true);
    std::string toolName;

    // add interfaces for tools and populate controller widget with tool widgets
    for (size_t tool = 0; tool < tracker->GetNumberOfTools(); tool++) {
        toolName = tracker->GetToolName(tool);
        mtsMicronTrackerToolQtComponent * toolWidget = new mtsMicronTrackerToolQtComponent(toolName);
        trackerWidget->AddTool(toolWidget,
                               toolWidget->GetWidget(),
                               toolWidget->GetMarkerProjectionLeft(),
                               toolWidget->GetMarkerProjectionRight());

        componentManager->AddComponent(toolWidget);
        componentManager->Connect(toolName, toolName,
                                  tracker->GetName(), toolName);

        std::string nameSpace = toolName;
        std::replace(nameSpace.begin(), nameSpace.end(), '-', '_');
        rosBridge->AddPublisherFromCommandRead<prmPositionCartesianGet, geometry_msgs::PoseStamped>
                (toolName, "GetPositionCartesian",
                 "/micron/" + nameSpace + "/measured_cp");
        rosBridge->AddPublisherFromEventWrite<bool, std_msgs::Bool>
                (toolName, "Visible",
                 "/micron/" + nameSpace + "/measured_cp_valid");
    }

    // add the bridge after all interfaces have been created
    componentManager->AddComponent(rosBridge);

    // connect all interfaces for the ROS bridge
    for (size_t tool = 0; tool < tracker->GetNumberOfTools(); tool++) {
        toolName = tracker->GetToolName(tool);
        componentManager->Connect(rosBridge->GetName(), toolName,
                                  tracker->GetName(), toolName);
    }

    // create a main window to hold QWidgets
    QMainWindow * mainWindow = new QMainWindow();
    mainWindow->setCentralWidget(trackerWidget->GetWidget());
    mainWindow->setWindowTitle("MicronTracker Controller");
    mainWindow->resize(0,0);
    mainWindow->show();

    mtsSystemQtWidgetComponent * systemWidget
        = new mtsSystemQtWidgetComponent("MTC-System");
    systemWidget->Configure();
    componentManager->AddComponent(systemWidget);
    componentManager->Connect(systemWidget->GetName(), "Component",
                              tracker->GetName(), "Controller");
    trackerWidget->GetWidget()->layout()->addWidget(systemWidget);

    // create and start all components
    componentManager->CreateAllAndWait(5.0 * cmn_s);
    componentManager->StartAllAndWait(5.0 * cmn_s);

    // run Qt user interface
    cmnQt::QApplicationExitsOnCtrlC();
    application.exec();

    // kill the micron tracker
    tracker->Kill();
    osaSleep(10 * rosPeriod); // to make sure the bridge has time to publish

    // kill all components and perform cleanup
    componentManager->KillAllAndWait(5.0 * cmn_s);
    componentManager->Cleanup();

    return 0;
}
