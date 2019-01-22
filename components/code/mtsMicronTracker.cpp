/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*
  Author(s):  Ali Uneri
  Created on: 2009-11-06

  (C) Copyright 2009-2019 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/

#include <cisstCommon/cmnXMLPath.h>
#include <cisstVector/vctDynamicMatrixTypes.h>
#include <cisstVector/vctDynamicVectorTypes.h>
#include <cisstOSAbstraction/osaSleep.h>
#include <cisstMultiTask/mtsInterfaceProvided.h>
#include <sawClaronMicronTracker/mtsMicronTracker.h>

#include <MTC.h>

CMN_IMPLEMENT_SERVICES_DERIVED_ONEARG(mtsMicronTracker, mtsTaskPeriodic, mtsTaskPeriodicConstructorArg);

// macro to check for and report MTC usage errors in class::method
#define MTC(func) {                                                     \
        int retval = func;                                              \
        if (retval != mtOK) {                                           \
            ControllerInterface->SendError(this->GetName() + ": MTC error " + MTLastErrorString()); \
        }                                                               \
    };

// same but used in global functions
#define MTC_func(func) {                                                \
        int retval = func;                                              \
        if (retval != mtOK) {                                           \
            CMN_LOG_RUN_ERROR << "MTC: " << MTLastErrorString() << std::endl; \
        }                                                               \
    };

struct mtsMicronTrackerData {
    mtHandle CurrentCamera;
    mtHandle IdentifyingCamera;
    mtHandle IdentifiedMarkers;
    mtHandle PoseXf;
    mtHandle Path;
};


vctFrm3 XfHandleToFrame(mtHandle & xfHandle)
{
    vctFrm3 frame;
    MTC_func(Xform3D_RotMatGet(xfHandle, frame.Rotation().Pointer()));
    frame.Rotation() = frame.Rotation().Transpose();  // MTC matrices are COL_MAJOR
    MTC_func(Xform3D_ShiftGet(xfHandle, frame.Translation().Pointer()));
    frame.Translation().Multiply(cmn_mm); // MTC reports positions in mm, convert to CISST units
    return frame;
}


mtHandle FrameToXfHandle(const mtHandle & poseXf, vctFrm3 & frame)
{
    frame.Rotation() = frame.Rotation().Transpose();  // MTC matrices are COL_MAJOR
    MTC_func(Xform3D_RotMatSet(poseXf, frame.Rotation().Pointer()));
    vct3 translation = frame.Translation();
    translation.Divide(cmn_mm); // convert back to mm
    MTC_func(Xform3D_ShiftSet(poseXf, translation.Pointer()));
    return poseXf;
}


void mtsMicronTracker::Construct(void)
{
    TrackerData = new mtsMicronTrackerData;

    IsCapturing = false;
    IsTracking = false;
    CameraIsThermallyStable = false;
    CameraIsThermallyStableTimer = 0.0;

    StateTable.AddData(IsCapturing, "IsCapturing");
    StateTable.AddData(IsTracking, "IsTracking");
    StateTable.AddData(XPointsMaxNum, "XPointsMaxNum");
    StateTable.AddData(XPoints, "XPoints");
    StateTable.AddData(XPointsProjectionLeft, "XPointsProjectionLeft");
    StateTable.AddData(XPointsProjectionRight, "XPointsProjectionRight");

    ImageTable = new mtsStateTable(10, "ImageTable");
    AddStateTable(ImageTable);
    ImageTable->SetAutomaticAdvance(false);
    ImageTable->AddData(ImageLeft, "ImageLeft");
    ImageTable->AddData(ImageRight, "ImageRight");

    ControllerInterface = AddInterfaceProvided("Controller");
    if (ControllerInterface) {
        ControllerInterface->AddMessageEvents();
        ControllerInterface->AddCommandWrite(&mtsMicronTracker::ToggleCapturing, this,
                                             "ToggleCapturing");
        ControllerInterface->AddCommandWrite(&mtsMicronTracker::ToggleTracking, this,
                                             "ToggleTracking");
        ControllerInterface->AddCommandReadState(StateTable, IsCapturing, "IsCapturing");
        ControllerInterface->AddCommandReadState(StateTable, IsTracking, "IsTracking");
        ControllerInterface->AddCommandReadState(StateTable, XPointsMaxNum, "GetXPointsMaxNum");
        ControllerInterface->AddCommandReadState(StateTable, XPoints, "GetXPoints");
        ControllerInterface->AddCommandReadState(StateTable, XPointsProjectionLeft, "GetXPointsProjectionLeft");
        ControllerInterface->AddCommandReadState(StateTable, XPointsProjectionRight, "GetXPointsProjectionRight");
        ControllerInterface->AddCommandReadState(*ImageTable, ImageLeft, "GetCameraFrameLeft");
        ControllerInterface->AddCommandReadState(*ImageTable, ImageRight, "GetCameraFrameRight");
        ControllerInterface->AddCommandWrite(&mtsMicronTracker::ComputeCameraModel, this,
                                             "ComputeCameraModel");
        ControllerInterface->AddCommandWrite(&mtsMicronTracker::SetJitterFilterEnabled, this,
                                             "SetJitterFilterEnabled", mtsBool());
        ControllerInterface->AddCommandWrite(&mtsMicronTracker::SetJitterCoefficient, this,
                                             "SetJitterCoefficient");
        ControllerInterface->AddCommandWrite(&mtsMicronTracker::SetKalmanFilterEnabled, this,
                                             "SetKalmanFilterEnabled", mtsBool());
        ControllerInterface->AddCommandReadState(StateTable, StateTable.PeriodStats,
                                                 "GetPeriodStatistics");
    }
}


void mtsMicronTracker::Startup(void)
{
    MarkAllToolsNotVisible();
}


void mtsMicronTracker::Configure(const std::string & filename)
{
    CMN_LOG_CLASS_INIT_VERBOSE << "Configure: using " << filename << std::endl;
    cmnXMLPath config;
    config.SetInputSource(filename);

    // get required paths
    config.GetXMLValue("/tracker/controller", "@calibration", CameraCalibrationDir);
    config.GetXMLValue("/tracker/controller", "@markers", MarkerTemplatesDir);

    // add tools
    int numberOfTools = 0;
    config.Query("count(/tracker/tools/*)", numberOfTools);
    std::string toolName, toolSerial;

    for (int i = 0; i < numberOfTools; i++) {
        std::stringstream context;
        context << "/tracker/tools/tool[" << i+1 << "]";  // XML is one-based, adding one here
        config.GetXMLValue(context.str().c_str(), "@name", toolName, "");
        if (toolName.empty()) {
            continue;
        }
        config.GetXMLValue(context.str().c_str(), "@marker", toolSerial);
        AddTool(toolName, toolSerial);
    }

    std::stringstream context;
    context << "/tracker/xpoints/xpoint";
    config.GetXMLValue(context.str().c_str(), "@max_number", XPointsMaxNum);
    // set the size of the XPoints
    XPoints.resize(XPointsMaxNum);
    XPointsProjectionLeft.resize(XPointsMaxNum);
    XPointsProjectionRight.resize(XPointsMaxNum);

    StartCameras();
}

void mtsMicronTracker::SetJitterFilterEnabled(const mtsBool & flag)
{
    Markers_JitterFilterEnabledSet(flag.GetData());

    bool status_JitterFilter = Markers_JitterFilterEnabled();

    if (status_JitterFilter == true)
        std::cout << "Jitter filter is enabled" << std::endl;
    else
        std::cout << "Jitter filter is disabled" << std::endl;
}

void mtsMicronTracker::SetJitterCoefficient(const double & coefficient)
{
    Markers_JitterFilterCoefficientSet(coefficient);
}

void mtsMicronTracker::SetKalmanFilterEnabled(const mtsBool & flag)
{
    Markers_KalmanFilterEnabledSet(flag.GetData());
    bool status_KalmanFilter = Markers_KalmanFilterEnabled();

    if (status_KalmanFilter == true)
        std::cout << "Kalman filter is enabled" << std::endl;
    else
        std::cout << "Kalman filter is disabled" << std::endl;
}

void mtsMicronTracker::MarkAllToolsNotVisible(void)
{
    ToolsType::const_iterator toolIterator;
    const ToolsType::const_iterator end = Tools.end();
    for (toolIterator = Tools.begin(); toolIterator != end; ++toolIterator) {
        Tool * tool = toolIterator->second;
        tool->VisibleEvent(false);
        tool->Visible = false;
        tool->TooltipPosition.SetValid(false);
    }
}

mtsMicronTracker::Tool * mtsMicronTracker::CheckTool(const std::string & serialNumber)
{
    const ToolsType::const_iterator end = Tools.end();
    ToolsType::const_iterator toolIterator;
    for (toolIterator = Tools.begin(); toolIterator != end; ++toolIterator) {
        if (toolIterator->second->SerialNumber == serialNumber) {
            CMN_LOG_CLASS_RUN_DEBUG << "CheckTool: found existing tool for serial number: " << serialNumber << std::endl;
            return toolIterator->second;
        }
    }
    return 0;
}


mtsMicronTracker::Tool * mtsMicronTracker::AddTool(const std::string & name, const std::string & serialNumber)
{
    Tool * tool = CheckTool(serialNumber);

    if (tool) {
        CMN_LOG_CLASS_INIT_WARNING << "AddTool: " << tool->Name << " already exists, renaming it to " << name << " instead" << std::endl;
        tool->Name = name;
    } else {
        tool = new Tool();
        tool->Name = name;
        tool->SerialNumber = serialNumber;
        tool->TooltipPosition.SetMovingFrame(name);
        tool->TooltipPosition.SetReferenceFrame("Micron");
        if (!Tools.AddItem(tool->Name, tool, CMN_LOG_LEVEL_INIT_ERROR)) {
            CMN_LOG_CLASS_INIT_ERROR << "AddTool: no tool created, duplicate name exists: " << name << std::endl;
            delete tool;
            return 0;
        }
        CMN_LOG_CLASS_INIT_VERBOSE << "AddTool: created tool \"" << name << "\" with serial number: " << serialNumber << std::endl;

        // create an interface for tool
        tool->Interface = AddInterfaceProvided(name);
        if (tool->Interface) {
            StateTable.AddData(tool->TooltipPosition, name + "Position");
            StateTable.AddData(tool->MarkerProjectionLeft, name + "MarkerProjectionLeft");
            StateTable.AddData(tool->MarkerProjectionRight, name + "MarkerProjectionRight");
            StateTable.AddData(tool->MarkerTemplatePositions, name + "MarkerTemplatePositions");
            StateTable.AddData(tool->MarkerTemplateTrackingPositions, name + "MarkerTemplateTrackingPositions");

            tool->Interface->AddCommandReadState(StateTable, tool->TooltipPosition, "GetPositionCartesian");
            tool->Interface->AddCommandReadState(StateTable, tool->MarkerProjectionLeft, "GetMarkerProjectionLeft");
            tool->Interface->AddCommandReadState(StateTable, tool->MarkerProjectionRight, "GetMarkerProjectionRight");
            tool->Interface->AddCommandReadState(StateTable, tool->MarkerTemplatePositions, "GetMarkerTemplatePositions");
            tool->Interface->AddCommandReadState(StateTable, tool->MarkerTemplateTrackingPositions, "GetMarkerTemplateTrackingPositions");
            tool->Interface->AddEventWrite(tool->VisibleEvent, "Visible", false);
        }
    }
    return tool;
}


std::string mtsMicronTracker::GetToolName(const unsigned int index) const
{
    ToolsType::const_iterator toolIterator = Tools.begin();
    if (index >= Tools.size()) {
        CMN_LOG_CLASS_RUN_ERROR << "GetToolName: requested index is out of range" << std::endl;
        return "";
    }
    for (unsigned int i = 0; i < index; i++) {
        toolIterator++;
    }
    return toolIterator->first;
}


void mtsMicronTracker::StartCameras(void)
{
    // attach cameras
    MTC(Cameras_AttachAvailableCameras(const_cast<char *>(CameraCalibrationDir.c_str())));
    if (Cameras_Count() < 1) {
        CMN_LOG_CLASS_INIT_ERROR << "StartCameras: no camera found" << std::endl;
    }
    CMN_LOG_CLASS_INIT_VERBOSE << "StartCameras: found " << Cameras_Count()
                               << " camera(s)" << std::endl;

    // load marker templates
    MTC(Markers_LoadTemplates(const_cast<char *>(MarkerTemplatesDir.c_str())));
    if (Markers_TemplatesCount() < 1) {
        CMN_LOG_CLASS_INIT_ERROR << "StartCameras: no marker template found" << std::endl;
    }
    CMN_LOG_CLASS_INIT_VERBOSE << "StartCameras: loaded " << Markers_TemplatesCount()
                               << " marker template(s)" << std::endl;

    TrackerData->IdentifiedMarkers = Collection_New();
    TrackerData->PoseXf = Xform3D_New();
    TrackerData->Path = Persistence_New();

    // select current camera
    MTC(Cameras_ItemGet(0, &(TrackerData->CurrentCamera)));

    // check if a camera is connected
    int retval = Cameras_GrabFrame(TrackerData->CurrentCamera);
    if (retval != mtOK) {
        if (retval == mtGrabFrameError) {
            CMN_LOG_CLASS_INIT_ERROR << "Startup: camera is not connected" << std::endl;
        } else {
            CMN_LOG_CLASS_INIT_ERROR << "Startup: " << MTLastErrorString() << std::endl;
        }
        return;
    }

    // get camera resolution and initialize buffers
    MTC(Camera_ResolutionGet(TrackerData->CurrentCamera, &FrameWidth, &FrameHeight));
    CMN_LOG_CLASS_INIT_VERBOSE << "Startup: resolution is " << FrameWidth
                               << " x " << FrameHeight << std::endl;

    ImageLeft.SetSize(FrameWidth * FrameHeight);
    ImageRight.SetSize(FrameWidth * FrameHeight);

    // get calibration info
    char lines[80][80];
    int numLines;
    MTC(Camera_CalibrationInfo(TrackerData->CurrentCamera, lines, 80, &numLines));
    std::stringstream calibrationInfo;
    for (int i = 0; i < numLines; i++) {
        calibrationInfo << " * " << lines[i] << "\n";
    }
    CMN_LOG_CLASS_INIT_DEBUG << "Startup: calibration parameters:\n"
                             << calibrationInfo.str() << std::endl;

    //Camera_HdrEnabledSet(CurrentCamera, true);
    Camera_HistogramEqualizeImagesSet(TrackerData->CurrentCamera, true);
    Camera_LightCoolnessSet(TrackerData->CurrentCamera, 0.56);  // obtain this value using CoolCard
}


void mtsMicronTracker::Run(void)
{
    ProcessQueuedCommands();

    int retval = Cameras_GrabFrame(TrackerData->CurrentCamera);
    if (retval != mtOK) {
        if (retval == mtGrabFrameError) {
            CMN_LOG_CLASS_RUN_ERROR << "Run: camera is not connected" << std::endl;
        }
        return;
    }

    if (IsCapturing) {
        int numFramesGrabbed;
        MTC(Camera_FramesGrabbedGet(TrackerData->CurrentCamera,
                                    &numFramesGrabbed));
        if (numFramesGrabbed > 0) {
            ImageTable->Start();
            MTC(Camera_ImagesGet(TrackerData->CurrentCamera,
                                 ImageLeft.Pointer(),
                                 ImageRight.Pointer()));
            ImageTable->Advance();
        }
    }
    mtMeasurementHazardCode hazardCode =
        Camera_LastFrameThermalHazard(TrackerData->CurrentCamera);
    // camera NOT stable
    if (hazardCode == mtCameraWarmingUp) {
        const double currentTime = this->StateTable.GetTic();
        bool sendWarning = false;
        if (CameraIsThermallyStable) {
            sendWarning = true;
            CameraIsThermallyStable = false;
        } else {
            // throttle to one message per minute
            if (currentTime - CameraIsThermallyStableTimer > 60.0 * cmn_s) {
                sendWarning = true;
            }
        }
        // finally send warning and update timer
        if (sendWarning) {
            ControllerInterface->SendWarning(this->GetName() + ": camera is not yet thermally stable.");
            CameraIsThermallyStableTimer = currentTime;
        }
    }
    // something else, i.e. camera should be stable
    else {
        if (!CameraIsThermallyStable) {
            ControllerInterface->SendWarning(this->GetName() + ": camera thermally stable.");
            CameraIsThermallyStable = true;
        }
    }

    if (IsTracking) {
        Track();
        // TrackXPoint();
    }
}


void mtsMicronTracker::Cleanup(void)
{
    MarkAllToolsNotVisible();

    MTC(Collection_Free(TrackerData->IdentifiedMarkers));
    MTC(Xform3D_Free(TrackerData->PoseXf));
    MTC(Persistence_Free(TrackerData->Path));
    Cameras_Detach();
}


void mtsMicronTracker::ToggleCapturing(const bool & toggle)
{
    if (toggle) {
        IsCapturing = true;
        CMN_LOG_CLASS_INIT_VERBOSE << "ToggleCapturing: capturing is on" << std::endl;
    } else {
        IsCapturing = false;
        CMN_LOG_CLASS_INIT_VERBOSE << "ToggleCapturing: capturing is off" << std::endl;
    }
}


void mtsMicronTracker::ToggleTracking(const bool & toggle)
{
    if (toggle) {
        IsTracking = true;
        CMN_LOG_CLASS_INIT_VERBOSE << "ToggleTracking: tracking is on" << std::endl;
    } else {
        IsTracking = false;
        MarkAllToolsNotVisible();
        CMN_LOG_CLASS_INIT_VERBOSE << "ToggleTracking: tracking is off" << std::endl;
    }
}


void mtsMicronTracker::Track(void)
{
    Tool * tool;
    mtHandle markerHandle;
    char markerName[MT_MAX_STRING_LENGTH];
    vctFrm3 markerPosition;
    vctFrm3 tooltipPosition;
    vctFrm3 tooltipCalibration;

    // initialize all marker positions to invalid
    const ToolsType::const_iterator end = Tools.end();
    ToolsType::const_iterator toolIterator;
    for (toolIterator = Tools.begin(); toolIterator != end; ++toolIterator) {
        toolIterator->second->TooltipPosition.SetValid(false);
    }

    MTC(Markers_ProcessFrame(TrackerData->CurrentCamera));
    MTC(Markers_IdentifiedMarkersGet(TrackerData->CurrentCamera,
                                     TrackerData->IdentifiedMarkers));
    const unsigned int numIdentifiedMarkers =
        Collection_Count(TrackerData->IdentifiedMarkers);
    CMN_LOG_CLASS_RUN_DEBUG << "Track: identified " << numIdentifiedMarkers
                            << " marker(s)" << std::endl;

    for (unsigned int i = 1; i <= numIdentifiedMarkers; i++) {
        markerHandle = Collection_Int(TrackerData->IdentifiedMarkers, i);
        MTC(Marker_NameGet(markerHandle, markerName, MT_MAX_STRING_LENGTH, 0));

        // check if tool exists, generate a name and add it otherwise
        tool = CheckTool(markerName);
        if (!tool) {
            std::string name = "tool-" + std::string(markerName);
            tool = AddTool(name, markerName);
        }

        MTC(Marker_Marker2CameraXfGet(markerHandle,
                                      TrackerData->CurrentCamera,
                                      TrackerData->PoseXf,
                                      &(TrackerData->IdentifyingCamera)));
        if (TrackerData->IdentifyingCamera != 0) {
            if (tool->Name == "tool-COOLCARD") {
                mtHandle identifiedFacets = Collection_New();
                MTC(Marker_IdentifiedFacetsGet(markerHandle,
                                               TrackerData->CurrentCamera,
                                               false, identifiedFacets));
                mtHandle longVectorHandle = Vector_New();
                mtHandle shortVectorHandle = Vector_New();
                MTC(Facet_IdentifiedVectorsGet(Collection_Int(identifiedFacets, 1), longVectorHandle, shortVectorHandle));
                if (longVectorHandle != 0) {
                    MTC(Camera_LightCoolnessAdjustFromColorVector(TrackerData->CurrentCamera, longVectorHandle, 0));
                    CMN_LOG_CLASS_RUN_VERBOSE << "Track: light coolness set to " << Cameras_LightCoolness() << std::endl;
                }
                MTC(Collection_Free(identifiedFacets));
                MTC(Vector_Free(longVectorHandle));
                MTC(Vector_Free(shortVectorHandle));
                continue;
            }

            markerPosition = XfHandleToFrame(TrackerData->PoseXf);
            tool->MarkerPosition.Position() = markerPosition;

            // get the calibration from marker template
            MTC(Marker_Tooltip2MarkerXfGet(markerHandle, TrackerData->PoseXf));
            tooltipCalibration = XfHandleToFrame(TrackerData->PoseXf);

            //            // update the calibration in marker template
            //            if (tool->TooltipOffset.All()) {
            //                tooltipCalibration.Translation() = tool->TooltipOffset;
            //                PoseXf = FrameToXfHandle(tooltipCalibration);
            //                MTC(Marker_Tooltip2MarkerXfSet(markerHandle, PoseXf));
            //                std::string markerPath = "C:\\Program Files\\Claron Technology\\MicronTracker\\Markers\\" + tool->SerialNumber + "_custom";
            //                MTC(Persistence_PathSet(Path, markerPath.c_str()));
            //                MTC(Marker_StoreTemplate(markerHandle, Path, ""));
            //                tool->TooltipOffset.SetAll(0.0);
            //            }

            tooltipPosition = markerPosition * tooltipCalibration;
            //            if (tool->Name == "Probe" && Tools.size() == 2) {
            //                tool->TooltipPosition.Position() = Tools.GetItem("Reference")->TooltipPosition.Position().ApplyInverseTo(tooltipPosition);
            //            } else {
            tool->TooltipPosition.Position() = tooltipPosition;
            //            }
            tool->TooltipPosition.SetValid(true);

            CMN_LOG_CLASS_RUN_DEBUG << "Track: " << markerName << " is at:\n" << tooltipPosition << std::endl;

            vct3 tipPosition(tooltipPosition.Translation().Pointer());
            tipPosition.Divide(cmn_mm); // convert back to mm to compute projectins
            MTC(Camera_ProjectionOnImage(TrackerData->CurrentCamera,
                                         LEFT_CAMERA, tipPosition.Pointer(),
                                         &(tool->MarkerProjectionLeft.X()),
                                         &(tool->MarkerProjectionLeft.Y())));

            MTC(Camera_ProjectionOnImage(TrackerData->CurrentCamera,
                                         RIGHT_CAMERA, tipPosition.Pointer(),
                                         &(tool->MarkerProjectionRight.X()),
                                         &(tool->MarkerProjectionRight.Y())));

        }

#if 0
        // get the tracking data of template points
        mtHandle IdentifiedFacets = Collection_New();
        MTC(Marker_IdentifiedFacetsGet(markerHandle, CurrentCamera, false, IdentifiedFacets));

        mtHandle lvHandle = Vector_New();
        mtHandle svHandle = Vector_New();
        MTC(Facet_IdentifiedVectorsGet(Collection_Int(IdentifiedFacets, 1), lvHandle, svHandle));

        double endPt[6];
        double endPt1[6];
        MTC(Vector_EndPosGet(lvHandle, endPt));
        MTC(Vector_EndPosGet(svHandle, endPt1));

        for (unsigned int j = 0; j < 2; j++) {
            for (unsigned int k = 0; k < 3; k++) {
                tool->MarkerTemplateTrackingPositions[3 * j + k] = endPt[3 * j + k];

                if (((endPt[0] != endPt1[3 * j]) || (endPt[1] != endPt1[3 * j + 1]) || (endPt[2] != endPt1[3 * j + 2])) && ((endPt[3] != endPt1[3 * j]) || (endPt[4] != endPt1[3 * j + 1]) || (endPt[5] != endPt1[3 * j + 2])) ) {

                    tool->MarkerTemplateTrackingPositions[3 * 2 + k] = endPt1[3 * j + k];
                }
            }
            CMN_LOG_CLASS_RUN_DEBUG << "Traking Template Data (" << tool->Name << ") [" << j << "]:  " << tool->MarkerTemplateTrackingPositions[3 * j] << ",   " << tool->MarkerTemplateTrackingPositions[3 * j + 1] << ",   " << tool->MarkerTemplateTrackingPositions[3 * j + 2] << std::endl;
        }
        CMN_LOG_CLASS_RUN_DEBUG << "Traking Template Data (" << tool->Name << ") [2]:  " << tool->MarkerTemplateTrackingPositions[3 * 2] << ",   " << tool->MarkerTemplatePositions[3 * 2 + 1] << ",   " << tool->MarkerTemplatePositions[3 * 2 + 2] << std::endl;

        // To get Template data from the markers
        mtHandle TemplateFacets = Collection_New();
        MTC(Marker_TemplateFacetsGet(markerHandle, &TemplateFacets));

        mtHandle facetHandle = Collection_Int(TemplateFacets, 1);

        double templatePos[12];
        MTC(Facet_TemplatePositionsGet(facetHandle, templatePos));

        for (unsigned int j = 0; j < 2; j++) {
            for (unsigned int k = 0; k < 3; k++) {
                tool->MarkerTemplatePositions[3 * j + k] = endPt[3 * j + k];

                if (((endPt[0] != endPt1[3 * j]) || (endPt[1] != endPt1[3 * j + 1]) || (endPt[2] != endPt1[3 * j + 2])) && ((endPt[3] != endPt1[3 * j]) || (endPt[4] != endPt1[3 * j + 1]) || (endPt[5] != endPt1[3 * j + 2])) ) {

                    tool->MarkerTemplatePositions[3 * 2 + k] = endPt1[3 * j + k];

                }
            }
            CMN_LOG_CLASS_RUN_DEBUG << "Traking Template Data (" << tool->Name << ") [" << j << "]:  " << tool->MarkerTemplatePositions[3 * j] << ",   " << tool->MarkerTemplatePositions[3 * j + 1] << ",   " << tool->MarkerTemplatePositions[3 * j + 2] << std::endl;
        }
        CMN_LOG_CLASS_RUN_DEBUG << "Traking Template Data (" << tool->Name << ") [2]:  " << tool->MarkerTemplatePositions[3 * 2] << ",   " << tool->MarkerTemplatePositions[3 * 2 + 1] << ",   " << tool->MarkerTemplatePositions[3 * 2 + 2] << std::endl;
#endif
        /*        for (unsigned int j = 0; j < 2; j++) {
                  for (unsigned int k = 0; k < 3; k++) {
                  tool->MarkerTemplateTrackingPositions[j].at(k) = endPt[3 * j + k];

                  if (((endPt[0] != endPt1[3 * j]) || (endPt[1] != endPt1[3 * j + 1]) || (endPt[2] != endPt1[3 * j + 2])) && ((endPt[3] != endPt1[3 * j]) || (endPt[4] != endPt1[3 * j + 1]) || (endPt[5] != endPt1[3 * j + 2])) ) {

                  tool->MarkerTemplateTrackingPositions[2].at(k) = endPt1[3 * j + k];
                  }
                  }
                  CMN_LOG_CLASS_RUN_DEBUG << "Traking Template Data (" << tool->Name << ") [" << j << "]:  " << tool->MarkerTemplateTrackingPositions[j].X() << ",   " << tool->MarkerTemplateTrackingPositions[j].Y() << ",   " << tool->MarkerTemplateTrackingPositions[j].Z()<< std::endl;
                  }
                  CMN_LOG_CLASS_RUN_DEBUG << "Traking Template Data (" << tool->Name << ") [2]:  " << tool->MarkerTemplateTrackingPositions[2].X() << ",   " << tool->MarkerTemplatePositions[2].Y() << ",   " << tool->MarkerTemplatePositions[2].Z()<< std::endl;


                  // To get Template data from the markers
                  mtHandle TemplateFacets = Collection_New();
                  MTC(Marker_TemplateFacetsGet(markerHandle, &TemplateFacets));

                  mtHandle facetHandle = Collection_Int(TemplateFacets, 1);

                  double * templatePos = new double[12];
                  MTC(Facet_TemplatePositionsGet(facetHandle, templatePos));

                  for (unsigned int j = 0; j < 2; j++) {
                  for (unsigned int k = 0; k < 3; k++) {
                  tool->MarkerTemplatePositions[j].at(k) = endPt[3 * j + k];

                  if (((endPt[0] != endPt1[3 * j]) || (endPt[1] != endPt1[3 * j + 1]) || (endPt[2] != endPt1[3 * j + 2])) && ((endPt[3] != endPt1[3 * j]) || (endPt[4] != endPt1[3 * j + 1]) || (endPt[5] != endPt1[3 * j + 2])) ) {

                  tool->MarkerTemplatePositions[2].at(k) = endPt1[3 * j + k];

                  }
                  }
                  CMN_LOG_CLASS_RUN_DEBUG << "Traking Template Data (" << tool->Name << ") [" << j << "]:  " << tool->MarkerTemplatePositions[j].X() << ",   " << tool->MarkerTemplatePositions[j].Y() << ",   " << tool->MarkerTemplatePositions[j].Z()<< std::endl;
                  }
                  CMN_LOG_CLASS_RUN_DEBUG << "Traking Template Data (" << tool->Name << ") [2]:  " << tool->MarkerTemplatePositions[2].X() << ",   " << tool->MarkerTemplatePositions[2].Y() << ",   " << tool->MarkerTemplatePositions[2].Z()<< std::endl;
        */
    }

    for (toolIterator = Tools.begin(); toolIterator != end; ++toolIterator) {
        Tool * tool = toolIterator->second;
        if (tool->Visible != tool->TooltipPosition.Valid()) {
            tool->Visible = tool->TooltipPosition.Valid();
            tool->VisibleEvent(tool->Visible);
        }
    }
}


void mtsMicronTracker::TrackXPoint(void)
{
    XPoints.clear();

    mtHandle markerHandle;
    char markerName[MT_MAX_STRING_LENGTH];

    MTC(Markers_ProcessFrame(TrackerData->CurrentCamera));
    MTC(Markers_IdentifiedMarkersGet(TrackerData->CurrentCamera,
                                     TrackerData->IdentifiedMarkers));
    const unsigned int numIdentifiedMarkers =
        Collection_Count(TrackerData->IdentifiedMarkers);
    CMN_LOG_CLASS_RUN_DEBUG << "Track: identified " << numIdentifiedMarkers << " marker(s)" << std::endl;

    if (numIdentifiedMarkers != 0) {
        for (unsigned int i = 1; i <= numIdentifiedMarkers; i++) {
            markerHandle = Collection_Int(TrackerData->IdentifiedMarkers, i);
            MTC(Marker_NameGet(markerHandle, markerName, MT_MAX_STRING_LENGTH, 0));

            // get the tracking data of template points
            mtHandle IdentifiedFacets = Collection_New();
            MTC(Marker_IdentifiedFacetsGet(markerHandle,
                                           TrackerData->CurrentCamera,
                                           false, IdentifiedFacets));

            mtHandle lvHandle = Vector_New();
            mtHandle svHandle = Vector_New();
            MTC(Facet_IdentifiedVectorsGet(Collection_Int(IdentifiedFacets, 1), lvHandle, svHandle));

            double endPt[6];
            double endPt1[6];
            MTC(Vector_EndPosGet(lvHandle, endPt));
            MTC(Vector_EndPosGet(svHandle, endPt1));

            for (unsigned int j = 0; j < 2; j++) {
                for (unsigned int k = 0; k < 3; k++) {
                    XPoints[3 * (i-1) + j].at(k) = endPt[3 * j + k];

                    if (((endPt[0] != endPt1[3 * j]) || (endPt[1] != endPt1[3 * j + 1]) || (endPt[2] != endPt1[3 * j + 2])) && ((endPt[3] != endPt1[3 * j]) || (endPt[4] != endPt1[3 * j + 1]) || (endPt[5] != endPt1[3 * j + 2])) ) {

                        XPoints[3*(i-1)+2].at(k) = endPt1[3 * j + k];

                    }
                }
                CMN_LOG_CLASS_RUN_DEBUG << "XPoint[" << 3*(i-1)+j << "]: " << XPoints[3*(i-1)+j].X() << ",   " << XPoints[3*(i-1)+j].Y() << ",   " << XPoints[3*(i-1)+j].Z()<< std::endl;

            }
            CMN_LOG_CLASS_RUN_DEBUG << "XPoint[" << 3*(i-1)+2 << "]: " << XPoints[3*(i-1)+2].X() << ",   " << XPoints[3*(i-1)+2].Y() << ",   " << XPoints[3*(i-1)+2].Z() << std::endl;

            for (unsigned int l = 0; l < 3; l++) {
                MTC(Camera_ProjectionOnImage(TrackerData->CurrentCamera,
                                             LEFT_CAMERA, XPoints[3*(i-1)+l].Pointer(),
                                             &(XPointsProjectionLeft[3*(i-1)+l].X()),
                                             &(XPointsProjectionLeft[3*(i-1)+l].Y())) );

                MTC(Camera_ProjectionOnImage(TrackerData->CurrentCamera,
                                             RIGHT_CAMERA, XPoints[3*(i-1)+l].Pointer(),
                                             &(XPointsProjectionRight[3*(i-1)+l].X()),
                                             &(XPointsProjectionRight[3*(i-1)+l].Y())) );
                CMN_LOG_CLASS_RUN_DEBUG << "XPoint[" << 3*(i-1)+l << "] left: " << XPointsProjectionLeft[3*(i-1)+l].X() << ",   " << XPointsProjectionLeft[3*(i-1)+l].Y() << std::endl;
                CMN_LOG_CLASS_RUN_DEBUG << "XPoint[" << 3*(i-1)+l << "] right: " << XPointsProjectionRight[3*(i-1)+l].X() << ",   " << XPointsProjectionRight[3*(i-1)+l].Y() << std::endl;
            }


            // To get Template data from the markers
            mtHandle TemplateFacets = Collection_New();
            MTC(Marker_TemplateFacetsGet(markerHandle, &TemplateFacets));

            mtHandle facetHandle = Collection_Int(TemplateFacets, 1);

            double * templatePos = new double[12];
            MTC(Facet_TemplatePositionsGet(facetHandle, templatePos));


            for (unsigned int r = 0; r < 4; r++)
                CMN_LOG_CLASS_RUN_DEBUG << "template positions " << "[" << r << "]:   " << templatePos[3 * r] << ",   " << templatePos[3 * r + 1] << ",   " << templatePos[3 * r + 2] << std::endl;
        }
    }

    mtHandle IdentifiedXPoints = Collection_New();

    MTC(XPoints_ProcessFrame(TrackerData->CurrentCamera));

    MTC(XPoints_DetectedXPointsGet(TrackerData->CurrentCamera, IdentifiedXPoints));
    CMN_LOG_CLASS_RUN_DEBUG << "identified XPoints: " << Collection_Count(IdentifiedXPoints) << std::endl;

    for (int i = 0; i < Collection_Count(IdentifiedXPoints); i++) {

        // check if tool exists, generate a name and add it otherwise
        mtHandle XPointHandle = Collection_Int(IdentifiedXPoints, i+1);

        MTC (XPoint_3DPositionGet(XPointHandle, &XPoints[3 * numIdentifiedMarkers + i].X(), &XPoints[3 * numIdentifiedMarkers + i].Y(), &XPoints[3 * numIdentifiedMarkers + i].Z()));

        CMN_LOG_CLASS_RUN_DEBUG << " XPoint" << 3 * numIdentifiedMarkers + i  << ": " << XPoints[3 * numIdentifiedMarkers + i].X()  << ", " << XPoints[3 * numIdentifiedMarkers + i].Y() << ", " << XPoints[3 * numIdentifiedMarkers + i].Z() << std::endl;

        MTC(Camera_ProjectionOnImage(TrackerData->CurrentCamera,
                                     LEFT_CAMERA,
                                     XPoints[3 * numIdentifiedMarkers + i].Pointer(),
                                     &(XPointsProjectionLeft[3 * numIdentifiedMarkers + i].X()),
                                     &(XPointsProjectionLeft[3 * numIdentifiedMarkers + i].Y())) );
        MTC(Camera_ProjectionOnImage(TrackerData->CurrentCamera,
                                     RIGHT_CAMERA,
                                     XPoints[3 * numIdentifiedMarkers + i].Pointer(),
                                     &(XPointsProjectionRight[3 * numIdentifiedMarkers + i].X()),
                                     &(XPointsProjectionRight[3 * numIdentifiedMarkers + i].Y())) );
        CMN_LOG_CLASS_RUN_DEBUG << "XPoint[" << 3 * numIdentifiedMarkers + i << "] left: " << XPointsProjectionLeft[3 * numIdentifiedMarkers + i].X() << ",   " << XPointsProjectionLeft[3 * numIdentifiedMarkers + i].Y() << std::endl;
        CMN_LOG_CLASS_RUN_DEBUG << "XPoint[" << 3 * numIdentifiedMarkers + i << "] right: " << XPointsProjectionRight[3 * numIdentifiedMarkers + i].X() << ",   " << XPointsProjectionRight[3 * numIdentifiedMarkers + i].Y() << std::endl;
    }
    Collection_Free(IdentifiedXPoints);
}

void mtsMicronTracker::ComputeCameraModel(const std::string & pathRectificationLUT)
{
    CMN_LOG_CLASS_RUN_WARNING << "ComputeCameraModel: exporting rectification LUT to " << pathRectificationLUT << std::endl;

#if CISST_HAS_CISSTNETLIB
    int resolutionX, resolutionY;
    int retval;
    calRay ray;

    MTC(Markers_ProcessFrame(CurrentCamera));
    MTC(Camera_ResolutionGet(CurrentCamera, &resolutionX, &resolutionY));

    vct2 imageCenterPX(resolutionX / 2.0, resolutionY / 2.0);
    double imagePlaneZ = 1500.0;

    unsigned int numPointsMax = resolutionX * resolutionY;
    vctMat interpolatedRay(numPointsMax, 6, VCT_COL_MAJOR);
    vctMat L(3 * numPointsMax, 4, VCT_COL_MAJOR);
    vctVec T(3 * numPointsMax);
    vctMat I(2 * numPointsMax, 4, VCT_COL_MAJOR);
    vctVec Q_2D(2 * numPointsMax);
    vctVec ray_dir_2D(2 * numPointsMax);
    vctDynamicMatrixRef<double> matrixRef;
    vctDynamicVectorRef<double> vectorRef;

    CMN_LOG_CLASS_RUN_WARNING << "ComputeCameraModel: computing camera model" << std::endl;
    unsigned int index = 0;
    for (double x = 0; x <= resolutionX; x++) {
        for (double y = 0; y <= resolutionY; y++) {
            retval = Camera_InterpolatedRayGet(CurrentCamera, x, y, LEFT_CAMERA, &ray);
            if (retval == mtOK) {
                matrixRef.SetRef(1, 6, interpolatedRay.row_stride(), interpolatedRay.col_stride(), interpolatedRay.Pointer(index, 0));
                matrixRef.Assign(x, y, ray.X0, ray.Y0, ray.Dx2Dz, ray.Dy2Dz);

                matrixRef.SetRef(3, 3, L.row_stride(), L.col_stride(), L.Pointer(3*index, 0));
                matrixRef.Assign(vctMat::Eye(3));

                matrixRef.SetRef(3, 1, L.row_stride(), L.col_stride(), L.Pointer(3*index, 3));
                matrixRef.Assign(ray.Dx2Dz, ray.Dy2Dz, 1.0);

                vectorRef.SetRef(3, T.Pointer(3*index));
                vectorRef.Assign(ray.X0, ray.Y0, 0.0);

                matrixRef.SetRef(2, 4, I.row_stride(), I.col_stride(), I.Pointer(2*index, 0));
                matrixRef.Assign(x-imageCenterPX[0], 0.0, 1.0, 0.0, 0.0, y-imageCenterPX[1], 0.0, 1.0);

                vectorRef.SetRef(2, Q_2D.Pointer(2*index));
                vectorRef.Assign(ray.X0+(imagePlaneZ*ray.Dx2Dz), ray.Y0+(imagePlaneZ*ray.Dy2Dz));

                vectorRef.SetRef(2, ray_dir_2D.Pointer(2*index));
                vectorRef.Assign(ray.Dx2Dz, ray.Dy2Dz);

                index++;
            }
        }
    }
    unsigned int numInterpolatedRays = index;

    interpolatedRay.resize(numInterpolatedRays, 6);
    L.resize(3 * numInterpolatedRays, 4);
    T.resize(3 * numInterpolatedRays);
    I.resize(2 * numInterpolatedRays, 4);
    Q_2D.resize(2 * numInterpolatedRays);
    ray_dir_2D.resize(2 * numInterpolatedRays);

    vctVec temp1(T.size());
    vct3 cameraOriginMM;
    nmrLSqLin(L, T, temp1);
    for (unsigned int i = 0; i < 3; i++) {
        cameraOriginMM[i] = temp1[i];
    }

    //    vctVec temp2(ray_dir_2D.size());
    //    vct2 solution2;
    //    nmrLSqLin(I, ray_dir_2D, temp2);
    //    for (unsigned int i = 2; i < 4; i++) {
    //        solution2[i] = temp2[i];
    //    }
    //    CMN_LOG_CLASS_RUN_WARNING << "ComputeCameraModel: direction of the ray at image center:\n" << solution2 << std::endl;

    vctVec temp3(Q_2D.size());
    vct2 pixelSizeMM;
    vct2 solution3;
    nmrLSqLin(I, Q_2D, temp3);
    for (unsigned int i = 0; i < 2; i++) {
        pixelSizeMM[i] = temp3[i];
        solution3[i] = temp3[i+2];
    }

    vct2 imageOriginPX(cameraOriginMM[0], cameraOriginMM[1]);
    imageOriginPX.Subtract(solution3);
    imageOriginPX.ElementwiseDivide(pixelSizeMM);
    imageOriginPX.Add(imageCenterPX);

    double focalLengthMM = imagePlaneZ + abs(cameraOriginMM[2]);
    vct2 focalLengthPX(focalLengthMM, focalLengthMM);
    focalLengthPX.ElementwiseDivide(pixelSizeMM);

    vct3x3 intrinsics(focalLengthPX[0], 0.0, imageOriginPX[0],
                      0.0, focalLengthPX[1], imageOriginPX[1],
                      0.0, 0.0, 1.0);
    vct4x4 extrinsics(1.0, 0.0, 0.0, cameraOriginMM[0],
                      0.0, 1.0, 0.0, cameraOriginMM[1],
                      0.0, 0.0, 1.0, cameraOriginMM[2],
                      0.0, 0.0, 0.0, 1.0);
    CMN_LOG_CLASS_RUN_WARNING << "ComputeCameraModel:\n"
                              << " * instrinsics:\n" << intrinsics << "\n"
                              << " * extrinsics:\n" << extrinsics << std::endl;

    CMN_LOG_CLASS_RUN_WARNING << "ComputeCameraModel: exporting rectification lookup table" << std::endl;

    std::ostringstream str_resolution;
    std::ostringstream str_ind_new;
    std::ostringstream str_ind_1, str_ind_2, str_ind_3, str_ind_4;
    std::ostringstream str_a1, str_a2, str_a3, str_a4;

    str_resolution.flags(std::ios::scientific);
    str_ind_new.flags(std::ios::scientific);
    str_ind_1.flags(std::ios::scientific);
    str_ind_2.flags(std::ios::scientific);
    str_ind_3.flags(std::ios::scientific);
    str_ind_4.flags(std::ios::scientific);
    str_a1.flags(std::ios::scientific);
    str_a2.flags(std::ios::scientific);
    str_a3.flags(std::ios::scientific);
    str_a4.flags(std::ios::scientific);

    str_resolution.precision(7);
    str_ind_new.precision(7);
    str_ind_1.precision(7);
    str_ind_2.precision(7);
    str_ind_3.precision(7);
    str_ind_4.precision(7);
    str_a1.precision(7);
    str_a2.precision(7);
    str_a3.precision(7);
    str_a4.precision(7);

    str_resolution << "  " << static_cast<double>(resolutionY) << "\n"
                   << "  " << static_cast<double>(resolutionX);
    for (unsigned int i = 0; i < numInterpolatedRays; i++) {
        double imageXYZ[3];
        imageXYZ[0] = (interpolatedRay(i,0) - imageCenterPX[0]) * pixelSizeMM[0];
        imageXYZ[1] = (interpolatedRay(i,1) - imageCenterPX[1]) * pixelSizeMM[1];
        imageXYZ[2] = 0.0;

        double trackerXYZ[3];
        trackerXYZ[0] = imageXYZ[0] + cameraOriginMM[0];
        trackerXYZ[1] = imageXYZ[1] + cameraOriginMM[1];
        trackerXYZ[2] = imageXYZ[2] + imagePlaneZ;

        double positionX, positionY;
        retval = Camera_ProjectionOnImage(CurrentCamera, LEFT_CAMERA, trackerXYZ, &positionX, &positionY);

        positionX += 0.5;
        positionY += 0.5;

        int pixelX = static_cast<int>(floor(positionX));
        int pixelY = static_cast<int>(floor(positionY));

        double alphaX = positionX - pixelX;
        double alphaY = positionY - pixelY;

        double a1 = (1 - alphaY) * (1 - alphaX);
        double a2 = (1 - alphaY) * alphaX;
        double a3 = alphaY * (1 - alphaX);
        double a4 = alphaY * alphaX;

        pixelX += 1;
        pixelY += 1;

        double ind_1 = pixelX * resolutionY + (pixelY + 1);
        double ind_2 = (pixelX + 1) * resolutionY + (pixelY + 1);
        double ind_3 = pixelX * resolutionY + (pixelY + 2);
        double ind_4 = (pixelX + 1) * resolutionY + (pixelY + 2);

        double ind_new = interpolatedRay(i,0) * resolutionY + (interpolatedRay(i,1) + 1);

        if (retval == mtOK) {
            str_ind_new << "  " << ind_new;
            str_ind_1 << "  " << ind_1;
            str_ind_2 << "  " << ind_2;
            str_ind_3 << "  " << ind_3;
            str_ind_4 << "  " << ind_4;
            str_a1 << "  " << a1;
            str_a2 << "  " << a2;
            str_a3 << "  " << a3;
            str_a4 << "  " << a4;
        }
    }

    std::ofstream file;
    file.open(pathRectificationLUT.c_str(), std::ios::out);
    file << str_resolution.str() << "\n"
         << str_ind_new.str() << "\n"
         << str_ind_1.str() << "\n"
         << str_ind_2.str() << "\n"
         << str_ind_3.str() << "\n"
         << str_ind_4.str() << "\n"
         << str_a1.str() << "\n"
         << str_a2.str() << "\n"
         << str_a3.str() << "\n"
         << str_a4.str() << std::endl;
    file.close();
    CMN_LOG_CLASS_RUN_WARNING << "ComputeCameraModel: done" << std::endl;

#else
    CMN_LOG_CLASS_RUN_WARNING << "ComputeCameraModel: requires cisstNetlib to solve for model parameters" << std::endl;
#endif
}


mtsMicronTracker::Tool::Tool(void) :
    TooltipOffset(0.0),
    Visible(false)
{
    TooltipPosition.SetValid(false);
    MarkerProjectionLeft.SetSize(2);
    MarkerProjectionLeft.SetAll(0.0);
    MarkerProjectionRight.SetSize(2);
    MarkerProjectionRight.SetAll(0.0);
    MarkerTemplateTrackingPositions.SetSize(9);
    MarkerTemplateTrackingPositions.SetAll(0.0);
    MarkerTemplatePositions.SetSize(9);
    MarkerTemplatePositions.SetAll(0.0);
    MarkerTemplateProjectionLeft.resize(3);
    MarkerTemplateProjectionLeft.clear();
    MarkerTemplateProjectionRight.resize(3);
    MarkerTemplateProjectionRight.clear();
}
