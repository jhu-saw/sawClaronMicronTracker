/* -*- Mode: C++; tab-width: 4; indent-tabs-mode: nil; c-basic-offset: 4 -*-    */
/* ex: set filetype=cpp softtabstop=4 shiftwidth=4 tabstop=4 cindent expandtab: */

/*

  Author(s):  Ali Uneri
  Created on: 2009-11-06

  (C) Copyright 2009-2012 Johns Hopkins University (JHU), All Rights Reserved.

--- begin cisst license - do not edit ---

This software is provided "as is" under an open source license, with
no warranty.  The complete license can be found in license.txt and
http://www.cisst.org/cisst/license.txt.

--- end cisst license ---
*/

/*!
  \file
  \brief cisst component for Claron MicronTracker trackers.
  \ingroup sawComponents

  \bug Hard coded to only support 640x480 resolution cameras.
  \bug Automatic light coolness adjustment using CoolCard is not working.

  \warning HdrEnabledSet is disabled, since it's removed in the new API.

  \todo Replace current custom video player with an svl Qt widget (mtsMicronTracker already provides an svlSampleBuffer).
  \todo ComputeCameraModel method needs error analysis.
  \todo Mapping from markerName to markerHandle.
  \todo Refactor the method of obtaining marker projections for the controller Qt Component.
  \todo Check for mtMeasurementHazardCode using Xform3D_HazardCodeGet.
  \todo Fix/suppress _WIN32_WINNT macro redefinition warning.
  \todo See sawNDITracker for other common tracking related todos.
*/

#ifndef _mtsMicronTracker_h
#define _mtsMicronTracker_h

#include <vector>
#include <cisstVector/vctDynamicNArray.h>
#include <cisstVector/vctFixedSizeVectorTypes.h>
#include <cisstMultiTask/mtsTaskPeriodic.h>
#include <cisstMultiTask/mtsVector.h>
#include <cisstMultiTask/mtsTransformationTypes.h>
#include <cisstOSAbstraction/osaGetTime.h>
#include <cisstParameterTypes/prmPositionCartesianGet.h>
#include <cisstStereoVision/svlBufferSample.h>
#include <sawClaronMicronTracker/sawClaronMicronTrackerExport.h>  // always include last

#include <MTC.h>


class CISST_EXPORT mtsMicronTracker : public mtsTaskPeriodic
{
    CMN_DECLARE_SERVICES(CMN_DYNAMIC_CREATION_ONEARG, CMN_LOG_ALLOW_DEFAULT);

 protected:
    class Tool
    {
     public:
        Tool(void);
        ~Tool(void) {};

        std::string Name;
        mtHandle Handle;
        std::string SerialNumber;
        mtsInterfaceProvided * Interface;
        prmPositionCartesianGet TooltipPosition;
        prmPositionCartesianGet MarkerPosition;
        mtsDoubleVec MarkerProjectionLeft;
        mtsDoubleVec MarkerProjectionRight;

        mtsDoubleVec MarkerTemplateTrackingPositions;
        mtsDoubleVec MarkerTemplatePositions;
        std::vector<vct3> MarkerTemplateProjectionLeft;
        std::vector<vct3> MarkerTemplateProjectionRight;

        vct3 TooltipOffset;
    };

 public:
    mtsMicronTracker(const std::string & taskName, const double period) :
        mtsTaskPeriodic(taskName, period, false, 5000) { Construct(); }
    mtsMicronTracker(const mtsTaskPeriodicConstructorArg & arg) :
        mtsTaskPeriodic(arg) { Construct(); }
    ~mtsMicronTracker(void) {};

    void Construct(void);
    void Configure(const std::string & filename = "");
    void Startup(void);
    void Run(void);
    void Cleanup(void);

    size_t GetNumberOfTools(void) const {
        return Tools.size();
    }
    std::string GetToolName(const unsigned int index) const;
    svlBufferSample * GetImageBufferLeft(void) {
        return ImageBufferLeft;
    }
    svlBufferSample * GetImageBufferRight(void) {
        return ImageBufferRight;
    }

 protected:
    enum { LEFT_CAMERA, RIGHT_CAMERA, MIDDLE_CAMERA };

    Tool * CheckTool(const std::string & serialNumber);
    Tool * AddTool(const std::string & name, const std::string & serialNumber);

    vctFrm3 XfHandleToFrame(mtHandle & xfHandle);
    mtHandle FrameToXfHandle(vctFrm3 & frame);

    void InitComponent(void);  // called from constructor

    void ToggleCapturing(const bool & toggle);
    void ToggleTracking(const bool & toggle);
    void Track(void);
    void TrackXPoint(void);
    void CalibratePivot(const std::string & toolName);
    void ComputeCameraModel(const std::string & pathRectificationLUT);
    void SetJitterFilterEnabled(const mtsBool & flag);
    void SetJitterCoefficient(const double & coefficient);
    void SetKalmanFilterEnabled(const mtsBool & flag);


    int FrameWidth;
    int FrameHeight;

    std::string CameraCalibrationDir;
    std::string MarkerTemplatesDir;

    typedef cmnNamedMap<Tool> ToolsType;
    ToolsType Tools;

    bool IsCapturing;
    bool IsTracking;

    int XPointsMaxNum;
    std::vector<vct3> XPoints;
    std::vector<vct3> XPointsProjectionLeft;
    std::vector<vct3> XPointsProjectionRight;

    mtHandle CurrentCamera;
    mtHandle IdentifyingCamera;
    mtHandle IdentifiedMarkers;
    mtHandle PoseXf;
    mtHandle Path;
    mtsDoubleVec MarkerProjectionLeft;

    mtsStateTable * ImageTable;
    mtsUCharVec ImageLeft;
    mtsUCharVec ImageRight;
    svlSampleImageRGB * RGB;
    svlBufferSample * ImageBufferLeft;
    svlBufferSample * ImageBufferRight;
};

CMN_DECLARE_SERVICES_INSTANTIATION(mtsMicronTracker);

#endif  // _mtsMicronTracker_h
