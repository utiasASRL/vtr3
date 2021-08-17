//=============================================================================
// Copyright © 2004 Point Grey Research, Inc. All Rights Reserved.
// 
// This software is the confidential and proprietary information of Point
// Grey Research, Inc. ("Confidential Information").  You shall not
// disclose such Confidential Information and shall use it only in
// accordance with the terms of the license agreement you entered into
// with Point Grey Research, Inc. (PGR).
// 
// PGR MAKES NO REPRESENTATIONS OR WARRANTIES ABOUT THE SUITABILITY OF THE
// SOFTWARE, EITHER EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR
// PURPOSE, OR NON-INFRINGEMENT. PGR SHALL NOT BE LIABLE FOR ANY DAMAGES
// SUFFERED BY LICENSEE AS A RESULT OF USING, MODIFYING OR DISTRIBUTING
// THIS SOFTWARE OR ITS DERIVATIVES.
//=============================================================================
//=============================================================================
// $Id: writecal.cpp,v 1.9 2010-08-05 20:40:32 arturp Exp $
//=============================================================================
//=============================================================================
// writecal:
//
// Retrieves a .cal calibration file from the camera and writes it to disk.
//
//=============================================================================

//=============================================================================
// System Includes
//=============================================================================
#include <stdio.h>
#include <stdlib.h>

//=============================================================================
// PGR Includes
//=============================================================================
#include "triclops.h"
#include "fc2triclops.h"

#include <FlyCapture2.h>

int
main( int /* argc */, char** /* argv */ )
{
    FlyCapture2::Camera camera;
    camera.Connect();

    FlyCapture2::CameraInfo cameraInfo;
    FlyCapture2::Error fc2Error = camera.GetCameraInfo(&cameraInfo);
    if (fc2Error != FlyCapture2::PGRERROR_OK)
    {
        fc2Error.PrintErrorTrace();
        return (int)fc2Error.GetType();
    }

    // Get the a triclops context from camera
    TriclopsContext context;
    Fc2Triclops::ErrorType fc2TriclopsError;
    fc2TriclopsError = Fc2Triclops::getContextFromCamera( cameraInfo.serialNumber, 
                                                         &context );
    if (fc2TriclopsError != Fc2Triclops::ERRORTYPE_OK)
    {
        return Fc2Triclops::handleFc2TriclopsError(fc2TriclopsError, 
		                                    "getContextFromCamera");
    }

    // A string to hold the calibration file name
    // We will construct a meaningful name for the written calibration file
    char szFilename[64];
#if defined( WIN64 ) || defined (WIN32) 
    sprintf_s( szFilename, sizeof(szFilename), "bumblebee%7.7u.cal", cameraInfo.serialNumber );
#else
    snprintf(szFilename, sizeof(szFilename), "bumblebee%7.7u.cal", cameraInfo.serialNumber);
#endif

    // Get the camera configuration
    TriclopsError te = triclopsWriteCurrentContextToFile( context, szFilename );
    if (te != TriclopsErrorOk)
    {
        printf("*** Triclops Error '%s' for call: %s\n",
	    triclopsErrorToString(te), "triclopsWriteCurrentContextToFile" );
    }

    printf("Calibration File successfully saved at %s", szFilename);
   
    // Destroy the triclops context
    triclopsDestroyContext( context );
   
    return 0;
}

