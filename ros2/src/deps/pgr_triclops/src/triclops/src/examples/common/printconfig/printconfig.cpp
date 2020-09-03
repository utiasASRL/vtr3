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
// printconfig:
//
// Prints the Triclops library version as well as camera information.
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

//=============================================================================
// Project Includes
//=============================================================================

//
// Macro to check, report on, and handle Triclops API error codes.
//
#define _HANDLE_TRICLOPS_ERROR( description, error )	\
{ \
   if( error != TriclopsErrorOk ) \
   { \
      printf( \
      "*** Triclops Error '%s' at line %d :\n\t%s\n", \
      triclopsErrorToString( error ), \
      __LINE__, \
      description );	\
      printf( "Press any key to exit...\n" ); \
      getchar(); \
      exit( 1 ); \
   } \
} \

int
main( int /* argc */, char** /* argv */ )
{
    TriclopsError   te;
    TriclopsContext triclops;

    float baseline;
    int   nrows, ncols;
    float focalLength;

    TriclopsCameraConfiguration triclopsConfig;

    // Camera setup
   
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
    Fc2Triclops::ErrorType fc2TriclopsError = 
        Fc2Triclops::getContextFromCamera( cameraInfo.serialNumber, &triclops );
    if (fc2TriclopsError != Fc2Triclops::ERRORTYPE_OK)
    {
        return Fc2Triclops::handleFc2TriclopsError(fc2TriclopsError, 
		                                    "getContextFromCamera");
    }

    // Print Triclops info
    printf( "Triclops Version  : %s\n", triclopsVersion() );
   
    // get the camera configuration 
    te = triclopsGetCameraConfiguration( triclops, &triclopsConfig );
    _HANDLE_TRICLOPS_ERROR( "triclopsGetCameraConfiguration()", te );

    switch( triclopsConfig )
    {
        case TriCfg_L:
            printf( "Configuration   : 3 Camera\n" );                   
            break;
        case TriCfg_2CAM_HORIZONTAL:      
            printf( "Configuration   : 2 Camera horizontal\n" );                   
            break;
        case TriCfg_2CAM_VERTICAL:
            printf( "Configuration   : 2 Camera vertical\n" );                   
            break;
        case TriCfg_2CAM_HORIZONTAL_WIDE:
            printf( "Configuration   : 2 Camera horizontal wide\n" );
            break;
        default:
            printf( "Unrecognized configuration: %d\n", triclopsConfig ); 
    }
   
    // Get the baseline
    te = triclopsGetBaseline( triclops, &baseline );
    _HANDLE_TRICLOPS_ERROR( "triclopsGetBaseline()", te );
    printf( "Baseline        : %f cm\n", baseline*100.0 );
   
    // Get the focal length
    te = triclopsGetFocalLength( triclops, &focalLength );
    _HANDLE_TRICLOPS_ERROR( "triclopsGetFocalLength()", te );
    te = triclopsGetResolution( triclops, &nrows, &ncols );
    _HANDLE_TRICLOPS_ERROR( "triclopsGetResolution()", te );
   
    printf( "Focal Length    : %f pixels for a %d x %d image\n", 
	        focalLength, 
	        ncols, 
	        nrows ) ;

    int   nRows, nCols;
    te = triclopsGetResolution( triclops, &nRows, &nCols );
    _HANDLE_TRICLOPS_ERROR( "triclopsGetResolution()", te );

    float fCenterRow, fCenterCol;
    te = triclopsGetImageCenter( triclops, &fCenterRow, &fCenterCol );
    _HANDLE_TRICLOPS_ERROR( "triclopsGetImageCenter()", te );

    printf( "The default image resolution for stereo processing is %d x %d\n", 
                nCols, nRows );
    printf( "For this resolution, the 'image center' or 'principal point' is:\n" );
    printf( "Center Row = %f\n", fCenterRow );
    printf( "Center Col = %f\n", fCenterCol );

   
    // Destroy the Triclops context
    triclopsDestroyContext( triclops ) ;
   
    return 0;
}
