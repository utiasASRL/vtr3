//=============================================================================
// Copyright (C) 2010 Point Grey Research, Inc. All Rights Reserved.
//
// This software is the confidential and proprietary information of Point
// Grey Research, Inc. ("Confidential Information").  You shall not
// disclose such Confidential Information and shall use it only in
// accordance with the terms of the license agreement you entered into
// with PGR.
//
// PGR MAKES NO REPRESENTATIONS OR WARRANTIES ABOUT THE SUITABILITY OF THE
// SOFTWARE, EITHER EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
// IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR
// PURPOSE, OR NON-INFRINGEMENT. PGR SHALL NOT BE LIABLE FOR ANY DAMAGES
// SUFFERED BY LICENSEE AS A RESULT OF USING, MODIFYING OR DISTRIBUTING
// THIS SOFTWARE OR ITS DERIVATIVES.
//=============================================================================

#ifndef FC2TRICLOPS_H
#define FC2TRICLOPS_H

//=============================================================================
// PGR Includes
//=============================================================================

#include "flycapture2bridge.h"
#include <FlyCapture2.h>

//=============================================================================
// System Includes
//=============================================================================

#include <assert.h>

namespace Fc2Triclops
{

    //
    // Name: StereoCameraMode
    // 
    // Description:
    //  The enumerated type StereoCameraMode indicates in which stereo mode 
    //  the camera captures images simultaneously. Format_7 Mode 3 is set 
    //  for all modes.
    //

    enum StereoCameraMode {
      // Two images are captured with Mono16 pixel format for monochrome 
      // cameras and Raw16 pixel format for color cameras. For two sensors 
      // cameras, images are tramsmitted in right-left order. If this mode 
      // is selected for three sensors cameras, it will be changed to 
      // TWO_CAMERA_NARROW mode. 
      TWO_CAMERA = 1,
      // Two images are captured with Mono16 pixel format for monochrome 
      // cameras and Raw16 pixel format for color cameras. This mode is 
      // available for three sensors cameras. The image are captured by the 
      // right and center sensors.
      TWO_CAMERA_NARROW,
      // Two images are captured with Mono16 pixel format for monochrome 
      // cameras and Raw16 pixel format for color cameras. This mode is 
      // available for three sensors cameras. The image are captured by the 
      // right and left sensors.
      TWO_CAMERA_WIDE,
      // Three images are captured with RGB8 pixel format for monochrome 
      // cameras and 8-bit Bayer interleaved pixel format for color cameras. 
      THREE_CAMERA
    };

    //
    // Name: setStereoMode
    // 
    // Synopsis:
    //  Sets a capturing and trasmitting mode according to the connected camera
    // 
    // Input:
    //  cam  - The camera
    //  mode - The stereo mode
    //
    // Returns:
    //  ERRORTYPE_OK                    - The operation succeeded.
    //  ERRORTYPE_CONFIG_READ_FAILED    - The reading of camera configuration failed.
    //  ERRORTYPE_NO_FMT7_SUPPORT       - The Format7 is not supported.
    //  ERRORTYPE_INVALID_FMT7          - The Format7 configuation is invalid.
    //  ERRORTYPE_NO_CORRECT_CAMERAMODE - The stero mode value is not correct.
    // 
    // Description:
    //  This function configures a capturing and trasmission mode for 
    //  the connected camera.
    //
    //
    ErrorType
    setStereoMode( FlyCapture2::Camera & cam, StereoCameraMode & mode );

    //
    // Name: isBB2
    // 
    // Synopsis:
    //  Checks if connected camera is a 2 sensors Bumblebee camera
    // 
    // Input:
    //  cam  - The camera
    //
    // Output:
    //  flag - the boolean flag
    //
    // Returns:
    //  ERRORTYPE_OK                    - The operation succeeded.
    //  ERRORTYPE_CONFIG_READ_FAILED    - The reading of camera configuration failed.
    // 
    // Description:
    //  This function is used to know if the connected camera is a two sensors 
    //  Bumblebee camera.  
    //
    //
    ErrorType 
    isBB2( const FlyCapture2::Camera &cam, bool & flag );
   
    //
    // Name: isBB3
    // 
    // Synopsis:
    //  Checks if connected camera is a 3 sensors Bumblebee camera
    // 
    // Input:
    //  cam  - The camera
    //
    // Output:
    //  flag - the boolean flag
    //
    // Returns:
    //  ERRORTYPE_OK                    - The operation succeeded.
    //  ERRORTYPE_CONFIG_READ_FAILED    - The reading of camera configuration 
    //                                     failed.
    // 
    // Description:
    //  This function is used to know if the connected camera is a three sensors 
    //  Bumblebee camera.  
    //
    //
    ErrorType
    isBBX3( const FlyCapture2::Camera &cam, bool & flag );

    //
    // Name: handleFc2Error
    // 
    // Synopsis:
    //  Handles FlyCap2 errors
    // 
    // Input:
    //  fc2error  - The error
    //
    // Returns:
    //  The value of the FlyCapture2::ErrorType enumerator
    // 
    // Description:
    //  This function is used to handles errors generated from FlyCap2 objects 
    //  (e.g. camera, image ). It also prints out a formatted log trace
    //
    //
    int
    handleFc2Error( FlyCapture2::Error const & fc2Error );

    //
    // Name: handleFc2TriclopsError
    // 
    // Synopsis:
    //  Handles FlyCapture2Triclops errors
    // 
    // Input:
    //  error        - The error
    //  pCallNameStr - The pointer to the string that is used in the formatted 
    //                 log trace
    //
    // Returns:
    //  The value of the Fc2Triclops::ErrorType enumerator plus the value 1000
    // 
    // Description:
    //  This function is used to handles errors generated from Fc2Triclops 
    //  functions. It also prints out a formatted log trace
    //
    //
    int 
    handleFc2TriclopsError( Fc2Triclops::ErrorType const & error, 
			   const char* pCallNameStr );
			   
} // end namespace Fc2Triclops

#endif // FC2TRICLOPS_H

