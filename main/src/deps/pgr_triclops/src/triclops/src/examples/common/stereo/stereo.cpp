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
// stereo:
//
// This example shows how to load raw stereo images from a PPM file, how
// to load camera calibration data from a file, and how to do stereo processing
// and generate a disparity image.
//
// This is the most basic Triclops example.
//
// Note: in this example invalid pixels are mapped to 255.  Other pixel values
// equal their disparity value.  Since all the disparity values are relatively
// small, the image appears to be binary in many viewers.  However, if the image
// is scaled (via stretching, adjusting the gamma or histogram equalization)
// you will see that there is quite a lot of information in the "black" region.
//
// You can perform this stretching by running the "scaleimage" example on the
// output disparity
//
//=============================================================================

//=============================================================================
// System Includes
//=============================================================================
#include <stdio.h>
#include <stdlib.h>
#include <string>

//=============================================================================
// PGR Includes
//=============================================================================
#include "triclops.h"
#include "pnmutils.h"

//=============================================================================
// Project Includes
//=============================================================================

// Print error and quit program
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
main( int argc, char** argv )
{
    TriclopsContext triclops;
    TriclopsError   error;
    TriclopsImage   triclopsImage;
    TriclopsImage16 triclopsImage16;
    TriclopsInput   triclopsInput;
    std::string szInputFile("input.ppm");
    std::string szCalFile("input.cal");
    std::string szDisparityFile("disparity.pgm");
    std::string szDisparity16File("disparity16.pgm");
    std::string szRectifiedFile("rectified.pgm");

    switch ( argc )
    {
        case 5:
            szCalFile	    = argv[1];
            szInputFile	    = argv[2];
            szDisparityFile = argv[3];
            szRectifiedFile = argv[4];
      break;
      case 1:
          // use defaults - no arguments
      break;
      default:
          printf( "Usage: stereo [<cal file> <input file>"
           "<output disparity file> <output rectified file>]\n" );
      break;
    }

    // Get the camera calibration data
    error = triclopsGetDefaultContextFromFile( &triclops, const_cast<char *>(szCalFile.c_str()) );
    _HANDLE_TRICLOPS_ERROR( "triclopsGetDefaultContextFromFile(): "
			   "Can't open calibration file", 
			   error );
   
    // Load images from file
    if ( !ppmReadToTriclopsInput( szInputFile.c_str(),  &triclopsInput ) )
    {
        printf( "ppmReadToTriclopsInput() failed. Can't read '%s'\n", szInputFile.c_str() );
        return 1;
    }
  
    // Rectify the images
    error = triclopsRectify( triclops, &triclopsInput );
    _HANDLE_TRICLOPS_ERROR( "triclopsRectify()", error );
    
    // Turn off subpixel
    error = triclopsSetSubpixelInterpolation( triclops, false );
    _HANDLE_TRICLOPS_ERROR( "triclopsSetSubpixelInterpolation()", error );

    // Do stereo processing
    error = triclopsStereo( triclops );
    _HANDLE_TRICLOPS_ERROR( "triclopsStereo()", error );
   
    // Retrieve the disparity image from the context
    error = triclopsGetImage( triclops, TriImg_DISPARITY, TriCam_REFERENCE, &triclopsImage );
    _HANDLE_TRICLOPS_ERROR( "triclopsGetImage(): Failed to get disparity image", 
			    error );
   
    // Save the disparity image
    error = triclopsSaveImage( &triclopsImage, const_cast<char *>(szDisparityFile.c_str()) );
    _HANDLE_TRICLOPS_ERROR( "triclopsSaveImage(): Failed to save disparity image", error );
    printf( "Wrote disparity image to '%s'\n", szDisparityFile.c_str() );

    // Turn on subpixel
    error = triclopsSetSubpixelInterpolation( triclops, true);
    _HANDLE_TRICLOPS_ERROR( "triclopsSetSubpixelInterpolation()", error );

    // Do stereo processing
    error =  triclopsStereo( triclops );
    _HANDLE_TRICLOPS_ERROR( "triclopsStereo()", error );
   
    // Retrieve the disparity image from the context
    error = triclopsGetImage16( triclops, TriImg16_DISPARITY, TriCam_REFERENCE, &triclopsImage16 );
    _HANDLE_TRICLOPS_ERROR( "triclopsGetImage16(): Failed to get disparity image", 
			    error );
   
    // Save the disparity image
    error = triclopsSaveImage16( &triclopsImage16, const_cast<char *>(szDisparity16File.c_str()) );
    _HANDLE_TRICLOPS_ERROR( "triclopsSaveImage16(): Failed to save disparity image", error );
    printf( "Wrote disparity image to '%s'\n", szDisparity16File.c_str() );

    // Do the same for the rectified reference image
    error = triclopsGetImage( triclops, TriImg_RECTIFIED, TriCam_REFERENCE, &triclopsImage );
    _HANDLE_TRICLOPS_ERROR( "triclopsGetImage(): Failed to get rectified image", 
			   error );
   
    error = triclopsSaveImage( &triclopsImage, const_cast<char *>(szRectifiedFile.c_str()) );
    _HANDLE_TRICLOPS_ERROR( "triclopsSaveImage(): Failed to save rectified image", error );
    printf( "Wrote rectified image to '%s'\n", szRectifiedFile.c_str() );

    // clean up memory allocated in context
    freeInput( &triclopsInput );
    error = triclopsDestroyContext( triclops );
    _HANDLE_TRICLOPS_ERROR( "triclopsDestroyContext()", error );
   
    return 0;
}

