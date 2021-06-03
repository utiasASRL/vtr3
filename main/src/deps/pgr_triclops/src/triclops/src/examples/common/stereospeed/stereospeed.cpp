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
// $Id: stereospeed.cpp,v 1.3 2005-02-18 02:48:40 mgibbons Exp $
//=============================================================================
//=============================================================================
// stereospeed:
//
// This example is a profiling program that shows how fast the stereo kernel can
// run on your PC.  It uses a dummy image, but swaps between two copies of the 
// dummy image each iteration.  This is important to make sure that the stereo
// kernel is having to load different memory into its cache each run.
//
// The timing is typically the same whether you swap images or not, but on 
// systems with very large cache this may not be true.
//
// The purpose of this example is to give the user an idea of their maximum
// disparity-pixels/second benchmark.
//
// Note, because the timer is using computer time, rather than  "wall clock" 
// time, the 10 second profiling run may take longer on your system if it is
// being slowed by other programs.
//
//=============================================================================

//=============================================================================
// System Includes
//=============================================================================
#include <stdio.h>
#include <string>

//=============================================================================
// PGR Includes
//=============================================================================
#include "triclops.h"
#include "pnmutils.h"

#include "Timer.h"

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
main( int /* argc */, char** /* * argv */ )
{
    TriclopsContext	triclops;
    TriclopsError    error;
    TriclopsInput 	triclopsInput1;
    TriclopsInput	   triclopsInput2;
    std::string 	szInputFile("input.ppm");
    std::string 	szCalFile("input.cal");

    //===================================================================================
    // *** Start of configuration section ***
    // Triclops settings that affect performance
    // Edit to match the configuration you are trying to profile

    int	nRows			         = 240;
    int	nCols			         = nRows*4/3;
    int	nDisparities	         = 64;
    bool bSubpixel		         = true;
    bool bTextureValidationOn	 = true;
    bool bUniquenessValidationOn = true;
    bool bSurfaceValidationOn	 = true;
   
    TriclopsCameraConfiguration	cameraConfig = TriCfg_2CAM_HORIZONTAL;
   
    // *** Endof configuration section ***
    // you shouldn't need to edit below this point
    //===================================================================================

    // Get the camera calibration data
    error = triclopsGetDefaultContextFromFile( &triclops, const_cast<char *>(szCalFile.c_str()) );
    _HANDLE_TRICLOPS_ERROR( "triclopsGetDefaultContextFromFile(): "
			   "Can't open calibration file", 
			   error );

    //===================================================================================
    //  update the TriclopsContext with the selected configuration
    triclopsSetCameraConfiguration( triclops, cameraConfig );
    triclopsSetResolution( triclops, nRows, nCols );
    triclopsSetDisparity( triclops, 0, nDisparities-1 );
    triclopsSetSubpixelInterpolation( triclops, bSubpixel );
    triclopsSetTextureValidation( triclops, bTextureValidationOn );
    triclopsSetUniquenessValidation( triclops, bUniquenessValidationOn );
    triclopsSetSurfaceValidation( triclops, bSurfaceValidationOn );

    //===================================================================================
      
    // Load images from file
    if ( !ppmReadToTriclopsInput( szInputFile.c_str(),  &triclopsInput1 ) )
    {
        printf( "ppmReadToTriclopsInput() failed. Can't read '%s'\n", szInputFile.c_str() );
        return 1;
    }

    // Load images from file
    if ( !ppmReadToTriclopsInput( szInputFile.c_str(),  &triclopsInput2 ) )
    {
        printf( "ppmReadToTriclopsInput() failed. Can't read '%s'\n", szInputFile.c_str() );
        return 1;
    }
   
    // Do processing once - this makes sure all necessarly lookup tables, etc are
    // build - these are one-time only constructions that make the first call to these
    // functions take longer

    // Rectify the images
    error = triclopsRectify( triclops, &triclopsInput1 );
    _HANDLE_TRICLOPS_ERROR( "triclopsRectify()", error );
     
    // Do stereo processing
    error =  triclopsStereo( triclops );
    _HANDLE_TRICLOPS_ERROR( "triclopsStereo()", error );

    //===================================================================================
    //  Do the time testing

    int nFrames = 0;
    double duration = 10.0;
    Timer stereoTimer, rectificationTimer, totalTimer;

    printf( "Starting %.0f second profiling run ...\n", duration );

    stereoTimer.Start();
    stereoTimer.Pause();
    rectificationTimer.Start();
    rectificationTimer.Pause();
    totalTimer.Start();

    while( totalTimer.GetElapsedTimeInSeconds() < duration )
    {
        TriclopsInput* pTriclopsInput = &triclopsInput1;
        if ( nFrames % 2 )
	        pTriclopsInput = &triclopsInput2;

        // Rectify the images
        rectificationTimer.Unpause();
        error = triclopsRectify( triclops, pTriclopsInput );
        _HANDLE_TRICLOPS_ERROR( "triclopsRectify()", error );
        rectificationTimer.Pause();
      
        // Do stereo processing
        stereoTimer.Unpause();
        error =  triclopsStereo( triclops );
        _HANDLE_TRICLOPS_ERROR( "triclopsStereo()", error );
        stereoTimer.Pause();

        ++nFrames;
    }

    totalTimer.Stop();
    stereoTimer.Stop();
    rectificationTimer.Stop();

    double dSecs 	   = totalTimer.GetElapsedTimeInSeconds();
    double dRectSecs   = rectificationTimer.GetElapsedTimeInSeconds();
    double dStereoSecs = stereoTimer.GetElapsedTimeInSeconds();
   
    printf( "\n==========\n" );
    printf( "Stereo resolution: %d x %d\n", nCols, nRows );
    printf( "Number of disparities: %d\n", nDisparities );
    printf( "Subpixel: %s\n", bSubpixel ? "on"  : "off" );
    printf( "Texture validation: %s\n", bTextureValidationOn ? "on"  : "off" );
    printf( "Uniqueness validation: %s\n", bUniquenessValidationOn ? "on"  : "off" );
    printf( "Surface validation: %s\n", bSurfaceValidationOn ? "on"  : "off" );

    printf( "\n==========\n" );
    printf( "Time spent on rectification was %f seconds\n", dRectSecs );
    printf( "Time spent on stereo was %f seconds\n", dStereoSecs );
    double dFps = (double) nFrames/dSecs;
    printf( "Processed %d frames in %f seconds\n", nFrames, dSecs );
    printf( "Frame rate of %f frames/sec\n", dFps );
    double dPixDispPerSec;
    dPixDispPerSec = nRows * nCols * nDisparities * dFps;
    printf( "Disparity-pixels/sec = %f\n", dPixDispPerSec );
    printf( "\n==========\n" );

    //===================================================================================

    // clean up memory allocated for context
    freeInput( &triclopsInput1 );
    freeInput( &triclopsInput2 );
    error = triclopsDestroyContext( triclops );
    _HANDLE_TRICLOPS_ERROR( "triclopsDestroyContext()", error );
   
    return 0;
}

