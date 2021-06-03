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
// grabstereo
//
// Gets input from the Bumblebee, and performs stereo processing
// to create a disparity image. A rectified image from the reference camera
// and the disparity image are both written out.
//
//=============================================================================

//=============================================================================
// System Includes
//=============================================================================
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

//=============================================================================
// PGR Includes
//=============================================================================
#include "triclops.h"
#include "fc2triclops.h"

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
}

//
// aliases namespaces
//
namespace FC2 = FlyCapture2;
namespace FC2T = Fc2Triclops;

// struct containing image needed for processing
struct ImageContainer
{
	FC2::Image tmp[2];
	FC2::Image unprocessed[2];	   
} ;

enum IMAGE_SIDE
{
	RIGHT = 0, LEFT
};

// configue camera to capture image
int configureCamera( FC2::Camera &camera );

// generate Triclops context from connected camera
int generateTriclopsContext( FC2::Camera & camera, 
                         TriclopsContext & triclops );

// capture image from connected camera
int grabImage (  FC2::Camera & camera, 
                 FC2::Image  & grabbedImage );

// color process the image and convert to monochrome
int convertColorToMonoImage( FC2::Image & colorImage, 
                             FC2::Image & monoImage );

// generate triclops input necessary to carry out stereo processing
int generateTriclopsInput( FC2::Image const & grabbedImage, 
                           ImageContainer   & imageContainer,
                           TriclopsInput    & triclopsInput );


// carry out stereo processin pipeline
int doStereo( TriclopsContext const & triclops, 
               const TriclopsInput & triclopsInput );




int
main( int /* argc */, char** /* argv */ )
{
    TriclopsContext   triclops;
    TriclopsInput     triclopsInput;

    // camera to be used to grab color image
    FC2::Camera camera;
    // grabbed unprocessed image
    FC2::Image grabbedImage;
   
    // connect camera
    camera.Connect();

    // configure camera
    if ( configureCamera( camera ) )
    {
		return EXIT_FAILURE;
    }

    // generate the Triclops context 
    if ( generateTriclopsContext( camera, triclops ) )
    {
		return EXIT_FAILURE;
    }

    // grab image from camera.
    // this image contains both right and left images
    if ( grabImage( camera, grabbedImage ) )
    {
		return EXIT_FAILURE;
    }
   
    // right and left image extracted from grabbed image
    ImageContainer imageCont;

    // generate triclops input from grabbed image
    if ( generateTriclopsInput( grabbedImage, imageCont, triclopsInput ) )
    {
		return EXIT_FAILURE;
    } 

    // carry out the stereo pipeline 
    if ( doStereo( triclops, triclopsInput ) )
    {
		return EXIT_FAILURE;
    }
   
    // Close the camera
    camera.StopCapture();
    camera.Disconnect();

    // Destroy the Triclops context
    TriclopsError     te;
    te = triclopsDestroyContext( triclops ) ;
    _HANDLE_TRICLOPS_ERROR( "triclopsDestroyContext()", te);
   
    return EXIT_SUCCESS;
}


int configureCamera( FC2::Camera & camera )
{
	FC2T::ErrorType fc2TriclopsError;	      
	FC2T::StereoCameraMode mode = FC2T::TWO_CAMERA;
    fc2TriclopsError = FC2T::setStereoMode( camera, mode );
    if ( fc2TriclopsError )
    {
        return FC2T::handleFc2TriclopsError(fc2TriclopsError, "setStereoMode");
    }

    return 0;
}

int generateTriclopsContext( FC2::Camera & camera, 
                         TriclopsContext & triclops )
{
	FC2::CameraInfo camInfo;
    FC2::Error fc2Error = camera.GetCameraInfo(&camInfo);
	if (fc2Error != FC2::PGRERROR_OK)
	{
		return FC2T::handleFc2Error(fc2Error);
	}
   
	FC2T::ErrorType fc2TriclopsError; 
    fc2TriclopsError = FC2T::getContextFromCamera( camInfo.serialNumber, 
	                                                &triclops );
    if (fc2TriclopsError != FC2T::ERRORTYPE_OK)
    {
        return FC2T::handleFc2TriclopsError(fc2TriclopsError, 
		                                    "getContextFromCamera");
    }
	
	return 0;
}

int grabImage ( FC2::Camera & camera, FC2::Image & grabbedImage )
{
	FC2::Error fc2Error = camera.StartCapture();
	if (fc2Error != FC2::PGRERROR_OK)
	{
		return FC2T::handleFc2Error(fc2Error);
	}

	fc2Error = camera.RetrieveBuffer(&grabbedImage);
	if (fc2Error != FC2::PGRERROR_OK)
	{
		return FC2T::handleFc2Error(fc2Error);
	}
	
	return 0;
}

int convertColorToMonoImage( FC2::Image & colorImage, FC2::Image & monoImage )
{
    FC2::Error fc2Error;
    fc2Error = colorImage.SetColorProcessing(FC2::HQ_LINEAR);
    if (fc2Error != FC2::PGRERROR_OK)
    {
        return FC2T::handleFc2Error(fc2Error);
    }

    fc2Error = colorImage.Convert(FC2::PIXEL_FORMAT_MONO8, 
                                                 &monoImage);
    if (fc2Error != FC2::PGRERROR_OK)
    {
        return FC2T::handleFc2Error(fc2Error);
    }

    return 0;
}

// generare triclops input
int generateTriclopsInput( FC2::Image const & grabbedImage, 
                           ImageContainer   & imageCont,
                           TriclopsInput    & triclopsInput )
{
   
    FC2::Error      fc2Error;
    FC2T::ErrorType fc2TriclopsError;
    TriclopsError   te;
 
    FC2::Image * tmpImage = imageCont.tmp;
    FC2::Image * unprocessedImage = imageCont.unprocessed;
      
    // Convert the pixel interleaved raw data to de-interleaved and color processed data
    fc2TriclopsError = FC2T::unpackUnprocessedRawOrMono16Image(
                                   grabbedImage, 
								   true /*assume little endian*/,
                                   tmpImage[RIGHT], 
                                   tmpImage[LEFT] );
										   
    if (fc2TriclopsError != FC2T::ERRORTYPE_OK)
    {
	    return FC2T::handleFc2TriclopsError(fc2TriclopsError, 
		                                   "unprocessedRawOrMono16Image()");
    }  

    // check if the unprocessed image is color
    if ( tmpImage[0].GetBayerTileFormat() != FC2::NONE )
    {
   	    for ( int i = 0; i < 2; ++i )
   	    {
   	  	    if ( convertColorToMonoImage(tmpImage[i], unprocessedImage[i]) )
   	  	    {
   	  	  	    return 1; 
   	  	    }
   	    }
    }
    else
    {
        unprocessedImage[RIGHT] = tmpImage[RIGHT];
        unprocessedImage[LEFT]  = tmpImage[LEFT];
    }
      
    // pack image data into a TriclopsInput structure
    te = triclopsBuildRGBTriclopsInput(
               grabbedImage.GetCols(), 
               grabbedImage.GetRows(), 
               grabbedImage.GetCols(),  
               (unsigned long)grabbedImage.GetTimeStamp().seconds, 
               (unsigned long)grabbedImage.GetTimeStamp().microSeconds, 
               unprocessedImage[RIGHT].GetData(), 
               unprocessedImage[LEFT].GetData(), 
               unprocessedImage[LEFT].GetData(), 
               &triclopsInput);

    _HANDLE_TRICLOPS_ERROR( "triclopsBuildRGBTriclopsInput()", te );

    // save monochrome generated right and left image
    FC2::PGMOption pgmOpt;
    pgmOpt.binaryFile = true;
    unprocessedImage[RIGHT].Save("rightGreyImage.pgm", &pgmOpt);
    unprocessedImage[LEFT].Save("leftGreyImage.pgm", &pgmOpt);

    return 0;
}



int doStereo( TriclopsContext const & triclops, 
                TriclopsInput const & triclopsInput )
{
    TriclopsImage disparityImage, edgeImage, rectifiedImage;
    TriclopsError te;
   
    // Rectify the images
    te = triclopsRectify( triclops, const_cast<TriclopsInput *>(&triclopsInput) );
    _HANDLE_TRICLOPS_ERROR( "triclopsRectify()", te );

    // Do stereo processing
    te = triclopsStereo( triclops );
    _HANDLE_TRICLOPS_ERROR( "triclopsStereo()", te );
   
    // Retrieve the disparity image from the triclops context
    te = triclopsGetImage( triclops, 
                          TriImg_DISPARITY, 
                          TriCam_REFERENCE, 
						  &disparityImage );
    _HANDLE_TRICLOPS_ERROR( "triclopsGetImage()", te );

    // Retrieve the rectified image from the triclops context
    te = triclopsGetImage( triclops, 
                          TriImg_RECTIFIED, 
                          TriCam_REFERENCE, 
						  &rectifiedImage );
    _HANDLE_TRICLOPS_ERROR( "triclopsGetImage()", te );

    // Retrieve the edge image from the triclops context
    te = triclopsGetImage( triclops, 
                          TriImg_EDGE, 
                          TriCam_REFERENCE, 
						  &edgeImage );
    _HANDLE_TRICLOPS_ERROR( "triclopsGetImage()", te );
   
    // Save the disparity, reference and edge images

    const char * pDisparityFilename = "disparity.pgm";
    te = triclopsSaveImage( &disparityImage, const_cast<char *>(pDisparityFilename) );
    _HANDLE_TRICLOPS_ERROR( "triclopsSaveImage()", te );

    const char * pEdgeFilename = "edge.pgm";
    te = triclopsSaveImage( &edgeImage, const_cast<char *>(pEdgeFilename) );
    _HANDLE_TRICLOPS_ERROR( "triclopsSaveImage()", te );

    const char * pRectifiedFilename = "rectified.pgm";
    te = triclopsSaveImage( &rectifiedImage, const_cast<char *>(pRectifiedFilename) );
    _HANDLE_TRICLOPS_ERROR( "triclopsSaveImage()", te );
   
    return 0;
}

