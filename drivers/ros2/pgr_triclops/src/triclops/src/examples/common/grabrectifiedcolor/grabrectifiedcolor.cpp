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
// $Id: grabrectifiedcolor.cpp,v 1.13 2010-08-06 00:06:46 arturp Exp $
//=============================================================================
//=============================================================================
// grabrectifiedcolor
//
// Takes input from a color stereo product, extracts the color image from the 
// right camera, rectifies this image, and saves it to a ppm file.
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


//
// Macro to check, report on, and handle Triclops API error codes.
//
#define _HANDLE_TRICLOPS_ERROR( function, error ) \
{ \
   if( error != TriclopsErrorOk ) \
   { \
      printf( \
	 "ERROR: %s reported %s.\n", \
	 function, \
	 triclopsErrorToString( error ) ); \
      exit( 1 ); \
   } \
} \

//
// aliases namespaces
//
namespace FC2 = FlyCapture2;
namespace FC2T = Fc2Triclops;

// struct containing image needed for processing
struct ImageContainer
{
    FC2::Image unprocessed[2];
    FC2::Image bgru[2];
    FC2::Image packed;
} ;

enum IMAGE_SIDE
{
	RIGHT = 0, LEFT
};

// configure the connected camera
int configureCamera( FC2::Camera & camera );

// generate triclops context from connected camera
int generateTriclopsContext( FC2::Camera & camera, 
                         TriclopsContext & triclops );

// capture image from connected camera
int grabImage ( FC2::Camera & camera, FC2::Image& rGrabbedImage );

// generate triclops input from grabbed color image 
int generateTriclopsInput( const FC2::Image & grabbedImage, 
                           ImageContainer   & imageContainer,
                           TriclopsInput    & triclopsColorInput );

// carry out rectification from triclops color input
int doRectification( const TriclopsContext    & triclops, 
                     const TriclopsInput      & colorTriclopsInput,
                     TriclopsPackedColorImage & rectifiedPackedColorImage
                     );

int
main( int /* argc */, char** /* argv */ )
{
    TriclopsInput triclopsColorInput;
    TriclopsPackedColorImage rectifiedPackedColorImage;
    TriclopsContext triclops;

    // camera to be used to grab color image
    FC2::Camera camera;

    // grabbed unprocessed color image
    FC2::Image grabbedImage;

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

    // declare container of images used for processing
    ImageContainer imageContainer;

    // generate color triclops input from grabbed image
    if ( generateTriclopsInput( grabbedImage, 
        imageContainer,
        triclopsColorInput ) 
       )
    {
		return EXIT_FAILURE;
    }

    // carry out rectification from triclops color input
    if ( doRectification( triclops, triclopsColorInput, rectifiedPackedColorImage ) )
    {
		return EXIT_FAILURE;
    }

    // Close the camera
    camera.StopCapture();
    camera.Disconnect();
   
    // clean up context
    TriclopsError te;
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

int grabImage ( FC2::Camera & camera, FC2::Image& grabbedImage )
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

int convertToBGRU( FC2::Image & image, FC2::Image & convertedImage )
{
    FC2::Error fc2Error;
    fc2Error = image.SetColorProcessing(FC2::HQ_LINEAR);
    if (fc2Error != FC2::PGRERROR_OK)
    {
        return FC2T::handleFc2Error(fc2Error);
    }

    fc2Error = image.Convert(FC2::PIXEL_FORMAT_BGRU, &convertedImage);
    if (fc2Error != FC2::PGRERROR_OK)
    {
        return FC2T::handleFc2Error(fc2Error);
    }

    return 0;
}

int generateTriclopsInput( const FC2::Image & grabbedImage, 
                            ImageContainer  & imageContainer,
                            TriclopsInput   & triclopsColorInput ) 
{
    FC2::Error fc2Error;
    FC2T::ErrorType fc2TriclopsError; 

    FC2::Image * unprocessedImage = imageContainer.unprocessed;

    fc2TriclopsError = FC2T::unpackUnprocessedRawOrMono16Image(
                                grabbedImage, 
                                true /*assume little endian*/,
                                unprocessedImage[RIGHT], unprocessedImage[LEFT]);

    if (fc2TriclopsError != FC2T::ERRORTYPE_OK)
    {
        return FC2T::handleFc2TriclopsError(fc2TriclopsError, 
                                     "deinterleaveRawOrMono16Image");
    }

    FC2::PGMOption pgmOpt;
    pgmOpt.binaryFile = true;
    unprocessedImage[RIGHT].Save("rawRightImage.pgm", &pgmOpt);
    unprocessedImage[LEFT].Save("rawLeftImage.pgm", &pgmOpt);

    FC2::Image * bgruImage = imageContainer.bgru;

    for ( int i = 0; i < 2; ++i )
    {
        if ( convertToBGRU(unprocessedImage[i], bgruImage[i]) )
        {
            return 1;
        }
    }

    FC2::PNGOption pngOpt;
    pngOpt.interlaced = false;
    pngOpt.compressionLevel = 9;
    bgruImage[RIGHT].Save("colorImageRight.png", &pngOpt);
    bgruImage[LEFT].Save("colorImageLeft.png", &pngOpt);

    FC2::Image & packedColorImage = imageContainer.packed;

    // pack BGRU right and left image into an image
    fc2TriclopsError = FC2T::packTwoSideBySideRgbImage( bgruImage[RIGHT], 
							                            bgruImage[LEFT],
							                            packedColorImage );

    if (fc2TriclopsError != FC2T::ERRORTYPE_OK)
    {
        return handleFc2TriclopsError(fc2TriclopsError, "packTwoSideBySideRgbImage");
    }
   
    // Use the row interleaved images to build up a packed TriclopsInput.
    // A packed triclops input will contain a single image with 32 bpp.
    TriclopsError te;
    te = triclopsBuildPackedTriclopsInput(
                    grabbedImage.GetCols(),
                    grabbedImage.GetRows(),
                    packedColorImage.GetStride(),
                    (unsigned long)grabbedImage.GetTimeStamp().seconds, 
                    (unsigned long)grabbedImage.GetTimeStamp().microSeconds, 
                    packedColorImage.GetData(),
                    &triclopsColorInput );

    _HANDLE_TRICLOPS_ERROR( "triclopsBuildPackedTriclopsInput()", te );

    // the following does not change the size of the image
    // and therefore it PRESERVES the internal buffer!
    packedColorImage.SetDimensions( 
                       packedColorImage.GetRows(), 
				           packedColorImage.GetCols(), 
				           packedColorImage.GetStride(),
				           packedColorImage.GetPixelFormat(),
				           FC2::NONE);

    packedColorImage.Save("packedColorImage.png",&pngOpt );

    return 0;
}

int doRectification( const TriclopsContext & triclops, 
                     const TriclopsInput & colorTriclopsInput,
                     TriclopsPackedColorImage & rectifiedPackedColorImage
                     )
{
    // rectify the color image
    TriclopsError te;
    te = triclopsRectifyPackedColorImage( triclops, 
				   TriCam_REFERENCE, 
				   const_cast<TriclopsInput *>(&colorTriclopsInput), 
				   &rectifiedPackedColorImage );
    _HANDLE_TRICLOPS_ERROR( "triclopsRectifyPackedColorImage()", te );
   
    // Save the color rectified image to file
    const char * rectifiedFilename = "right-rectified.pgm";
    triclopsSavePackedColorImage(&rectifiedPackedColorImage, const_cast<char *>(rectifiedFilename) );

    return 0;
}

