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
// roistereo:
//
// Takes input from the camera, and sets up four regions of interest on 
// which to perform stereo processing; the resultant disparity image is 
// saved.
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


// aliases namespaces
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

// capture image from connected camera
int grabImage (  FC2::Camera & camera, FC2::Image & grabbedImage );

// color process the image and convert to monochrome
int convertToMono( FC2::Image & image, FC2::Image & monoImage );

// generate triclops input necessary to carry out stereo processing
int generateTriclopsInput( FC2::Image const & grabbedImage, 
                           ImageContainer   & imageContainer, 
                           TriclopsInput    & triclopsInput );

// generate Triclops context from connected camera
int generateTriclopsContext( FC2::Camera & camera, 
                         TriclopsContext & triclops );

// configure ROIs on which carry out stereo processing 
int configureROIs( const TriclopsContext & triclops );

// carry out stereo processin pipeline
int doStereo( const TriclopsContext & triclops, 
              const TriclopsInput   & triclopsInput );

int
main( int /* argc */, char** /* argv */ )
{
    TriclopsContext    triclops;
    TriclopsInput      triclopsInput;

    FC2::Camera camera;
    FC2::Image grabbedImage;

    camera.Connect();

    // configure camera
    if ( configureCamera( camera ) )
    {
		return EXIT_FAILURE;
    }


    // temporary unprocessed right and left images extracted from grabbed image
    FC2::Image unprocessedRightImage;
    FC2::Image unprocessedLeftImage;

    // temporary right and left images for preparing the stereo images
    FC2::Image rightGreyImage;
    FC2::Image leftGreyImage;

    // generate the Triclops context 
    if ( generateTriclopsContext( camera, triclops ) )
    {
		return EXIT_FAILURE;
    }

    if ( configureROIs( triclops ) )
    {
		return EXIT_FAILURE;
    }

    // grab image from camera.
    // this image contains both right and left images
    if ( grabImage( camera, grabbedImage ) )
    {
		return EXIT_FAILURE;
    }

    // declare container of Images used during processing
    ImageContainer imageContainer;

    // generate triclops input from grabbed image
    if ( generateTriclopsInput( grabbedImage, 
                                imageContainer,
                                triclopsInput ) 
       )
    {
		return EXIT_FAILURE;
    }


    // carry out the stereo pipeline 
    if ( doStereo( triclops, triclopsInput ) )
    {
		return EXIT_FAILURE;
    }
      
    // Destroy the Triclops context
    TriclopsError te;
    te = triclopsDestroyContext( triclops ) ;
    _HANDLE_TRICLOPS_ERROR( "triclopsDestroyContext()", te );
   
    // Close the camera and disconnect
    camera.StopCapture();
    camera.Disconnect();
   
    return 0;
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

int generateTriclopsContext( FC2::Camera     & camera, 
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

int convertToMono( FC2::Image & image, FC2::Image & monoImage )
{
    FC2::Error fc2Error;
    fc2Error = image.SetColorProcessing(FC2::HQ_LINEAR);
    if (fc2Error != FC2::PGRERROR_OK)
    {
        return FC2T::handleFc2Error(fc2Error);
    }

    fc2Error = image.Convert(FC2::PIXEL_FORMAT_MONO8, &monoImage);
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
    if ( tmpImage[RIGHT].GetBayerTileFormat() != FC2::NONE)
    {
   	    for ( int i = 0; i < 2; ++i )
   	    {
   	  	    if ( convertToMono(tmpImage[i], unprocessedImage[i]) )
   	  	    {
   	  	  	    return 1; 
   	  	    }
   	    }
    }
    else
    {
        unprocessedImage[RIGHT] = tmpImage[RIGHT];
        unprocessedImage[LEFT] = tmpImage[LEFT];
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


int configureROIs( const TriclopsContext &triclops ) 
{
    TriclopsROI * pRois;
    int           nMaxRois;
    TriclopsError te;

    // Get the pointer to the regions of interest array
    te = triclopsGetROIs( triclops, &pRois, &nMaxRois );
    _HANDLE_TRICLOPS_ERROR( "triclopsGetROIs()", te );
   
    if ( nMaxRois >= 4 ) 
    {
        // Set up four regions of interest: 
      
        // Entire upper left quadrant of image
        pRois[0].row   = 0;
        pRois[0].col   = 0;
        pRois[0].nrows = 240;
        pRois[0].ncols = 320;
      
        // Part of the lower right
        pRois[1].row   = 240;
        pRois[1].col   = 320;
        pRois[1].nrows = 180;
        pRois[1].ncols = 240;
      
        // Centered in upper right quadrant
        pRois[2].row   = 60;
        pRois[2].col   = 400;
        pRois[2].nrows = 120;
        pRois[2].ncols = 160;
      
        // Small section of lower left
        pRois[3].row   = 300;
        pRois[3].col   = 30;
        pRois[3].nrows = 80;
        pRois[3].ncols = 80;
      
        // Tell the TriclopsContext how many ROIs we want to process
        te = triclopsSetNumberOfROIs( triclops, 4 );
        _HANDLE_TRICLOPS_ERROR( "triclopsSetNumberOfROIs()", te );
    }
    else
    {
        printf( "Only %d ROIs available in the TriclopsContext "
                "- this should never happen!\n" 
	            "Aborting!\n", nMaxRois );
	      
        // Destroy the Triclops context
        triclopsDestroyContext( triclops ) ;

        return 1;
    }

    return 0;
}


int doStereo( const TriclopsContext & triclops, 
              const TriclopsInput   & triclopsInput )
{
    TriclopsImage disparityImage, edgeImage, rectifiedImage;
    TriclopsError te;

    // Set up some stereo parameters:
    // Set to 640x480 output images
    te = triclopsSetResolution( triclops, 480, 640 );
    _HANDLE_TRICLOPS_ERROR( "triclopsSetResolution()", te );

    // Set disparity range to be quite wide
    te = triclopsSetDisparity( triclops, 0, 200 );
    _HANDLE_TRICLOPS_ERROR( "triclopsSetDisparity()", te );

    // Set subpixel interpolation off - so we know we don't need to use 
    // TriclopsImage16 structures when we access and save the disparity image
    te = triclopsSetSubpixelInterpolation( triclops, 0 );
    _HANDLE_TRICLOPS_ERROR( "triclopsSetSubpixelInterpolation()", te );

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


