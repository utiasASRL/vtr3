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
// stereoto3dpoints
//
// Takes input from a Bumblebee and performs subpixel
// interpolation to create a 16-bit disparity image, which is saved.
// The disparity data is then converted to 3-dimensional X/Y/Z
// coordinates which is written to a file. 
//
// This point file can be viewed with PGRView under windows.
//
//=============================================================================

#include "triclops.h"

#include "fc2triclops.h"

#include <stdio.h>
#include <stdlib.h>


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

// aliases namespaces
namespace FC2 = FlyCapture2;
namespace FC2T = Fc2Triclops;

struct ImageContainer
{
    FC2::Image unprocessed[2];	
    FC2::Image bgru[2];
    FC2::Image mono[2];
    FC2::Image packed;
};

enum IMAGE_SIDE
{
	RIGHT = 0, LEFT
};

// configue camera to capture image
int configureCamera( FC2::Camera &camera );

// generate Triclops context from connected camera
int generateTriclopsContext( FC2::Camera     & camera, 
                             TriclopsContext & triclops );

// capture image from connected camera
int grabImage ( FC2::Camera & camera, FC2::Image & grabbedImage );

// convert image to BRGU
int convertToBGRU( FC2::Image & image, FC2::Image & convertedImage );

// generate triclops input necessary to carry out stereo processing
int generateTriclopsInput( FC2::Image const & grabbedImage, 
                           ImageContainer   & imageContainer,
                           TriclopsInput    & colorData,
                           TriclopsInput    & stereoData );

// carry out stereo processing pipeline
int doStereo( TriclopsContext const & triclops, 
               TriclopsInput  const & stereoData,
               TriclopsImage16      & depthImage );

// save 3d points generated from stereo processing
int save3dPoints( FC2::Image      const & grabbedImage, 
                  TriclopsContext const & triclops, 
                  TriclopsImage16 const & disparityImage16, 
                  TriclopsInput   const & colorData );



int
main( int /* argc */, char** /* argv */ )
{
    TriclopsInput triclopsColorInput, triclopsMonoInput;
    TriclopsContext triclops;

    FC2::Camera camera;
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

    // Container of Images used for processing
    ImageContainer imageContainer;

    // generate triclops inputs from grabbed image
    if ( generateTriclopsInput( grabbedImage, 
                                imageContainer,
                                triclopsColorInput, 
                                triclopsMonoInput ) 
       )
    {
		return EXIT_FAILURE;
    }

    // output image disparity image with subpixel interpolation
    TriclopsImage16 disparityImage16;

    // carry out the stereo pipeline 
    if ( doStereo( triclops, triclopsMonoInput, disparityImage16 ) )
    {
		return EXIT_FAILURE;
    }

    // save text file containing 3d points
    if ( save3dPoints( grabbedImage, triclops, disparityImage16, triclopsColorInput ) )
    {
		return EXIT_FAILURE;
    }

    // Close the camera and disconnect
    camera.StopCapture();
    camera.Disconnect();
   
    // Destroy the Triclops context
    TriclopsError te;
    te = triclopsDestroyContext( triclops ) ;
    _HANDLE_TRICLOPS_ERROR( "triclopsDestroyContext()", te );
         
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
    fc2TriclopsError = FC2T::getContextFromCamera( camInfo.serialNumber, &triclops );
    if (fc2TriclopsError != FC2T::ERRORTYPE_OK)
    {
        return FC2T::handleFc2TriclopsError(fc2TriclopsError, 
		                                    "getContextFromCamera");
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

int generateTriclopsInput( FC2::Image const & grabbedImage, 
                            ImageContainer  & imageContainer,
                            TriclopsInput   & triclopsColorInput,
                            TriclopsInput   & triclopsMonoInput ) 
{
    FC2::Error fc2Error;
    FC2T::ErrorType fc2TriclopsError; 
    TriclopsError te;

    FC2::Image * unprocessedImage = imageContainer.unprocessed;

    fc2TriclopsError = FC2T::unpackUnprocessedRawOrMono16Image(
                            grabbedImage, 
                            true /*assume little endian*/,
                            unprocessedImage[RIGHT], 
                            unprocessedImage[LEFT]);

    if (fc2TriclopsError != FC2T::ERRORTYPE_OK)
    {
        return FC2T::handleFc2TriclopsError(fc2TriclopsError, 
                                     "unpackUnprocessedRawOrMono16Image");
    }

    FC2::PGMOption pgmOpt;
    pgmOpt.binaryFile = true;
    unprocessedImage[RIGHT].Save("rawRightImage.pgm", &pgmOpt);
    unprocessedImage[LEFT].Save("rawLeftImage.pgm", &pgmOpt);

    FC2::Image * monoImage = imageContainer.mono;

    // check if the unprocessed image is color
    if ( unprocessedImage[RIGHT].GetBayerTileFormat() != FC2::NONE )
    {
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
            return handleFc2TriclopsError(fc2TriclopsError, 
                                            "packTwoSideBySideRgbImage");
        }

        // Use the row interleaved images to build up a packed TriclopsInput.
        // A packed triclops input will contain a single image with 32 bpp.
        te = triclopsBuildPackedTriclopsInput( grabbedImage.GetCols(),
                                                grabbedImage.GetRows(),
                                                packedColorImage.GetStride(),
                                                (unsigned long)grabbedImage.GetTimeStamp().seconds, 
                                                (unsigned long)grabbedImage.GetTimeStamp().microSeconds, 
                                                packedColorImage.GetData(),
                                                &triclopsColorInput );

        _HANDLE_TRICLOPS_ERROR( "triclopsBuildPackedTriclopsInput()", te );


        // the following does not change the size of the image
        // and therefore it PRESERVES the internal buffer!
        packedColorImage.SetDimensions( packedColorImage.GetRows(), 
                                        packedColorImage.GetCols(), 
                                        packedColorImage.GetStride(),
                                        packedColorImage.GetPixelFormat(),
                                        FC2::NONE);

        packedColorImage.Save("packedColorImage.png",&pngOpt );

        for ( int i = 0; i < 2; ++i )
        {
            fc2Error = bgruImage[i].Convert(FlyCapture2::PIXEL_FORMAT_MONO8, &monoImage[i]);
            if (fc2Error != FlyCapture2::PGRERROR_OK)
            {
                return Fc2Triclops::handleFc2Error(fc2Error);
            }
        }

        monoImage[RIGHT].Save("monoImageRight.pgm", &pgmOpt);
        monoImage[LEFT].Save("monoImageLeft.pgm", &pgmOpt);
    }
    else
    {
        monoImage[RIGHT] = unprocessedImage[RIGHT];
        monoImage[LEFT] = unprocessedImage[LEFT];
    }
   
    // Use the row interleaved images to build up an RGB TriclopsInput.  
    // An RGB triclops input will contain the 3 raw images (1 from each camera).
    te = triclopsBuildRGBTriclopsInput( grabbedImage.GetCols(), 
                                        grabbedImage.GetRows(), 
                                        grabbedImage.GetCols(),  
                                        (unsigned long)grabbedImage.GetTimeStamp().seconds, 
                                        (unsigned long)grabbedImage.GetTimeStamp().microSeconds, 
                                        monoImage[RIGHT].GetData(), 
                                        monoImage[LEFT].GetData(), 
                                        monoImage[LEFT].GetData(), 
                                        &triclopsMonoInput );

    _HANDLE_TRICLOPS_ERROR( "triclopsBuildRGBTriclopsInput()", te );

    return 0;
}

int doStereo( TriclopsContext const & triclops, 
               TriclopsInput  const & stereoData, 
               TriclopsImage16      & depthImage )
{
    TriclopsError te;

    // Set subpixel interpolation on to use 
    // TriclopsImage16 structures when we access and save the disparity image
    te = triclopsSetSubpixelInterpolation( triclops, 1 );
    _HANDLE_TRICLOPS_ERROR( "triclopsSetSubpixelInterpolation()", te );

    // Rectify the images
    te = triclopsRectify( triclops, const_cast<TriclopsInput *>(&stereoData) );
    _HANDLE_TRICLOPS_ERROR( "triclopsRectify()", te );

    // Do stereo processing
    te = triclopsStereo( triclops );
    _HANDLE_TRICLOPS_ERROR( "triclopsStereo()", te );
   
    // Retrieve the interpolated depth image from the context
    te = triclopsGetImage16( triclops, 
                            TriImg16_DISPARITY, 
                            TriCam_REFERENCE, 
                            &depthImage );
    _HANDLE_TRICLOPS_ERROR( "triclopsGetImage()", te );

    // Save the interpolated depth image
    char const * pDispFilename = "disparity16.pgm";
    te = triclopsSaveImage16( &depthImage, const_cast<char *>(pDispFilename) );
    _HANDLE_TRICLOPS_ERROR( "triclopsSaveImage()", te );

    return 0;
}

int save3dPoints( FC2::Image      const & grabbedImage, 
                  TriclopsContext const & triclops, 
                  TriclopsImage16 const & disparityImage16, 
                  TriclopsInput   const & colorData )
{
    TriclopsImage monoImage = {0};
    TriclopsColorImage colorImage = {0};
    TriclopsError te;

    float            x, y, z; 
    int	            r, g, b;
    FILE             * pPointFile;
    int              nPoints = 0;
    int	             pixelinc ;
    int	             i, j, k;
    unsigned short * row;
    unsigned short   disparity;

    // Rectify the color image if applicable
    bool isColor = false;
    if ( grabbedImage.GetPixelFormat() == FC2::PIXEL_FORMAT_RAW16 )
    {
        isColor = true;
        te = triclopsRectifyColorImage( triclops, 
                                        TriCam_REFERENCE, 
	                                    const_cast<TriclopsInput *>(&colorData), 
	                                    &colorImage );
        _HANDLE_TRICLOPS_ERROR( "triclopsRectifyColorImage()", te );
    }
    else
    {
        te = triclopsGetImage( triclops,
	                            TriImg_RECTIFIED,
	                            TriCam_REFERENCE,
	                            &monoImage );
        _HANDLE_TRICLOPS_ERROR( "triclopsGetImage()", te );
    }

  
    // Save points to disk
    const char * pFilename = "out.pts";
#if defined( WIN64 ) || defined (WIN32) 
    errno_t error;
    error = fopen_s( &pPointFile, pFilename, "w+" );
    if ( error == 0 )
#else
    pPointFile = fopen( pFilename, "w+" );
    if ( pPointFile != NULL )
#endif
    {
        printf("Opening output file %s\n", pFilename);
    }
    else
    {
        printf("Error opening output file %s\n", pFilename);
        return 1;
    }

    // The format for the output file is:
    // <x> <y> <z> <red> <grn> <blu> <row> <col>
    // <x> <y> <z> <red> <grn> <blu> <row> <col>
    // ...

    // Determine the number of pixels spacing per row
    pixelinc = disparityImage16.rowinc/2;
    for ( i = 0, k = 0; i < disparityImage16.nrows; i++ )
    {
        row = disparityImage16.data + i * pixelinc;
        for ( j = 0; j < disparityImage16.ncols; j++, k++ )
        {
            disparity = row[j];

            // do not save invalid points
            if ( disparity < 0xFF00 )
            {
                // convert the 16 bit disparity value to floating point x,y,z
                triclopsRCD16ToXYZ( triclops, i, j, disparity, &x, &y, &z );

                // look at points within a range
                if ( z < 5.0 )
                {
                    if ( isColor )
                    {
                        r = (int)colorImage.red[k];
                        g = (int)colorImage.green[k];
                        b = (int)colorImage.blue[k];		  
                    }
                    else
                    {
                        // For mono cameras, we just assign the same value to RGB
                        r = (int)monoImage.data[k];
                        g = (int)monoImage.data[k];
                        b = (int)monoImage.data[k];
                    }

                    fprintf( pPointFile, "%f %f %f %d %d %d %d %d\n", x, y, z, r, g, b, i, j );
                    nPoints++;
                }
            }
        }
    }

    fclose( pPointFile );
    printf( "Points in file: %d\n", nPoints );

    return 0;

}

