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
// $Id: customizedstereo.cpp,v 1.4 2008-04-22 22:01:14 mgriffin Exp $
//=============================================================================
//=============================================================================
// customizedstereo:
//
// This example demonstrates the effect of a variety of Triclops library 
// parameters.  The idea is to show by example just some of the many parameters
// you can control to tweak the behaviour of the stereo algorithm.  
//
// The best way to get an idea of how well the stereo is working is to
// run the triclopsDemo program. This allows you to change all the parameters
// and observe their effects in real time. The purpose of this example is to
// show how easy it is to control there parameters in your own stereo program.
//
// All the images in this series except for the 16-bit subpixel value are 
// scaled for better viewing.  Invalid pixels are black.
// 
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



// Scale a 16 bit image to an 8 bit image. 
//
// note: it is assumed that the TriclopsImage has been pre-allocated
void
scaleImage16ToImage8( TriclopsImage16 * pImage16,
                        TriclopsImage * pImage,
                        unsigned char	 ucMinOut,
                        unsigned char	 ucMaxOut );

// Scale an 8 bit image 
//
// note: this scaling is done "in place" so it stomps over the current image
void
scaleImage( TriclopsImage * pImage,
            unsigned char   ucMinOut,
            unsigned char	ucMaxOut );

void
doRectifyStereoSave( TriclopsContext triclops,
                     TriclopsInput * pTriclopsInput,
                     const char    * szDisparityBase,
                     const char    * szNameModifier );


int
main( int /* argc */, char** /* * argv */ )
{
    TriclopsContext	triclops;
    TriclopsError   error;
    TriclopsInput 	triclopsInput;
    std::string 	szInputFile("input.ppm");
    std::string 	szCalFile("input.cal");
    std::string 	szDisparityBase("disparity");


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
   
    doRectifyStereoSave( triclops, &triclopsInput, szDisparityBase.c_str(), "default" );

    // try doing stereo with a big stereo mask
    triclopsSetStereoMask( triclops, 21 );
    doRectifyStereoSave( triclops, &triclopsInput, szDisparityBase.c_str(), "bigmask" );

    // try doing stereo with a small stereo mask
    triclopsSetStereoMask( triclops, 5 );
    doRectifyStereoSave( triclops, &triclopsInput, szDisparityBase.c_str(), "smallmask" );

    // set mask to a neutral value
    triclopsSetStereoMask( triclops, 11 );

    // try doing stereo without any validation
    triclopsSetSurfaceValidation( triclops, 0 );
    triclopsSetUniquenessValidation( triclops, 0 );
    triclopsSetTextureValidation( triclops, 0 );
    doRectifyStereoSave( triclops, &triclopsInput, szDisparityBase.c_str(), "novalidation" );

    // try doing stereo with only texture and surface
    triclopsSetSurfaceValidation( triclops, 1 );
    triclopsSetTextureValidation( triclops, 1 );
    doRectifyStereoSave( triclops, &triclopsInput, szDisparityBase.c_str(), "surf-tex-val" );

    // try doing stereo in 2 camera mode with back and forth validation
    triclopsSetCameraConfiguration( triclops, TriCfg_2CAM_HORIZONTAL );
    triclopsSetBackForthValidation( triclops, 1 );
    doRectifyStereoSave( triclops, &triclopsInput, szDisparityBase.c_str(), "backforth" );

    // try doing stereo in subpixel mode to compare the results of subpixel with
    // whole pixel - try running "convertimage16" example to convert this image to
    // one that is more easily viewed
    triclopsSetSubpixelInterpolation( triclops, 1 );
    doRectifyStereoSave( triclops, &triclopsInput, szDisparityBase.c_str(), "subpixel" );
   
    // clean up memory allocated in context
    freeInput( &triclopsInput );
    error = triclopsDestroyContext( triclops );
    _HANDLE_TRICLOPS_ERROR( "triclopsDestroyContext()", error );
   
    return 0;
}

void
scaleImage16ToImage8( TriclopsImage16 * pImage16,
                        TriclopsImage * pImage,
                        unsigned char	 ucMinOut,
                        unsigned char	 ucMaxOut )
{

    double dMinOut = (double) ucMinOut;
    double dMaxOut = (double) ucMaxOut;

    // find the max and minimum disparities
    double dMaxDisp = 0;
    double dMinDisp = 255;
    int    r;

    for ( r = 0; r < pImage16->nrows; r++ )
    {
        unsigned short* 	pusSrc = pImage16->data + r*pImage16->rowinc/sizeof(unsigned short);
        for ( int c = 0; c < pImage16->ncols; c++ )
        {
            if ( pusSrc[c] < 0xff00 )
            {
                double dDisp = (double) pusSrc[c]/256.0;
                if ( dMaxDisp < dDisp )
                {
                    dMaxDisp = dDisp;
                }
                if ( dMinDisp > dDisp )
                {
                   dMinDisp = dDisp;
                }
            }
        }
    }

    // scale the output to take the disparity values of the input image that fall within
    // dMinDisp to dMaxDisp to fall within ucMinOut and ucMaxOut for the 8 bit output image
    for ( r = 0; r < pImage16->nrows; r++ )
    {
        unsigned short * pusSrc = pImage16->data + r*pImage16->rowinc/sizeof(unsigned short);
        unsigned char *	 pucDst = pImage->data + r*pImage->rowinc;
        for ( int c = 0; c < pImage16->ncols; c++ )
        {
	        if ( pusSrc[c] < 0xff00 )
	        {
	            double dDisp = (double) pusSrc[c]/256.0;
	            double dOut = (dDisp-dMinDisp)*(dMaxOut-dMinOut)/(dMaxDisp-dMinDisp);
	            dOut += dMinOut;
	            pucDst[c]	= (unsigned char) dOut;
	        }
	        else
	        {
	            pucDst[c]	= 0;
	        }
        }
    }
}

// Scale an 8 bit image 
//
// note: this scaling is done "in place" so it stomps over the current image
void
scaleImage( TriclopsImage * pImage,
            unsigned char	 ucMinOut,
            unsigned char	 ucMaxOut )
{
    int r, c;

    double dMinOut = (double) ucMinOut;
    double dMaxOut = (double) ucMaxOut;

    // find the max and minimum disparities
    double dMaxDisp = 0;
    double dMinDisp = 255;
    for ( r = 0; r < pImage->nrows; r++ )
    {
        unsigned char * pucSrc = pImage->data + r*pImage->rowinc;
        for ( c = 0; c < pImage->ncols; c++ )
        {
	        // note: 240 is the limit of the normal disparity range
	        if ( pucSrc[c] < 240 )
	        {
	            double dDisp = (double) pucSrc[c];
	            if ( dMaxDisp < dDisp )
                {
	                dMaxDisp = dDisp;
                }
	            if ( dMinDisp > dDisp )
                {
	                dMinDisp = dDisp;
                }
	        }
        }
    }

    // scale the output to take the disparity values of the input image that fall within
    // dMinDisp to dMaxDisp to fall within ucMinOut and ucMaxOut for the 8 bit output image
    for ( r = 0; r < pImage->nrows; r++ )
    {
        unsigned char* 	pucSrc = pImage->data + r*pImage->rowinc;
        for ( c = 0; c < pImage->ncols; c++ )
        {
            if ( pucSrc[c] < 240 )
            {
                double dDisp = (double) pucSrc[c];
                double dOut = (dDisp-dMinDisp)*(dMaxOut-dMinOut)/(dMaxDisp-dMinDisp);
                dOut += dMinOut;
                pucSrc[c]	= (unsigned char) dOut;
            }
            else
            {
                pucSrc[c]	= 0;
            }
        }
    }
}

void
doRectifyStereoSave( TriclopsContext triclops,
                    TriclopsInput *	 pTriclopsInput,
                    const char    *	 pszDisparityBase,
                    const char    *  pszNameModifier )
{
    TriclopsError error;
    // Rectify the images
    error = triclopsRectify( triclops, pTriclopsInput );
    _HANDLE_TRICLOPS_ERROR( "triclopsRectify()", error );

    // Do stereo processing
    error =  triclopsStereo( triclops );
    _HANDLE_TRICLOPS_ERROR( "triclopsStereo()", error );
   
    // test whether we are doing subpixel stereo
    TriclopsBool bSubpixel;
    error =  triclopsGetSubpixelInterpolation( triclops, &bSubpixel );
    _HANDLE_TRICLOPS_ERROR( "triclopsGetSubpixelInterpolation()", error );

    char szDisparityFile[1024];
    if ( bSubpixel )
    {
        // Retrieve the disparity image from the context
        TriclopsImage16 triclopsImage16;
        error = triclopsGetImage16( triclops, 
				  TriImg16_DISPARITY, 
				  TriCam_REFERENCE, 
				  &triclopsImage16 );

        _HANDLE_TRICLOPS_ERROR( "triclopsGetImage16(): Failed to get disparity image", 
			      error );
      
        // create the disparity name
#if defined( WIN64 ) || defined (WIN32) 
        sprintf_s( szDisparityFile, sizeof(szDisparityFile),"%s-%s.pgm", pszDisparityBase, pszNameModifier );
#else
        snprintf( szDisparityFile, sizeof(szDisparityFile), "%s-%s.pgm", pszDisparityBase, pszNameModifier );
#endif

        // Save the disparity 16-bit image
        error = triclopsSaveImage16( &triclopsImage16, szDisparityFile );
        _HANDLE_TRICLOPS_ERROR( "triclopsSaveImage16(): Failed to save disparity image", error );
        printf( "Wrote 16-bit disparity image to '%s'\n", szDisparityFile );

        // scale it into an 8 bit image and save that as well
        TriclopsImage triclopsImage;
        // need to allocate the 8-bit TriclopsImage to scale this one into
        triclopsImage.nrows  = triclopsImage16.nrows;
        triclopsImage.ncols  = triclopsImage16.ncols;
        triclopsImage.rowinc = triclopsImage16.ncols;
        triclopsImage.data	 = (unsigned char*) malloc( triclopsImage.nrows*triclopsImage.ncols );

        scaleImage16ToImage8( &triclopsImage16, &triclopsImage, 90, 255 );

        // create the disparity name
#if defined( WIN64 ) || defined (WIN32)
        sprintf_s( szDisparityFile, sizeof(szDisparityFile), "%s-%s-scaled.pgm", pszDisparityBase, pszNameModifier );
#else
        snprintf( szDisparityFile, sizeof(szDisparityFile), "%s-%s-scaled.pgm", pszDisparityBase, pszNameModifier );
#endif
        triclopsSaveImage( &triclopsImage, szDisparityFile );
        printf( "Wrote disparity image to '%s'\n", szDisparityFile );
      
        free( triclopsImage.data );
    }	
    else
    {
        // Retrieve the disparity image from the context
        TriclopsImage triclopsImage;
        error = triclopsGetImage( triclops, TriImg_DISPARITY, TriCam_REFERENCE, &triclopsImage );
        _HANDLE_TRICLOPS_ERROR( "triclopsGetImage(): Failed to get disparity image", 
			      error );

        // scale the image so it is easier to see
        scaleImage( &triclopsImage, 90, 255 );
      
        // create the disparity name
#if defined( WIN64 ) || defined (WIN32)
        sprintf_s( szDisparityFile, sizeof(szDisparityFile), "%s-%s.pgm", pszDisparityBase, pszNameModifier );
#else
        snprintf( szDisparityFile, sizeof(szDisparityFile), "%s-%s.pgm", pszDisparityBase, pszNameModifier );
#endif
        // Save the disparity image
        error = triclopsSaveImage( &triclopsImage, szDisparityFile );
        _HANDLE_TRICLOPS_ERROR( "triclopsSaveImage(): Failed to save disparity image", error );
        printf( "Wrote disparity image to '%s'\n", szDisparityFile );
   }

   return;
}

