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
// convertimage16:
//
// Usage: convertimage16 [<input image> <output image>]
//
// Default input image = "disparity-subpixel.pgm"
// Default output image = "disparity-scaled.pgm"
//
// This example shows how to convert a 16 bit disparity image into an 8 bit 
// disparity image for viewing purposes.  It can also be used to convert 16-bit
// images that have been saved by your own processing into an easily viewed
// 8-bit image.  
//
// The PGM file format readily supports 16-bits per image.  The problem is that
// almost all viewers for displaying these images either assume that all images
// are 8-bit (such as PaintShop Pro) or throw away the bottom 8 bits (such as
// the Linux image viewer 'xv').
// 
// This example converts a 16-bit image to an 8-bit image, and also remaps the
// image so that it has more contrast and is easier to view than the original
// image.  It remaps the input image into the 90 to 255 greyscale range.  This
// will generally be both brightening and stretching the range of the input 
// image.
//
// The example image, "disparity-subpixel.pgm" was obtained from the 
// customizedstereo.cpp example in the stereo directory.
//
//=============================================================================

//=============================================================================
// System Includes
//=============================================================================
#include <stdio.h>
#include <stdlib.h>
#include <assert.h>
#include <string>

//=============================================================================
// PGR Includes
//=============================================================================
#include "triclops.h"
#include "pgrpnmio.h"
#include "pnmutils.h"

//=============================================================================
// Project Includes
//=============================================================================

// Scale a 16 bit image to an 8 bit image. 
//
// note: it is assumed that the TriclopsImage has been pre-allocated
void
scaleImage16ToImage8( TriclopsImage16*  pImage16,
                      TriclopsImage*    pImage,
                      unsigned char     ucMinOut,
                      unsigned char     ucMaxOut );


int
main( int argc, char** argv )
{
    std::string szInputFile    = "disparity-subpixel.pgm";
    std::string szOutputFile16 = "disparity-subpixel-16.pgm";
    std::string szOutputFile   = "disparity-scaled.pgm";

    switch ( argc )
    {
        case 3:
        szInputFile  = argv[1];
        szOutputFile = argv[2];
        break;
        case 1:
        // use default parameters
        break;
        default:
        printf( "Usage: convertimage16 <input pgm> <output pgm>\n" );
        return 1;
    }

    // Load the 16-bit image from file - note that this file must be a 16 bit
    // PGM or else the read function will fail
    TriclopsImage16 image16;
    if ( !pgmReadToTriclopsImage16( szInputFile.c_str(),  &image16 ) )
    {
        printf( "pgmReadToTriclopsImage16() failed. Can't read '%s'\n", szInputFile.c_str() );
        return 1;
    }

    // Save 16bpp image to file
    TriclopsBool rc;
    rc = pgmWriteFromTriclopsImage16 ( szOutputFile16.c_str() , &image16 );
    if ( false == rc )
    {
        printf( "pgmWriteFromTriclopsImage16() failed. Can't write '%s'\n", szOutputFile16.c_str() );
        return 1;
    }

    // allocate data members for a TriclopsImage that will hold the 8bit version of
    // the input image.
    TriclopsImage image;
    image.nrows  = image16.nrows;
    image.ncols  = image16.ncols;
    image.rowinc = image.ncols;
    image.data   = (unsigned char*) malloc( image.nrows * image.ncols );
   

    // scale the output image
    scaleImage16ToImage8( &image16, &image, 90, 255 );

    triclopsSaveImage( &image, const_cast<char *>(szOutputFile.c_str()) );
   
    // clean up memory allocated in context
    freeImage16( &image16 );
    freeImage( &image );
   
    return 0;
}


void
scaleImage16ToImage8( TriclopsImage16*  pImage16,
                      TriclopsImage*    pImage,
                      unsigned char     ucMinOut,
                      unsigned char     ucMaxOut )
{

   double dMinOut = (double) ucMinOut;
   double dMaxOut = (double) ucMaxOut;

   // find the max and minimum disparities
   double dMaxDisp = 0;
   double dMinDisp = 255;
   int    r;

   for ( r = 0; r < pImage16->nrows; r++ )
   {
      unsigned short*   pusSrc = pImage16->data + r*pImage16->rowinc/sizeof(unsigned short);
      for ( int c = 0; c < pImage16->ncols; c++ )
      {
         if ( pusSrc[c] < 0xff00 )
         {
            double dDisp = (double) pusSrc[c]/256.0;
            if ( dMaxDisp < dDisp )
               dMaxDisp = dDisp;
            if ( dMinDisp > dDisp )
               dMinDisp = dDisp;
         }
      }
   }

   // scale the output to take the disparity values of the input image that fall within
   // dMinDisp to dMaxDisp to fall within ucMinOut and ucMaxOut for the 8 bit output image
   for ( r = 0; r < pImage16->nrows; r++ )
   {
      unsigned short*   pusSrc = pImage16->data + r*pImage16->rowinc/sizeof(unsigned short);
      unsigned char*    pucDst = pImage->data + r*pImage->rowinc;
      for ( int c = 0; c < pImage16->ncols; c++ )
      {
         if ( pusSrc[c] < 0xff00 )
         {
            double dDisp = (double) pusSrc[c]/256.0;
            double dOut = (dDisp-dMinDisp)*(dMaxOut-dMinOut)/(dMaxDisp-dMinDisp);
            dOut += dMinOut;
            pucDst[c]   = (unsigned char) dOut;
         }
         else
         {
            pucDst[c]   = 0;
         }
      }
   }
}
