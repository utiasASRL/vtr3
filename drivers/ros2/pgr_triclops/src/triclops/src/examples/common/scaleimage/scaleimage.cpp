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
// scaleimage:
//
// Usage: scaleimage [<input image> <output image> <min val output> <max val output>]
//
// Default input image = "disparity.pgm"
// Default output image = "disparity-scaled.pgm"
// Default min val output = 90
// Default max val output = 255
//
// This example shows how to linearly scale a 8 bit disparity image. The 
// disparity values of the input image that fall within a minimum and a 
// maximum are mapped to fall within min output and max output for the 8 bit 
// output image
//
// The default example image "disparity.pgm" was obtained from the 
// stereo.cpp example in the stereo directory or grabstereo.cpp example in 
// grastereo directory.
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
#include "pnmutils.h"

//=============================================================================
// Project Includes
//=============================================================================


// Scale an 8 bit image 
// note: this scaling is done "in place" so it stomps over the current image
void
scaleImage( TriclopsImage * pImage,
	        unsigned char	ucMinOut,
	        unsigned char	ucMaxOut );


int
main( int argc, char** argv )
{
    std::string  szInputFile  = "disparity.pgm";
    std::string  szOutputFile = "disparity-scaled.pgm";
    unsigned char ucMinOut	 = 90;
    unsigned char ucMaxOut	 = 255;

    switch ( argc )
    {
        case 5:
            ucMinOut = (unsigned char) atoi( argv[3] );
            ucMaxOut = (unsigned char) atoi( argv[4] );
            // deliberately fall through to 3-parm case
        case 3:
            szInputFile	= argv[1];
            szOutputFile	= argv[2];
            break;
        case 1:
            // use default parameters
            break;
        default:
            printf( "Usage: scaleimage <input pgm> <output pgm> [<min out> <max out>]\n" );
            return 1;
    }

    // Load the bit image from file 
    TriclopsImage image;
    if ( !pgmReadToTriclopsImage( szInputFile.c_str(),  &image ) )
    {
        printf( "pgmReadToTriclopsImage() failed. Can't read '%s'\n", szInputFile.c_str() );
        return 1;
    }

    // scale the output image
    scaleImage( &image, ucMinOut, ucMaxOut );

    triclopsSaveImage( &image, const_cast<char *>(szOutputFile.c_str()) );
   
    // clean up memory allocated in context
    freeImage( &image );
   
    return 0;
}


void
scaleImage( TriclopsImage * pImage,
	        unsigned char	ucMinOut,
	        unsigned char	ucMaxOut )
{
    int r, c;

    double dMinOut = (double) ucMinOut;
    double dMaxOut = (double) ucMaxOut;

    // find the max and minimum disparities
    double dMaxDisp = 0;
    double dMinDisp = 255;
    for ( r = 0; r < pImage->nrows; r++ )
    {
        unsigned char* 	pucSrc = pImage->data + r*pImage->rowinc;
        for ( c = 0; c < pImage->ncols; c++ )
        {
            // note: 240 is the limit of the normal disparity range
            if ( pucSrc[c] < 240 )
            {
                double dDisp = (double) pucSrc[c];
                if ( dMaxDisp < dDisp )
                    dMaxDisp = dDisp;
                if ( dMinDisp > dDisp )
                    dMinDisp = dDisp;
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



