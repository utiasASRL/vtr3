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

#ifndef FLYCAPTURE2BRIDGE_H
#define FLYCAPTURE2BRIDGE_H

//////////////////////////////////////////////////////////////////////////


// fwd declarations
typedef void* TriclopsContext;
namespace FlyCapture2
{
   class Image;
}

//////////////////////////////////////////////////////////////////////////

namespace Fc2Triclops
{

    //
    // Name: ErrorType
    //
    // Description:
    //    All FlyCapture2Triclops functions return an error value that 
    //    indicates whether an error occurred, and if so what error.  The 
    //    following enumerated type lists the kinds of errors that may be 
    //    returned.
    //
    enum ErrorType
    {
       // Function succeeded.
       ERRORTYPE_OK,
       // The reading of camera configuration failed.
       ERRORTYPE_CONFIG_READ_FAILED,
       // The configuration file is empty or not exist
       ERRORTYPE_NO_CONFIG_FILE,
       // Camera is not connected
       ERRORTYPE_NO_CAMERA,
       // The configuration file is corrupted
       ERRORTYPE_CORRUPT_CONFIG_FILE,
       // The allocation of output failed
       ERRORTYPE_FAILED_OUTPUT_ALLOCATION,
       // The pixel format conversion failed
       ERRORTYPE_FAILED_CONVERSION,
       // Generic error to indicate that an input variable is invalid
       ERRORTYPE_INVALID_INPUT,
       // Generic error to indicate that an output variable or operation is invalid
       ERRORTYPE_INVALID_OUTPUT,
       // The format Format7 is not supported
       ERRORTYPE_NO_FMT7_SUPPORT,
       // The format Format7 is invalid
       ERRORTYPE_INVALID_FMT7,
       // The stereo camera mode is not correct
       ERRORTYPE_NO_CORRECT_CAMERAMODE 
    };

    //
    // Name: getContextFromCamera
    // 
    // Synopsis:
    //  Generates a TriclopsContext from the connected camera
    // 
    // Input:
    //  cameraSerialNumber - The camera serial number
    //
    // Output:
    //  outputContext - the context
    //
    // Returns:
    //  ERRORTYPE_OK                  - The operation succeeded.
    //  ERRORTYPE_NO_CAMERA           - Camera is not connected
    //  ERRORTYPE_CONFIG_READ_FAILED  - The reading of camera configuration 
    //                                   failed.
    //  ERRORTYPE_NO_CONFIG_FILE      - The config file does not exist or 
    //                                   is empty
    //  ERRORTYPE_CORRUPT_CONFIG_FILE - The configuration file is corrupted
    //
    // Description:
    //  This function is used to generate a TriclopsContext starting from the
    //  serial number of the connected camera.  
    //
    //
    ErrorType
    getContextFromCamera( unsigned long cameraSerialNumber, 
	       		 TriclopsContext* outputContext );

    //
    // Name: unpackUnprocessedRawOrMono16Image
    // 
    // Synopsis:
    //  Unpacks a raw image into a right and left raw images 
    // 
    // Input:
    //  unprocessedImage - The image to unpack 
    //  isLittleEndian   - the image endianess. 
    //                     Little endian = right-left image pixel
    //                     Big endian    = left-right image pixel
    // 
    //
    // Output:
    //  unprocessedImageRight - the unpacked right raw image
    //  unprocessedImageLeft  - the unpacked left raw image
    //
    // Returns:
    //  ERRORTYPE_OK                  - The operation succeeded.
    //  ERRORTYPE_FAILED_CONVERSION   - The pixel format conversion failed
    //  ERRORTYPE_INVALID_INPUT       - Generic error to indicate that an input variable is invalid
    //  ERRORTYPE_INVALID_OUTPUT      - Generic error to indicate that an output
    //                                   variable or operation is invalid
    //
    // Description:
    //  This function converts a pixel interleaved raw image to two de-interleaved images
    //
    //
    ErrorType
    unpackUnprocessedRawOrMono16Image( const FlyCapture2::Image& unprocessedImage, 
				 bool isLittleEndian, 
				 FlyCapture2::Image& unprocessedImageRight, 
				 FlyCapture2::Image& unprocessedImageLeft);

    //
    // Name: unpackUnprocessedRgbImage
    // 
    // Synopsis:
    //  Unpacks a raw image into a right and left raw images 
    // 
    // Input:
    //  unprocessedImage - The image to unpack 
    // 
    //
    // Output:
    //  unprocessedImageRight  - the unpacked right raw image
    //  unprocessedImageMiddle - the unpacked middle raw image
    //  unprocessedImageLeft   - the unpacked left raw image
    //
    // Returns:
    //  ERRORTYPE_OK                  - The operation succeeded.
    //  ERRORTYPE_FAILED_CONVERSION   - The pixel format conversion failed
    //  ERRORTYPE_INVALID_INPUT       - Generic error to indicate that an input variable 
    //                                   is invalid
    //  ERRORTYPE_INVALID_OUTPUT      - Generic error to indicate that an output
    //                                   variable or operation is invalid
    //
    // Description:
    //  This function converts a pixel interleaved raw image to three de-interleaved images.
    //
    //
    ErrorType 
    unpackUnprocessedRgbImage( const FlyCapture2::Image& unprocessedImage, 
			     FlyCapture2::Image& unprocessedImageRight, 
			     FlyCapture2::Image& unprocessedImageMiddle, 
			     FlyCapture2::Image& unprocessedImageLeft );

    //
    // Name: packTwoSideBySideRgbImage
    // 
    // Synopsis:
    //  Packs side by side two RGB images into a single image
    // 
    // Input:
    //  unprocessedImage - The image to unpack 
    // 
    //
    // Output:
    //  imageRgbRight - the RGB right image
    //  imageRgbLeft  - the RGB left image
    //
    // Returns:
    //  ERRORTYPE_OK                  - The operation succeeded.
    //  ERRORTYPE_FAILED_CONVERSION   - The pixel format conversion failed
    //  ERRORTYPE_INVALID_INPUT       - Generic error to indicate that an input variable 
    //                                   is invalid
    //  ERRORTYPE_INVALID_OUTPUT      - Generic error to indicate that an output
    //                                   variable or operation is invalid
    //
    // Description:
    //  This function packs two RGB images into a single RGB image. Each row of the 
    //  packed image is composed by the corresponding line of the first image followed 
    //  by the line of the second image
    //
    //				 
    ErrorType 
    packTwoSideBySideRgbImage( const FlyCapture2::Image& imageRgbRight, 
			      const FlyCapture2::Image& imageRgbLeft, 
			      FlyCapture2::Image& packedRgbImage);


    //
    // Name: packThreeSideBySideRgbImage
    // 
    // Synopsis:
    //  Packs side by side three RGB images into a single image
    // 
    // Input:
    //  unprocessedImage - The image to unpack 
    // 
    //
    // Output:
    //  imageRgbRight  - the RGB right image
    //  imageRgbMiddle - the RGB middle image
    //  imageRgbLeft   - the RGB left image
    //
    // Returns:
    //  ERRORTYPE_OK                  - The operation succeeded.
    //  ERRORTYPE_FAILED_CONVERSION   - The pixel format conversion failed
    //  ERRORTYPE_INVALID_INPUT       - Generic error to indicate that an input variable 
    //                                   is invalid
    //  ERRORTYPE_INVALID_OUTPUT      - Generic error to indicate that an output
    //                                   variable or operation is invalid
    //
    // Description:
    //  This function packs three RGB images into a single RGB image. Each row of the 
    //  packed image is composed by the corresponding line of the first, second
    //  and third image, respectively.
    //
    //				 
    ErrorType 
    packThreeSideBySideRgbImage( const FlyCapture2::Image& imageRgbRight,
				const FlyCapture2::Image& imageRgbMiddle, 
				const FlyCapture2::Image& imageRgbLeft, 
				FlyCapture2::Image& packedRgbImage);
}

//////////////////////////////////////////////////////////////////////////

#endif // FLYCAPTURE2BRIDGE_H

