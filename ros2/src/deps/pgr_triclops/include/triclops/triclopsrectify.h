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
// SOFTWARE, EITHER EXPRESSED OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR
// PURPOSE, OR NON-INFRINGEMENT. PGR SHALL NOT BE LIABLE FOR ANY DAMAGES
// SUFFERED BY LICENSEE AS A RESULT OF USING, MODIFYING OR DISTRIBUTING
// THIS SOFTWARE OR ITS DERIVATIVES.
//
//=============================================================================
//=============================================================================
// $Id: triclopsrectify.h,v 2.5 2010-06-24 22:04:18 arturp Exp $
//=============================================================================
#ifndef TRICLOPSRECTIFY_H
#define TRICLOPSRECTIFY_H

//=============================================================================
//
// This file defines the API for the rectification and camera geometry
// functions of the Triclops Stereo Vision SDK.
//
//=============================================================================


//=============================================================================
// Defines
//=============================================================================

//=============================================================================
// System Includes  
//=============================================================================

#include <triclops.h>

#ifdef __cplusplus
extern "C"
{
#endif
   
//=============================================================================
// Macros  
//=============================================================================


//=============================================================================
// Enumerations  
//=============================================================================

//=============================================================================
// Types 
//=============================================================================
//
// Group = Types

//
// Name: TriclopsRectImgQuality
//
// Description:
//    This enumerated type identifies 
//
typedef enum TriclopsRectImgQuality
{
   TriRectQlty_FAST,
   TriRectQlty_STANDARD,
   TriRectQlty_ENHANCED_1,
   TriRectQlty_ENHANCED_2
} TriclopsRectImgQuality;

//=============================================================================
// Function Prototypes  
//=============================================================================

//=============================================================================
// Rectification and Camera Geometry
//=============================================================================
// Group = Rectification and Camera Geometry

//
// Name: triclopsSetSourceResolution
//
// Synopsis:
//  Sets the resolution of the raw images that the library will be processing 
// later.
//
// Input:
//  context - The context.
//  nSrcRows   - Number of rows in the raw images.
//  nSrcCols   - Number of columns in the raw images.
//
// Returns:
//  TriclopsErrorOk             - The operation succeeded.
//  InvalidContext - The input context was invalid.
//  InvalidSetting - The aspect ratio of the requested image was not 
//                   4 columns to 3 rows, or was a negative size.	
//
// Description:
//  This function sets the expected resolution of the raw images. 
//  This function is provided primarily to support feature based stereo application
//  where one is expected to make direct calls to triclopsRectifyPixel() and 
//  triclopsUnrectifyPixel() on a point by point basis.  For regular stereo
//  application where an entire image will be rectified every time, the application
//  should use triclopsSetResolutionAndPrepare() which in addition to setting
//  up both the source and rectification resolution, it also creates the rectification 
//  table to speed up the full image rectification calls.
//
// See Also:
//  triclopsGetSourceResolution(), triclopsRectifyPixel(), triclopsUnrectifyPixel(),
//	triclopsSetResolutionAndPrepare()
//
TriclopsError
triclopsSetSourceResolution( TriclopsContext   context, 
		       int		 nSrcRows,
		       int		 nSrcCols );

//
// Name: triclopsSetResolution
//
// Synopsis:
//  Sets the resolution of the resultant images.  This includes rectified, 
//  disparity and edge images.
//
// Input:
//  context - The context.
//  nrows   - Number of rows in the output images.
//  ncols   - Number of columns in the output images.
//
// Returns:
//  TriclopsErrorOk             - The operation succeeded.
//  InvalidContext - The input context was invalid.
//  InvalidSetting - The aspect ratio of the requested image was not 
//                   4 columns to 3 rows, or was a negative size.	
//
// Description:
//  This function sets the desired resolution of the output images.  These 
//  images include the rectified, disparity and edge images.  This resolution
//  must maintain the 640x480 columns to rows ratio.  If the user wishes to 
//  have an image of a different aspect ratio, he/she must use Regions Of 
//  Interest to control the size of the image that is being processed.
//
// See Also:
//  triclopsGetResolution(), triclopsGetROIs()
//
TriclopsError
triclopsSetResolution( TriclopsContext   context, 
		       int		 nrows,
		       int		 ncols );

//
// Name: triclopsSetResolutionAndPrepare
//
// Synopsis:
//  Sets the resolution of the resultant images.  This includes rectified, 
//  disparity and edge images.
//
// Input:
//  context    - The context.
//  nrows      - Number of rows in the output images.
//  ncols      - Number of columns in the output images.
//  nInputRows - Number of rows in the input images.
//  nInputCols - Number of columns in the input images.
//
// Returns:
//  TriclopsErrorOk - The operation succeeded.
//  InvalidContext - The input context was invalid.
//  InvalidSetting - The aspect ratio of the requested image was not 4 columns 
//                   to 3 rows, or was of negative size.
//
// Description:
//  This function sets the desired resolution of the output images and also 
//  immediately constructs the rectification tables.  For large images, the 
//  construction of the rectification can take a while. This function allows 
//  you to control when the construction takes place, otherwise it will occur 
//  during the first call to triclopsPreprocess().  The resolution of the input 
//  images must be specified at this time, as this is necessary for the 
//  construction of the tables.  The output images include the rectified, 
//  disparity and edge images.  This requested resolution must maintain the 
//  640x480 columns to rows ratio.  If the user wishes to have an image of 
//  a different aspect ratio, he/she must use Regions Of Interest to control 
//  the size of the image that is being processed.
//  For feature based stereo application where rectification of the entire image
//  is not needed, one should call triclopsSetResolution() and triclopsSetSourceResolution()
//  only (these are much simpler functions), and then simply proceeds to call
//  triclopsRectifyPixel() and triclopsUnrectifyPixel() for the small set of feature
//  pixels needed.
//
// See Also:
//  triclopsGetResolution(), triclopsSetResolution(), triclopsSetSourceResolution(),
//  triclopsSetRectify()
//
TriclopsError
triclopsSetResolutionAndPrepare( TriclopsContext   context, 
				 int		   nrows,
				 int		   ncols,
				 int		   nInputRows,
				 int		   nInputCols );

//
// Name: triclopsGetSourceResolution
//
// Synopsis:
//  Retrieves the resolution of the resultant images.  This includes rectified, 
//  disparity and edge images.
//
// Input:
//  context - The context.
//
// Output:
//  nSrcRows - Number of rows in the raw images.
//  nSrcCols - Number of columns in the raw images.
//
// Returns:
//  TriclopsErrorOk             - The operation succeeded.
//  InvalidContext - The input context was invalid.
// 
// Description:
//  This returns the current resolution for raw images of the given context.
//
// See Also:
//  triclopsSetSourceResolution()
//
TriclopsError
triclopsGetSourceResolution( const TriclopsContext	context, 
		       int*			nSrcRows,
		       int*			nSrcCols );

//
// Name: triclopsGetResolution
//
// Synopsis:
//  Retrieves the resolution of the resultant images.  This includes rectified, 
//  disparity and edge images.
//
// Input:
//  context - The context.
//
// Output:
//  nrows - Number of rows in the output images.
//  ncols - Number of columns in the output images.
//
// Returns:
//  TriclopsErrorOk             - The operation succeeded.
//  InvalidContext - The input context was invalid.
// 
// Description:
//  This returns the current resolution for output images of the given context.
//
// See Also:
//  triclopsSetResolution()
//
TriclopsError
triclopsGetResolution( const TriclopsContext	context, 
		       int*			nrows,
		       int*			ncols );

//=============================================================================
// Rectification
//=============================================================================
//
// Group = Rectification 


//
// Name: triclopsSetLowpass
//
// Synopsis:
//  Turns low-pass filtering before rectification on or off.
// 
// Input:
//  context - The context.
//  on      - A Boolean value indicating whether it should be turned on or off.	
//
// Returns:
//  TriclopsErrorOk             - The operation succeeded.
//  InvalidContext - The input context was invalid.	
//
// See Also:
//  triclopsGetLowpass()
//
TriclopsError
triclopsSetLowpass( TriclopsContext	context,
		    TriclopsBool	on );

//
// Name: triclopsGetLowpass
//
// Synopsis:
//  Retrieves the state of the low-pass filtering feature.
//
// Input:
//  context - The context.
//  
// Output:
//  on  - A pointer to a Boolean variable that will store the current setting.
//
// Returns:
//  TriclopsErrorOk             - The operation succeeded.
//  InvalidContext - The input context was invalid.	
//
// See Also:
//  triclopsSetLowpass()
//
TriclopsError
triclopsGetLowpass( const TriclopsContext	context,
		    TriclopsBool*		on );

//
// Name: triclopsSetRectify
//
// Synopsis:
//  Turns rectification on or off.
//
// Input:
//  context - The context.
//  on      - A Boolean indicating whether rectification should be turned on or 
//            off.	
// 
// Returns:
//  TriclopsErrorOk             - The operation succeeded.
//  InvalidContext - The input context was invalid.
//
// See Also:
//  triclopsGetRectify(), triclopsSetRectImgQuality(), triclopsGetRectImgQuality()
//
TriclopsError
triclopsSetRectify( TriclopsContext	  context,
		    TriclopsBool	  on );

//
// Name: triclopsGetRectify
//
// Synopsis:
//  Retrieves the state of the rectification feature.
// 
// Input:
//  context - The context.
//
// Output:
//  on - A pointer to a Boolean that will store the current setting.
//
// Returns:
//  TriclopsErrorOk             - The operation succeeded.
//  InvalidContext - The input context was invalid.	
// 
// See Also:
//  triclopsSetRectify(), triclopsSetRectImgQuality(), triclopsGetRectImgQuality()
//
TriclopsError
triclopsGetRectify( const TriclopsContext	context,
		    TriclopsBool*		on );

//
// Name: triclopsSetRectImgQuality
//
// Description:
//    Sets the quality of the algorithm to use during image rectification.
// Higher-quality algorithms will generally require more processing time, but
// give more accurate results. Currently available are
// TriRectQlty_STANDARD, our classic rectification algorithm, and
// TriRectQlty_ENHANCED, which uses a more elaborate kernel in the rectification 
// process.
//
// Input:
//    context	  - The current TriclopsContext.
//    quality	  - The desired quality of the rectification algorithm to use.
//
// Returns:
//    TriclopsErrorOk		    - The operation succeeded.
//    TriclopsErrorInvalidContext   - If the context is invalid.
//
// See Also:
//    triclopsGetRectImgQuality()
//
TriclopsError
triclopsSetRectImgQuality(TriclopsContext context, TriclopsRectImgQuality quality);

//
// Name: triclopsGetRectImgQuality
//
// Description:
//    Gets the quality of the algorithm currently in use for image rectification.
//
// Input:
//    context	  - The current TriclopsContext.
//    quality	  - The quality of the rectification algorithm currently in use.
//
// Returns:
//    TriclopsErrorOk		    - The operation succeeded.
//    TriclopsErrorInvalidContext   - If the context is invalid.
//
// See Also:
//    triclopsSetRectImgQuality()
//
TriclopsError
triclopsGetRectImgQuality(TriclopsContext context, TriclopsRectImgQuality* quality);

//
// Name: triclopsRectifyPackedColorImage
//
// Description:
//  This function rectifies a packed color image.
//  This function will only rectify a packed TriclopsInput (a TriclopsInput
//  of type TriInp_RGB_32BIT_PACKED).  It is useful for creating rectified
//  color images for display, since bitmap displays commonly require the
//  data format to be packed.
//
// Input:
//  context - The TriclopsContext to use to rectify the color image.
//  nCamera - The camera from which this TriclopsInput originated.
//    If the system is a Color Triclops, nCamera should be set to
//    TriCam_COLOR.
//  input   - The color image to be rectified.
//
// Output:
//  output  - The rectified color image.
//
// Returns:
//  TriclopsErrorOk - Upon the successful completion of the operation.
//
TriclopsError
triclopsRectifyPackedColorImage( TriclopsContext	       	context,
				 TriclopsCamera	       		nCamera,
				 TriclopsInput*	       		input,
				 TriclopsPackedColorImage*  	output );

//
// Name: triclopsRectifyColorImage
//
// Synopsis:
//  Rectifies a TriclopsInput structure into a TriclopsColorImage.
//  This function is used for TriclopsInput's that have been
//  obtained from a Color Triclops.  If the system is
//  a Color Triclops, nCamera should be set to TriCam_COLOR.
//
// Input:
//  context - The context.
//  nCamera - A TriclopsCamera enumerated value indicating which camera
//            the input image came from.
//  input   - The raw color image encoded into a TriclopsInput structure.	
// 
// Output:
//  output - The resultant rectified color image.
//
// Returns:
//  TriclopsErrorOk              - The operation succeeded.
//  InvalidContext  - The input context was invalid.
//  InvalidRequest  - The input raw image had a size of 0.
//  InvalidCamera   - There is a corruption in the color
//                    camera calibration data in the calibration file.
//  InvalidSettings - The TriclopsInput structure did not have a recognizable 
//                    data type.
//
TriclopsError
triclopsRectifyColorImage( TriclopsContext      context,
			   TriclopsCamera       nCamera,
			   TriclopsInput*	input,
			   TriclopsColorImage*  output );


//
// Name: triclopsRectifyPixel
//
// Synopsis:
//  Converts a pixel coordinate location from the unrectified image coordinates 
//  to rectified image coordinates.
//  The source image dimension must have been previously set either via a call
//  to triclopsSetSourceResolution() or triclopsSetResolutionAndPrepare().
//
// Input:
//  context - The context.
//  camera  - The camera for which the pixel should be rectified.
//  rowIn   - The location of the pixel to rectify.
//  colIn   - The location of the pixel to rectify.	
//
// Output:
//  rowOut - The location of the rectified pixel.
//  colOut - The location of the rectified pixel.
//
// Returns:
//  TriclopsErrorOk             - The operation succeeded.
//  InvalidContext - The input context was invalid.
//  InvalidCamera  - The camera argument was invalid.
//  TriclopsErrorInvalidSetting - The raw image dimension has not been set	
//
// See Also:
//  triclopsPreprocess(), triclopsRectifyColorImage(), triclopsSetSourceResolution()
//
TriclopsError
triclopsRectifyPixel( const TriclopsContext	context, 
		      TriclopsCamera	   	camera,
		      float		   	rowIn, 		
		      float		   	colIn, 
		      float*		   	rowOut,	
		      float*		   	colOut );

//
// Name: triclopsUnrectifyPixel
//
// Synopsis:
//  Converts a pixel coordinate location from the rectified image coordinates 
//  to unrectified image coordinates.
//
// Input:
//  context - The context.
//  camera  - The camera for which the pixel should be unrectified.
//  rowIn   - The location of the pixel to unrectify.
//  colIn   - The location of the pixel to unrectify.	
// 
// Output:
//  rowOut - The location of the unrectified pixel.
//  colOut - The location of the unrectified pixel.
//
// Returns:
//  TriclopsErrorOk  - The operation succeeded.
//  InvalidContext   - The input context was invalid.
//  InvalidCamera    - The camera argument was invalid.
//  TriclopsErrorInvalidSetting - The raw image dimension has not been set	
//  InvalidRequest   - The requested rectified location cannot be unrectified 
//                     into a location that is within the proper raw image bounds
//
// Remarks:
//  This version will accommodate input rectified pixel locations that are 
//  outside of the normal rectified image bounds, as long as the corresponding 
//  unrectified location is within its own image bounds.
//  The source image dimension must have been previously set either via a call
//  to triclopsSetSourceResolution() or triclopsSetResolutionAndPrepare().
//
// See Also:
//  triclopsRectifyPixel(), triclopsSetSourceResolution()
//
TriclopsError
triclopsUnrectifyPixel( const TriclopsContext	context, 
			TriclopsCamera          camera,
			float		   	rowIn, 		
			float		   	colIn, 
			float*		   	rowOut,	
			float*		   	colOut );


//
// Name: triclopsPreprocess
//
// Synopsis:
//  Does image unpacking, smoothing, rectification and edge detection, 
//  as specified by parameters.
//
// Input:
//  context - The context.
//  input   - The image to be processed.	
//
// Returns:
//  TriclopsErrorOk             - The operation succeeded.
//  InvalidContext - The input context was invalid.
//  NotImplemented - TriclopsInput has rowinc < ncols*pixelsize
//
// Description:
//  This function does all necessary preprocessing on the input data.  It 
//  unpacks the data, which strips individual channels from 32-bit packed 
//  data and puts them into 3 TriImg_RAW channels.  It applies a low-pass 
//  filter on these channels if requested, and corrects for lens distortion 
//  and camera misalignment, saving these images into TriImg_RECTIFIED.  
//  Finally it performs 2nd derivative Gaussian edge processing on the 
//  rectified images and saves them into TriImg_EDGE internal images.
//
// See Also:
//  triclopsStereo(), triclopsSetResolution(), triclopsSetEdgeCorrelation(), 
//  triclopsSetRectify(), triclopsSetLowpass()
//
TriclopsError
triclopsPreprocess( TriclopsContext   context,
		    TriclopsInput*    input );

//=============================================================================
// Configuration
//=============================================================================
//
// Group = Configuration 

//
// Name: triclopsGetBaseline
//
// Synopsis:
//  Retrieves the baseline of the cameras.
// 
// Input:
//  context - The context.
// 
// Output:
//  base -  The baseline in meters.
//
// Returns:
//  TriclopsErrorOk             - The operation succeeded.
//  InvalidContext - The input context was invalid.	
// 
// Description:
//  This function retrieves the baseline of the cameras in meters.  The 
//  stereo context must have already been read.
//
// See Also:
//  triclopsGetDefaultContextFromFile()
//
TriclopsError
triclopsGetBaseline( TriclopsContext	  context,
		     float*		  base );

//
// Name: triclopsGetFocalLength
//
// Synopsis:
//  Retrieves the focal length of the cameras.
// 
// Input:
//  context	  - The context.
//
// Output:
//  focallength	  - The focal length in pixels.
//  
// Returns:
//  TriclopsErrorOk - The operation succeeded.
//  InvalidContext - The input context was invalid.	
//
// Description:
//  This function returns the focal length of the system. The focal length 
//  is in 'pixels' for the current selected output resolution.  All cameras' 
//  rectified images have the same focal length.  The default stereo context 
//  must have been read before this call can be made.
//
// See Also:
//  triclopsGetDefaultContextFromFile(), triclopsSetResolution()
//
TriclopsError
triclopsGetFocalLength( const TriclopsContext   context,
			float*		   	focallength );

//
// Name: triclopsGetImageCenter
//
// Synopsis:
//  Returns the optical center for pinhole calculations.
//
// Input:
//  context - TriclopsContext for the operation.
// 
// Output:
//  centerRow - A pointer that will contain the row position 
//              of the image center for the current resolution.
//  centerCol - A pointer that will contain the column position
//              of the image center for the current resolution.	
//
// Returns:
//  TriclopsErrorOk                          - Operation successful.
//  TriclopsErrorInvalidContext - Context is not a valid TriclopsContext.	
//
// Description:
//  It is important that the context already has the resolution set.  
//  If triclopsSetResolution is not set, the returned value cannot be 
//  used for calculations.  This image center can be used as the position 
//  in the image plane of the optical center for pinhole camera calculations.
//
// See Also:
//  triclopsSetResolution()
//
TriclopsError
triclopsGetImageCenter( TriclopsContext	context,
			float*	     	centerRow, 
			float*	     	centerCol );


#ifdef __cplusplus
}
#endif

#endif  // #ifndef TRICLOPSRECTIFY_H
