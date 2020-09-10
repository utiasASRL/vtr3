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
// $Id: triclops.h,v 2.96 2010-07-12 19:09:52 arturp Exp $
//=============================================================================
#ifndef TRICLOPS_H
#define TRICLOPS_H

//=============================================================================
//
// This file defines the the API for the Triclops Stereo Vision SDK.
//
//=============================================================================


//=============================================================================
// Defines
//=============================================================================

//
// The version of the library.
//
#define TRICLOPS_VERSION 3402

//=============================================================================
// System Includes  
//=============================================================================

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
//
// Group = Enumerations

//
// Name: TriclopsError
//
// Description:
//    All Triclops functions return an error value that indicates whether
//    an error occurred, and if so what error.  The following enumerated
//    type lists the kinds of errors that may be returned.
//
typedef enum TriclopsError
{
   // Function succeeded.
   TriclopsErrorOk = 0,
   // User requested an option that is not yet implemented. 
   TriclopsErrorNotImplemented,
   // User specified input to the function that was not a valid value for 
   // this function.
   TriclopsErrorInvalidSetting,
   // The TriclopsContext passed in was corrupt or invalid.
   TriclopsErrorInvalidContext,
   // The specified camera is not valid for this camera configuration.
   TriclopsErrorInvalidCamera,
   // An impossible Region Of Interest was specified.  
   // For example, a region of interest with negative height or width. 
   TriclopsErrorInvalidROI,
   // Given the current specified options, the request is not valid. 
   // For example, requesting the disparity image from a context that has 
   // not executed 'triclopsStereo()'. 
   TriclopsErrorInvalidRequest,
   // Some options are illegal or contradictory. 
   TriclopsErrorBadOptions,
   // The configuration file is corrupted, missing mandatory fields,
   // or is for the wrong Triclops version. 
   TriclopsErrorCorruptConfigFile,
   // Can not find the configuration file. 
   TriclopsErrorNoConfigFile,
   // An indeterminable error occurred. 
   TriclopsErrorUnknown,
   // A function that requires MMX was called on a non-MMX enabled CPU. 
   TriclopsErrorNonMMXCpu,
   // An invalid parameter has been passed.
   TriclopsErrorInvalidParameter,
   // A call to TriclopsSetSurfaceValidation has caused an overflow. 
   TriclopsErrorSurfaceValidationOverflow,
   // An error has occurred during a system call.  Check 'errno' to 
   // determine the reason.
   // This includes the system running out of memory.
   TriclopsErrorCorruptTransformFile,
   // Can not find the transform file or its contents is invalid
   TriclopsErrorSystemError,
   // Error reading in a file.
   TriclopsErrorFileRead,
   // The PGR comment in a PGM header was corrupted
   TriclopsErrorCorruptPGRComment,
   // This Triclops functionality has been deprecated
   TriclopsErrorDeprecated

} TriclopsError;

//
// Name: TriclopsImageType
// 
// Description:
//  The enumerated type TriclopsImageType indicates what kind of image 
//  is either being requested or returned. It also indicates the size 
//  of each pixel in the image.
//
typedef enum TriclopsImageType
{
   // Disparity image: This is the resultant depth image after stereo 
   // processing.
   TriImg_DISPARITY = 0,
   // Raw unrectified image: This is an image with the aspect ratio 
   // that was supplied by the input.  There is no correction for lens 
   // distortion or camera misalignment. 
   TriImg_RAW,
   // Rectified image: This is an image that has been corrected for lens 
   // distortion and camera misalignment to fit a pinhole camera model.
   TriImg_RECTIFIED,
   // Edge image: A Bandpass filter has been applied to this image.  
   // This image has values that range about 128.   A value of 128 
   // indicates no edge.  Values greater and less than 128 indicate 
   // an edge with strength relative to the difference between 128 and 
   // the pixel value. 
   TriImg_EDGE,

} TriclopsImageType;

//
// Name: TriclopsImage16Type 
//
// Description:
//  This enumerated type defines the various 16bit image types.
//
typedef enum TriclopsImage16Type
{
   // Disparity Image:  This is the 16-bit resultant depth image after
   // stereo processing with subpixel on.
   TriImg16_DISPARITY = 0

} TriclopsImage16Type;


//
// Name: TriclopsCamera
// 
// Description:
//  This enumerated type identifies individual cameras given a specific
//  camera configuration.
//
// Remarks: 
//  TriCam_COLOR is only for use with the Color Triclops.  It is not
//  used for any Bumblebee product (TriCam_COLOR may be phased 
//  out in future releases of Triclops SDK).
//
typedef enum TriclopsCamera
{
   TriCam_REFERENCE,
   TriCam_RIGHT,
   TriCam_TOP,
   TriCam_LEFT,

   // These are kept here as legacy code for now... should be replaced in user
   // code to not include the configuration specific "_L_"
   // These values may be phased out in future releases of Triclops SDK
   TriCam_L_RIGHT      	= TriCam_RIGHT,
   TriCam_L_TOP	       	= TriCam_TOP,
   TriCam_L_LEFT       	= TriCam_LEFT,

   TriCam_COLOR,
   TriCam_L_COLOR	= TriCam_COLOR

} TriclopsCamera;

//
// Name: TriclopsInputType
//
// Description:
//  This enumerated type identifies the format of the input to the 
//  Triclops library.  This input has generally either been read from 
//  a set of files (for off line processing) or has just been delivered 
//  by a frame grabber.  There are two formats of input data that are 
//  currently being supported.  RGB format indicates that the data is 
//  supplied with a separate buffer for each R, G and B channel.  RGB 
//  packed indicates that the 3 channels are interleaved.
//
typedef enum TriclopsInputType
{   
   // This is used to mark that the input has not yet been set. 
   TriInp_NONE,
   // An array of pixels with color bands interleaved in the 
   // following order: [B G R U] [B G R U]. 
   TriInp_RGB_32BIT_PACKED,	
   // An array of separated bands with the following ordering: 
   // [R R R....][G G G...][B B B...]. 
   TriInp_RGB

} TriclopsInputType;


//=============================================================================
// Types 
//=============================================================================
//
// Group = Types 

//
// Name: TriclopsBool 
// 
// Description:
//  Definition for Boolean variables. 
//
typedef int TriclopsBool;

//
// Name: TriclopsContext
//
// Description:
//  Triclops context is a pointer to an internal structure that maintains
//  all image and bookkeeping information necessary to the Triclops library.  
//  It is the container for all parameters
//
typedef void*  TriclopsContext;

//
// Name: TriclopsTimestamp
// 
// Description:
//  This structure describes the format by which time is represented in the 
//  Triclops Stereo Vision SDK.
//
typedef struct TriclopsTimestamp
{
   // The number of seconds since the epoch. 
   long sec;
   // The number of microseconds within the second. 
   long u_sec;
   
} TriclopsTimestamp;

//
// Name: TriclopsImage
//	
// Description:
//  This structure is used both for image input and output from the
//  Triclops system.  
//
typedef struct TriclopsImage
{
   // The number of rows in the image. 
   int   nrows;
   // The number of columns in the image. 
   int   ncols;
   // This is the pitch, or row increment for the image.  Data must be
   // contiguous within each row, and the rows must be equally spaced.
   // Rowinc indicates the number of bytes between the beginning of a 
   // row and the beginning of the following row. 
   int   rowinc;
   // The area for the pixel data.  This must be numRows * numCols bytes 
   // large. 
   unsigned char*	data; 

} TriclopsImage;

//
// Name: TriclopsImage16
//
// Description: This structure is used for image output from the Triclops
//   system for image types that require 16-bits per pixel.  This is the format
//   for subpixel interpolated images.   The structure is identical to the 
//   TriclopsImage structure except that the data contains unsigned shorts 
//   rather than unsigned chars.  Rowinc is still the number of bytes between
//   the beginning of a row and the beginning of the following row 
//   (NOT number of pixels).
//
typedef struct TriclopsImage16
{
   // The number of rows in the image.
   int 		     nrows;
   // The number of columns in the image.
   int 		     ncols;
   // The number row increment of the image.
   int		     rowinc;
   // The pixel data of the image.
   unsigned short*   data;

} TriclopsImage16;

//
// Name: TriclopsColorImage
//
// Description:
//  This structure is used for image output from the Triclops system for color
//  images.  The structure is the same as TriclopsImage except that the data
//  field is replaced by three color bands; 'red', 'green' and 'blue'.  Each
//  band is a complete image for that particular color band.  In this case, 
//  rowinc is the row increment for each color band.
//
typedef struct TriclopsColorImage
{
   // The number of rows in the image.
   int 		     nrows;
   // The number of columns in the image.
   int 		     ncols;
   // The row increment of the image.
   int		     rowinc;
   // The pixel data for red band of the image.
   unsigned char*    red;
   // The pixel data for green band of the image.
   unsigned char*    green;
   // The pixel data for blue band of the image.
   unsigned char*    blue;

} TriclopsColorImage;

//
// Name: TriclopsPackedColorPixel 
//
// Description:
//  This structure defines the format for a 32bit color pixel.
//
typedef struct TriclopsPackedColorPixel
{
   // The 32 bit pixel data.
   unsigned char     value[4];

} TriclopsPackedColorPixel;

//
// Name: TriclopsPackedColorImage 
//
// Description:
//  This structure defines the format of a 32bit packed color image.
//
typedef struct TriclopsPackedColorImage
{
   // The number of rows in the image.
   int   nrows;
   // The number of columns in the image.
   int   ncols;  
   // The number of bytes in each row of the image.
   int   rowinc; 
   // A pointer to the pixel data.
   TriclopsPackedColorPixel*	 data;  

} TriclopsPackedColorImage;

//
// Name: TriclopsInputRGB32BitPacked
//
// Description:
//  This structure contains RGB packed data.  
//
typedef struct TriclopsInputRGB32BitPacked
{
   //  a pointer to an array of nrows*ncols*4 pixels. The pixels are 
   //  organized in the following fashion [RGBU][RGBU] ... 
   void*    data;

} TriclopsInputRGB32BitPacked;

//
// Name: TriclopsInputRGB
// 
// Description:
//  This structure consists of three separate buffers, 
//  each containing nrows * ncols pixels for each of the RGB bands.
typedef struct TriclopsInputRGB
{
   // The red band.
   void*    red;
   // The green band.
   void*    green;
   // The blue band.
   void*    blue;

} TriclopsInputRGB;

//
// Name: TriclopsInput 
//
// Description: 
//  TriclopsInput structure contains image input to the Triclops library. 
//  The library accepts two formats: 32-bit packed, and RGB separate buffer 
//  inputs.  Field u contains a pointer to one of the two typed structures 
//  that contain the image data.  The inputType field indicates which type of 
//  input is actually present in the structure.  
//
typedef struct TriclopsInput
{
   // The input type indicating packed or unpacked data.
   TriclopsInputType	inputType;
   // The timestamp on the image data (generated by the camera.)
   TriclopsTimestamp	timeStamp;
   // The number of rows in the input images. 
   int   nrows;
   // The number of columns in the input images. 
   int   ncols;
   // The row increment of the input images.
   int   rowinc;

   // The actual image data is either packed or unpacked and can be accessed
   // with either TriclopsinputRGB or TriclopsInputRGB32BitPacked.
   union
   {
      TriclopsInputRGB		    rgb;
      TriclopsInputRGB32BitPacked   rgb32BitPacked;
   } u;

} TriclopsInput;



//=============================================================================
// Function Prototypes  
//=============================================================================

//=============================================================================
// Debugging and Error Reporting
//=============================================================================
//
// Group = Debugging and Error Reporting

//
// Name: triclopsErrorToString
// 
// Synopsis:
//  Converts a Triclops error into a meaningful string.
// 
// Input:
//  error -	The value of a TriclopsError variable.
//
// Returns:
//  char* - A string describing the nature of the TriclopsError.
//
// Description:
//  This function returns a string that describes the given TriclopsError.  
//  This allows error reporting software to report meaningful error messages
//  to the user.
//
// See Also:
//  TriclopsError
//
char*
triclopsErrorToString( TriclopsError error );

//=============================================================================
// General
//=============================================================================
//
// Group = General 

//
// Name: triclopsVersion
//
// Synopsis:
//  Returns a string with the Triclops library version.
//
// Input:
//  
//  
// Returns:
//  char* - A string containing the version information for the context.
//
// Description:
//  This function returns internally-handled memory.  The caller should not
//  free the returned pointer.
//  
//
const char*
triclopsVersion();

//
// Name: triclopsGetVersionMajor
//
// Synopsis:
//  Returns an integer with the Triclops library major version number.
//
// Input:
//  
//  
// Returns:
//  int - An integer containing the major version number.
//
// Description:
//  <see above>
//  
//
int
triclopsGetVersionMajor();

//
// Name: triclopsGetVersionMinor
//
// Synopsis:
//  Returns an integer with the Triclops library minor version number.
//
// Input:
//  
//  
// Returns:
//  int - An integer containing the minor version number.
//
// Description:
//  <see above>
//  
//
int
triclopsGetVersionMinor();

//
// Name: triclopsGetVersionReleaseType
//
// Synopsis:
//  Returns an integer with the Triclops library release type.
//
// Input:
//  
//  
// Returns:
//  int - An integer containing the release type.
//
// Description:
//  <see above>
//  
//
int
triclopsGetVersionReleaseType();

//
// Name: triclopsGetVersionReleaseNumber
//
// Synopsis:
//  Returns an integer with the Triclops library release number.
//
// Input:
//  
//  
// Returns:
//  int - An integer containing the release number.
//
// Description:
//  <see above>
//  
//
int
triclopsGetVersionReleaseNumber();


//
// Name: triclopsGetDynamicLibPath
//
// Synopsis:
//  Returns a string with the filesystem path to 'this' Triclops library.
//
// Input:
//  
//  
// Returns:
//  char* - A string containing the filesystem path where 'this' lib is located.
//
// Description:
//  This function returns internally-handled memory.  The caller should not
//  free the returned pointer.
//  
//
const char*
triclopsGetDynamicLibPath();


//
// Name: triclopsGetImage
//
// Synopsis:
//  Retrieves a specified type of image associated with the specified camera.
//
// Input:
//  context   - The context.
//  imageType - The image type requested.
//  camera    - The camera that generated the requested image.	
//
// Output:
//  image - A TriclopsImage with the image data.
//
// Returns:
//  TriclopsErrorOk  - The operation succeeded.
//  InvalidContext - The input context was invalid.
//  InvalidCamera  - The camera does not match a camera in this configuration.
//  InvalidRequest - The image type does not match the camera or is not being
//                   generated by this system with the current context options.	
// 
// Description:
//  This function supplies the data to a TriclopsImage.  The user is 
//  responsible for allocating the TriclopsImage structure.  The data 
//  value of the structure will point to memory within the stereo kernel 
//  working space.  Therefore, if a permanent copy of this image is desired, 
//  the user must copy it out.  
//
// See Also:
//  triclopsSetImageBuffer()
//
TriclopsError
triclopsGetImage( const TriclopsContext   context,
		  TriclopsImageType	  imageType,
		  TriclopsCamera	  camera,
		  TriclopsImage*	  image );

//
// Name: triclopsBuildRGBTriclopsInput
//
// Synopsis:
//  Uses the parameters passed in to build up a Triclops Input
//
// Input:
//  iCols	     - The number of columns in the image
//  iRows	     - The number of rows in the image
//  iRowInc	     - The number of bytes in each row of the image
//  ulSeconds	     - The second value of the image timestamp
//  ulMicroSeconds   - The microsecond value of the image timestamp
//  pDataR	     - A buffer to hold the R data
//  pDataG	     - A buffer to hold the G data
//  pDataB	     - A buffer to hold the B data
//
// Output:
//  triclopsInput - A TriclopsInput created by the image data passed in.
//
// Returns:
//  TriclopsErrorOk  - The operation succeeded.
//  TriclopsErrorFailed - The operation failed.
// 
// Description:
//  This function constructs a TriclopsInput struct from given parameters.
// 
TriclopsError
triclopsBuildRGBTriclopsInput (int iCols, 
	   int iRows, 
	   int iRowInc, 
	   unsigned long ulSeconds, 
	   unsigned long ulMicroSeconds, 
	   unsigned char* pDataR, 
	   unsigned char* pDataG, 
	   unsigned char* pDataB, 
	   TriclopsInput* triclopsInput);

//
// Name: triclopsBuildPackedTriclopsInput
//
// Synopsis:
//  Uses the parameters passed in to build up a Triclops Input
//
// Input:
//  iCols	     - The number of columns in the image
//  iRows	     - The number of rows in the image
//  iRowInc	     - The number of bytes in each row of the image
//  ulSeconds	     - The second value of the image timestamp
//  ulMicroSeconds   - The microsecond value of the image timestamp 
//  pDataPacked	     - An array to hold the packed data
//
// Output:
//  triclopsInput    - A TriclopsInput created by the image data passed in.
//
// Returns:
//  TriclopsErrorOk  - The operation succeeded.
//  TriclopsErrorFailed - The operation failed.
//
// Description:
//  This function constructs a TriclopsInput struct from given parameters.
// 
TriclopsError
triclopsBuildPackedTriclopsInput (int iCols, 
	   int iRows, 
	   int iRowInc, 
	   unsigned long ulSeconds, 
	   unsigned long ulMicroSeconds, 
	   unsigned char* pDataPacked, 
	   TriclopsInput* triclopsInput);

//
// Name: triclopsSaveImage
//
// Synopsis:
//  Saves an image to the specified filename.  The file format currently 
//  supported is PGM format.
//
// Input:
// image    - The TriclopsImage to be saved.
// filename - The file name in which to save the image.	
// 
// Returns:
//  SystemError     - The file could not be opened
//  TriclopsErrorOk - The operation succeeded.
//
// Description:
//  This function saves the input image to the requested file.  Currently, this
//  function will not detect if the file could not be opened, and will always 
//  show successful return.
//
TriclopsError
triclopsSaveImage( TriclopsImage* image, 
		   char*	  filename );


//
// Name: triclopsGetImage16
//
// Synopsis:
//  Retrieves a specified 16-bit image associated with the specified camera. 
//
// Input:
//  context   - The context.
//  imageType - The image type requested.
//  camera    - The camera that generated the requested image.	
//
// Output:
//  image - A TriclospImage16 with the image data.
//
// Returns:
//  TriclopsErrorOk             - The operation succeeded.
//  InvalidContext - The input context was invalid.
//  InvalidCamera  - The camera does not match a camera in this configuration.
//  InvalidRequest - The image type does not match the camera or is not being 
//                   generated by this system with the current context options.	
//
// Description:
//  This function performs the same action as triclopsGetImage(), except that 
//  it retrieves a 16-bit image type.  Currently the only supported 16-bit 
//  image is the resultant disparity image from subpixel interpolation.  
//
// See Also:
//  triclopsGetImage()
//
TriclopsError
triclopsGetImage16( const TriclopsContext	context,
		    TriclopsImage16Type		imageType,
		    TriclopsCamera		camera,
		    TriclopsImage16*		image );


//
// Name: triclopsSaveImage16
//
// Synopsis:
//  Saves an image to the specified filename.  The file format currently 
//  supported is PGM format.
//
// Input:
//  image    - The TriclopsImage16 to be saved.
//  filename - The file name in which to save the image.
//
// Returns:
//  SystemError     - The file could not be opened
//  TriclopsErrorOk - The operation succeeded.
//
// Description:
//  This function saves the input image to the requested file. Currently,
//  this function will not detect if the file could not be opened, and will 
//  always return successful.
//
// See Also:
//  triclopsSaveImage()
//
TriclopsError
triclopsSaveImage16( TriclopsImage16*  	image, 
		     char*		filename );



//=============================================================================
// Rectification
//=============================================================================
//
// Group = Rectification

//
// Name: triclopsRectify
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
//  TriclopsErrorOk   - The operation succeeded.
//  InvalidContext    - The input context was invalid.
//  NotImplemented    - TriclopsInput has rowinc < ncols*pixelsize
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
triclopsRectify( TriclopsContext   context,
		 TriclopsInput*    input );


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
//  TriclopsErrorOk  - The operation succeeded.
//  InvalidContext   - The input context was invalid.
//  NotImplemented   - TriclopsInput has rowinc < ncols*pixelsize
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
// Remarks:
//  This function has been renamed as "triclopsRectify()", which is a
//  more descriptive name.  "triclopsPreprocess()" will be deprecated in
//  the future.
//
// See Also:
//  triclopsStereo(), triclopsSetResolution(), triclopsSetEdgeCorrelation(), 
//  triclopsSetRectify(), triclopsSetLowpass()
//
TriclopsError
triclopsPreprocess( TriclopsContext   context,
		    TriclopsInput*    input );


//=============================================================================
// Stereo
//=============================================================================
//
// Group = Stereo 


//
// Name: triclopsStereo
//
// Synopsis:
//  Does stereo processing, validation, and subpixel interpolation, as 
//  specified by parameters.
//
// Input:
//  context - The context.
//
// Returns:
//  TriclopsErrorOk  - The operation succeeded.
//  BadOptions	     - There are some contradictory options set.
//  NotImplemented   - The context camera configuration is not supported.
//  InvalidROI	     - ROI has negative dimensions or illegal starting position.
//  InvalidRequest   - triclopsStereo called before triclopsPreprocess was
//		       called.
//
// Description:
//  This function performs the stereo processing and validation, and generates 
//  the internal image TriImg_DISPARITY.
//
// Remarks:
//  An illegal starting position is a upper/left corner that is closer to the 
//  right/bottom edge of the image than the stereo mask size.	
//
// See Also:
//  triclopsPreprocessing(), triclopsSetDoStereo(), triclopsEdgeCorrelation(), 
//  triclopsSetTextureValidation(), triclopsSetUniquenessValidation(), 
//  triclopsGetROIs(), triclopsSetSubpixelInterpolation()
//
TriclopsError
triclopsStereo( TriclopsContext context );

#include <triclops3d.h>
#include <triclopsbuffer.h>
#include <triclopscontext.h>
#include <triclopsvalidation.h>
#include <triclopsrectify.h>
#include <triclopsstereo.h>
#include <triclopsimageio.h>



#ifdef __cplusplus
}
#endif

#endif  // #ifndef TRICLOPS_H
