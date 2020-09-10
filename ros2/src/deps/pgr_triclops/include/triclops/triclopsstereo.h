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
// $Id: triclopsstereo.h,v 2.3 2010-06-21 18:46:08 arturp Exp $
//=============================================================================
#ifndef TRICLOPSSTEREO_H
#define TRICLOPSSTEREO_H

//=============================================================================
//
// This file defines the API for Stereo related functions in the 
// Triclops Stereo Vision SDK.
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
// Name: TriclopsStereoQuality
//
// Description:
//    This enumerated type identifies stereo algorithm used
// to compute the disparity images. In general, the higher quality of algorithm
// chosen, the more processing time required to compute the result.
//
typedef enum TriclopsStereoQuality
{
   TriStereoQlty_STANDARD,
   TriStereoQlty_ENHANCED      
} TriclopsStereoQuality;

//
// Name: TriclopsROI
// 
// Description:
//  This structure describes a Region Of Interest for selective processing 
//  of the input images.  
//
typedef struct TriclopsROI
{
   // The row value for the upper-left corner of the region of interest. 
   int   row;   
   // The column value for the upper-left corner of the region of interest. 
   int   col;
   // The height of the region of interest. 
   int   nrows;
   // The width of the region of interest. 
   int   ncols;

} TriclopsROI;

//=============================================================================
// Function Prototypes  
//=============================================================================

//=============================================================================
// Stereo
//=============================================================================
//
// Group = Stereo

//
// Name: triclopsGetROIs
//
// Synopsis:
//  Retrieves a handle to an array of Regions Of Interest.
//
// Input:
//  context - The context.
//
// Output:
//  rois    - A pointer to a 1D array of ROI structures.
//  maxROIs - The maximum number of ROIs allowed.	
//
// Returns:
//  TriclopsErrorOk             - The operation succeeded.
//  InvalidContext - The input context was invalid.	
//
// Description:
//  This function returns a pointer to an array of region of interest (ROI) 
//  structures.  The user may set requested region of interest boundaries in 
//  this array.  The Regions Of Interest that will be valid must begin with 
//  the 0th element in the array and continue in a contiguous manner.  After 
//  having set the Regions Of Interest, the user must call 
//  triclopsSetNumberOfROIs() to indicate to the stereo kernel how many ROIs 
//  he/she wishes processed.  Currently, ROIs are only applied during the 
//  stereo processing, not during preprocessing.
//
// See Also:
//  triclopsSetNumberOfROIs(), triclopsStereo(), TriclopsROI
//
TriclopsError
triclopsGetROIs( TriclopsContext  context,
		 TriclopsROI**	  rois,
		 int*		  maxROIs );

//
// Name: triclopsSetNumberOfROIs
//
// Synopsis:
//  Sets the number of Regions Of Interest the user currently wants active.
//
// Input:
//  context - The context.
//  nrois   - Number of ROIs.
//
// Returns:
//  TriclopsErrorOk             - The operation succeeded.
//  InvalidContext - The input context was invalid.
//  InvalidSetting - Too many ROIs.	
//
// Description:
//  This function indicates to the stereo kernel how many ROIs in the ROI array 
//  will be processed.  Before calling this function, the user must first call 
//  triclopsGetROIs() and set up the ROIs he/she intends to use.  If nrois is
//  set to 0, the entire image will be processed.  This is the default setting.
//
// See Also:
//  triclopsGetROIs()
//
TriclopsError
triclopsSetNumberOfROIs( TriclopsContext context, 
			 int		 nrois );


//
// Name: triclopsGetMaxThreadCount
//
// Synopsis:
//  Retrieves the maximum number of threads used in multi-threaded calls
//
// Input:
//  context - The context.
//
// Output:
//  rois    - A pointer to a 1D array of ROI structures.
//  maxROIs - The maximum number of ROIs allowed.	
//
// Returns:
//  TriclopsErrorOk - The operation succeeded.
//  InvalidContext - The input context was invalid.	
//
// Description:
//  This function returns the maximum number of threads allowed in multuthreaded
//  operations.
//
// See Also:
//  triclopsSetMaxThreadCount(), triclopsStereo()
//
TriclopsError
triclopsGetMaxThreadCount( TriclopsContext  context, int* maxThreadCount );

//
// Name: triclopsSetMaxThreadCount
//
// Synopsis:
//  Sets the maximum number of threads to use in multi-threaded operations
//
// Input:
//  context - The context.
//  nrois   - maximum number of threads (minimum 1)
//
// Returns:
//  TriclopsErrorOk - The operation succeeded.
//  InvalidContext - The input context was invalid.
//  InvalidSetting - Invalid number passed in (less then 1).	
//  InvalidRequest - the kernel is busy executing in the background
//
// Description:
//  This function indicates to the stereo kernel at what number to cap the number
//  of threads used in multi-threaded operations. If not set, the kernel will use
//  a default number based on the architecture of the machine and the expected 
//  sppedup.
//
// See Also:
//  triclopsGetMaxThreadCount(), triclopsStereo()
//
TriclopsError
triclopsSetMaxThreadCount( TriclopsContext context, int maxThreadCount );

//
// Name: triclopsSetEdgeCorrelation
//
// Synopsis:
//  Turns edge based correlation on or off.
//
// Input:
//  context - The context.
//  on      - A Boolean value indicating whether correlation should be turned on
//            or off.	
//
// Returns:
//  TriclopsErrorOk             - The operation succeeded.
//  InvalidContext - The input context was invalid.	
//
// Description:
//  Edge based correlation is required for texture and uniqueness 
//  validation to be used.
//
// See Also:
//  triclopsGetEdgeCorrelation(), triclopsSetTextureValidation(), 
//  triclopsSetUniquenessValidation()
//
TriclopsError
triclopsSetEdgeCorrelation( TriclopsContext   context,
			    TriclopsBool      on );

//
// Name: triclopsGetEdgeCorrelation
//
// Synopsis:
//  Retrieves the state of the edge based correlation flag.
// 
// Input:
//  context - The context.
//  
// Output: 	
//  on - A pointer to a Boolean that will contain the current setting.
//
// Returns:
//  TriclopsErrorOk             - The operation succeeded.
//  InvalidContext - The input context was invalid.	
//
// See Also:
//  triclopsSetEdgeCorrelation()
//
TriclopsError
triclopsGetEdgeCorrelation( const TriclopsContext	context,
			    TriclopsBool*		on );

//
// Name: triclopsSetEdgeMask
//
// Synopsis:
//  Sets the edge detection mask size.
// 
// Input:
//  context  - The context.
//  masksize - The new mask size, which is valid between 3 and 11.
//
// Returns:
//  TriclopsErrorOk             - The operation succeeded.
//  InvalidContext - The input context was invalid.	
//
// See Also:
//  triclopsGetEdgeMask()
// 
TriclopsError
triclopsSetEdgeMask( TriclopsContext	  context,
		     int		  masksize );

//
// Name: triclopsGetEdgeMask
//
// Synopsis:
//  Retrieves the edge detection mask size.
//
// Input:
//  context - The context.
//
// Output:
//  masksize - A pointer to an integer that will contain the current mask size.
//
// Returns:
//  TriclopsErrorOk             - The operation succeeded.
//  InvalidContext - The input context was invalid.
//
// See Also:
//  triclopsSetEdgeMask()
//
TriclopsError
triclopsGetEdgeMask( const TriclopsContext	context,
		     int*			masksize );

//
// Name: triclopsSetDoStereo
//
// Synopsis:
//  Turns stereo processing on or off.
//
// Input:
//  context - The context.
//  on      - A Boolean indicating whether stereo processing should be turned on
//            or off.	
//
// Returns:
//  TriclopsErrorOk             - The operation succeeded.
//  InvalidContext - The input context was invalid.	
//
// See Also:
//  triclopsGetDoStereo(), triclopsSetStereoQuality(), triclopsGetStereoQuality()
//
TriclopsError
triclopsSetDoStereo( TriclopsContext	context,
		     TriclopsBool	on );

//
// Name: triclopsGetDoStereo
//
// Synopsis:
//  Retrieves the state of the stereo processing.
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
//  triclopsSetDoStereo(), triclopsSetStereoQuality(), triclopsGetStereoQuality()
//
TriclopsError
triclopsGetDoStereo( const TriclopsContext	context,
		     TriclopsBool*		on );

//
// Name: triclopsSetStereoQuality
//
// Description:
//    Sets the quality of the stereo algorithm to use during stereo processing.
// Higher-quality algorithms will generally require more processing time, but
// give more accurate and/or more complete results. Currently available are 
// TriStereoQlty_STANDARD, our classic stereo algorithm, and 
// TriStereoQlty_ENHANCED, which provides a more reliable sub-pixel disparity 
// estimate.
//
// Input:
//    context	  - The current TriclopsContext.
//    quality	  - The desired quality of the stereo algorithm to use
//
// Returns:
//    TriclopsErrorOk		    - The operation succeeded.
//    TriclopsErrorInvalidContext   - If the context is invalid.
//
// See Also:
//    triclopsGetStereoAlgQuality()
//
TriclopsError
triclopsSetStereoQuality(TriclopsContext context, TriclopsStereoQuality quality);

//
// Name: triclopsGetStereoQuality
//
// Description:
//    Gets the quality of the stereo algorithm currently in use for stereo 
// processing.
//
// Input:
//    context	  - The current TriclopsContext.
//    quality	  - The quality of the stereo algorithm currently in use.
//
// Returns:
//    TriclopsErrorOk		    - The operation succeeded.
//    TriclopsErrorInvalidContext   - If the context is invalid.
//
// See Also:
//    triclopsSetStereoAlgQuality()
//
TriclopsError
triclopsGetStereoQuality(TriclopsContext context, TriclopsStereoQuality* quality);

//
// Name: triclopsSetSubpixelInterpolation
//
// Synopsis:
//  Turns subpixel interpolation stereo improvements on or off.
//
// Input:
//  context - The context.
//  on      - A Boolean indicating whether it should be on or off.	
//
// Returns:
//  TriclopsErrorOk             - The operation succeeded.
//  InvalidContext - The input context was invalid.	
//
// See Also:
//  triclopsGetSubpixelInterpolation(), triclopsSetStrictSubpixelValidation()
//
TriclopsError
triclopsSetSubpixelInterpolation( TriclopsContext	context,
				  TriclopsBool	 	on );

//
// Name: triclopsGetSubpixelInterpolation
//
// Synopsis:
//  Retrieves the state of the subpixel interpolation feature.
//
// Input:
//  context - The context.
//
// Output:
//  on - A pointer to a Boolean that will store the current setting.
//
// Returns:
//  TriclopsErrorOk             - The operation succeeded.
//  InvalidContext - The specified context was invalid.	
//
// See Also
//  triclopsSetSubpixelInterpolation(), triclopsSetStrictSubpixelValidation()
//
TriclopsError
triclopsGetSubpixelInterpolation( const TriclopsContext	  context,
				  TriclopsBool*		  on );

//
// Name: triclopsSetDisparity
//
// Synopsis:
//  Sets the disparity range for stereo processing. Currently, the range
// must be such that (maxDisparity-minDisparity<=240) and (maxDisparity<=1024). 
// Note that a side-effect of setting maxDisparity > 240 is that the disparity
// offset is set to maxDisparity-240.
//
// Input:
//  context      - The context.
//  minDisparity - The disparity range minimum.
//  maxDisparity - The disparity range maximum.	
//
// Returns:
//  TriclopsErrorOk             - The operation succeeded.
//  InvalidContext - The input context was invalid.	
//
// See Also:
//  triclopsGetDisparityOffset.
//
TriclopsError
triclopsSetDisparity( TriclopsContext   context, 
		      int		minDisparity, 
		      int		maxDisparity );

//
// Name: triclopsGetDisparity
//
// Synopsis:
//  Retrieves the disparity range from the given context.
//
// Input:
//  context - The context
//
// Output:
//  minDisparity - A pointer to an integer that will store the current value.
//  maxDisparity - A pointer to an integer that will store the current value.	
//
// Returns:
//  TriclopsErrorOk             - The operation succeeded.
//  InvalidContext - The input context was invalid.	
//
// See Also:
//  triclopsGetDisparityOffset.
//
TriclopsError
triclopsGetDisparity( const TriclopsContext	context, 
		      int*			minDisparity, 
		      int*			maxDisparity );

//
// Name: triclopsGetDisparityOffset
//
// Synopsis:
//  Retrieves the disparity offset from the given context. Adding the disparity
//  offset to a valid disparity value from the disparity image gives the 
//  true disparity value. The disparity offset is set automatically when the
//  disparity range is set using triclopsSetDisparity, and is equal to
//  MAX( 0, maxDisparity-240). The disparity offset allows the disparity range, 
//  (which has a maximum extent of 240) to be shifted up away from zero, so 
//  that objects very close to the camera may be imaged.
//
// Input:
//  context - The context
//
// Output:
//  nDisparityOffset - A pointer to an integer that will store the current value.
//
// Returns:
//  TriclopsErrorOk - The operation succeeded.
//  InvalidContext  - The input context was invalid.	
//
TriclopsError
triclopsGetDisparityOffset( const TriclopsContext	context, 
			    int*			nDisparityOffset );

//
// Name: triclopsSetDisparityMappingOn
//
// Synopsis:
//  Enables or disables disparity mapping.
// 
// Input:
//  context - TriclopsContext for the operation.		
//  on      - A Boolean flag indicating if the mapping should be on or off.		
//
// Returns:
//  TriclopsErrorOk                          - Operation successful.
//  TriclopsErrorInvalidContext	- Context is not a valid TriclopsContext. 
//
// Description:
//  This function is used to enable or disable disparity mapping.  See the 
//  comments section on this subject in Chapter 7 in the Triclops manual for 
//  why disparity mapping can cause trouble.
//
//
TriclopsError
triclopsSetDisparityMappingOn( TriclopsContext 	context, 
			       TriclopsBool	on );

//
// Name: triclopsGetDisparityMappingOn
//
// Synopsis:
//  Retrieves the current setting.
//
// Input:
//  context - TriclopsContext for the operation.
//
// Output:
//  on - A pointer that will contain the current value of this flag.
//
// Returns:
//  TriclopsErrorOk                          - Operation successful. 
//  TriclopsErrorInvalidContext - Context is not a valid TriclopsContext.
//
// Description:
//  This function is used to extract the current disparity mapping setting.
//
// See Also:
//  triclopsSetDisparityMappingOn.
//
TriclopsError
triclopsGetDisparityMappingOn( TriclopsContext 	context, 
			       TriclopsBool*	on );

//
// Name: triclopsSetDisparityMapping
//
// Synopsis:
//  Sets the disparity range for stereo processing.
//
// Input:
//  context      - The context.
//  minDisparity - The disparity range in the output disparity image minimum.
//                 The range is between 0 and 255.
//  maxDisparity - The disparity range in the output disparity image maximum.
//                 The range is between 0 and 255.
//
// Returns:
//  TriclopsErrorOk             - The operation succeeded.
//  InvalidContext - The input context was invalid.	
// 
// Description:
//  This function sets the disparity mapping values.  The disparity mapping 
//  values control what range of pixels values appear in the output disparity 
//  image.  The true disparity ranges between the minimum and maximum values 
//  set with triclopsSetDisparity().  The output image has its pixel values 
//  linearly scaled from minimum => maximum disparity  to minimum => maximum 
//  disparity mapping.  This is primarily used when one wishes to use a screen 
//  display buffer as the disparity image buffer.  Note:  it is advisable to 
//  set the disparity mapping to the exact values of the input disparities if 
//  one is not using the screen buffer display feature.
//
// See Also:
//  triclopsGetDisparityMapping(), triclopsSetDisparity(), triclopsRCD8ToXYZ(), 
//  triclopsRCD16ToXYZ(), triclopsRCDMappedToXYZ()
//
TriclopsError
triclopsSetDisparityMapping( TriclopsContext   context, 
			     unsigned char     minDisparity, 
			     unsigned char     maxDisparity );

//
// Name: triclopsGetDisparityMapping
//
// Synopsis:
//  Retrieves the disparity range from the given context.
//
// Input:
//  context - The context.
// 
// Output:	
//  minDisparity - The disparity range in the output disparity image minimum.
//  maxDisparity - The disparity range in the output disparity image maximum.
//
// Returns:
//  TriclopsErrorOk             - The operation succeeded.
//  InvalidContext - The input context was invalid.	
//
// See Also:
//  triclopsSetDisparityMapping(), triclopsSetDisparity()
//
TriclopsError
triclopsGetDisparityMapping( const TriclopsContext	context, 
			     unsigned char*		minDisparity, 
			     unsigned char*		maxDisparity );

//
// Name: triclopsSetStereoMask
//
// Synopsis:
//  Set the stereo correlation mask size.
//
// Input:
//  context  - The context.
//  masksize - The new correlation mask size, which is valid between 1 and 15.
//
// Returns:
//  TriclopsErrorOk             - The operation succeeded.
//  InvalidContext - The input context was invalid.	
//  
TriclopsError
triclopsSetStereoMask( TriclopsContext	context,
		       int		masksize );

//
// Name: triclopsSetAnyStereoMask
//
// Synopsis:
//  Allows the user to set stereomask to any size. 
//
// Input:
//  context - TriclopsContext for the operation.		
//  size    - The size for a new stereo correlation mask.			
//
// Returns:
//  TriclopsErrorOk                          - Operation successful.
//  TriclopsErrorInvalidContext - Context is not a valid TriclopsContext.
//  TriclopsErrorInvalidSetting - A value of less than 1	
//
// Description:
//  This function allows you to set a stereo correlation mask of any size.  
//  There is a danger that an internal counter in the stereo correlation 
//  engine may overflow if mask sizes over 15 are provided.  Thus, the 
//  current limit for triclopsSetStereoMask is 15.  However, in practice, 
//  much larger mask sizes can be used successfully.  If an overflow results, 
//  it may not affect the stereo result, and it will generally only happen for 
//  pathological cases.  Therefore, this function is provided as an 
//  'experimental' function to allow users to set any mask size they wish.
//
// See Also:
//  triclopsSetStereoMask()
//
TriclopsError
triclopsSetAnyStereoMask( TriclopsContext	context,
			  int			size );

//
// Name: triclopsGetStereoMask
//
// Synopsis: 
//  Retrieves the stereo correlation mask size.
// 
// Input:
//  context - The context.
// 
// Output: 
//  size - A pointer to an integer that will store the current value.
//
// Returns:
//  TriclopsErrorOk             - The operation succeeded.
//  InvalidContext - The input context was invalid.	
//
TriclopsError
triclopsGetStereoMask( const TriclopsContext	context,
		       int*			size );

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
//  TriclopsErrorOk             - The operation succeeded.
//  BadOptions     - There are some contradictory options set.
//  NotImplemented - The context camera configuration is not supported.
//  InvalidROI     - A negative dimensioned ROI, or ROI with illegal starting 
//                   positions was inputted.  Illegal starting position is a
//		     upper/left corner that is closer to the right/bottom edge
//		     of the image than the stereo mask size.	
//  InvalidRequest - triclopsStereo called before triclopsPreprocess ever called.
//
// Description:
//  This function performs the stereo processing and validation, and generates 
//  the internal image TriImg_DISPARITY.
//
// See Also:
//  triclopsPreprocessing(), triclopsSetDoStereo(), triclopsEdgeCorrelation(), 
//  triclopsSetTextureValidation(), triclopsSetUniquenessValidation(), 
//  triclopsGetROIs(), triclopsSetSubpixelInterpolation()
//
TriclopsError
triclopsStereo( TriclopsContext context );

#ifdef __cplusplus
}
#endif

#endif  // #ifndef TRICLOPS_H
