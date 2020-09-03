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
// $Id: triclopsvalidation.h,v 2.2 2007-11-30 00:43:03 soowei Exp $
//=============================================================================
#ifndef TRICLOPSVALIDATION_H
#define TRICLOPSVALIDATION_H

//=============================================================================
//
// This file defines the API for Stereo Validation methods in the 
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

//=============================================================================
// Function Prototypes  
//=============================================================================

//=============================================================================
// Validation Support
//=============================================================================
//
// Group = Validation Support 

//
// Name: triclopsSetTextureValidation
//
// Synopsis:
//  Turns texture validation on or off.
//
// Input:
//  context - The context.
//  on      - Texture validation flag.
//
// Returns:
//  TriclopsErrorOk             - The operation succeeded.
//  InvalidContext - The input context was invalid.
//
// Description:
//  Sets the context texture validation flag.  When set to true, 
//  texture-based validation is enabled.  Pixels that do not pass 
//  the validation test will be marked with the texture validation 
//  mapping value in the disparity image.
//
// See Also:
//  triclopsGetTextureValidation()
//
TriclopsError
triclopsSetTextureValidation( const TriclopsContext	 context,
			      TriclopsBool		 on );

//
//  Name: triclopsGetTextureValidation
//
// Synopsis:
//  Retrieves the state of the texture validation flag.
//
// Input:
//  context - The context.
//
// Output:
//  on - Container that receives the texture validation flag value.
//
// Returns:
//  TriclopsErrorOk             - The operation succeeded.
//  InvalidContext - The input context was invalid.
//
// See Also:
//  TriclopsSetTextureValidation()
//
TriclopsError
triclopsGetTextureValidation( const TriclopsContext	 context,
			      TriclopsBool*		 on );

//
// Name: triclopsSetTextureValidationThreshold
//
// Synopsis:
//  Sets the texture validation threshold.
//
// Input:
//  context - The context
//  value   - The new texture validation threshold.
//
// Returns:
//  TriclopsErrorOk             - The operation succeeded.
//  InvalidContext - The input context was invalid.
//
// Description:
//  Sets the texture validation threshold.  This threshold allows one
//  to tune the texture-based rejection of pixels.  Values range from 
//  0.0 (no rejection) to 128.0 (complete rejection) but good operating 
//  range is between 0.0 and 2.0.
//
// See Also:
//  TriclopsGetTextureValidationThreshold()
//
TriclopsError
triclopsSetTextureValidationThreshold( TriclopsContext   context,
				       float		 value );

//
// Name: triclopsGetTextureValidationThreshold
//
// Synopsis:
//  Retrieves the texture validation threshold.
//
// Input:
//  context - The context.
// 
// Output:	
//  value - The location for the retrieved threshold.
//
// Returns:
//  TriclopsErrorOk             - The operation succeeded.
//  InvalidContext - The input context was invalid.
//
// See Also:
//  TriclopsSetTextureValidationThreshold()
//
TriclopsError
triclopsGetTextureValidationThreshold( const TriclopsContext	context,
				       float*		  	value );

//
// Name: triclopsSetTextureValidationMapping 
//
// Description:
//  Sets the value that appears in the disparity image for pixels that
//  fail texture validation.
//
// Input:
//  context - The TriclopsContext to set the texture validation mapping for.
//  value   - The new value to map invalid pixels to.  The range is from
//            0 to 255.
//
// Returns:
//  TriclopsErrorOk - Upon the successful completion of the operation.
//
TriclopsError
triclopsSetTextureValidationMapping( TriclopsContext	context,
				     unsigned char	value );

//
// Name: triclopsGetTextureValidationMapping
//
// Description:
//  Gets the value that appears in the disparity image for pixels 
//  that fail texture validation.
//
// Input:
//  context - The TriclopsContext to get the texture validation mapping from.
//
// Output:
//  value - The current texture validation mapping setting.
//
// Returns:
//  TriclopsErrorOk - Upon the successful completion of the operation.
//
TriclopsError
triclopsGetTextureValidationMapping( const TriclopsContext	context,
				     unsigned char*	  	value );

//
// Name: triclopsSetUniquenessValidation
//
// Synopsis:
//  Turns uniqueness validation on or off.
//
// Input:
//  context - The context.
//  on      - A Boolean value indicating whether validation is on or off.
//
// Returns:
//  TriclopsErrorOk             - The operation succeeded.
//  InvalidContext - The input context was invalid.
//
// Description:
//  Turns uniqueness validation on or off.  Uniqueness validation verifies 
//  that the match of a given pixel to its corresponding pixels in the top 
//  and left images is unique enough to be considered the definite correct 
//  match.  If pixels at other disparities have almost as good matching to the 
//  given pixel as the best disparity, the pixel can be rejected as an 
//  unconfident match.
//
// See Also:
//  TriclopsGetUniquenessValidation()
//
TriclopsError
triclopsSetUniquenessValidation( TriclopsContext context,
				 TriclopsBool	 on );

//
// Name: triclopsGetUniquenessValidation
//
// Synopsis:
//  Retrieves the state of the uniqueness validation
//
// Input:
//  context - The context.
//
// Output:
//  on - A pointer to a Boolean.
//
// Returns:
//  TriclopsErrorOk             - The operation succeeded.
//  InvalidContext - The input context was invalid.
//
// See Also:
//  TriclopsSetUniquenessValidation()
//
TriclopsError
triclopsGetUniquenessValidation( const TriclopsContext context,
				 TriclopsBool*	       on );

//
// Name: triclopsSetUniquenessValidationThreshold
//
// Synopsis:
//  Sets the uniqueness validation threshold.
// 
// Input:
//  context - The context.
//  value   - A float indicating the new threshold.
//
// Returns:
//  TriclopsErrorOk             - The operation succeeded.
//  InvalidContext - The input context was invalid.
//
// Description:
//  This function sets the uniqueness matching criteria threshold.  
//  This value can range from 0.0 to 10.0. The larger the number, 
//  the less rejection occurs.  Common operating ranges are between 
//  0.5 and 3.0.
//
// See Also:
//  TriclopsGetUniquenessValidationThreshold()
//
TriclopsError
triclopsSetUniquenessValidationThreshold( TriclopsContext context,
					  float	       	  value );

//
// Name: triclopsGetUniquenessValidationThreshold
//
// Synopsis:
//  Retrieves the uniqueness validation threshold.
//
// Input:
//  context - The context.
// 
// Output:
//  value - A pointer to a float indicating the current threshold.
//
// Returns:
//  TriclopsErrorOk             - The operation succeeded.
//  InvalidContext - The input context was invalid.
//
// See Also:
//  TriclopsSetUniquenessThreshold()
//
TriclopsError
triclopsGetUniquenessValidationThreshold( const TriclopsContext	context,
					  float*		value );

//
// Name: triclopsSetUniquenessValidationMapping
//
// Synopsis:
//  Sets the value that appears in the disparity image for pixels 
//  that fail uniqueness validation.
//
// Input:
//  context - The context.
//  value   - The value to which failing pixels are mapped.
//  
// Returns:
//  TriclopsErrorOk             - The operation succeeded.
//  InvalidContext - The input context was invalid.
//
// Description:
//  Sets the unique matching validation mapping value.  This is the value that 
//  is assigned to pixels in the disparity image that fail the uniqueness 
//  validation test.  The range is between 0 and 255.
//
// See Also:
//  TriclopsGetUniquenessValidationMapping()
//
TriclopsError
triclopsSetUniquenessValidationMapping( TriclopsContext   context,
					unsigned char     value );

//
// Name: triclopsGetUniquenessValidationMapping
//
// Synopsis:
//  Retrieves the value that appears in the disparity image for pixels 
//  that fail uniqueness validation.
// 
// Input:
//  context - The context.
// 
// Output:
//  value - A pointer to an unsigned char that will contain the current value 
//          of the mapping.
//
// Returns:
//  TriclopsErrorOk             - The operation succeeded.
//  InvalidContext - The input context was invalid.
//
// See Also:
//  TriclopsSetUniquenessValidationMapping()
//
TriclopsError
triclopsGetUniquenessValidationMapping( const TriclopsContext	context,
					unsigned char*		value );

//
// Name: triclopsSetBackForthValidation
//
// Synopsis:
//  Enables or disables back-forth validation.
//
// Input:
//  context - TriclopsContext for the operation.
//  on      - A Boolean flag indicating if back-forth validation should be enabled
//            or disabled.
//
// Returns:
//  TriclopsErrorOk             - Operation successful.
//  TriclopsErrorInvalidContext - Context is not a valid TriclopsContext.
// 
// Description:
//  This function is used to enable or disable back-forth validation. 
//
// See Also:
//  triclopsGetSurfaceValidation ()
//
TriclopsError
triclopsSetBackForthValidation( TriclopsContext 	context,
				TriclopsBool 		on );
//
// Name: triclopsGetBackForthValidation
//
// Synopsis:
//  Gets the current back-forth validation setting.
//
// Input:
//  context - TriclopsContext for the operation.
//
// Output:
//  on  - A pointer for the returned value of the current setting of the 
//        back-forth validation flag.
//
// Returns:
//  TriclopsErrorOk             - Operation successful.
//  TriclopsErrorInvalidContext - Context is not a valid TriclopsContext.
//
// Description:
//  This function is used to obtain the current setting of back-forth validation.  
//
// See Also:
//  triclopsSetBackForthValidation ()
//
TriclopsError
triclopsGetBackForthValidation( const TriclopsContext	context,
				 TriclopsBool	*on );

//
// Name: triclopsSetBackForthValidationMapping
//
// Synopsis:
//  Sets the value that appears in the disparity image for pixels 
//  that fail back-forth validation.
//
// Input:
//  context - The context.
//  value   - The value to which failing pixels are mapped.
//  
// Returns:
//  TriclopsErrorOk  - The operation succeeded.
//  InvalidContext   - The input context was invalid.
//
// Description:
//  Sets the back-forth validation mapping value.  This is the value that 
//  is assigned to pixels in the disparity image that fail the back-forth 
//  validation test.  The range is between 0 and 255.
//
// See Also:
//  triclopsGetBackForthValidationMapping()
//
TriclopsError
triclopsSetBackForthValidationMapping( TriclopsContext	context,
				       unsigned char	value );

//
// Name: triclopsGetBackForthValidationMapping
//
// Synopsis:
//  Retrieves the back-forth validation threshold.
//
// Input:
//  context - The context.
// 
// Output:
//  value - A pointer that will return the current value of the back-forth
//          validation mapping parameter.
//
// Returns:
//  TriclopsErrorOk  - The operation succeeded.
//  InvalidContext   - The input context was invalid.
//
// See Also:
//  triclopsSetBackForthValidationMapping()
//
TriclopsError
triclopsGetBackForthValidationMapping( const TriclopsContext	context,
				       unsigned char		*value );

//
// Name: triclopsSetSurfaceValidation
//
// Synopsis:
//  Enables or disables surface validation.
//
// Input:
//  context - TriclopsContext for the operation.
//  on      - A Boolean flag indicating if surface validation should be enabled
//            or disabled.
//
// Returns:
//  TriclopsErrorOk                          - Operation successful.
//  TriclopsErrorInvalidContext - Context is not a valid TriclopsContext.
// 
// Description:
//  This function is used to enable or disable surface validation. 
//
TriclopsError
triclopsSetSurfaceValidation( TriclopsContext   context,
			      TriclopsBool      on );

//
// Name: triclopsGetSurfaceValidation
//
// Synopsis:
//  Gets the current surface validation setting.
//
// Input:
//  context - TriclopsContext for the operation.
//
// Output:
//  on  - A pointer for the returned value of the current  setting of the 
//        surface validation flag.
//
// Returns:
//  TriclopsErrorOk                          - Operation successful.
//  TriclopsErrorInvalidContext - Context is not a valid TriclopsContext.
//
// Description:
//  This function is used to obtain the current setting of surface validation.  
//
// See Also:
//  triclopsSetSurfaceValidation ()
//
TriclopsError
triclopsGetSurfaceValidation( const TriclopsContext	 context,
			      TriclopsBool*		 on );


//
// Name: triclopsSetSurfaceValidationSize
//
// Synopsis:
//  Sets the minimum number of pixels a surface can cover and still be
//  considered valid.
//
// Input:
//  context - TriclopsContext for the operation.			
//  size    - The minimum number of pixels a surface can cover and still be
//            considered valid.
//
// Returns:
//  TriclopsErrorOk                          - Operation successful.
//  TriclopsErrorInvalidContext - Context is not a valid TriclopsContext.                                             
//  TriclopsErrorInvalidSetting - A negative size was given.
//
// Description:
//  This function is used to set the minimum number of pixels a surface can
//  cover and still be considered valid.  The larger the number is, the fewer
//  surfaces will be accepted.  The lower the number is, the more surfaces will
//  be accepted.  Common parameter values range from 100 to 500, depending on 
//  image resolution.
//
TriclopsError
triclopsSetSurfaceValidationSize( TriclopsContext context,
				  int		  size );

//
// Name: triclopsGetSurfaceValidationSize
//
// Synopsis:
//  Retrieves the current validation size.
//
// Input:
//  context - TriclopsContext for the operation.
// 
// Output:
//  size - A pointer that returns the current value of the size  parameter.
//
// Returns:
//  TriclopsErrorOk                          - Operation Successful.	
//  TriclopsErrorInvalidContext - Context is not a valid TriclopsContext.
//
// Description:
//  This function is used to extract the surface size parameter for surface
//  validation. 
// 
// See Also:
//  triclopsSetSurfaceValidationSize 
//
TriclopsError
triclopsGetSurfaceValidationSize( const TriclopsContext	context,
				  int*			size );

//
// Name: triclopsSetSurfaceValidationDifference
//
// Synopsis:
//  Set the maximum disparity difference between two adjacent pixels that will
//  still allow the two pixels to be considered part of the same surface.
//
// Input:
//  context - TriclopsContext for the operation.			
//  diff    - The maximum disparity difference between two  adjacent pixels 
//            that will still allow the two pixels  to be considered part of 
//            the same surface.
//
// Returns:
//  TriclopsErrorOk                          - Operation successful. 
//  TriclopsErrorInvalidContext - Context is not a valid TriclopsContext.
//  TriclopsErrorInvalidSetting - A negative difference was given.	
// 
TriclopsError
triclopsSetSurfaceValidationDifference( TriclopsContext	context,
					float		diff  );

//
// Name: triclopsGetSurfaceValidationDifference
// 
// Synopsis:
//  Returns the maximum disparity difference between two adjacent pixels that 
//  will still allow the two pixels to be considered part of the same surface.
// 
// Input:
//  context - TriclopsContext for the operation.
//  
// Output:
//  diff - A pointer that returns the current value of the surface validation 
//         difference parameter.
//
// Returns:
//  TriclopsErrorOk                          - Operation successful.
//  TriclopsErrorInvalidContext - Context is not a valid TriclopsContext.
//
TriclopsError
triclopsGetSurfaceValidationDifference( const TriclopsContext	context,
					float*			diff  );

//
// Name: triclopsSetSurfaceValidationMapping
//
// Synopsis:
//  Sets the current surface validation mapping.
//
// Input:
//  context - TriclopsContext for the operation.			
//  value   - Mapping value for pixels that fail surface validation. The range
//            is between 0 and 255.
//
// Returns:
//  TriclopsErrorOk                          - Operation successful.
//  TriclopsErrorInvalidContext	- Context is not a valid TriclopsContext
//
// Description:
//  Surface validation is a noise rejection method.  By setting the mapping 
//  value to 0x??, an invalid pixel that failed this validation step will be
//  marked 0xFF??.
//
TriclopsError
triclopsSetSurfaceValidationMapping( TriclopsContext	context,
					unsigned char	value );

//
// Name: triclopsGetSurfaceValidationMapping
//
// Synopsis:
//  Retrieves the current surface validation mapping.
//
// Input:
//  context - TriclopsContext for the operation.
//
// Output:
//  value - A pointer that will return the current value of the surface
//          validation mapping parameter.
//
// Returns:
//  TriclopsErrorOk                          - Operation successful. 
//  TriclopsErrorInvalidContext - Context is not a valid TriclopsContext.
//
// Description:
//  This function returns the current setting for the surface validation 
//  parameter.
//
TriclopsError
triclopsGetSurfaceValidationMapping( const TriclopsContext context,
				     unsigned char*	   value );

//
// Name: triclopsSetStrictSubpixelValidation
//
// Synopsis:
//  Sets the state of the subpixel validation setting.
//
// Input:
//  context -  The context
//  on      - A Boolean value indicating whether validation is on or off. 
//
// Returns:
//  TriclopsErrorOk             - The operation succeeded.
//  InvalidContext - The input context was invalid.
//
// Description:
//  The strict subpixel validation option is used when the subpixel 
//  interpolation flag is turned on.  Strict subpixel validation enables 
//  a more restrictive validation method for subpixel validation.  With 
//  strict subpixel validation on, there will be less data in the output
//  image, but it will be more reliable.
//
// See Also:
//  triclopsGetStrictSubpixelValidation(), triclopsSetSubpixelInterpolation(), 
//  triclopsGetSubpixelInterpolation()
//
TriclopsError
triclopsSetStrictSubpixelValidation( TriclopsContext	 context,
				     TriclopsBool	 on );

//
// Name: triclopsGetStrictSubpixelValidation
//
// Synopsis:
//  Retrieves the value that appears in the disparity image for pixels that 
//  fail uniqueness validation.
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
//  triclopsSetStrictSubpixelValidation(), triclopsSetSubpixelInterpolation(), 
//  triclopsGetSubpixelInterpolation()
//
TriclopsError
triclopsGetStrictSubpixelValidation( TriclopsContext	context,
				     TriclopsBool*	on );

//
// Name: triclopsSetSubpixelValidationMapping
//
// Synopsis:
//  Sets the value that appears in the disparity image for pixels that fail 
//  subpixel validation.
//
// Input:
//  context - TriclopsContext for the operation.
//  value   - The new subpixel validation mapping value.  The range is between
//            0 and 255.
//
// Returns:
//  TriclopsErrorOk                          - The operation succeeded.
//  TriclopsErrorInvalidContext	- Context is not a valid TriclopsContext.
//
// Description:
//  Strict subpixel validation verifies that the disparity values contribute 
//  to the subpixel interpolation make sense.  By setting the mapping value to 
//  0x??, an invalid pixel that failed this validation step will be marked 
//  0xFF??.
//
TriclopsError
triclopsSetSubpixelValidationMapping( TriclopsContext   context,
				      unsigned char	value );

//
// Name: triclopsGetSubpixelValidationMapping
//
// Synopsis:
//  Retrieves the value that appears in the disparity image for pixels that 
//  fail subpixel validation.
//
// Input:	
//  context - TriclopsContext for the operation.
//
// Output:
//  value - A pointer that will hold the subpixel validation mapping value.
//
// Returns:
//  TriclopsErrorOk                          - Operation successful.
//  TriclopsErrorInvalidContext	- Context is not a valid TriclopsContext.
//
// See Also:
//  triclopsSetSubpixelValidationMapping
//
TriclopsError
triclopsGetSubpixelValidationMapping( const TriclopsContext	context,
				      unsigned char*		value );


#ifdef __cplusplus
}
#endif

#endif  // #ifndef TRICLOPSVALIDATION_H
