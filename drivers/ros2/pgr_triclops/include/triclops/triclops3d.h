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
// $Id: triclops3d.h,v 2.2 2007-11-30 00:43:03 soowei Exp $
//=============================================================================
#ifndef TRICLOPS3D_H
#define TRICLOPS3D_H

//=============================================================================
//
// This file defines the API for 3D extraction and manipulation for the
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
// Name: TriclopsPoint3d
//
// Description:
//  This structure defines a single 3d pixel.  It is only used in the 
//  TriclopsImage3d structure.
//
typedef struct TriclopsPoint3d
{
   // The 3 values for the (x,y,z) point in order x = 0, y = 1, z = 2.
   float point[3];
   
} TriclopsPoint3d;

//
// Name: TriclopsImage3d
//
// Description:
//  This structure defines the format of an image consisting of 3d points.
//
typedef struct TriclopsImage3d
{
   // The number of rows in the image. 
   int nrows;
   // The number of columns in the image. 
   int ncols;
   // The number of bytes between the beginning of a row and the beginning
   // of the following row
   int rowinc;
   // The area for the pixel data.  This must be numRows * numCols bytes 
   // large. 
   TriclopsPoint3d*     points; 
   
} TriclopsImage3d;

//
// Name: TriclopsTransform 
//
// Description: 
//  A transformation matrix.
//
//
typedef struct 
{
   double	matrix[4][4];

} TriclopsTransform;

//=============================================================================
// Function Prototypes  
//=============================================================================
//
// Group = 3D

//
// Name: triclopsRCDToXYZ
//
// Synopsis:
//  Converts image coordinates with disparity values that have been mapped 
//  using the disparity mapping function into true 3D points.
//
// Input:
//  context - The stereo context.
//  row     - The row of the input pixel.
//  col     - The column of the input pixel.
//  disp    - The disparity value of the input pixel.	
//
// Output:
//  x - The x coordinate of the point represented by the input row column 
//      disparity in the camera coordinate system.
//  y - The y coordinate of the point represented by the input row column
//      disparity in the camera coordinate system.
//  z - The z coordinate of the point represented by the input row
//      column disparity in the camera coordinate system.
//
// Returns:
//  TriclopsErrorOk  - The operation succeeded.
//  InvalidContext   - The input context was invalid.	
//
// Description:
//  This function takes disparity values that have been scaled by the disparity 
//  mapping feature.  
//
// Remarks:
//  If you do not have the disparity mapping values set to 
//  the same as the disparity values, you should use this function.  However, 
//  it is less efficient than the other XYZ conversion functions and may have 
//  some round-off errors.  It is preferable to set the disparity mapping range 
//  to the same as the disparity range and use one of the other conversion 
//  functions.
//
//  It is up to the user to supply valid pixel locations.  Pixels that have 
//  been invalidated may give negative results.
// 
//  Disparity offset SHOULD NOT be applied to input disp values. In this 
//  way, users can step through a disparity image calling this function
//  and passing image disparity values straight in with no changes.
//
// See Also:
//  triclopsRCDFloatToXYZ(), triclopsRCD8ToXYZ(), triclopsRCD16ToXYZ()
//
TriclopsError
triclopsRCDToXYZ( TriclopsContext	context,
		  float		  	row, 
		  float		  	col, 
		  float		  	disp,
		  float*		x,  
		  float*		y,  
		  float*		z );

//
// Name: triclopsRCDMappedToXYZ
//
// Synopsis:
//  Converts image coordinates with disparity values that have been mapped 
//  using the disparity mapping function into true 3D points.
//
// Input:
//  context - The stereo context.
//  row     - The row of the input pixel.
//  col     - The column of the input pixel.
//  disp    - The disparity value of the input pixel.	
//
// Output:
//  x - The x coordinate of the point represented by the input row
//      column disparity in the camera coordinate system.
//  y - The y coordinate of the point represented by the input row
//      column disparity in the camera coordinate system.
//  z - The z coordinate of the point represented by the input row 
//      column disparity in the camera coordinate system.
//
// Returns:
//  TriclopsErrorOk             - The operation succeeded.
//  InvalidContext - The input context was invalid.
//
// Description:
//  This function takes disparity values that have been scaled by the disparity 
//  mapping feature.
//
// Remarks:
//  If you do not have the disparity mapping values set to the same as the
//  disparity values, you should use this function.  However, it is less
//  efficient than the other XYZ conversion functions and may have some
//  round-off errors.  It is preferable to set the disparity mapping range
//  to the same as the disparity range and use one of the other conversion 
//  functions.
//
//  It is up to the user to supply valid pixel locations.  Pixels that have
//  been invalidated may give negative results.
//
//  Disparity offset SHOULD NOT be applied to input disp values. In this 
//  way, users can step through a disparity image calling this function
//  and passing image disparity values straight in with no changes.
//
// See Also:
//  triclopsRCDFloatToXYZ(), triclopsRCD8ToXYZ(), triclopsRCD16ToXYZ()
//
TriclopsError
triclopsRCDMappedToXYZ( TriclopsContext	context, 
			int		row, 
			int		col, 
			unsigned char	disp,
			float*		x, 
			float*		y, 
			float*		z );

//
// Name: triclopsRCDFloatToXYZ
//
// Synopsis:
//  Converts image coordinates and a floating-point disparity value into true 
//  3D points.
//
// Input:
//  context - The stereo context.
//  row     - The row of the input pixel.
//  col     - The column of the input pixel.
//  disp    - The disparity value of the input pixel.	
//
// Output:
//  x - The x coordinate of the point represented by the input row
//      column disparity in the camera coordinate system.
//  y - The y coordinate of the point represented by the input row
//      column disparity in the camera coordinate system.
//  z - The z coordinate of the point represented by the input row 
//      column disparity in the camera coordinate system.
//
// Returns:
//  TriclopsErrorOk  - The operation succeeded.
//  InvalidContext   - The input context was invalid.	
//
// Description:
//  This function takes a floating-point disparity value and converts it to 
//  XYZ coordinates.
//
// Remarks:
//  It is up to the user to supply valid pixel locations.  Pixels which 
//  have been invalidated may give negative results.
//
//  Disparity offset MUST be applied to input disp values, if offset != 0,
//  in order to produce valid XYZ values.
//
// See Also:
//  triclopsRCDMappedToXYZ(), triclopsRCD8ToXYZ(), triclopsRCD16ToXYZ(),
//  triclopsGetDisparityOffset()
//
TriclopsError
triclopsRCDFloatToXYZ( TriclopsContext	context, 
		       float		row, 
		       float		col, 
		       float		disp,
		       float*		x, 
		       float*		y, 
		       float*		z );

//
// Name: triclopsRCD8ToXYZ
//
// Synopsis:
//  Converts image coordinates and an *-bit disparity into true 3D points.
//
// Input:
//  context - The stereo context.
//  row     - The row of the input pixel.
//  col     - The column of the input pixel.
//  disp    - The disparity value of the input pixel.	
//
// Output:
//  x - The x coordinate of the point represented by the input row
//      column disparity in the camera coordinate system.
//  y - The y coordinate of the point represented by the input row
//      column disparity in the camera coordinate system.
//  z - The z coordinate of the point represented by the input row 
//      column disparity in the camera coordinate system.
//
// Returns:
//  TriclopsErrorOk             - The operation succeeded.
//  InvalidContext - The input context was invalid.	
//
// Description:
//  When using this function, you should ensure that the values for the 
//  Triclops disparity mapping feature are the same as the disparity range.
//
// Remarks:
//  It is up to the user to supply valid pixel locations.  Pixels that have
//  been invalidated may give negative results.
//
//  Disparity offset SHOULD NOT be applied to input disp values. In this 
//  way, users can step through a disparity image calling this function
//  and passing image disparity values straight in with no changes.
//
// See Also:
//  triclopsRCDFloatToXYZ(), triclopsRCDMappedToXYZ(), triclopsRCD16ToXYZ(), 
//  triclopsSetDisparity(), triclopsSetDisparityMapping(), 
//  triclopsGetDisparityOffset()
//
TriclopsError
triclopsRCD8ToXYZ( TriclopsContext   	context, 
		   int		  	row, 
		   int		  	col, 
		   unsigned char     	disp,
		   float*		x, 
		   float*		y, 
		   float*		z );

//
// Name: triclopsRCD16ToXYZ
//
// Synopsis:
//  Converts image coordinates and a 16-bit disparity into true 3D points.
//
// Input:
//  context - The stereo context.
//  row     - The row of the input pixel.
//  col     - The column of the input pixel.
//  disp    - The disparity value of the input pixel.	
//
// Output:
//  x - The x coordinate of the point represented by the input 
//      row column disparity in the camera coordinate system.
//  y - The y coordinate of the point represented by the input 
//      row column disparity in the camera coordinate system.
//  z - The z coordinate of the point represented by the input 
//      row column disparity in the camera coordinate system.
//
// Returns:
//  TriclopsErrorOk             - The operation succeeded.
//  InvalidContext - The input context was invalid.	
//
// Description:
//  When using this function, you should ensure that the values for the 
//  Triclops disparity mapping feature are the same as the disparity range.
//
// Remarks:
//  It is up to the user to supply valid pixel locations.  Pixels that have
//  been invalidated may give negative results.
//
//  Disparity offset SHOULD NOT be applied to input disp values. In this 
//  way, users can step through a disparity image calling this function
//  and passing image disparity values straight in with no changes.
//
// See Also:
//  triclopsRCDFloatToXYZ(), triclopsRCDMappedToXYZ(), triclopsRCD8ToXYZ(), 
//  triclopsSetDisparity(), triclopsSetDisparityMapping()
//  triclopsGetDisparityOffset()
//
//
TriclopsError
triclopsRCD16ToXYZ( TriclopsContext	context, 
		    int		  	row, 
		    int		  	col, 
		    unsigned short    	disp,
		    float*		x, 
		    float*		y, 
		    float*		z );

//
// Name: triclopsXYZToRCD
//
// Synopsis:
//  Converts true 3D points into image coordinates. 
//
// Input:
//  context - TriclopsContext set up for desired resolution.
//  x       - X value of a point in the Triclops coordinate system.
//  y       - Y value of a point in the Triclops coordinate system.
//  z       - Z value of a point in the Triclops coordinate system.
//
// Output:
//  row  - The row in a disparity image.
//  col  - The column in a disparity image.
//  disp - The disparity value that would match the point specified in XYZ.
//
// Returns:
//  TriclopsErrorOk		 - The Operation succeeded.
//  TriclopsErrorInvalidContext	 - Context is not valid TriclopsContext.
//  TriclopsErrorInvalidRequest	 - An impossible XYZ value has been provided
//                                 (ie: negative Z).
//
// Description:
//  This function takes as input the XYZ position of a point in the Triclops 
//  coordinate system, and determines what row, column, and disparity value 
//  would result from a sensed point at XYZ.
//
TriclopsError
triclopsXYZToRCD( TriclopsContext context,
		  float		  x,   
		  float		  y,   
		  float		  z,
		  float*	  row, 
		  float*	  col, 
		  float*	  disp );

//
// Name: triclopsRCDToWorldXYZ
//
// Synopsis:
//  Converts image coordinates and disparity values to world 3D points.
//
// Input:
//  context - The stereo context.
//  row     - The row of the input pixel.
//  col     - The column of the input pixel.
//  disp    - The disparity value of the input pixel.	
//
// Output:
//  x - The x coordinate of the corresponding 3D point in the world
//	coordinate system
//  y - The y coordinate of the corresponding 3D point in the world
//	coordinate system
//  z - The z coordinate of the corresponding 3D point in the world
//	coordinate system
//
// Returns:
//  TriclopsErrorOk             - The operation succeeded.
//  InvalidContext - The input context was invalid.	
//
// Description:
//  This function takes a pixel location and matching disparity value and
//  calculates the 3D position that this combination represents.  The 
//  position is calculated in the "world" coordinate system.  That is to say,
//  the position is calculated in the Triclops coordinate system and then
//  transformed by the TriclopsContext transform to a new coordinate system.
//
// Remarks: 
//  It is up to the user to supply valid disparity values.  Values
//  taken from invalid pixels in the disparity image give negative 
//  results.
//
//  Disparity offset SHOULD NOT be applied to input disp values. In this 
//  way, users can step through a disparity image calling this function
//  and passing image disparity values straight in with no changes.
//
// See Also:
//  triclopsRCDFloatToXYZ(), triclopsRCD8ToXYZ(), triclopsRCD16ToXYZ(),
//  triclopsRCDFloatToWorldXYZ(), triclopsRCD8ToWorldXYZ(), 
//  triclopsRCD16ToWorldXYZ(), triclopsSetTriclopsToWorldTransform(), 
//  triclopsGetTriclopsToWorldTransform(),  triclopsGetDisparityOffset()
//
TriclopsError
triclopsRCDToWorldXYZ( TriclopsContext	context,
		       float		row, 
		       float		col, 
		       float		disp,
		       float*		x,  
		       float*		y,  
		       float*		z );

//
// Name: triclopsRCDMappedToWorldXYZ
//
// Synopsis:
//  Converts image coordinates with disparity values that have been mapped 
//  using the disparity mapping function into world 3D points.
//
// Input:
//  context - The stereo context.
//  row     - The row of the input pixel.
//  col     - The column of the input pixel.
//  disp    - The disparity value of the input pixel.	
//
// Output:
//  x - The x coordinate of the point represented by the input row
//      column disparity in the camera coordinate system.
//  y - The y coordinate of the point represented by the input row
//      column disparity in the camera coordinate system.
//  z - The z coordinate of the point represented by the input row 
//      column disparity in the camera coordinate system.
//
// Returns:
//  TriclopsErrorOk  - The operation succeeded.
//  InvalidContext   - The input context was invalid.
//
// Description:
//  This function takes disparity values that have been scaled by the 
//  disparity mapping feature and transforms them into a "world"
//  coordinate system based on the transform recorded in the TriclopsContext.
// 
// Remarks:
//  If you have set "Disparity Mapping" on you should use this function.
//  However, it is less efficient than the other XYZ conversion functions and 
//  may have some round-off errors. It is preferable to set the disparity
//  mapping off.
//
//  It is up to the user to supply valid disparity values, invalid
//  disparity values (as taken from an invalid pixel in the a disparity
//  image) may give negative results.
//
//  Disparity offset SHOULD NOT be applied to input disp values. In this 
//  way, users can step through a disparity image calling this function
//  and passing image disparity values straight in with no changes.
//
// See Also:
//  triclopsRCDFloatToXYZ(), triclopsRCD8ToXYZ(), triclopsRCD16ToXYZ()
//  triclopsRCDToWorldXYZ(), triclopsRCDFloatToWorldXYZ(), 
//  triclopsRCD8ToWorldXYZ(), triclopsRCD16ToWorldXYZ(), 
//  triclopsSetTriclopsToWorldTransform(), triclopsGetTriclopsToWorldTransform()
//  triclopsGetDisparityOffset()
//
TriclopsError
triclopsRCDMappedToWorldXYZ( TriclopsContext	context, 
			     int		row, 
			     int		col, 
			     unsigned char     	disp,
			     float*		x, 
			     float*		y, 
			     float*		z );

//
// Name: triclopsRCDFloatToWorldXYZ
//
// Synopsis:
//  Converts an image location and a floating-point disparity value into a 
//  world 3D point.
//
// Input:
//  context - The stereo context.
//  row     - The row of the input pixel.
//  col     - The column of the input pixel.
//  disp    - The disparity value of the input pixel.	
//
// Output:
//  x - The x coordinate of the corresponding 3D point 
//  y - The y coordinate of the corresponding 3D point
//  z - The z coordinate of the corresponding 3D point
//
// Returns:
//  TriclopsErrorOk             - The operation succeeded.
//  InvalidContext - The input context was invalid.	
//
// Description:
//  This function takes a floating-point disparity value and converts it to 
//  XYZ coordinates in world coordinates.  
//
// Remarks:
//  The world coordinates are determined by transforming the point from the 
//  Triclops coordinate system to the world coordinate system based on the 
//  TriclopsContext transform.
//
//  It is up to the user to supply valid disparity values.
//
//  Disparity offset MUST be applied to input disp values, if offset != 0,
//  in order to produce valid XYZ values.
//
// See Also:
//  triclopsRCDMappedToXYZ(), triclopsRCD8ToXYZ(), triclopsRCD16ToXYZ()
//  triclopsRCDToWorldXYZ(), 
//  triclopsRCD8ToWorldXYZ(), triclopsRCD16ToWorldXYZ(), 
//  triclopsSetTriclopsToWorldTransform(), triclopsGetTriclopsToWorldTransform()
//  triclopsGetDisparityOffset()
//
TriclopsError
triclopsRCDFloatToWorldXYZ( TriclopsContext	context, 
			    float		row, 
			    float		col, 
			    float		disp,
			    float*		x, 
			    float*		y, 
			    float*		z );

//
// Name: triclopsRCD8ToWorldXYZ
//
// Synopsis:
//  Converts image coordinates and an 8-bit disparity into a world 3D point.
//
// Input:
//  context - The stereo context.
//  row     - The row of the input pixel.
//  col     - The column of the input pixel.
//  disp    - The disparity value of the input pixel.	
//
// Output:
//  x - The x coordinate of the corresponding 3D point 
//  y - The y coordinate of the corresponding 3D point
//  z - The z coordinate of the corresponding 3D point
//
// Returns:
//  TriclopsErrorOk             - The operation succeeded.
//  InvalidContext - The input context was invalid.	
//
// Description:
//  This function takes an 8-bit disparity value and converts it to 
//  XYZ coordinates in world coordinates.  The world coordinates are determined
//  by transforming the point from the Triclops coordinate system to the
//  world coordinate system based on the TriclopsContext transform.
//
// Remarks:
//  It is up to the user to supply valid disparity values 
//
//  Disparity offset SHOULD NOT be applied to input disp values. In this 
//  way, users can step through a disparity image calling this function
//  and passing image disparity values straight in with no changes.
//
// See Also:
//  triclopsRCDFloatToXYZ(), triclopsRCDMappedToXYZ(), triclopsRCD16ToXYZ(), 
//  triclopsSetDisparity(), triclopsSetDisparityMapping()
//  triclopsRCDToWorldXYZ(), triclopsRCDFloatToWorldXYZ(), 
//  triclopsRCD16ToWorldXYZ(), 
//  triclopsSetTriclopsToWorldTransform(), triclopsGetTriclopsToWorldTransform()
//  triclopsGetDisparityOffset()
//
TriclopsError
triclopsRCD8ToWorldXYZ( TriclopsContext	context, 
			int		row, 
			int		col, 
			unsigned char	disp,
			float*		x, 
			float*		y, 
			float*		z );

//
// Name: triclopsRCD16ToWorldXYZ
//
// Synopsis:
//  Converts image coordinates and a 16-bit disparity into a world 3D point.
//
// Input:
//  context - The stereo context.
//  row     - The row of the input pixel.
//  col     - The column of the input pixel.
//  disp    - The disparity value of the input pixel.	
//
// Output:
//  x - The x coordinate of the corresponding 3D point 
//  y - The y coordinate of the corresponding 3D point
//  z - The z coordinate of the corresponding 3D point
//
// Returns:
//  TriclopsErrorOk             - The operation succeeded.
//  InvalidContext - The input context was invalid.	
//
// Description:
//  This function takes a 16-bit disparity value and converts it to 
//  XYZ coordinates in world coordinates.  The world coordinates are determined
//  by transforming the point from the Triclops coordinate system to the
//  world coordinate system based on the TriclopsContext transform.
//
// Remarks: 
//  It is up to the user to supply valid disparity values 
//
//  Disparity offset SHOULD NOT be applied to input disp values. In this 
//  way, users can step through a disparity image calling this function
//  and passing image disparity values straight in with no changes.
//
// See Also:
//  triclopsRCDFloatToXYZ(), triclopsRCDMappedToXYZ(), triclopsRCD8ToXYZ(), 
//  triclopsRCD16ToXYZ(),
//  triclopsSetDisparity(), triclopsSetDisparityMapping()
//  triclopsRCDToWorldXYZ(), triclopsRCDFloatToWorldXYZ(), 
//  triclopsRCD8ToWorldXYZ(), 
//  triclopsSetTriclopsToWorldTransform(), triclopsGetTriclopsToWorldTransform()
//  triclopsGetDisparityOffset()
//
//
TriclopsError
triclopsRCD16ToWorldXYZ( TriclopsContext	context, 
			 int			row, 
			 int			col, 
			 unsigned short		disp,
			 float*			x, 
			 float*			y, 
			 float*			z );

//
// Name: triclopsWorldXYZToRCD
//
// Synopsis:
//  Converts world 3D points into image coordinates. 
//
// Input:
//  context - TriclopsContext set up for desired resolution.
//  x       - X value of a point in the World coordinate system.
//  y       - Y value of a point in the World coordinate system.
//  z       - Z value of a point in the World coordinate system.
//
// Output:
//  row  - The row in a disparity image.
//  col  - The column in a disparity image.
//  disp - The disparity value that would match the point specified in XYZ.
//
// Returns:
//  TriclopsErrorOk                          - The Operation succeeded.
//  TriclopsErrorInvalidContext - Context is not valid TriclopsContext.
//  TriclopsErrorInvalidRequest - An impossible XYZ value has been provided
//                                (ie: negative Z).
//
// Description:
//  This function takes as input the XYZ position of a point in the World 
//  coordinate system, moves the point to the Triclops coordinate system 
//  (as described by the TriclopsContext transform), and determines what 
//  row, column, and disparity value would result from the resulting point.
//
// See Also:
//  triclopsRCDFloatToXYZ(), triclopsRCDMappedToXYZ(), triclopsRCD8ToXYZ(), 
//  triclopsSetDisparity(), triclopsSetDisparityMapping()
//  triclopsRCDToWorldXYZ(), triclopsRCDFloatToWorldXYZ(), 
//  triclopsRCD8ToWorldXYZ(), triclopsRCD16ToWorldXYZ(), 
//  triclopsXYZToRCD(),
//  triclopsSetTriclopsToWorldTransform(), triclopsGetTriclopsToWorldTransform()
//
TriclopsError
triclopsWorldXYZToRCD( TriclopsContext	context,
		       float		x,   
		       float		y,   
		       float		z,
		       float*		row, 
		       float*		col, 
		       float*		disp );

//
// Name: triclopsSetTriclopsToWorldTransform()
//
// Synopsis:
//  Sets the Triclops to World transform.
//
// Input:
//  context	- the TriclopsContext
//  transform   - the 4x4 homogeneous transform
//
// Output:
//  none
//
// Returns:
//  TriclopsErrorOk                          - The Operation succeeded.
//  TriclopsErrorInvalidContext - Context is not valid TriclopsContext.
//  TriclopsErrorInvalidRequest - The provided transform matrix has
//				  a singular rotation component 
//                               
// Description:
//  This function sets the internal TriclopsContext transform to match the
//  provided transform.  
//
// Remarks:
//  There are several things to note:
//
//  1. The transform is the Triclops-to-World transform. ie: when this
//  transform is applied to a Triclops point, it will change it to a world
//  point.
//
//  2. The transform must be of the approved format.  This means it has a 3x3
//  rotational component that is orthonormal and the bottom row must be of
//  format (0 0 0 n).  This function will try to normalize the rotational
//  component and clean up the bottom row of the matrix.  One can verify 
//  whether modifications to the transform were necessary by obtaining the
//  current transform using triclopsGetTriclopsToWorldTransform() and comparing.
//
// See Also:
//  triclopsRCDToWorldXYZ(), triclopsRCDFloatToWorldXYZ(), 
//  triclopsRCD8ToWorldXYZ(), triclopsRCD16ToWorldXYZ(), 
//  triclopsWorldXYZToRCD(), triclopsGetTriclopsToWorldTransform()
//  triclopsGetTransformFromFile(), triclopsWriteTransformToFile()
//
TriclopsError
triclopsSetTriclopsToWorldTransform( TriclopsContext	context, 
				     TriclopsTransform	transform );

//
// Name: triclopsGetTriclopsToWorldTransform()
//
// Synopsis:
//  Gets the Triclops to World transform.
//
// Input:
//  context	- the TriclopsContext
//
// Output:
//  transform   - the 4x4 homogeneous transform
//
// Returns:
//  TriclopsErrorOk		 - The Operation succeeded.
//  TriclopsErrorInvalidContext	 - Context is not valid TriclopsContext.
//                               
// Description:
//  This function fills in the provided TriclopsTransform structure with the
//  current contents of the Triclops to World transform for this 
//  TriclopsContext
//
// See Also:
//  triclopsRCDToWorldXYZ(), triclopsRCDFloatToWorldXYZ(), 
//  triclopsRCD8ToWorldXYZ(), triclopsRCD16ToWorldXYZ(), 
//  triclopsWorldXYZToRCD(), triclopsSetTriclopsToWorldTransform()
//  triclopsGetTransformFromFile(), triclopsWriteTransformToFile()
//
TriclopsError
triclopsGetTriclopsToWorldTransform( TriclopsContext	context, 
				     TriclopsTransform*	transform );

//
// Name: triclopsGetTransformFromFile()
//
// Synopsis:
//  Loads the contents of a TriclopsTransform from the input file.
//
// Input:
//  fileName	- name of the file from which to load the transform
//
// Output:
//  transform   - the 4x4 homogeneous transform
//
// Returns:
//  TriclopsErrorOk                          - The Operation succeeded.
//  CorruptTransformFile - either the file is not found or it contains invalid fields.
//                               
// Description:
//  This function fills in the provided TriclopsTransform structure based on
//  the contents read from the specified file.  If the specified file is not
//  found, contains invalid fields, the value of CorruptTransformFile will be
//  returned.  
//
// See Also:
//  triclopsRCDToWorldXYZ(), triclopsRCDFloatToWorldXYZ(), 
//  triclopsRCD8ToWorldXYZ(), triclopsRCD16ToWorldXYZ(), 
//  triclopsWorldXYZToRCD(), triclopsSetTriclopsToWorldTransform(),
//  triclopsGetTriclopsToWorldTransform(), triclopsWriteTransformToFile
//
TriclopsError
triclopsGetTransformFromFile( char* 		 fileName, 
			      TriclopsTransform* transform );

//
// Name: triclopsWriteTransformToFile()
//
// Synopsis:
//  Saves the contents of a TriclopsTransform to the output file.
//
// Input:
//  fileName	- name of the file to which to save the transform
//  transform   - the 4x4 homogeneous transform to save
//
// Output:
//
// Returns:
//  TriclopsErrorOk		 - The Operation succeeded.
//  TriclopsErrorInvalidContext	 - Context is not valid TriclopsContext.
//                               
// Description:
//  This function saves the contents of the specified transform to an
//  external file. 
//
// See Also:
//  triclopsRCDToWorldXYZ(), triclopsRCDFloatToWorldXYZ(), 
//  triclopsRCD8ToWorldXYZ(), triclopsRCD16ToWorldXYZ(), 
//  triclopsWorldXYZToRCD(), triclopsSetTriclopsToWorldTransform(),
//  triclopsGetTriclopsToWorldTransform(), triclopsGetTransformFromFile
//
TriclopsError
triclopsWriteTransformToFile( char*              fileName, 
			      TriclopsTransform* transform );

//
// Name: triclopsCreateImage3d
//
// Description:
//  Allocates a TriclopsImage3d to the correct size as specified by
//  the resolution of the TriclopsContext
//
// Input:
//   context   - The current TriclopsContext.
//   ppimage   - Pointer to the address of the TriclopsImage3d structure to
//               allocate memory for.
//
// Returns:
//  TriclopsErrorOk	      - The operation succeeded.
//  TriclopsErrorSystemError  - If there was a problem allocating the memory.
//
// See Also:
//    triclopsDestroyImage3d(), triclopsExtractImage3d(), 
//    triclopsExtractWorldImage3d()
//
TriclopsError  
triclopsCreateImage3d( TriclopsContext   context,
		       TriclopsImage3d** ppimage );

//
// Name: triclopsDestroyImage3d
//
// Description:
//  Deallocates a TriclopsImage3d allocated by triclopsCreateImage3d()
//
// Input:
//   ppimage   - Pointer to the address of the TriclopsImage3d structure to
//               destroy.
//
// See Also:
//    triclopsCreateImage3d(), triclopsExtractImage3d(), 
//    triclopsExtractWorldImage3d()
//
void  
triclopsDestroyImage3d( TriclopsImage3d** ppimage );

//
// Name: triclopsExtractImage3d
//
// Description:
//  Creates a 3D image given the current disparity result of a TriclopsContext
//
// Input:
//   context   - The current TriclopsContext.
//   pimage    - Pointer to the TriclopsImage3d structure.
//
// Returns:
//  TriclopsErrorOk		    - The operation succeeded.
//  TriclopsErrorInvalidContext	    - If the context is invalid.
//  TriclopsErrorInvalidParameter   - If there is a geometry mismatch between
//				      the context and the TriclopsImage3d.
//  TriclopsErrorInvalidRequest	    - If the (subpixel) disparity image does
//				      not match the current resolution.
//
// Remarks:
//  Invalid points will be assigned the value of (0,0,0) in the returned image.
//
// See Also:
//    triclopsDestroyImage3d(), triclopsCreateImage3d(), 
//    triclopsExtractWorldImage3d()
//
TriclopsError  
triclopsExtractImage3d( TriclopsContext  context,
		        TriclopsImage3d* pimage	);

//
// Name: triclopsExtractWorldImage3d
//
// Description:
//  Creates a 3D image given the current disparity result of a TriclopsContext
//  that is transformed to the world coordinate system
//
// Input:
//   context   - The current TriclopsContext.
//   pimage    - Pointer to the TriclopsImage3d structure.
//
// Returns:
//  TriclopsErrorOk		    - The operation succeeded.
//  TriclopsErrorInvalidContext	    - If the context is invalid.
//  TriclopsErrorInvalidParameter   - If there is a geometry mismatch between
//				      the context and the TriclopsImage3d.
//  TriclopsErrorInvalidRequest	    - If the (subpixel) disparity image does
//				      not match the current resolution.
//
// Remarks:
//  Invalid points will be assigned the value of (0,0,0) in the returned image.
//
// See Also:
//    triclopsDestroyImage3d(), triclopsCreateImage3d(), 
//    triclopsExtractImage3d()
//
TriclopsError  
triclopsExtractWorldImage3d( TriclopsContext     context,
			     TriclopsImage3d*    pimage );


#ifdef __cplusplus
}
#endif

#endif  // #ifndef TRICLOPS3D_H
