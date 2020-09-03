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
// $Id: triclopscontext.h,v 2.7 2010-07-27 19:19:58 arturp Exp $
//=============================================================================
#ifndef TRICLOPSCONTEXT_H
#define TRICLOPSCONTEXT_H

//=============================================================================
//
// This file defines the the API for TriclopsContext management functions
// within the Triclops Stereo Vision SDK.
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
//
// Group = Enumerations

//
// Name: TriclopsCameraConfiguration
//
// Description:
//  This enumerated type defines the camera configuration. The symbols in
//  the table represent the cameras as they would be seen when the camera 
//  module is viewed from the front. This type is either read from the camera
//  or is set manually to indicate 2-Camera mode.
//
typedef enum TriclopsCameraConfiguration
{
   // L shaped three camera system ** obsolete **.
   TriCfg_L = 0,
   // 2 Camera Unit or 3 Camera Unit in 2 camera stereo mode.
   TriCfg_2CAM_HORIZONTAL = 1,
   TriCfg_2CAM_HORIZONTAL_NARROW = 1,	// 12cm baseline for BB2 or XB3
   // 2 Camera Vertical Unit or 3 Camera Unit in 2 camera vertical mode.
   TriCfg_2CAM_VERTICAL = 2,
   TriCfg_2CAM_HORIZONTAL_WIDE = 3,	// 24cm baseline for XB3
   TriCfg_MAX_VALUE = 3
      
} TriclopsCameraConfiguration;


//=============================================================================
// Types 
//=============================================================================

//=============================================================================
// Function Prototypes  
//=============================================================================

//=============================================================================
// Triclops Context Manipulation
//=============================================================================
//
//Group = Triclops Context Manipulation 

//
// Name: triclopsGetDefaultContextFromFile
// 
// Synopsis: 
//  Setup the initial context with data obtained from a file.
//
// Input:
//  filename - The configuration file name.
//
// Output:
//  defaultContext - The default context.
//
// Returns:
//  TriclopsErrorOk                - The operation succeeded.
//  CorruptConfigFile - The specified file was corrupt, or was for the 
//                      wrong Triclops version.
//
// Description:
//  This function reads in the default option list and the camera 
//  calibration data from a file.  If the specified file is not found, 
//  contains invalid fields, or is for the wrong product, the value of 
//  CorruptConfigFile will be returned.
//
// See Also:
//  triclopsWriteDefaultContextToFile(), triclopsWriteCurrentContextToFile()
//
TriclopsError
triclopsGetDefaultContextFromFile( TriclopsContext* 	defaultContext,
				   char*	 	filename );

//
// Name: triclopsGetDefaultContextFromMemory
// 
// Synopsis: 
//  Setup the initial context with data obtained from a file loaded into memory.
//
// Input:
//  fileData - The configuration file data location in memory.
//
// Output:
//  defaultContext - The default context.
//
// Returns:
//  TriclopsErrorOk - The operation succeeded.
//  CorruptConfigFile - The specified file was corrupt, or was for the 
//                      wrong Triclops version.
//
// Description:
//  This function reads in the default option list and the camera 
//  calibration data from a file loaded into memory. If the specified file data
//  contains invalid fields, or is for the wrong product, the value of 
//  CorruptConfigFile will be returned.
//
// See Also:
//  triclopsWriteDefaultContextToFile(), triclopsWriteCurrentContextToFile()
//
TriclopsError
triclopsGetDefaultContextFromMemory(TriclopsContext* defaultContext,
				    void* fileData);

//
// Name: triclopsWriteDefaultContextToFile
// 
// Synopsis: 
//  This writes the default calibration file from the TriclopsContext
//  to a file.
//
// Input:
//  context  - The TriclopsContext
//  filename - The name of the file to be written
//
// Output:
//  none
//
// Returns:
//  TriclopsErrorOk   - The operation succeeded.
//  InvalidContext    - The context provided was invalid
//  SystemError       - Could not open the specified file
//
// Description:
//  This function writes the default context parameters and calibration
//  data to a file.  It does not write the current configuration, rather the
//  original configuration that would obtained when getting the default
//  from either a file or a device.
//
// See Also:
//  TriclopsGetDefaultContextFromFile(), triclopsWriteCurrentContextToFile()
//
TriclopsError
triclopsWriteDefaultContextToFile( TriclopsContext 	context,
				   char*	 	filename );


//
// Name: triclopsWriteCurrentContextToFile
// 
// Synopsis: 
//  This writes the calibration and parameter file from the TriclopsContext
//  to a file.  It is the "current" context, so it uses the parameters
//  that are currently active in the TriclopsContext.  Any parameter changes
//  that have been made through the API will be reflected in this 
//  calibration file.
//
// Input:
//  context  - The TriclopsContext
//  filename - The name of the file to be written
//
// Output:
//  none
//
// Returns:
//  TriclopsErrorOk   - The operation succeeded.
//  InvalidContext    - The context provided was invalid
//  SystemError       - Could not open the specified file
//
// Description:
//  This function writes the current context parameters and calibration
//  data to a file. 
//
// See Also:
//  TriclopsGetDefaultContextFromFile(), triclopsWriteDefaultContextToFile()
//
TriclopsError
triclopsWriteCurrentContextToFile( TriclopsContext 	context,
				   char*	 	filename );


//
// Name: triclopsCopyContext
//
// Synopsis:
//  Copies the context parameters and images.  Care must be taken when 
//  using triclopsCopyContext() as contexts will share image buffers.
//
// Input:
//  contextIn - A context that has already been constructed.
//
// Output:
//  contextOut - A copy of the input context.
//
// Returns:
//  TriclopsErrorOk             - The operation succeeded.
//  InvalidContext - The input context is invalid.
// 
// Description:
//  This function creates a copy of the input context.  This allows 
//  the user to have two contexts with the same options and also to 
//  share the same image buffers.  Care must be taken with this function.  
//  The ensuing contexts will share image buffers.  This allows them to 
//  share some of the previous processing, however, any changes in resolution 
//  or calls to set image buffers will create new buffers to be used.  These 
//  buffers will no longer be shared and thus programs may not operate as 
//  expected at this time.  The recommendation is to use this operation when 
//  one wants to run stereo with different stereo options.  After the 
//  preprocessing step, the context can be copied and different stereo options
//  set before the call to triclopsStereo().
//
// Note: This only copies the options, not the images 
//
// See Also:
//  triclopsGetDefaultContext(), triclopsDestroyContext()
//
TriclopsError
triclopsCopyContext( const TriclopsContext	contextIn,
		     TriclopsContext*		contextOut );

//
// Name: triclopsDestroyContext
// 
// Synopsis:
//  Destroys the given context.
//
// Input:
//  Context - The context to be destroyed.
//
// Returns:
// TriclopsErrorOk             - The operation succeeded.
// InvalidContext - The context was invalid.
//
// Description:
//  This function destroys a context and frees all associated memory.
//
TriclopsError
triclopsDestroyContext( TriclopsContext   context );

//=============================================================================
// Configuration
//=============================================================================
//
// Group = Configuration 

//
// Name: triclopsGetCameraConfiguration
//
// Synopsis:
//  Retrieves the current configuration of the stereo camera.  This 
//  configuration is the configuration that specifies the stereo 
//  algorithm used.  For example, 2CAM_HORIZONTAL configuration can be
//  used to do 2 camera stereo on a 3 camera device.
//
// Input:
//  context - The context.
//
// Output:
//  config - A pointer that will hold the result.
// 
// Returns:
//  TriclopsErrorOk  - The operation succeeded.
//  InvalidContext   - The input context was invalid.
//
// See Also:
//  TriclopsCameraConfiguration, TriclopsSetCameraConfiguration(),
//  TriclopsGetDeviceConfiguration()
//
TriclopsError
triclopsGetCameraConfiguration(  const TriclopsContext	  	context,
				 TriclopsCameraConfiguration*	config );

//
// Name: triclopsSetCameraConfiguration
//
// Synopsis:
//  Sets the configuration of the cameras.  This configuration determines 
//  the configuration for the stereo algorithm.  For example, a three camera
//  stereo device may be set into "2 camera horizontal" mode for faster
//  stereo processing.
//
// Input:
//  context - The context.
//  config  - The new CameraConfiguration.
// 
// Returns:
//  TriclopsErrorOk             - The operation succeeded.
//  InvalidContext 		- The input context was invalid.
//  InvalidParameter		- 'config' is not a valid camera configuration
//
// See Also:
//  TriclopsCameraConfiguration, TriclopsGetCameraConfiguration(),
//  TriclopsGetDeviceConfiguration()
//
TriclopsError
triclopsSetCameraConfiguration( const TriclopsContext	  	context,
				TriclopsCameraConfiguration	config );

//
// Name: triclopsGetDeviceConfiguration
//
// Synopsis:
//  This function returns the physical configuration of the stereo device.
//  This allows the user to determine what algorithms they have available
//  on the current device.
//
// Input:
//  context - The context.
// 
// Output:
//  config  - The physical CameraConfiguration.
// 
// Returns:
//  TriclopsErrorOk             - The operation succeeded.
//  InvalidContext 		- The input context was invalid.
//
// See Also:
//  TriclopsCameraConfiguration, TriclopsGetCameraConfiguration(), 
//  TriclopsSetCameraConfiguration()
//
TriclopsError
triclopsGetDeviceConfiguration( const TriclopsContext	  	context,
				TriclopsCameraConfiguration*	config );

//
// Name: triclopsGetSerialNumber
//
// Description:
//  This function returns the serial number of the stereo camera 
//  product associated with the given TriclopsContext.
//
// Input:
//  context - The context to extract the serial number from.
// 
// Output:
//  serialNumber - The serial number of the stereo camera product
//                 associated with the given context.
//
// Returns:
//  TriclopsErrorOk - Upon the successful completion of the operation.
// 
TriclopsError
triclopsGetSerialNumber( const TriclopsContext	context,
			 int*		      	serialNumber );

#ifdef __cplusplus
}
#endif

#endif  // #ifndef TRICLOPSCONTEXT_H
