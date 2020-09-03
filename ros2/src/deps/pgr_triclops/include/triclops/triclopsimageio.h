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
// $Id: triclopsimageio.h,v 2.6 2007-11-30 00:43:03 soowei Exp $
//=============================================================================
#ifndef TRICLOPSIMAGEIO_H
#define TRICLOPSIMAGEIO_H

//=============================================================================
//
// This file defines the API for the Triclops Stereo Vision SDK
// for functions that deal with saving and loading images
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
// Name: RectifiedImageInfo 
//
// Description: 
//   Information that is stored in a Pgm file created using 
// triclopsSaveImageExtra when passed the TriclopsImageType TriImg_RECTIFIED
// as the second parameter.
// 
typedef struct 
{
   TriclopsRectImgQuality rectQuality;
} RectifiedImageInfo;

//
// Name: EdgeImageInfo 
//
// Description: 
//   Information that is stored in a Pgm file created using 
// triclopsSaveImageExtra when passed the TriclopsImageType TriImg_EDGE
// as the second parameter.
// 
typedef struct 
{
   TriclopsRectImgQuality rectQuality;
   int edgeMaskSize;
   bool  lowpassOn;
} EdgeImageInfo;

//
// Name: DisparityImageInfo 
//
// Description: 
//   Information that is stored in a Pgm file created using 
// triclopsSaveImageExtra when passed the TriclopsImageType TriImg_DISPARITY
// as the second parameter.
// 
typedef struct 
{
   TriclopsRectImgQuality	 rectQuality;
   bool				 edgeCorrelationOn;
   int				 edgeMaskSize;
   bool				 lowpassOn;
   TriclopsStereoQuality	 stereoQuality;
   TriclopsCameraConfiguration	 camConfig;
   int				 stereoMaskSize;
   int				 minDisparity;
   int				 maxDisparity;
   int				 nDisparityOffset;
   bool				 disparityMappingOn;
   int				 minDispMapping;
   int				 maxDispMapping;
   bool				 textureValidOn;
   double			 textureValidThreshold;
   unsigned char		 textureValidMapping;
   bool				 uniqueValidOn;
   double			 uniqueValidThreshold;
   unsigned char		 uniqueValidMapping;
   bool				 backForthValidOn;
   unsigned char		 backForthValidMapping;
   bool				 surfaceValidOn;
   double			 surfaceValidDiff;
   int				 surfaceValidSize;
   unsigned char		 surfaceValidMapping;
} DisparityImageInfo;

//
// Name: TriclopsImageInfo 
//
// Description: 
//   This structure is used to hold any image information available when an
// image is read in to the triclopsLibrary. It currently works with the 
// function calls triclopsReadImage[16]Extra. Different information is available
// depending on the TriclopsImageType of the image -- that is the function 
// of the union. The information will be available if the image was created
// using the triclopsSaveImage[16]Extra function.
typedef struct 
{
   bool commentFound;
   char product[100];
   int serialNumber;
   TriclopsCamera nCamera;
   union 
   {
      TriclopsImageType	   imageType;
      TriclopsImage16Type  image16Type;
   } type;
   union
   {
      RectifiedImageInfo rectified;
      EdgeImageInfo	 edge;
      DisparityImageInfo disparity;
   } info;

} TriclopsImageInfo;


//=============================================================================
// Image I/O Operations
//=============================================================================
// Group = Image I/O Operations

//
// Name: triclopsSaveImageExtra
//
// Synopsis:
//  Saves the specified type of image associated with the specified camera to a 
//  file named "filename". Currently, the only output file format supported is PGM.
//
// Input:
//  context   - The context.
//  imageType - The image type requested.
//  camera    - The camera that generated the requested image.	
//  filename  - The file name in which to save the image.	
// 
// Returns:
//  TriclopsErrorOk - The operation succeeded.
//  SystemError     - The file could not be opened
//  InvalidContext  - The input context was invalid.
//  InvalidCamera   - The camera does not match a camera in this configuration.
//  InvalidRequest  - The image type does not match the camera or is not being
//			generated by this system with the current context options.
//  ErrorUnknown    - The image was unable to be written properly for an unknown
//			reason.	
//
// Description:
//  Saves the specified type of image associated with the specified camera to a 
//  file named "filename".  Depending on the image type, certain useful 
//  comments are filled in to the header.
//  For all images, the camera type and serial number are filled in. For each
//  image type, only relevant parameters are filled in. E.g., for the 
//  TriImg_RECTIFIED type, the only additional comment is the rectification
//  quality.
//
TriclopsError
triclopsSaveImageExtra( const TriclopsContext context, 
		        TriclopsImageType imageType,
		        TriclopsCamera camera,
		        char* filename);

//
// Name: triclopsSaveImage16Extra
//
// Synopsis:
//  Saves the specified type of 16-bit image associated with the specified 
//  camera to a file named "filename". Currently, the only output file format 
//  supported is PGM.
//
// Input:
//  context     - The context.
//  image16Type - The 16-bit image type requested.
//  camera      - The camera that generated the requested image.	
//  filename    - The file name in which to save the image.	
// 
// Returns:
//  TriclopsErrorOk - The operation succeeded.
//  SystemError     - The file could not be opened
//  InvalidContext  - The input context was invalid.
//  InvalidCamera   - The camera does not match a camera in this configuration.
//  InvalidRequest  - The image type does not match the camera or is not being
//			generated by this system with the current context options.
//  ErrorUnknown    - The image was unable to be written properly for an unknown
//			reason.	
//
// Description:
//  Saves the specified type of 16-bit image associated with the specified camera 
//  to a file named "filename".  Depending on the image type, certain useful 
//  comments are filled in to the header.
//  For all images, the camera type and serial number are filled in. 
//
// See Also:
//  triclopsSaveImageExtra()
//
TriclopsError
triclopsSaveImage16Extra( const TriclopsContext context, 
			  TriclopsImage16Type image16Type,
			  TriclopsCamera camera,
			  char* filename );

//
// Name: triclopsReadImageExtra
//
// Synopsis:
//  Reads an image from a file named "filename" into a triclopsImage,
// filling in as much of the imageInfo structure as possible from the input
// file's header. Currently, the only input file format supported is PGM.
//
// Input:
//  filename	     - The name of the file from which the image is read.
//  triclopsImage    - The image that gets read.
//  imageInfo	     - Relevant information about the image that is read.
// 
// Returns:
//  TriclopsErrorOk		    - The operation succeeded.
//  TriclopsErrorFileRead	    - Could not read in the specified file (not
//				       found, or corrupted).
//  TriclopsErrorCorruptPGRComment  - The PGR comment in the header was 
//    corrupted. The imageInfo is invalid, but the file body ("the image")
//    was read in successfully.
// 
// Description:
//  Reads an image from a file named "filename" into a triclopsImage. If 
// the image was created using the triclopsSaveImageExtra function, it should
// contain information about the image, depending on the image type. This 
// information is parsed from the header and placed into the imageInfo
// structure. If the image is a rectified, edge, or disparity image, the
// appropriate parameter in the TriclopsImageInfo union will be filled in.
// ** NOTE: Even if the function returns triclopsErrorOk, you still need to 
// check the TriclopsImageInfo "commentFound" boolean field, in case there
// was no PGRComment associated with the input file.
//
// See Also:
//  triclopsReadImage16Extra()
//
TriclopsError
triclopsReadImageExtra( char* filename, 
		        TriclopsImage* triclopsImage, 
		        TriclopsImageInfo* imageInfo);

//
// Name: triclopsReadImage16Extra
//
// Synopsis:
//  Reads an image from a file named "filename" into a TriclopsImage16,
// filling in as much of the imageInfo structure as possible from the input
// file's header. Currently, the only input file format supported is PGM.
//
// Input:
//  filename	     - The name of the file from which the image is read.
//  triclopsImage16  - The 16-bit image that gets read.
//  imageInfo	     - Relevant information about the image that is read.
// 
// Returns:
//  TriclopsErrorOk		    - The operation succeeded.
//  TriclopsErrorFileRead	    - Could not read in the specified file 
//				       (not found, or corrupted).
//  TriclopsErrorCorruptPGRComment  - The PGR comment in the header was 
//    corrupted. The imageInfo is invalid, but the file body ("the image")
//    was read in successfully.
// 
// Description:
//  Reads a 16-bit image from a file named "filename" into a triclopsImage. 
// If the image was created using the triclopsSaveImage16Extra function, it should
// contain information about the image, depending on the image type. This 
// information is parsed from the header and placed into the imageInfo
// structure. If the image is a rectified, edge, or disparity image, the
// appropriate parameter in the TriclopsImageInfo union will be filled in.
// ** NOTE: Even if the function returns triclopsErrorOk, you still need to 
// check the TriclopsImageInfo "commentFound" boolean field, in case there
// was no PGRComment associated with the input file.
//
// See Also:
//  triclopsReadImageExtra()
//
TriclopsError
triclopsReadImage16Extra( char* filename, 
			  TriclopsImage16* triclopsImage16, 
			  TriclopsImageInfo* imageInfo);


//
// Name: triclopsSaveColorImage
//
// Synopsis:
//  Saves an image to the specified filename.  The file format currently 
//  supported is PGM format.
//
// Input:
//  image    - The TriclopsColorImage to be saved.
//  filename - The file name in which to save the image.
//
// Returns:
//  SystemError     - The file could not be opened
//  TriclopsErrorOk - The operation succeeded.
//
// Description:
//  This function saves the input image to the requested file.  Currently, this 
//  function will not detect if the file could not be opened, and will always 
//  return successful.  Color images are saved in PPM format.
//
// See Also:
//  triclopsSaveImage()
//
TriclopsError
triclopsSaveColorImage( TriclopsColorImage*	image, 
			char*			filename );

//
// Name: triclopsSavePackedColorImage
//
// Description:
//  Allows the user to save a packed color image to the given file.
//
// Input:
//  image    - A pointer to the buffer containing the image.
//  filename - The name of the file to be written to.
//
// Returns:
//  SystemError     - The file could not be opened
//  TriclopsErrorOk - Upon the successful completion of the operation.
//
TriclopsError
triclopsSavePackedColorImage( TriclopsPackedColorImage*  image, 
			      char*			 filename );





#ifdef __cplusplus
}
#endif

#endif  // #ifndef TRICLOPSIMAGEIO_H

