//=============================================================================
// Copyright � 2000 Point Grey Research, Inc. All Rights Reserved.
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
//=============================================================================
// $Id: pgrpnmio.h,v 1.5 2003-12-09 22:43:09 jbeis Exp $
//=============================================================================
#ifndef _PGRPNMIO_H_
#define _PGRPNMIO_H_

//=============================================================================
//
// pnmutils:
//
// This file provides users utilities for reading and writing PGM and
// PPM files from and to a generic character buffer.
// Users are encouraged to modify this source to suit their own needs.
//=============================================================================


//=============================================================================
// System Includes
//=============================================================================
#include <stdio.h>
#include <stdlib.h>
#include <string.h>


//=============================================================================
// PGR Includes
//=============================================================================

#ifdef __cplusplus
extern "C"
{
#endif


/* BYTESWAP for uint16_t values */
#if defined(_MSC_VER)                       /* MS or Intel Compilers */
 /* for windows function signature is :
    unsigned short _byteswap_ushort ( unsigned short val ); */
 #define PGR_BYTESWAP(val)    _byteswap_ushort(val)

#elif defined(__GNUC__)                     /* GCC Compilers */

/* missing __builtin_bswap16(val) in gcc < 4.8 */
 #if ( ( (__GNUC__ == 4 ) &&         \
         (__GNUC_MINOR__ >= 8 ) ) || \
       (__GNUC__ >= 5 ) )
 /* for gnu gcc linux function signature is :
    uint16_t __builtin_bswap16 (uint16_t x) */
 #define PGR_BYTESWAP(val)    __builtin_bswap16(val)

 #else /* gcc < 4.8 */
 
 static inline unsigned short __builtin_bswap16(unsigned short a)
 {
    return (a<<8)|(a>>8);
 }

 #define PGR_BYTESWAP(val)    __builtin_bswap16(val)

 #endif

#else                                       /* Unsupported compiler */
#error "Please provide your own byteswap function for uint16_t values." 
#endif

typedef enum _MachineEndianness 
{
   PGR_ENDIANNESS_LITTLE = 0, /**< Intel, ARM (selectable?) */
   PGR_ENDIANNESS_BIG    = 1  /**< PowerPC */

} MachineEndianness;

/**
 * \fn MachineEndianness pgrGetEndianness ( void ).
 *
 * \brief   Determine endianness of run-time architecture.
 * \return  PGR_ENDIANNESS_LITTLE if architecture is LE
 *          PGR_ENDIANNESS_BIG if architecture is BE.
 *
 * \note This function makes the assumption that 
 *       uint16_t is 2 bytes long and uint8_t is 1 byte long.
 */
MachineEndianness 
pgrGetEndianness ( void );

//=============================================================================
//
// pgm8Read() -
//      This function reads an 8 bit pgm file.
//      (Note: it is assumed the caller knows that it is an 8-bit file)
//
bool
pgm8Read(const char*       filename,
         char*             szComment,
         int*              nrows,
         int*              ncols,
         unsigned char**   data  );

//=============================================================================
//
// pgm16Read() -
//      This function reads an 16 bit pgm file.
//      (Note: it is assumed the caller knows that it is an 16-bit file)
//
bool
pgm16Read(const char*         filename,
          char*               szComment,
          int*                nrows,
          int*                ncols,
          unsigned short**    data  );

//=============================================================================
//
// ppm8ReadPacked() -
//      This function reads an 8-bit per color per pixel ppm image.
//      The input image therefore has 24 bits per pixel.
//      However, the output is assumed to be 'RGBPacked' format,
//      ie: [B G R U][B G R U]...
//
bool
ppm8ReadPacked(const char*      filename,
               char*            szComment,
               int*             nrows,
               int*             ncols,
               unsigned char**  data );

//=============================================================================
//
// ppm8ReadRGB() -
//      This function reads an 8-bit per color per pixel ppm image.
//      The input image therefore has 24 bits per pixel.
//      However, the output is assumed to be 3 separate color channels
//
bool
ppm8ReadRGB(const char*       filename,
            char*             szComment,
            int*              nrows,
            int*              ncols,
            unsigned char**   red,
            unsigned char**   green,
            unsigned char**   blue );

//=============================================================================
// parsePPMHeader() - this function parses the header of a ppm file
// it returns in the provided variables all the information provided
// in the header:
// 	size (nrows x ncols)
//	maxval (if over 255, indicates the pixels are 2 byte pixels)
//	ascii (indicates the format of the input)
//
// Note that this function moves the current location of the file stream.
//
bool
parsePPMHeader(	  FILE*		 stream,
		  char*		 szComment,
		  int*		 nrows,
		  int* 		 ncols,
		  int*		 maxval,
		  bool*  	ascii );


//=============================================================================
// parsePGMHeader() - this function parses the header of a pgm file
// it returns in the provided variables all the information provided
// in the header:
// 	size (nrows x ncols)
//	maxval (if over 255, indicates the pixels are 2 byte pixels)
//	ascii (indicates the format of the input)
//
// Note that this function moves the current location of the file stream.
//
bool
parsePGMHeader(	  FILE*		 stream,
		  char*		 szComment,
		  int*		 nrows,
		  int* 		 ncols,
		  int*		 maxval,
		  bool*	 	 ascii );

//=============================================================================
// readPGM8BinaryData() - this function reads the data of a binary format
// 8-bit-pixel pgm file into the provided buffer
//
bool
readPGM8BinaryData( FILE * 		stream,
		    int 		nrows,
		    int  		ncols,
		    unsigned char *	data );

//=============================================================================
// readPGM8AsciiData() - this function reads the data of an ascii format
// 8-bit-pixel pgm file into the provided buffer
//
bool
readPGM8AsciiData( FILE * 		stream,
		   int 			nrows,
		   int  		ncols,
		   unsigned char *	data );

//=============================================================================
// readPPM8BinaryPacked() - this function reads the data of a binary format
// 24-bit-pixel ppm file into the provided buffer
// The data is packed into the provided buffer [B G R U][B G R U]
// Consequently there are 32 bits per pixel.
//
bool
readPPM8BinaryPacked( FILE * 		stream,
		      int 		nrows,
		      int  		ncols,
		      unsigned char *	data );


bool
readPPM24BinaryPackedBGR( FILE* 		stream,
			 int 		nrows,
			 int  		ncols,
			 unsigned char*	data );

//=============================================================================
// readPPM8AsciiPacked() - this function reads the data of an ascii format
// 24-bit-pixel ppm file into the provided buffer
// The data is packed into the provided buffer [B G R U][B G R U]
// Consequently there are 32 bits per pixel.
//
bool
readPPM8AsciiPacked( FILE * 		stream,
		     int 		nrows,
		     int  		ncols,
		     unsigned char *	data );

//=============================================================================
//
// readPPM8BinaryPackedRGB() - this function reads the data of a binary format
// 24-bit-pixel ppm file into the provided buffer
// The data is packed into the provided buffer [R G B U][R G B U]
// Consequently there are 32 bits per pixel.
//
bool
readPPM8BinaryPackedRGB( FILE* 		stream,
		      int 		nrows,
		      int  		ncols,
		      unsigned char*	data );

//=============================================================================
//
// readPPM8AsciiPacked() - this function reads the data of an ascii format
// 24-bit-pixel ppm file into the provided buffer
// The data is packed into the provided buffer [R G B U][R G B U]
// Consequently there are 32 bits per pixel.
//
bool
readPPM8AsciiPackedRGB( FILE* 		stream,
		     int 		nrows,
		     int  		ncols,
		     unsigned char*	data );


#ifdef __cplusplus
}
#endif


#endif //#ifndef _PGRPNMIO_H_
