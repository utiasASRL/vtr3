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
// $Id: pgrpnmio.cpp,v 1.10 2008-03-05 18:49:41 donm Exp $
//=============================================================================

//=============================================================================
// System Includes
//=============================================================================
#include <stdio.h>
#include <ctype.h>
#include <stdlib.h>
#include <stdarg.h>
#include <math.h>
#include <stdint.h>

//=============================================================================
// Project Includes
//=============================================================================
#include "pgrpnmio.h"

//=============================================================================
// Definitions
//=============================================================================
//#define DEBUG

//
// Disable some uncorrectable warnings so this file compiles cleanly with MSDEV
// W4.
//
#if (defined WIN32 | defined WIN64)
#pragma warning (disable: 4514) // unreferenced inline function has been removed
#pragma warning (disable: 4505) // unreferenced local function has been removed
#pragma warning (disable: 4100) // unreferenced formal parameter
#endif

static void printDebug(const char* funcName, const char* format, ... )
{
#ifdef DEBUG
		va_list args;
		fprintf( stderr, "Error:%s:\n\t", funcName );
		va_start( args, format );
		vfprintf( stderr, format, args );
		va_end( args );
		fprintf( stderr, "\n" );
#endif
	}

//
// eatComment() - this function removes all characters until it hits
// a newline.  It is called after a '#' character is encountered in
// the header.  Its purpose is to remove all characters in a comment
// line.
//
// It returns false if it encounters an end-of-file within the comment
//
static bool eatComment(FILE* stream) {
	int ch = fgetc(stream);

	while (ch != '\n') {
		ch = fgetc(stream);
		if (ch == EOF)
			return false;
	}
	return true;
}

//
// getToken() -  this function gets a whitespace delineated token out
// of the input stream and returns it in the provided buffer 'buf'
//
// 'buf' is null terminated
//
// it returns the number of characters in 'buf', or 0 if there is
// a failure
//
// failures include:
//	hitting end-of-file before a non-whitespace character
//	hitting a comment and never getting out of it
//	running out of space provided in 'buf'
//
static int getToken(FILE* stream, char* buf, int buflen) {
	// remove white space
	// read non-white space chars until a white space
	int ch = fgetc(stream);
	int i = 0;

	// read until we hit a non whitespace char
	for (;;) {
		// if white space, keep reading
		if (isspace(ch))
			ch = fgetc(stream);

		// did we run out of file?
		else if (ch == EOF)
			return 0;

		// is this the beginning of a comment?
		else if (ch == '#') {
			// read till end of comment
			if (!eatComment(stream))
				return 0;
			ch = fgetc(stream);
		}
		// we got a non-white space, non-comment character
		else
			break;
	}

	// okay, now we are at the start of the token
	while (ch != EOF && !isspace(ch)) {
		buf[i++] = (char) ch;
		if (i >= buflen - 1)
			// out of buffer
			return 0;
		ch = fgetc(stream);
	}

	// null terminate the string
	buf[i] = 0;

	return i;
}

//
// getInt() - parses a whitespace delineated integer from the stream
//
static bool getInt(FILE* stream, int* val) {
	char buf[512];

	if (!getToken(stream, buf, 512))
		return false;

	// convert token to int
	*val = atoi(buf);
	if (!*val)
		return false;
	return true;
}

//
// getByte() - parses a whitespace delineated integer from the stream
// with a value from 0 => 255 from the stream
//
static bool getByte(FILE* stream, unsigned char* val) {
	char buf[512];

	if (!getToken(stream, buf, 512))
		return false;

#if 0
	// old broken way
	// convert token to int
	*val = (unsigned char) atoi( buf );
	if ( !*val )
	return false;
#else
	// new good way...
	char* endptr;
	int ret = strtol(buf, &endptr, 10);
	if (endptr[0] != '\0')
		return false;
	*val = (unsigned char) ret;
#endif
	return true;
}

//
// getShort() - parses a whitespace delineated integer from the stream
// with a legal unsigned short value
//
static bool getShort(FILE* stream, unsigned short* val) {
	char buf[512];

	if (!getToken(stream, buf, 512))
		return false;

	// convert token to int : why in the world is this casted
        // to unsigned char ?
	*val = (unsigned char) atoi(buf);
	if (!*val)
		return false;
	return true;
}

static bool getComment(FILE* stream, char* szComment) {
	int ch;

	// make sure we have a valid pointer for a string to be stored.
	if (szComment == NULL) {
		return true;
	}

	// get all of the white space characters up to the '#'
	ch = fgetc(stream);

	while (isspace(ch) && ch != EOF) {
		ch = fgetc(stream);
	}

	// Loop and collect all comments into a single string
	char tmp[256];
	sprintf(szComment, "");
	bool done = false;
	bool foundComment = false;
	while (!done) {
		// if the first non-white space character is not a '#', put
		// it back on the stream and exit... we don't have a comment.
		if (ch != '#') {
			ungetc(ch, stream);
			done = true;
		} else {
			foundComment = true;
			// get the comment and print it to the user buffer.
			fgets(tmp, 250, stream);
			strcat(szComment, tmp);
			ch = fgetc(stream);
			while (isspace(ch) && ch != EOF) {
				ch = fgetc(stream);
			}
		}
	}
	return foundComment;
}

static bool getCommentLine(char** stream, char* szCommentLine) {
	int ch;

	// make sure we have a valid pointer for a string to be stored.
	if (szCommentLine == NULL) {
		return true;
	}

	// get all of the white space characters up to the '#'
	int indx = 0;
	ch = (int) (*stream)[indx];

	while (isspace(ch) && ch != EOF) {
		ch = (int) (*stream)[++indx];
	}

	// if the first non-white space character is not a '#', put
	// it back on the stream and exit... we don't have a comment.
	if (ch != '#') {
		return false;
	}

	//char tmp[256];
	// get the comment and print it to the user buffer.
	//fgets( tmp, 250, *stream );
	//strcat( szCommentLine, tmp );

	return true;
}

//
// ppmSidebySideto32bitPacked this function is called to convert a side by side ppm image to normal 32 bit triclops input ready image
//
bool ppmSidebySideto32bitPacked(int nrows, int ncols, unsigned char * data,
		unsigned char * data_temp) {
	int RR, GR, BR, RL, GL, BL;
	printf("data channel *data %i", *data);
	printf("data channel *(data+1) %i", *(data + 1));
	printf("data channel *(data+2) %i", *(data + 2));
	printf("data channel *(data+3) %i", *(data + 3));
	printf("data channel *data %i", *(data + 4));
	printf("data channel *(data+1) %i", *(data + 5));
	printf("data channel *(data+2) %i", *(data + 6));
	printf("data channel *(data+3) %i", *(data + 7));
	for (int row = 0; row < nrows; row++) {
		for (int col = 0; col < ncols; col++) {
			BR = *(data + (row * ncols * 2 + col) * 4);
			GR = *(data + (row * ncols * 2 + col) * 4 + 1);
			RR = *(data + (row * ncols * 2 + col) * 4 + 2);
			BL = *(data + (row * ncols * 2 + col + ncols) * 4);
			GL = *(data + (row * ncols * 2 + col + ncols) * 4 + 1);
			RL = *(data + (row * ncols * 2 + col + ncols) * 4 + 2);

			*(data_temp + ((row * ncols + col) * 4)) = 0; //(RR+GR+BR)/3;
			*(data_temp + ((row * ncols + col) * 4 + 1)) = 0; //(RL+GL+BL)/3;
			*(data_temp + ((row * ncols + col) * 4 + 2)) = 0;
			*(data_temp + ((row * ncols + col) * 4 + 3)) = 0;

		}
	}
	free(data);

	return true;
}

bool ppmIsSideBySide(const char* filename) {
	unsigned char* data;
	char szComment[200];

	int ncols, nrows;


	if (!ppm8ReadPacked(filename, szComment, &nrows, &ncols, &data)) {
		free(data);
		return false;
	}

	if (ncols > (nrows * 2)) // if this is a side by side image ,then num of cols should be greater than 2* num of rows.
			{
		free(data);
		return true;
	} else {
		free(data);
		return false;
	}
}
//
// readPGM8BinaryData() - this function reads the data of a binary format
// 8-bit-pixel pgm file into the provided buffer
//
bool readPGM8BinaryData(FILE* stream, int nrows, int ncols,
		unsigned char* data) {
	size_t len = fread(data, 1, nrows * ncols, stream);
	if ((size_t) nrows * ncols != len)
		return false;

	return true;
}

//
// readPGM8AsciiData() - this function reads the data of an ascii format
// 8-bit-pixel pgm file into the provided buffer
//
bool readPGM8AsciiData(FILE* stream, int nrows, int ncols,
		unsigned char* data) {
	for (int row = 0; row < nrows; row++) {
		for (int col = 0; col < ncols; col++) {
			if (!getByte(stream, data++)) {
				return false;
			}
		}
	}
	return true;
}

//
// readPPM8BinaryPackedRGB() - this function reads the data of a binary format
// 24-bit-pixel ppm file into the provided buffer
// The data is packed into the provided buffer [R G B U][R G B U]
// Consequently there are 32 bits per pixel.
//
bool readPPM8BinaryPackedRGB(FILE* stream, int nrows, int ncols,
		unsigned char* data) {

	for (int row = 0; row < nrows; row++) {
		for (int col = 0; col < ncols; col++) {
			// get red byte
			if (!fread(data + 0, 1, 1, stream)) {
				return false;
			}

			// get green byte
			if (!fread(data + 1, 1, 1, stream)) {
				return false;
			}

			// get blue byte
			if (!fread(data + 2, 1, 1, stream)) {
				return false;
			}

			data += 4;
		}
	}

	return true;
}

//
// readPPM8BinaryPacked() - this function reads the data of a binary format
// 24-bit-pixel ppm file into the provided buffer
// The data is packed into the provided buffer [R G B U][R G B U]
// Consequently there are 32 bits per pixel.
//
bool readPPM8BinaryPacked(FILE* stream, int nrows, int ncols,
		unsigned char* data) {

	for (int row = 0; row < nrows; row++) {
		for (int col = 0; col < ncols; col++) {
			// get red byte
			if (!fread(data + 2, 1, 1, stream)) {
				return false;
			}

			// get green byte
			if (!fread(data + 1, 1, 1, stream)) {
				return false;
			}

			// get blue byte
			if (!fread(data + 0, 1, 1, stream)) {
				return false;
			}

			data += 4;
		}
	}

	return true;
}

bool readPPM24BinaryPackedBGR(FILE* stream, int nrows, int ncols,
		unsigned char* data) {
	for (int row = 0; row < nrows; row++) {
		for (int col = 0; col < ncols; col++) {
			// get red byte
			if (!fread(data + 2, 1, 1, stream)) {
				return false;
			}

			// get green byte
			if (!fread(data + 1, 1, 1, stream)) {
				return false;
			}

			// get blue byte
			if (!fread(data + 0, 1, 1, stream)) {
				return false;
			}

			data += 3;
		}
	}

	return true;
}

//
// readPPM8AsciiPacked() - this function reads the data of an ascii format
// 24-bit-pixel ppm file into the provided buffer
// The data is packed into the provided buffer [R G B U][R G B U]
// Consequently there are 32 bits per pixel.
//
bool readPPM8AsciiPacked(FILE* stream, int nrows, int ncols,
		unsigned char* data) {
	for (int row = 0; row < nrows; row++) {
		for (int col = 0; col < ncols; col++) {
			// get red byte
			if (!getByte(stream, data + 2)) {
				return false;
			}

			// get green byte
			if (!getByte(stream, data + 1)) {
				return false;
			}

			// get blue byte
			if (!getByte(stream, data + 0)) {
				return false;
			}

			data += 4;
		}
	}
	return true;
}

//
// readPPM8AsciiPacked() - this function reads the data of an ascii format
// 24-bit-pixel ppm file into the provided buffer
// The data is packed into the provided buffer [R G B U][R G B U]
// Consequently there are 32 bits per pixel.
//
bool readPPM8AsciiPackedRGB(FILE* stream, int nrows, int ncols,
		unsigned char* data) {
	for (int row = 0; row < nrows; row++) {
		for (int col = 0; col < ncols; col++) {
			// get red byte
			if (!getByte(stream, data + 0)) {
				return false;
			}

			// get green byte
			if (!getByte(stream, data + 1)) {
				return false;
			}

			// get blue byte
			if (!getByte(stream, data + 2)) {
				return false;
			}

			data += 4;
		}
	}
	return true;
}

//
// readPPM8BinaryRGB() - this function reads the data of a binary format
// 24-bit-pixel ppm file into the provided RGB buffers
//
bool readPPM8BinaryRGB(FILE* stream, int nrows, int ncols, unsigned char* red,
		unsigned char* green, unsigned char* blue) {

	for (int row = 0; row < nrows; row++) {
		for (int col = 0; col < ncols; col++) {
			// get red byte
			if (!fread(red, 1, 1, stream)) {
				return false;
			}

			// get green byte
			if (!fread(green, 1, 1, stream)) {
				return false;
			}

			// get blue byte
			if (!fread(blue, 1, 1, stream)) {
				return false;
			}

			red++;
			green++;
			blue++;
		}
	}

	return true;
}

//
// readPPM8AsciiRGB() - this function reads the data of an ascii format
// 24-bit-pixel ppm file into the provided buffer
// The data is packed into the provided buffer [R G B U][R G B U]
// Consequently there are 32 bits per pixel.
//
bool readPPM8AsciiRGB(FILE* stream, int nrows, int ncols, unsigned char* red,
		unsigned char* green, unsigned char* blue) {
	for (int row = 0; row < nrows; row++) {
		for (int col = 0; col < ncols; col++) {
			// get red byte
			if (!getByte(stream, red)) {
				return false;
			}

			// get green byte
			if (!getByte(stream, green)) {
				return false;
			}

			// get blue byte
			if (!getByte(stream, blue)) {
				return false;
			}

			red++;
			green++;
			blue++;
		}
	}
	return true;
}

MachineEndianness 
pgrGetEndianness ( void )
{
   uint16_t            word = 0x0001;
   char              * byte;
   MachineEndianness   endianness;
   
   byte = (char*) &word;
   endianness = (*byte) ? PGR_ENDIANNESS_LITTLE : PGR_ENDIANNESS_BIG;

   return (endianness);
}

//
// readPGM16BinaryData() - this function reads the data of 16-bit-pixel
// binary format pgm file into the provided buffer
//
static bool 
readPGM16BinaryData ( FILE* stream , 
                      int nrows , int ncols ,
                      unsigned short* data ) 
{
   size_t            len_read;
   uint32_t          len_expected;
   MachineEndianness endianness;
   uint16_t          val;
   int               i;

   len_expected = nrows * ncols;
   len_read = fread( data , sizeof(uint16_t) , len_expected , stream );
   if ( len_expected != len_read ) 
   {
      return false;
   }

   /* see if we need to swap data */
   endianness = pgrGetEndianness();
   if ( PGR_ENDIANNESS_BIG == endianness )
   {
      return true;
   }
   
   /* we are running on little endian arch
      so byte swap is necessary */
   for ( i = 0 ; i < len_expected ; ++i , ++data )
   {
      val = *data;
      *data = PGR_BYTESWAP(val);
   }
   
   return true;
}

//
// readPGM16AsciiData() - this function reads the data of 16-bit-pixel
// ascii format pgm file into the provided buffer
//
static bool 
readPGM16AsciiData ( FILE * stream , 
                     int nrows , int ncols ,
                     unsigned short * data ) 
{
   uint32_t          len_expected;
   MachineEndianness endianness;
   uint16_t          val;
   uint32_t          i;
   bool              rc;
   
   len_expected = nrows * ncols;   

   endianness = pgrGetEndianness();

   if ( PGR_ENDIANNESS_BIG == endianness )
   {
      for ( i = 0 ; i < len_expected ; ++i , ++data )
      {
         rc = getShort( stream , data );
         if ( false == rc )
         {
            return false;
         }
      }

      return true;
   }
   else if ( PGR_ENDIANNESS_LITTLE == endianness )
   {
      for ( i = 0 ; i < len_expected ; ++i , ++data )
      {
         rc = getShort( stream , &val );
         if ( false == rc )
         {
            return false;
         }

         *data = PGR_BYTESWAP(val);
      }

      return true;
   }

   return false;
}

//
// pgm8Read() -
//	This function reads an 8 bit pgm file.
//	(Note: it is assumed the caller knows that it is an 8-bit file)
//
bool pgm8Read(const char* filename, char* szComment, int* nrows, int* ncols,
		unsigned char** data) {
	FILE* stream = fopen(filename, "rb");
	if (!stream) {
		// can't open file
		return false;
	}

	// now we want to parse the file to extract the PGM header
	int maxval;
	bool ascii;
	if (!parsePGMHeader(stream, szComment, nrows, ncols, &maxval, &ascii)) {
		// couldn't parse header
		//printDebug( fname, "Couldn't parse header", filename );
		return false;
	}

	if (maxval > 0xff) {
		// this is a more than 8 bit pgm
		//printDebug( fname, "Maxval is too high", filename );
		return false;
	}

	// allocate a buffer to hold this image
	*data = (unsigned char*) malloc(
			(*ncols) * (*nrows) * sizeof(unsigned char));
	if (!*data) {
		// can't allocate buffer
		//printDebug( fname, "Can't allocate image buffer", filename );
		return false;
	}

	if (ascii) {
		if (!readPGM8AsciiData(stream, *nrows, *ncols, *data)) {
			// error reading data
			//printDebug( fname, "Error reading ascii data", filename );
			free(*data);
			return false;
		}
	} else {
		if (!readPGM8BinaryData(stream, *nrows, *ncols, *data)) {
			// error reading data
			//printDebug( fname, "Error reading binary data", filename );
			free(*data);
			return false;
		}
	}

	fclose(stream);
	return true;
}

//
// pgm16Read() -
//	This function reads an 16 bit pgm file.
//	(Note: it is assumed the caller knows that it is an 16-bit file)
//
bool pgm16Read(const char* filename, char* szComment, int* nrows, int* ncols,
		unsigned short** data) {
	FILE* stream = fopen(filename, "rb");
	if (!stream) {
		// can't open file
		return false;
	}

	// now we want to parse the file to extract the PGM header
	int maxval;
	bool ascii;
	if (!parsePGMHeader(stream, szComment, nrows, ncols, &maxval, &ascii)) {
		// couldn't parse header
		//printDebug( fname, "Couldn't parse header", filename );
		return false;
	}

	if (maxval > 0xffff) {
		// this is a more than 16 bit pgm
		//printDebug( fname, "Maxval is too high", filename );
		return false;
	}

	// allocate a buffer to hold this image
	*data = (unsigned short*) malloc(
			(*ncols) * (*nrows) * sizeof(unsigned short));

	if (!*data) {
		// can't allocate buffer
		//printDebug( fname, "Can't allocate image buffer", filename );
		return false;
	}

	if (ascii) {
		if (!readPGM16AsciiData(stream, *nrows, *ncols, *data)) {
			// error reading data
			//printDebug( fname, "Error reading ascii data", filename );
			free(*data);
			return false;
		}
	} else {
		if (!readPGM16BinaryData(stream, *nrows, *ncols, *data)) {
			// error reading data
			//printDebug( fname, "Error reading binary data", filename );
			free(*data);
			return false;
		}
	}

	fclose(stream);
	return true;
}

//
// ppm8ReadPacked() -
//	This function reads an 8-bit per color per pixel ppm image.
//	The input image therefore has 24 bits per pixel.
//	However, the output is assumed to be 'RGBPacked' format,
//	ie: [R G B U][R G B U]...
//
bool ppm8ReadPacked(const char* filename, char* szComment, int* nrows,
		int* ncols, unsigned char** data) {
	//const char * fname	= "ppm8ReadPacked";

	FILE* stream = fopen(filename, "rb");
	if (!stream) {
		// can't open file
		return false;
	}

	// now we want to parse the file to extract the PGM header
	int maxval;
	bool ascii;
	if (!parsePPMHeader(stream, szComment, nrows, ncols, &maxval, &ascii)) {
		// couldn't parse header
		//printDebug( fname, "Couldn't parse header", filename );
		return false;
	}

	if (maxval > 0xff) {
		// this is a more than 8 bit pgm
		//printDebug( fname, "Maxval is too high", filename );
		return false;
	}

	// allocate a buffer to hold this image (note, since its packed...
	// need 4 bytes per pixel)
	*data = (unsigned char*) malloc(
			4 * (*ncols) * (*nrows) * sizeof(unsigned char));
	if (!*data) {
		// can't allocate buffer
		//printDebug( fname, "Can't allocate image buffer", filename );
		return false;
	}

	if (ascii) {
		if (!readPPM8AsciiPacked(stream, *nrows, *ncols, *data)) {
			// error reading data
			//printDebug( fname, "Error reading ascii data", filename );
			free(*data);
			return false;
		}
	} else {
		if (!readPPM8BinaryPacked(stream, *nrows, *ncols, *data)) {
			// error reading data
			//printDebug( fname, "Error reading binary data", filename );
			free(*data);
			return false;
		}
	}

	fclose(stream);
	return true;
}

//
// ppm8ReadRGB() -
//	This function reads an 8-bit per color per pixel ppm image.
//	The input image therefore has 24 bits per pixel.
//	However, the output is assumed to be 3 separate color channels
//
bool ppm8ReadRGB(const char* filename, char* szComment, int* nrows, int* ncols,
		unsigned char** red, unsigned char** green, unsigned char** blue) {
	FILE* stream = fopen(filename, "rb");
	if (!stream) {
		// can't open file
		return false;
	}

	// now we want to parse the file to extract the PGM header
	int maxval;
	bool ascii;
	if (!parsePPMHeader(stream, szComment, nrows, ncols, &maxval, &ascii)) {
		// couldn't parse header
		//printDebug( fname, "Couldn't parse header", filename );
		return false;
	}

	if (maxval > 0xff) {
		// this is a more than 8 bit pgm
		//printDebug( fname, "Maxval is too high", filename );
		return false;
	}

	// allocate a buffer to hold this image (note, since its packed...
	// need 4 bytes per pixel)
	*red = (unsigned char*) malloc((*ncols) * (*nrows) * sizeof(unsigned char));
	*green = (unsigned char*) malloc(
			(*ncols) * (*nrows) * sizeof(unsigned char));
	*blue = (unsigned char*) malloc(
			(*ncols) * (*nrows) * sizeof(unsigned char));

	if (!*red || !*green || !*blue) {
		// can't allocate buffer
		//printDebug( fname, "Can't allocate image buffer", filename );
		if (*red) {
			free(*red);
		}

		if (*green) {
			free(*green);
		}

		if (*blue) {
			free(*blue);
		}

		*red = NULL;
		*green = NULL;
		*blue = NULL;

		return false;
	}

	if (ascii) {
		if (!readPPM8AsciiRGB(stream, *nrows, *ncols, *red, *green, *blue)) {
			// error reading data
			//printDebug( fname, "Error reading ascii data", filename );
			free(*red);
			free(*green);
			free(*blue);
			*red = NULL;
			*green = NULL;
			*blue = NULL;
			return false;
		}
	} else {
		if (!readPPM8BinaryRGB(stream, *nrows, *ncols, *red, *green, *blue)) {
			// error reading data
			//printDebug( fname, "Error reading binary data", filename );
			free(*red);
			free(*green);
			free(*blue);
			*red = NULL;
			*green = NULL;
			*blue = NULL;
			return false;
		}
	}

	fclose(stream);
	return true;
}

//
// parsePGMHeader() - this function parses the header of a pgm file
// it returns in the provided variables all the information provided
// in the header:
// 	size (nrows x ncols)
//	maxval (if over 255, indicates the pixels are 2 byte pixels)
//	ascii (indicates the format of the input)
//
//
bool parsePGMHeader(FILE* stream, char* szComment, int* nrows, int* ncols,
		int* maxval, bool* ascii) {
	// header looks like
	// <P5/P2> [ws] <ncols> [ws] <nrows> [ws] <maxval> [1 ws] data
	// between P5/P2 and maxval, comment lines can be inserted if there is
	// a '#' character in the first column of the line
	//const char * fname	= "parsePGMHeader";

	int ch;

	// read the magic identifier
	if (fgetc(stream) != 'P') {
		return false;
	}

	ch = fgetc(stream);
	if (ch == '2') {
		*ascii = true;
	} else if (ch == '5') {
		*ascii = false;
	} else {
		//printDebug( fname, "Header didnot contain PGM identifier" );
		return false;
	}

	// try and get the comment if there is one.
	getComment(stream, szComment);

	// get ncols
	if (!getInt(stream, ncols)) {
		//printDebug( fname, "Parse failed on 'ncols'" );
		return false;
	}

	// get nrows
	if (!getInt(stream, nrows)) {
		//printDebug( fname, "Parse failed on 'nrows'" );
		return false;
	}

	// get maxval
	if (!getInt(stream, maxval)) {
		//printDebug( fname, "Parse failed on 'maxval'" );
		return false;
	}

	return true;
}

//
// parsePPMHeader() - this function parses the header of a ppm file
// it returns in the provided variables all the information provided
// in the header:
// 	size (nrows x ncols)
//	maxval (if over 255, indicates the pixels are 2 byte pixels)
//	ascii (indicates the format of the input)
//
bool parsePPMHeader(FILE* stream, char* szComment, int* nrows, int* ncols,
		int* maxval, bool* ascii) {
	// header looks like
	// <P5/P2> [ws] <ncols> [ws] <nrows> [ws] <maxval> [1 ws] data
	// between P5/P2 and maxval, comment lines can be inserted if there is
	// a '#' character in the first column of the line
	//const char * fname	= "parsePPMHeader";

	int ch;

	// read the magic identifier
	if (fgetc(stream) != 'P')
		return false;

	ch = fgetc(stream);

	if (ch == '3') {
		*ascii = true;
	} else if (ch == '6') {
		*ascii = false;
	} else {
		//printDebug( fname, "Header didnot contain PGM identifier" );
		return false;
	}

	// try and get the comment;
	getComment(stream, szComment);

	// get ncols
	if (!getInt(stream, ncols)) {
		//printDebug( fname, "Parse failed on 'ncols'" );
		return false;
	}

	// get nrows
	if (!getInt(stream, nrows)) {
		//printDebug( fname, "Parse failed on 'nrows'" );
		return false;
	}

	// get maxval
	if (!getInt(stream, maxval)) {
		//printDebug( fname, "Parse failed on 'maxval'" );
		return false;
	}

	return true;
}

