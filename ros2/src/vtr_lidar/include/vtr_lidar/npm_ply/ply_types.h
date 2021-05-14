/*----------------------------------------------------------------------------*\
  NPM Toolbox: Various functions and classes for point cloud processing
--------------------------------------------------------------------------------
Author(s): Hassan Bouchiba
Creation: 09 Nov 2015
Description: This file contains auxiliary enums and functions for PLY I/O.
\*----------------------------------------------------------------------------*/

#ifndef NPM_TOOLBOX_IO_PLY_TYPES_H_
#define NPM_TOOLBOX_IO_PLY_TYPES_H_

#include <string>


namespace npm {

/*----------------------------------------------------------------------------*\
  enums definition 
\*----------------------------------------------------------------------------*/

// PLY internal data format
enum PLYFormat { 
  PLY_BINARY_LITTLE_ENDIAN, 
  PLY_BINARY_BIG_ENDIAN, 
  PLY_ASCII 
};

// Basic type of a PLY property
enum PLYType { 
  PLY_CHAR, 
  PLY_UCHAR,
  PLY_SHORT,
  PLY_USHORT,
  PLY_INT,
  PLY_UINT,
  PLY_FLOAT,
  PLY_DOUBLE 
};


/*----------------------------------------------------------------------------*\
  static functions declaration 
\*----------------------------------------------------------------------------*/

// Return the string of each type handled by PLY format 
static std::string typeStr(PLYType type);
// Return the PLY type associated with the input string
static PLYType strType(std::string str);
// Return the size of each type handled by PLY format
static size_t typeSize(PLYType type);

}  // namespace npm




/*----------------------------------------------------------------------------*\
  static functions definition 
\*----------------------------------------------------------------------------*/

#include <cassert>


inline
std::string npm::typeStr(PLYType type) {
  switch (type) {
    case PLY_CHAR: {return "char";}
    case PLY_UCHAR: {return "uchar";}
    case PLY_SHORT: {return "short";}
    case PLY_USHORT: {return "ushort";}
    case PLY_INT: {return "int";}
    case PLY_UINT: {return "uint";}
    case PLY_FLOAT: {return "float";}
    case PLY_DOUBLE: {return "double";}
    default: {assert(false); return "unknown";}
  }
}

inline
npm::PLYType npm::strType(std::string str) {
	if (str.compare("char") == 0) {return PLY_CHAR;} 
  else if (str.compare("uchar") == 0) {return PLY_UCHAR;} 
  else if (str.compare("short") == 0) {return PLY_SHORT;} 
  else if (str.compare("ushort") == 0) {return PLY_USHORT;} 
  else if (str.compare("int") == 0) {return PLY_INT;}	
  else if (str.compare("uint") == 0) {return PLY_UINT;}
  else if ((str.compare("float32") == 0) | 
           (str.compare("float") == 0)) {return PLY_FLOAT;} 
  else if ((str.compare("float64") == 0) | 
           (str.compare("double") == 0)) {return PLY_DOUBLE;} 
  else {assert(false); return PLY_INT;}
}

inline
size_t npm::typeSize(PLYType type) {
  switch (type) {
    case PLY_CHAR: {return 1;}
    case PLY_UCHAR: {return 1;}
    case PLY_SHORT: {return 2;}
    case PLY_USHORT: {return 2;}
    case PLY_INT: {return 4;}
    case PLY_UINT: {return 4;}
    case PLY_FLOAT: {return 4;}
    case PLY_DOUBLE: {return 8;}
    default: {assert(false); return 4;}
  }
}

#endif  // NPM_TOOLBOX_IO_PLY_TYPES_H_