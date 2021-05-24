/*----------------------------------------------------------------------------*\
  NPM Toolbox: Various functions and classes for point cloud processing
--------------------------------------------------------------------------------
Author(s): Hassan Bouchiba
Creation: 04 Nov 2015

TODO(hassan): modify binary file writing in order to reduce memory footprint.
  Instanciate only one small buffer to gather data instead of one big buffer
  and swap between filling and writing.
TODO(hassan): find a way to modify ASCII writing operations wrinting on a
  buffer (may be use other native c function for numbers formatting) and then
  wrinting on the file (as binary operations).
TODO(hassan): add verifications when file already have been written and unbind
  data pointers when Write() is called.
\*----------------------------------------------------------------------------*/


#include <cassert>
#include <cstdint>
#include <cstdio>

#include <fstream>
#include <initializer_list>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include "vtr_lidar/npm_ply/ply_file_out.h"
#include "vtr_lidar/npm_ply/ply_types.h"


uint64_t npm::PLYFileOut::kWriteBufferSize = 16000;
uint64_t npm::PLYFileOut::kASCIIWritePrecision = 15;

npm::PLYFileOut::PLYFileOut(std::string file_path, PLYFormat format)
    : file_path_(file_path),
      internal_format_(format),
      //open_(false),
      num_points_(0),
      header_("") {}

bool npm::PLYFileOut::write() {
  using std::cout; using std::endl;


  // Initialize output stream
  // ------------------------------------------------------------------------ //
  std::ofstream file_out(file_path_, std::ios::binary);
  if (!file_out.good()) {
    cout
    << "[PLYFileOut] error: can't open " << file_path_
    << " in output mode" << endl;
    return false;
  }


  // Write header
  // ------------------------------------------------------------------------ //
  genHeader();
  file_out << header_;


  // Merge vectors data into one single binary buffer
  // ------------------------------------------------------------------------ //
  uint64_t point_size(0);
  for (auto s: data_sizes_) {point_size += s;}
  uint64_t total_data_size = point_size * num_points_;

  char* data_buffer = new char[total_data_size];

  uint64_t num_data_entries = data_.size();
  uint64_t buffer_offset(0);
  for (uint64_t i(0); i<num_points_; ++i) {
    for (size_t k(0); k<num_data_entries; ++k) {
      uint64_t vector_offset = i * data_sizes_[k];
      for (size_t j(0); j<data_sizes_[k]; ++j) {
        data_buffer[buffer_offset] = data_[k][vector_offset];
        ++buffer_offset;
        ++vector_offset;
      }
    }
  }


  // Write buffer data
  // ------------------------------------------------------------------------ //
  switch (internal_format_) {
    case PLY_BINARY_LITTLE_ENDIAN: {
      uint64_t n = total_data_size / kWriteBufferSize;
      uint64_t r = total_data_size % kWriteBufferSize;

      for (uint64_t i(0); i < n; i++) {
	      file_out.write(data_buffer + i*kWriteBufferSize, kWriteBufferSize);
      }
      file_out.write(data_buffer + n*kWriteBufferSize, r);

      break;
    }
    case PLY_BINARY_BIG_ENDIAN: {
      cout
      << "[PLYFileOut] error: function not implemented yet for BBE format"
      << endl;
      return false;
    }
    case PLY_ASCII: {
      file_out.precision(kASCIIWritePrecision);

      uint64_t buffer_offset(0);
      for (uint64_t i(0); i<num_points_; ++i) {
        buffer_offset = i * point_size;

        for (auto& type : types_) {
          if(type==PLY_CHAR)
            file_out << +*reinterpret_cast<int8_t*>(data_buffer+buffer_offset);
          if(type==PLY_UCHAR)
            file_out << +*reinterpret_cast<uint8_t*>(data_buffer+buffer_offset);
          if(type==PLY_SHORT)
            file_out << *reinterpret_cast<int16_t*>(data_buffer+buffer_offset);
          if(type==PLY_USHORT)
            file_out << *reinterpret_cast<uint16_t*>(data_buffer+buffer_offset);
          if(type==PLY_INT)
            file_out << *reinterpret_cast<int32_t*>(data_buffer+buffer_offset);
          if(type==PLY_UINT)
            file_out << *reinterpret_cast<uint32_t*>(data_buffer+buffer_offset);
          if(type==PLY_FLOAT)
            file_out << *reinterpret_cast<float*>(data_buffer+buffer_offset);
          if(type==PLY_DOUBLE)
            file_out << *reinterpret_cast<double*>(data_buffer+buffer_offset);
          file_out << " ";
          buffer_offset += typeSize(type);
        }

        file_out << std::endl;
      }

      break;
    }
    default: {
      assert(false);
    }
  }

  delete[] data_buffer;


	return true;
}

void npm::PLYFileOut::genHeader() {
  header_ = "";
  std::stringstream header_stream;

  header_stream
  << "ply" << std::endl
  << "format ";

  switch (internal_format_) {
    case PLY_BINARY_LITTLE_ENDIAN: {
      header_stream << "binary_little_endian 1.0" << std::endl;
      break;
    }
    case PLY_BINARY_BIG_ENDIAN: {
      header_stream << "binary_big_endian 1.0" << std::endl;
      break;
    }
    case PLY_ASCII: {
      header_stream << "ascii 1.0" << std::endl;
      break;
    }
  }

  header_stream << "element vertex " << num_points_ << std::endl;

  size_t num_properties = properties_.size();
  for (uint64_t i(0); i<num_properties; ++i) {
    header_stream
    << "property " + typeStr(types_[i]) + " " + properties_[i] << std::endl;
  }

  header_stream << "end_header" << std::endl;


  header_ = header_stream.str();
}
