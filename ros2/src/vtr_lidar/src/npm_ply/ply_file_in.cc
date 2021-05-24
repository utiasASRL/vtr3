/*----------------------------------------------------------------------------*\
  NPM Toolbox: Various functions and classes for point cloud processing
--------------------------------------------------------------------------------
Author(s): Hassan Bouchiba
Creation: 04 Nov 2015

TODO(Hassan): here read function is twice slower than previous implementation 
because we basically do the job twice : we fill a big binary buffer with data
and then we parse data into smaller buffers. May be this second step is not 
needed.
\*----------------------------------------------------------------------------*/


#include <cstdint>

#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include "vtr_lidar/npm_ply/ply_file_in.h"
#include "vtr_lidar/npm_ply/ply_types.h"

uint64_t npm::PLYFileIn::kReadBufferSize = 16000;

bool npm::PLYFileIn::read(uint64_t* num_pt, std::vector<PLYType>* types,
    std::vector<std::string>* properties) {
  using std::cout; using std::endl;
  

  // Initialize input stream
  // ------------------------------------------------------------------------ //
  std::ifstream file_in(file_path_, std::ios::binary);
  if (!file_in.good()) {
    cout
    << "[PLYFileIn] error: can't open " << file_path_ 
    << " in input mode" << endl; 
    return false;
  }


  // Read and parse header
  // ------------------------------------------------------------------------ //
  std::string line("");
  std::getline(file_in, line);
  std::string magic_word = std::string(line, 0, 3);

  if (magic_word.compare("ply") != 0) {
    cout 
    << "[PLYFileIn] error: magic word ply not found (found instead " 
    <<  magic_word << ") this file is probably not PLY" << endl;
    return false;
  }

	while (std::string(line, 0, 10).compare("end_header") != 0) {
		std::getline(file_in, line);
		header_ += line + "\n";
	}
  

  if (!parseHeader()) return false;
  

  // Read file data
  // ------------------------------------------------------------------------ //  
  for (auto t: types_) {fields_sizes_.push_back(typeSize(t));}

	uint64_t point_size(0);
  for (auto s: fields_sizes_) {point_size += s;}
  uint64_t total_data_size = point_size * num_points_;

  char* data_buffer = new char[total_data_size];


	switch (internal_format_)	{
	  case PLY_BINARY_LITTLE_ENDIAN: {
		  uint64_t n = total_data_size / kReadBufferSize;
		  uint64_t r = total_data_size % kReadBufferSize;

		  for (uint64_t i(0); i < n; ++i) {
			  file_in.read(data_buffer + i*kReadBufferSize, kReadBufferSize);
		  }
		  file_in.read(data_buffer + n*kReadBufferSize, r);

		  break;
	  }
	  case PLY_BINARY_BIG_ENDIAN: {
      cout 
      << "[PLYFileIn] error: function not implemented yet for BBE format" 
      << std::endl;
      return false;
	  }
    case PLY_ASCII: {
      uint64_t buffer_offset(0);
      std::string line("");

      for (uint64_t i(0); i<num_points_; ++i) {
        std::getline(file_in, line);
        std::stringstream line_stream(line);

        buffer_offset = i * point_size;
        for (auto& type : types_) {
          // Remark:
          // if we read ASCII data with
          // if(type==PLYType::kChar)
          //   line_stream >> *reinterpret_cast<int8_t*>
          //   (data_buffer+buffer_offset);
          // (Which is what we do with floating point types).
          // it does not work because line_stream will understand 'read next 
          // ASCII character and not "read next uint8_t integer" as we can
          // expect.

          if((type==PLY_CHAR)  | (type==PLY_UCHAR)  | 
             (type==PLY_SHORT) | (type==PLY_USHORT) | 
             (type==PLY_INT)   | (type==PLY_UINT)) {
            int interger_tmp;
            line_stream >> interger_tmp;  
            if(type==PLY_CHAR)
              *reinterpret_cast<int8_t*>(data_buffer+buffer_offset) = 
              static_cast<int8_t>(interger_tmp);
            if(type==PLY_UCHAR)
              *reinterpret_cast<uint8_t*>(data_buffer+buffer_offset) = 
              static_cast<uint8_t>(interger_tmp);
            if(type==PLY_SHORT)
              *reinterpret_cast<int16_t*>(data_buffer+buffer_offset) = 
              static_cast<int16_t>(interger_tmp);
            if(type==PLY_USHORT)
              *reinterpret_cast<uint16_t*>(data_buffer+buffer_offset) = 
              static_cast<uint16_t>(interger_tmp);
            if(type==PLY_INT)
              *reinterpret_cast<int32_t*>(data_buffer+buffer_offset) = 
              static_cast<int32_t>(interger_tmp);
            if(type==PLY_UINT)
              *reinterpret_cast<uint32_t*>(data_buffer+buffer_offset) = 
              static_cast<uint32_t>(interger_tmp);         
          } else if (type==PLY_FLOAT) {
            line_stream >> *reinterpret_cast<float*>(data_buffer+buffer_offset);
          } else if (type==PLY_DOUBLE) {
            line_stream >> *reinterpret_cast<double*>(data_buffer+buffer_offset);
          }

          buffer_offset += typeSize(type);
        }
      }      
      break;
    }
    default: {
      assert(false);
    }
	}

  // Parse data into fields_
  for (auto& f: fields_) {delete[] f;}
  std::vector<char*>().swap(fields_);
  for (auto& s: fields_sizes_) {
    char* tmp_field = nullptr;
    tmp_field = new char[s*num_points_];
    fields_.push_back(tmp_field);
  }

  uint64_t num_fields = fields_.size();
  uint64_t buffer_offset(0);
  uint64_t field_offset(0);

  for (uint64_t i(0); i<num_points_; ++i) {
    buffer_offset = i * point_size;
    
    for (uint64_t k(0); k<num_fields; ++k) {
      field_offset = i * fields_sizes_[k];

      for (size_t j(0); j<fields_sizes_[k]; ++j) {
        fields_[k][field_offset] = data_buffer[buffer_offset];
        ++buffer_offset;
        ++field_offset;
      }
    } 
  }

  delete[] data_buffer;


  // Fill return values
  // ------------------------------------------------------------------------ // 
  if (num_pt != nullptr) {*num_pt = num_points_;}
  if (types != nullptr) {*types = types_;}
  if (properties != nullptr) {*properties = properties_;}
  

  return true;      
}

bool npm::PLYFileIn::parseHeader() {
  using std::cout; using std::endl;


  std::stringstream header_stream(header_);
  std::streamoff tmp_file_position(0);
  std::string token("");

  while (!header_stream.eof()) {
    header_stream >> token;

    if (token.compare("format") == 0)	{
      header_stream >> token;
      
      if (token.compare("binary_little_endian") == 0) {
        internal_format_ = PLY_BINARY_LITTLE_ENDIAN;
      } else if (token.compare("binary_big_endian") == 0) {
        internal_format_ = PLY_BINARY_BIG_ENDIAN;
      } else if (token.compare("ascii") == 0) {
        internal_format_ = PLY_ASCII;
      } else {
        cout 
        << "[PLYFileIn] warning: unknown format (" << token << ")" << endl;
      }
    }

    if (token.compare("element") == 0) {
      header_stream >> token;

      if (token.compare("vertex") == 0) {
        header_stream >> num_points_;
				
        header_stream >> token;
        while (token.compare("property") == 0) {
          header_stream >> token;
          types_.push_back(strType(token));
          header_stream >> token;
          properties_.push_back(token);
          tmp_file_position = header_stream.tellg();
          header_stream >> token;
        }

        // get back to before the last element extracted
        header_stream.seekg(tmp_file_position); 
      } else {
        cout 
        << "[PLYFileIn] warning: : PLY file contain unhandled data (" 
        << token << ")" << endl;
      }

    } 
  }


  return true;
}