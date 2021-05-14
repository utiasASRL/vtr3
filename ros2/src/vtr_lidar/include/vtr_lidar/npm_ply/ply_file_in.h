/*----------------------------------------------------------------------------*\
  NPM Toolbox: Various functions and classes for point cloud processing
--------------------------------------------------------------------------------
Author(s): Hassan Bouchiba
Creation: 04 Nov 2015
\*----------------------------------------------------------------------------*/

#ifndef NPM_TOOLBOX_IO_PLY_FILE_IN_H_
#define NPM_TOOLBOX_IO_PLY_FILE_IN_H_

#include <cstdint>

#include <iostream>
#include <string>
#include <vector>

#include "ply_types.h"


namespace npm {

/*----------------------------------------------------------------------------*\
  class PLYFileIn declaration 
\*----------------------------------------------------------------------------*/

// Class to handle point cloud PLY file input
// Warining : This class does not handle other information that may be present 
// in the PLY file (e.g : triangles).
class PLYFileIn {
public:
	static uint64_t kReadBufferSize;

  // Constructor.
	PLYFileIn(std::string file_path);
  // Destructor.
  ~PLYFileIn();
  // Read point cloud data from PLY file this data. At this point data is only
  // stored internally in this class. Data must then be retrieved with 
  // PopNextField() function.
	bool read(uint64_t* num_pt=nullptr, std::vector<PLYType>* types=nullptr,
      std::vector<std::string>* properties=nullptr);
  // Pop dim-dimentional field. A field is componed by multiple scalar fields
  // of the same type.
  template<typename T>
  bool getField(int field_index, int dim, std::vector<T>& data);
  // Returns file header.
  std::string header() const;
  // Returns internal format.
  PLYFormat internal_format() const;
  // Modify binary read buffer size in bytes.
  static void set_read_buffer_size(uint64_t read_buffer_size);

 private:
  // Auxiliary function to parse the PLY header header_.
	bool parseHeader();

 private:
  std::string file_path_;
	PLYFormat	internal_format_;
  uint64_t num_points_;
  std::string header_;
  std::vector<std::string> properties_;
  std::vector<PLYType> types_;
  std::vector<char*> fields_;
  std::vector<size_t> fields_sizes_;
};

}  // namespace npm




/*----------------------------------------------------------------------------*\
  class PLYFileIn definition (inline functions) 
\*----------------------------------------------------------------------------*/

inline
npm::PLYFileIn::PLYFileIn(std::string file_path) 
    : file_path_(file_path),
      internal_format_(PLY_BINARY_LITTLE_ENDIAN),
      num_points_(0),
      header_("") {}

inline
npm::PLYFileIn::~PLYFileIn() {
  for (auto& f: fields_) {delete[] f;}
}

inline
std::string npm::PLYFileIn::header() const {
  return header_;
}

inline
npm::PLYFormat npm::PLYFileIn::internal_format() const {
  return internal_format_;
}

inline
void npm::PLYFileIn::set_read_buffer_size(uint64_t read_buffer_size) {
  kReadBufferSize = read_buffer_size;
}

template<typename T>
inline
bool npm::PLYFileIn::getField(int field_index, int dim, std::vector<T>& data) {
  using std::cout; using std::endl;


  // Perform checks on input data
  // ------------------------------------------------------------------------ //
  if ((field_index + dim) > properties_.size()) {
    cout 
    << "[PLYFileIn] error: mismatch between requested field dimension (" 
    << dim << ") and remaining fields to read in PLY file ("
    << properties_.size()-field_index  << "/" << properties_.size() 
    << ")" << endl;
    return false;
  }

  size_t input_vec_size(sizeof(T));
  size_t expected_vec_size(0);
  for (int i(0); i<dim; ++i) {
    expected_vec_size += fields_sizes_[field_index+i];
  }
  if (input_vec_size != expected_vec_size) {
    cout 
    << "[PLYFileIn] error: mismatch between input field size (" 
    << input_vec_size << ") (sizeof (T) for std::vector<T>) and expected "
    << "size (" << expected_vec_size  << ")" << endl;
    return false;
  }


  // Fill input vector
  // ------------------------------------------------------------------------ //
  data.resize(num_points_);
  char* in_data_tab = reinterpret_cast<char*>(data.data());

  uint64_t in_data_offset(0);
  uint64_t field_offset(0);

  for (uint64_t i(0); i<num_points_; ++i) {
    in_data_offset = i * input_vec_size;

    for (uint64_t j(0); j<dim; ++j) {
      size_t field_ind = field_index + j;
      size_t field_size = fields_sizes_[field_ind];
      field_offset = i * field_size;

      for (uint64_t k(0); k<field_size; ++k) {
        in_data_tab[in_data_offset] = fields_[field_ind][field_offset];
        ++in_data_offset;
        ++field_offset;
      }
      
    }
  }


  // delete used scalar fields.
  for (uint64_t j(0); j<dim; ++j) {
    size_t field_ind = field_index + j;
    delete[] fields_[field_ind];
    fields_[field_ind] = nullptr;
  }

  return true;
}

#ifdef NPM_UNCOMMENT
// PLYFileIn class definition
template<typename... Targs>
bool npm::PLYFileIn::Read(Targs&... vectors) {
  // ----- Check file stream state ------------------------------------------ //
  switch (state_)	{
    case State::kError: {
      return false;
    }
    case State::kOpen: {
      if (!ReadHeader()) 
        return false;
      break;
    }
    case State::kHeaderRead: {
      // if ReadHeader() function have been already called
      break;
    }
    case State::kFileRead: {
      std::cout << "Error: file already read" << std::endl;
      return false;
    }
  }


  // ----- Retrieve vectors meta-information -------------------------------- //
  std::vector<size_t> vector_sizes;  // sizeof(T) for each vector
  std::vector<char*> vector_data;    // data() of each vector
  size_t num_vectors = sizeof...(vectors);
  AllocateMemoryAndGatherData(num_points_, vector_sizes, 
                              vector_data, vectors...);

  // Check if input vectors match file data
  size_t vec_point_size_in_bytes = 0;
  for (auto& s:vector_sizes) {
    vec_point_size_in_bytes += s;
  }

  size_t file_point_size_in_bytes = 0;
  for (auto& t:types_) {
    file_point_size_in_bytes += TypeSize(t);
  }
  size_t file_data_size_in_bytes = file_point_size_in_bytes * num_points_;

  if (vec_point_size_in_bytes > file_point_size_in_bytes) {
    std::cout << "Error: provided vectors do not match the input file data "
              << "(file point size "
              << file_point_size_in_bytes 
              << " < vector point size "
              << vec_point_size_in_bytes << ")" << std::endl;
    state_ = State::kError;
    return false;
  }


  // ----- Read data from file ---------------------------------------------- //
	char* data_buffer = new char[file_data_size_in_bytes];

  // Copy data in data_buffer
	switch (format_)	{
    // ---------------------------------------------------------------------- //
	  case PLYFormat::kBinaryLittleEendian: {
		  uint64_t n = file_data_size_in_bytes / kReadBufferSize;
		  uint64_t r = file_data_size_in_bytes % kReadBufferSize;

		  for (uint64_t i(0); i < n; ++i) {
			  file_in_.read(data_buffer + i*kReadBufferSize, kReadBufferSize);
		  }
		  file_in_.read(data_buffer + n*kReadBufferSize, r);

		  break;
	  }
    // ---------------------------------------------------------------------- //
	  case PLYFormat::kBinaryBigEendian: {
      std::cout << "Error: function not implemented yet for BBE format" 
                << std::endl;
      state_ = State::kError;
      return false;
	  }
    // ---------------------------------------------------------------------- //
    case PLYFormat::kASCII: {
      uint64_t buffer_offset(0);
      std::string line("");

      for (uint64_t i(0); i<num_points_; ++i) {
        std::getline(file_in_, line);
        std::stringstream line_stream(line);

        buffer_offset = i * file_point_size_in_bytes;
        for (auto& type:types_) {
          // Remark:
          // if we read ASCII data with
          // if(type==PLYType::kChar)
          //   line_stream >> *reinterpret_cast<int8_t*>(data_buffer+buffer_offset);
          // it does not work because line_stream will understand 'read next 
          // ASCII character and not "read next uint8_t integer" as we can
          // expect
          int interger_tmp;
          if((type==PLYType::kChar) | (type==PLYType::kUChar) | 
             (type==PLYType::kShort) | (type==PLYType::kUShort) | 
             (type==PLYType::kInt) | (type==PLYType::kUInt)) {
            line_stream >> interger_tmp;  
          }
          if(type==PLYType::kChar)
            *reinterpret_cast<int8_t*>(data_buffer+buffer_offset) = static_cast<int8_t>(interger_tmp);
          if(type==PLYType::kUChar)
            *reinterpret_cast<uint8_t*>(data_buffer+buffer_offset) = static_cast<uint8_t>(interger_tmp);
          if(type==PLYType::kShort)
            *reinterpret_cast<int16_t*>(data_buffer+buffer_offset) = static_cast<int16_t>(interger_tmp);
          if(type==PLYType::kUShort)
            *reinterpret_cast<uint16_t*>(data_buffer+buffer_offset) = static_cast<uint16_t>(interger_tmp);
          if(type==PLYType::kInt)
            *reinterpret_cast<int32_t*>(data_buffer+buffer_offset) = static_cast<int32_t>(interger_tmp);
          if(type==PLYType::kUInt)
            *reinterpret_cast<uint32_t*>(data_buffer+buffer_offset) = static_cast<uint32_t>(interger_tmp);

          if(type==PLYType::kFloat)
            line_stream >> *reinterpret_cast<float*>(data_buffer+buffer_offset);
          if(type==PLYType::kDouble)
            line_stream >> *reinterpret_cast<double*>(data_buffer+buffer_offset);
          buffer_offset += TypeSize(type);
        }
      }      
      break;
    }
    // ---------------------------------------------------------------------- //
    default: {
      assert(false);
    }
	}

  // Parse data into vectors
  uint64_t buffer_offset(0);
  uint64_t vector_offset(0);

  for (uint64_t i(0); i<num_points_; ++i) {
    buffer_offset = i * file_point_size_in_bytes;
    for (size_t k(0); k<num_vectors; ++k) {
      vector_offset = i * vector_sizes[k];
      for (size_t j(0); j<vector_sizes[k]; ++j) {
        vector_data[k][vector_offset] = data_buffer[buffer_offset];
        ++buffer_offset;
        ++vector_offset;
      }
    } 
  }

  delete[] data_buffer;


  state_ = State::kFileRead;
	return true;  
}

template<typename T>
inline
void npm::PLYFileIn::AllocateMemoryAndGatherData(
    uint64_t num_points,
    std::vector<size_t>& vec_sizes,
    std::vector<char*>& vec_data,
    std::vector<T>& first_vec) {
  first_vec.resize(num_points);
  vec_sizes.push_back(sizeof(T));
  vec_data.push_back(reinterpret_cast<char*>(first_vec.data()));
}

template<typename T, typename... Targs>
inline
void npm::PLYFileIn::AllocateMemoryAndGatherData(
    uint64_t num_points,
    std::vector<size_t>& vec_sizes,
    std::vector<char*>& vec_data,
    std::vector<T>& first_vec,
    Targs&... other_vec) {
  first_vec.resize(num_points);
  vec_sizes.push_back(sizeof(T));
  vec_data.push_back(reinterpret_cast<char*>(first_vec.data()));
  AllocateMemoryAndGatherData(num_points, vec_sizes, vec_data, other_vec...);
}
#endif  // NPM_UNCOMMENT


#endif  // NPM_TOOLBOX_IO_PLY_FILE_IN_H_