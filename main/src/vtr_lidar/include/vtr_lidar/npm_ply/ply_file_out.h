/*----------------------------------------------------------------------------*\
  NPM Toolbox: Various functions and classes for point cloud processing
--------------------------------------------------------------------------------
Author(s): Hassan Bouchiba
Creation: 04 Nov 2015
\*----------------------------------------------------------------------------*/

#ifndef NPM_TOOLBOX_IO_PLY_FILE_OUT_H_
#define NPM_TOOLBOX_IO_PLY_FILE_OUT_H_

#include <cstdint>

#include <initializer_list>
#include <string>
#include <vector>

#include "ply_types.h"


namespace npm {

/*----------------------------------------------------------------------------*\
  class PLYFileOut declaration 
\*----------------------------------------------------------------------------*/

// Class to handle point cloud PLY file output
// Warining : This class does not handle other information that may be present 
// in the PLY file (e.g : triangles) 
// Usage:
//   std::vector<Eigen::Vector3f> points;
//   std::vector<double> ts;
//
//   /*code here to fill the two vectors*/
//
//   uint64_t n = points.size();
//   PLYFileOut out(path);
//   out.pushField(n, 3, PLY_FLOAT,  {"x", "y", "z"}, points);
//   out.pushField(n, 1, PLY_DOUBLE, {"ts"}, ts);
//   out.write();
class PLYFileOut {
public:
  static uint64_t kASCIIWritePrecision;
	static uint64_t kWriteBufferSize;

  // Constructor
	PLYFileOut(std::string file_path, PLYFormat format=PLY_BINARY_LITTLE_ENDIAN);
  // Push data field by field. A field is componed by multiple scalar fields
  // of the same type.
  template<typename T>
  bool pushField(uint64_t num_pt, int dim, PLYType type,
      std::initializer_list<std::string> properties, std::vector<T>& data);
  // Writes data to external PLY file.
  bool write();
  // Returns file header.
  std::string header() const;
  // Modify to internal format.
  void set_internal_format(PLYFormat new_format);
  // Modify number of digits written for PLY_ASCII internal.
  static void set_ascii_write_precision(uint64_t ascci_write_prec);
  // Modify binary write buffer size in bytes.
  static void set_write_buffer_size(uint64_t write_buffer_size);

 private:
  // Auxiliary function to generate the PLY file header.
	void genHeader();

 private:
  std::string file_path_;
	PLYFormat	internal_format_;
  //bool open_;
  uint64_t num_points_;
  std::string header_;
  std::vector<std::string> properties_;
  std::vector<PLYType> types_;
  // TODO(hassan): change the name data_ into field_ (and field_sizes_).
  std::vector<char*> data_;
  std::vector<size_t> data_sizes_;
};

}  // namespace npm




/*----------------------------------------------------------------------------*\
  class PLYFileOut definition (inline functions)
\*----------------------------------------------------------------------------*/

#include <cassert>

#include <iostream>


template<typename T>
inline
bool npm::PLYFileOut::pushField(uint64_t num_pt, int dim, PLYType type,
    std::initializer_list<std::string> properties, std::vector<T>& data) {
  using std::cout; using std::endl;


  // Perform checks on input data sizes
  // ------------------------------------------------------------------------ //
  if (num_points_ == 0) {num_points_ = num_pt;}
  size_t in_num_props = properties.size();
  size_t in_expected_data_size = dim * typeSize(type) * num_pt;
  size_t in_data_size = sizeof(T) * data.size();
  
  if (num_pt == 0) {
    cout
    << "[PLYFileOut] error: field {";
    for(auto& p: properties) {cout << p << " ";}
    cout << "}" << endl;
    cout
    << "Null number of points" << endl;
    return false;
  }

  if (num_pt != num_points_) {
    cout
    << "[PLYFileOut] error: field {";
    for(auto& p: properties) {cout << p << " ";}
    cout << "}" << endl;
    cout
    << "Mismatch between previously specified number of "
    << "points (" << num_points_ << ") and requested number of points for "
    << "this field (" << num_pt << ")" << endl;
    return false;
  }

  if ((size_t)dim != in_num_props) {
    cout
    << "[PLYFileOut] error: field {";
    for(auto& p: properties) {cout << p << " ";}
    cout << "}" << endl;
    cout
    << "Mismatch between specified dimension (" << dim 
    << ") and number of input properties of this field (" 
    << properties.size() << ")" << endl;
    return false;  
  }

  if (in_expected_data_size != in_data_size) {
    cout
    << "[PLYFileOut] error: field {";
    for(auto& p: properties) {cout << p << " ";}
    cout << "}" << endl;
    cout
    << "Mismatch between expected data size (" 
    << in_expected_data_size << ") and number provided data size for this "
    << "field("<< in_data_size << ")" << endl;
    return false;  
  }


  // Save data internally
  // ------------------------------------------------------------------------ //
  for (auto& p: properties) {properties_.push_back(p);}
  for (int k(0); k<dim; ++k) {types_.push_back(type);}
  data_.push_back(reinterpret_cast<char*>(data.data()));
  data_sizes_.push_back(sizeof(T));


  return true;
}

inline
std::string npm::PLYFileOut::header() const {
  return header_;
}

inline
void npm::PLYFileOut::set_internal_format(PLYFormat new_format) {
  internal_format_ = new_format;
}

inline
void npm::PLYFileOut::set_ascii_write_precision(uint64_t ascci_write_prec) {
  kASCIIWritePrecision = ascci_write_prec;
}

inline
void npm::PLYFileOut::set_write_buffer_size(uint64_t write_buffer_size) {
  kWriteBufferSize = write_buffer_size;
}

#endif  // NPM_TOOLBOX_IO_PLY_FILE_OUT_H_