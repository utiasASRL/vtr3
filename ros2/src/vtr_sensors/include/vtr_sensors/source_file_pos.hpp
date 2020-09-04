#ifndef UTR_FILE_POS_HPP
#define UTR_FILE_POS_HPP

#include <iostream>
#include <sstream>
#include <string>
#include <utility>
// A little class and macro that gives you the current file position.

namespace asrl {

class source_file_pos {
 public:
  std::string function;
  std::string file;
  int line;

  source_file_pos(std::string function, std::string file, int line)
      : function(std::move(function)), file(std::move(file)), line(line) {}

  explicit operator std::string() const {
    std::stringstream s;
    s << file << ":" << line << ": " << function << "()";
    ;
    return s.str();
  }
};

}  // namespace asrl

inline std::ostream& operator<<(std::ostream& out,
                                asrl::source_file_pos const& sfp) {
  out << sfp.file << ":" << sfp.line << ": " << sfp.function << "()";
  return out;
}

#define ASRL_SOURCE_FILE_POS \
  asrl::source_file_pos(__FUNCTION__, __FILE__, __LINE__)

#endif
