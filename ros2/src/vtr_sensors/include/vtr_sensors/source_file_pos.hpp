#ifndef UTR_FILE_POS_HPP
#define UTR_FILE_POS_HPP

#include <string>
#include <iostream>
#include <sstream>
// A little class and macro that gives you the current file position.

namespace asrl {

  class source_file_pos
  {
  public:
    std::string function;
    std::string file;
    int line;

    source_file_pos(std::string function, std::string file, int line) :
      function(function), file(file), line(line) {}

    operator std::string()
    {
      std::stringstream s;
      s << file << ":" << line << ": " << function << "()";;
      return s.str();
    }

  };

}// namespace utr

inline std::ostream & operator<<(std::ostream & out, asrl::source_file_pos const & sfp)
{
  out << sfp.file << ":" << sfp.line << ": " << sfp.function << "()";
  return out;
}


#define ASRL_SOURCE_FILE_POS asrl::source_file_pos(__FUNCTION__,__FILE__,__LINE__)

#endif

