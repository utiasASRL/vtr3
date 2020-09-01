//////////////////////////////////////////////////////////////////////////////////////////////
/// \file ParseUtils.hpp
/// \brief Helper functions for parsing files
///
/// \author Sean Anderson, ASRL
//////////////////////////////////////////////////////////////////////////////////////////////

#ifndef STEAM_PARSE_UTILS_HPP
#define STEAM_PARSE_UTILS_HPP

#include <stdlib.h>
#include <vector>
#include <iostream>
#include <fstream>
#include <stdexcept>

namespace steam {
namespace parse {

//////////////////////////////////////////////////////////////////////////////////////////////
/// \brief Template function to write simple types to strings
/// \param t, a value of basic type (int, float, etc.)
/// \return a string evaluation of 't'
//////////////////////////////////////////////////////////////////////////////////////////////
template <typename T>
static std::string toString(const T& t) {
  std::ostringstream ss; ss<<t; return ss.str();
}

//////////////////////////////////////////////////////////////////////////////////////////////
/// \brief Helper function to read delimited files into 'matrix' of strings
//////////////////////////////////////////////////////////////////////////////////////////////
static std::vector<std::vector<std::string> > loadData(const std::string& fileName, char delim = ',') {

  // Open file stream and check that it is error free
  std::ifstream inFileStream( fileName.c_str() );
  if (!inFileStream.good()) {
    std::stringstream ss; ss << "error opening input file " << fileName;
    throw std::invalid_argument(ss.str());
  }

  // Create output variable
  std::vector<std::vector<std::string> > strMatrix;

  // Loop over lines (rows) of CSV file
  std::string line;
  while(std::getline(inFileStream, line))
  {
    // Make string stream out of line
    std::istringstream ss(line);
    std::vector<std::string> strRow;
    // Loop over comma separated fields of CSV line
    std::string field;
    while (getline(ss, field, delim)) {
      strRow.push_back(field); // add column entry to row
    }
    strMatrix.push_back(strRow); // add row to matrix
  }
  inFileStream.close();

  return strMatrix;
}

//////////////////////////////////////////////////////////////////////////////////////////////
/// \brief Helper function to write 'matrix' of strings into a delimited file
//////////////////////////////////////////////////////////////////////////////////////////////
static void writeData(const std::string& fileName,
                      const std::vector<std::vector<std::string> >& strMatrix, char delim = ',') {

  if (strMatrix.size() < 1u) {
    throw std::invalid_argument("Provided matrix of strings had no entries.");
  }

  std::ofstream outFileStream;
  outFileStream.open(fileName.c_str());

  // Iterate over the rows of the string matrix and write comma-separated fields
  for (std::vector<std::vector<std::string> >::const_iterator itRow = strMatrix.begin() ; itRow != strMatrix.end(); ++itRow) {
    const unsigned fields = itRow->size();

    if (fields < 1u) {
      throw std::invalid_argument("String matrix row has no entries.");
    }

    outFileStream << itRow->at(0);
    for (unsigned i = 1; i < fields; i++) {
      outFileStream << delim << itRow->at(i);
    }
    outFileStream << std::endl;
  }

  outFileStream.close();
}

} // parse
} // steam

#endif // STEAM_PARSE_UTILS_HPP
