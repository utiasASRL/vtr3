//////////////////////////////////////////////////////////////////////////////////////////////
/// \file BlockMatrix.cpp
///
/// \author Sean Anderson, ASRL
//////////////////////////////////////////////////////////////////////////////////////////////

#include <steam/blockmat/BlockMatrix.hpp>

#include <stdexcept>
#include <iostream>

namespace steam {

//////////////////////////////////////////////////////////////////////////////////////////////
/// \brief Default constructor, matrix size must still be set before using
//////////////////////////////////////////////////////////////////////////////////////////////
BlockMatrix::BlockMatrix() : BlockMatrixBase() {
}

//////////////////////////////////////////////////////////////////////////////////////////////
/// \brief Rectangular matrix constructor
//////////////////////////////////////////////////////////////////////////////////////////////
BlockMatrix::BlockMatrix(const std::vector<unsigned int>& blkRowSizes,
                         const std::vector<unsigned int>& blkColSizes)
  : BlockMatrixBase(blkRowSizes, blkColSizes) {

  // Setup data structures
  data_.clear();
  data_.resize(this->getIndexing().rowIndexing().numEntries());
  for (unsigned int r = 0; r < data_.size(); r++) {
    data_[r].resize(this->getIndexing().colIndexing().numEntries());
  }
  this->zero();
}

//////////////////////////////////////////////////////////////////////////////////////////////
/// \brief Block-size-symmetric matrix constructor, pure scalar symmetry is still optional
//////////////////////////////////////////////////////////////////////////////////////////////
BlockMatrix::BlockMatrix(const std::vector<unsigned int>& blkSizes, bool symmetric)
  : BlockMatrixBase(blkSizes, symmetric) {

  // Setup data structures
  data_.clear();
  data_.resize(this->getIndexing().rowIndexing().numEntries());
  for (unsigned int r = 0; r < data_.size(); r++) {
    data_[r].resize(this->getIndexing().colIndexing().numEntries());
  }
  this->zero();
}

//////////////////////////////////////////////////////////////////////////////////////////////
/// \brief Keep the existing entries and sizes, but set them to zero
//////////////////////////////////////////////////////////////////////////////////////////////
void BlockMatrix::zero() {

  for (unsigned int r = 0; r < data_.size(); r++) {
    for (unsigned int c = 0; c < data_[r].size(); c++) {
      data_[r][c] = Eigen::MatrixXd::Zero(this->getIndexing().rowIndexing().blkSizeAt(r),
                                          this->getIndexing().colIndexing().blkSizeAt(c));
    }
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
/// \brief Adds the matrix to the block entry at index (r,c), block dim must match
//////////////////////////////////////////////////////////////////////////////////////////////
void BlockMatrix::add(unsigned int r, unsigned int c, const Eigen::MatrixXd& m) {

  // Get references to indexing objects
  const BlockDimIndexing& blkRowIndexing = this->getIndexing().rowIndexing();
  const BlockDimIndexing& blkColIndexing = this->getIndexing().colIndexing();

  // Check that indexing is valid
  if (r >= blkRowIndexing.numEntries() ||
      c >= blkColIndexing.numEntries()) {
    throw std::invalid_argument("Requested row or column indice did not fall in valid "
                                "range of existing block structure.");
  }

  // If symmetric, check that we are indexing into upper-triangular portion
  if (this->isSymmetric() && r > c) {
    std::cout << "[STEAM WARN] Attempted to add to lower half of upper-symmetric, "
                 "block-sparse matrix: operation was ignored for efficiency." << std::endl;
    return;
  }

  // Check that provided matrix is of the correct dimensions
  if (m.rows() != (int)blkRowIndexing.blkSizeAt(r) ||
      m.cols() != (int)blkColIndexing.blkSizeAt(c)) {

    std::stringstream ss; ss << "Size of matrix did not align with block structure; row: "
                             << r << " col: " << c << " failed the check: "
                             << m.rows() << " == " << (int)blkRowIndexing.blkSizeAt(r) << " && "
                             << m.cols() << " == " << (int)blkColIndexing.blkSizeAt(c);
    throw std::invalid_argument(ss.str());
  }

  // Add to entry
  data_[r][c] += m;
}

//////////////////////////////////////////////////////////////////////////////////////////////
/// \brief Returns a reference to the value at (r,c), if it exists
///        *Note this throws an exception if matrix is symmetric and you request a lower
///         triangular entry. For read operations, use copyAt(r,c).
//////////////////////////////////////////////////////////////////////////////////////////////
Eigen::MatrixXd& BlockMatrix::at(unsigned int r, unsigned int c) {

  // Check that indexing is valid
  if (r >= this->getIndexing().rowIndexing().numEntries() ||
      c >= this->getIndexing().colIndexing().numEntries()) {
    throw std::invalid_argument("Requested row or column indice did not fall in valid "
                                "range of existing block structure.");
  }

  // If symmetric, check that we are indexing into upper-triangular portion
  if (this->isSymmetric() && r > c) {
    std::cout << "[STEAM WARN] Attempted to add to lower half of upper-symmetric, "
                 "block-sparse matrix: cannot return reference." << std::endl;
  }

  // Return reference to data
  return data_[r][c];
}

//////////////////////////////////////////////////////////////////////////////////////////////
/// \brief Returns a copy of the entry at index (r,c)
//////////////////////////////////////////////////////////////////////////////////////////////
Eigen::MatrixXd BlockMatrix::copyAt(unsigned int r, unsigned int c) const {

  // Check that indexing is valid
  if (r >= this->getIndexing().rowIndexing().numEntries() ||
      c >= this->getIndexing().colIndexing().numEntries()) {
    throw std::invalid_argument("Requested row or column indice did not fall in valid "
                                "range of existing block structure.");
  }

  // If symmetric, check that we are indexing into upper-triangular portion
  if (this->isSymmetric() && r > c) {

    // Accessing lower triangle of symmetric matrix
    // Return matrix
    return data_[c][r].transpose();

  } else {

    // Not symmetric OR accessing upper-triangle
    // Return matrix
    return data_[r][c];
  }
}

} // steam
