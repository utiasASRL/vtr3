//////////////////////////////////////////////////////////////////////////////////////////////
/// \file BlockSparseMatrix.cpp
///
/// \author Sean Anderson, ASRL
//////////////////////////////////////////////////////////////////////////////////////////////

#include <steam/blockmat/BlockSparseMatrix.hpp>

#include <stdexcept>
#include <iostream>

#include <steam/common/Timer.hpp>

namespace steam {

//////////////////////////////////////////////////////////////////////////////////////////////
/// \brief Default constructor, matrix size must still be set before using
//////////////////////////////////////////////////////////////////////////////////////////////
BlockSparseMatrix::BlockSparseMatrix() : BlockMatrixBase() {
}

//////////////////////////////////////////////////////////////////////////////////////////////
/// \brief Rectangular matrix constructor
//////////////////////////////////////////////////////////////////////////////////////////////
BlockSparseMatrix::BlockSparseMatrix(const std::vector<unsigned int>& blkRowSizes,
                                     const std::vector<unsigned int>& blkColSizes)
  : BlockMatrixBase(blkRowSizes, blkColSizes) {

  // Setup data structures
  cols_.clear();
  cols_.resize(this->getIndexing().colIndexing().numEntries());
}

//////////////////////////////////////////////////////////////////////////////////////////////
/// \brief Block-size-symmetric matrix constructor, pure scalar symmetry is still optional
//////////////////////////////////////////////////////////////////////////////////////////////
BlockSparseMatrix::BlockSparseMatrix(const std::vector<unsigned int>& blkSizes, bool symmetric)
  : BlockMatrixBase(blkSizes, symmetric) {

  // Setup data structures
  cols_.clear();
  cols_.resize(this->getIndexing().colIndexing().numEntries());
}

//////////////////////////////////////////////////////////////////////////////////////////////
/// \brief Clear sparse entries, maintain size
//////////////////////////////////////////////////////////////////////////////////////////////
void BlockSparseMatrix::clear() {
  for (unsigned int c = 0; c < this->getIndexing().colIndexing().numEntries(); c++) {
    cols_[c].rows.clear();
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
/// \brief Keep the existing entries and sizes, but set them to zero
//////////////////////////////////////////////////////////////////////////////////////////////
void BlockSparseMatrix::zero() {
  for (unsigned int c = 0; c < this->getIndexing().colIndexing().numEntries(); c++) {
    for(std::map<unsigned int, BlockRowEntry>::iterator it = cols_[c].rows.begin();
        it != cols_[c].rows.end(); ++it) {
      it->second.data.setZero();
    }
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
/// \brief Adds the matrix to the block entry at index (r,c), block dim must match
//////////////////////////////////////////////////////////////////////////////////////////////
void BlockSparseMatrix::add(unsigned int r, unsigned int c, const Eigen::MatrixXd& m) {

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

  // Find if row entry exists
  std::map<unsigned int, BlockRowEntry>::iterator it = cols_[c].rows.find(r);

  // Check if found, and create new entry, or add to existing one
  if (it == cols_[c].rows.end()) {
    cols_[c].rows[r].data = m;
  } else {
    it->second.data += m;
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
/// \brief Returns a reference to the row entry at (r,c), if it exists; if it does not exist
///        it will be inserted if 'allowInsert' is set to true.
///        *Note this throws an exception if matrix is symmetric and you request a lower
///         triangular entry.
//////////////////////////////////////////////////////////////////////////////////////////////
BlockSparseMatrix::BlockRowEntry& BlockSparseMatrix::rowEntryAt(unsigned int r, unsigned int c, bool allowInsert) {

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

  // Find if row entry exists
  BlockSparseColumn& colRef = cols_[c];

  // Lock read/write to the column
  omp_set_lock(&colRef.lock);

  std::map<unsigned int, BlockRowEntry>::iterator it = colRef.rows.find(r);

  // If it does not exist, throw exception
  if (it == colRef.rows.end()) {
    if (allowInsert) {
      BlockRowEntry& result = colRef.rows[r];
      result.data = Eigen::MatrixXd::Zero(this->getIndexing().rowIndexing().blkSizeAt(r),
                                         this->getIndexing().rowIndexing().blkSizeAt(c));
      omp_unset_lock(&colRef.lock); // Unlock read/write to the column
      return result;
    } else {
      throw std::invalid_argument("Tried to read entry that did not exist");
    }
  } else {
    BlockRowEntry& result = it->second;
    omp_unset_lock(&colRef.lock); // Unlock read/write to the column
    return result;
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
/// \brief Returns a reference to the value at (r,c), if it exists
///        *Note this throws an exception if matrix is symmetric and you request a lower
///         triangular entry. For read operations, use copyAt(r,c).
//////////////////////////////////////////////////////////////////////////////////////////////
Eigen::MatrixXd& BlockSparseMatrix::at(unsigned int r, unsigned int c) {

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

  // Find if row entry exists
  std::map<unsigned int, BlockRowEntry>::iterator it = cols_[c].rows.find(r);

  // If it does not exist, throw exception
  if (it == cols_[c].rows.end()) {
    throw std::invalid_argument("Tried to read entry that did not exist");
  }

  // Return reference to data
  return it->second.data;
}

//////////////////////////////////////////////////////////////////////////////////////////////
/// \brief Returns a copy of the entry at index (r,c)
//////////////////////////////////////////////////////////////////////////////////////////////
Eigen::MatrixXd BlockSparseMatrix::copyAt(unsigned int r, unsigned int c) const {

  // Check that indexing is valid
  if (r >= this->getIndexing().rowIndexing().numEntries() ||
      c >= this->getIndexing().colIndexing().numEntries()) {
    throw std::invalid_argument("Requested row or column indice did not fall in valid "
                                "range of existing block structure.");
  }

  // If symmetric, check that we are indexing into upper-triangular portion
  if (this->isSymmetric() && r > c) {

    // Accessing lower triangle of symmetric matrix

    // Find if row entry exists
    std::map<unsigned int, BlockRowEntry>::const_iterator it = cols_[r].rows.find(c);

    // If it does not exist, throw exception
    if (it == cols_[r].rows.end()) {
      return Eigen::MatrixXd::Zero(this->getIndexing().rowIndexing().blkSizeAt(r),
                                   this->getIndexing().colIndexing().blkSizeAt(c));
    }

    // Return reference to data
    return it->second.data.transpose();

  } else {

    // Not symmetric OR accessing upper-triangle

    // Find if row entry exists
    std::map<unsigned int, BlockRowEntry>::const_iterator it = cols_[c].rows.find(r);

    // If it does not exist, throw exception
    if (it == cols_[c].rows.end()) {
      return Eigen::MatrixXd::Zero(this->getIndexing().rowIndexing().blkSizeAt(r),
                                   this->getIndexing().colIndexing().blkSizeAt(c));
    }

    // Return reference to data
    return it->second.data;
  }
}

//////////////////////////////////////////////////////////////////////////////////////////////
/// \brief Convert to Eigen sparse matrix format
//////////////////////////////////////////////////////////////////////////////////////////////
Eigen::SparseMatrix<double> BlockSparseMatrix::toEigen(bool getSubBlockSparsity) const {

  // Get references to indexing objects
  const BlockDimIndexing& blkRowIndexing = this->getIndexing().rowIndexing();
  const BlockDimIndexing& blkColIndexing = this->getIndexing().colIndexing();

  // Allocate sparse matrix and reserve memory for estimates number of non-zero (nnz) entries
  Eigen::SparseMatrix<double> mat(blkRowIndexing.scalarSize(), blkColIndexing.scalarSize());
  mat.reserve(this->getNnzPerCol());

  // Iterate over block-sparse columns and rows
  for (unsigned int c = 0; c < blkColIndexing.numEntries(); c++) {

    unsigned colBlkSize = blkColIndexing.blkSizeAt(c);
    unsigned colCumSum = blkColIndexing.cumSumAt(c);
    for(std::map<unsigned int, BlockRowEntry>::const_iterator it = cols_[c].rows.begin(); it != cols_[c].rows.end(); ++it) {

      // Get row index of block entry
      unsigned int r = it->first;
      unsigned int rowBlkSize = blkRowIndexing.blkSizeAt(r);
      unsigned int rowCumSum = blkRowIndexing.cumSumAt(r);

      // Iterate over internal matrix dimensions
      // Eigen matrix storage is column-major, outer iterator should be over column first for speed
      unsigned int colIdx = colCumSum;
      for (unsigned int j = 0; j < colBlkSize; j++, colIdx++) {
        unsigned int rowIdx = rowCumSum;
        for (unsigned int i = 0; i < rowBlkSize; i++, rowIdx++) {

          // Get scalar element
          double v_ij = it->second.data(i,j);

          // Add entry to sparse matrix
          // ** The case where we do not add the element is when sub-block sparsity is enabled
          //    and an element is exactly zero
          if (fabs(v_ij) > 0.0 || !getSubBlockSparsity) {
            mat.insert(rowIdx, colIdx) = v_ij;
          }
        }
      }
    }
  }

  // (optional) Compress into compact format
  mat.makeCompressed();

  return mat;
}

//////////////////////////////////////////////////////////////////////////////////////////////
/// \brief Gets the number of non-zero entries per scalar-column
//////////////////////////////////////////////////////////////////////////////////////////////
Eigen::VectorXi BlockSparseMatrix::getNnzPerCol() const {

  // Get references to indexing objects
  const BlockDimIndexing& blkColIndexing = this->getIndexing().colIndexing();

  // Allocate vector of ints
  Eigen::VectorXi result = Eigen::VectorXi(blkColIndexing.scalarSize());

  // Iterate over columns and determine number of non-zero entries
  for (unsigned int c = 0; c < blkColIndexing.numEntries(); c++) {

    // Sum
    unsigned int nnz = 0;

    // Iterate over sparse row entries of column 'c'
    for(std::map<unsigned int, BlockRowEntry>::const_iterator it = cols_[c].rows.begin();
        it != cols_[c].rows.end(); ++it) {
      nnz += it->second.data.rows();
    }

    // Add to result
    result.block(blkColIndexing.cumSumAt(c), 0, blkColIndexing.blkSizeAt(c), 1) =
        Eigen::VectorXi::Constant(blkColIndexing.blkSizeAt(c), nnz);
  }

  return result;
}

} // steam
