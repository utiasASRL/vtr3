#include <gtest/gtest.h>

#include <iostream>

#include <steam/blockmat/BlockSparseMatrix.hpp>
#include <steam/blockmat/BlockVector.hpp>

TEST(Steam, SparsityPattern) {
  std::vector<unsigned int> blockSizes;
  blockSizes.resize(3);
  blockSizes[0] = 2;
  blockSizes[1] = 2;
  blockSizes[2] = 2;

  // Setup blocks
  Eigen::Matrix2d m_tri_diag;
  m_tri_diag << 2, 1, 1, 2;
  Eigen::Matrix2d m_tri_offdiag;
  m_tri_offdiag << 0, 0, 1, 0;
  Eigen::Matrix2d m_z = Eigen::Matrix2d::Zero();
  Eigen::Matrix2d m_o = Eigen::Matrix2d::Ones();

  // Setup A - Case 1
  steam::BlockSparseMatrix tri(blockSizes, true);
  tri.add(0, 0, m_tri_diag);
  tri.add(0, 1, m_tri_offdiag);
  tri.add(1, 1, m_tri_diag);
  tri.add(1, 2, m_tri_offdiag);
  tri.add(2, 2, m_tri_diag);

  // Setup A - Case 2
  steam::BlockSparseMatrix blkdiag(blockSizes, true);
  blkdiag.add(0, 0, m_tri_diag);
  blkdiag.add(0, 1, m_z);
  blkdiag.add(1, 1, m_tri_diag);
  blkdiag.add(1, 2, m_z);
  blkdiag.add(2, 2, m_tri_diag);

  // Setup A - Case 3
  steam::BlockSparseMatrix tri_ones(blockSizes, true);
  tri_ones.add(0, 0, m_o);
  tri_ones.add(0, 1, m_o);
  tri_ones.add(1, 1, m_o);
  tri_ones.add(1, 2, m_o);
  tri_ones.add(2, 2, m_o);

  // Setup B
  Eigen::VectorXd b(6);
  b << 1, 2, 3, 4, 5, 6;

  // Test sub sparsity
  {
    // Maximum sparsity
    Eigen::SparseMatrix<double> eig_sparse = tri.toEigen(true);
    std::cout << "case1: " << eig_sparse << std::endl;
    std::cout << "nonzeros: " << eig_sparse.nonZeros() << std::endl;
    EXPECT_EQ(eig_sparse.nonZeros(), 14);

    // Get only block-level sparsity (important for re-using pattern)
    Eigen::SparseMatrix<double> eig_blk_sparse = tri.toEigen(false);
    std::cout << "case2: " << eig_blk_sparse << std::endl;
    std::cout << "nonzeros: " << eig_blk_sparse.nonZeros() << std::endl;
    EXPECT_EQ(eig_blk_sparse.nonZeros(), 20);
  }

  // Test solve
  {
    // Maximum sparsity
    Eigen::SparseMatrix<double> eig_sparse = tri.toEigen(true);
    std::cout << "case1: " << eig_sparse << std::endl;
    std::cout << "nonzeros: " << eig_sparse.nonZeros() << std::endl;

    // Solve sparse
    Eigen::SimplicialLLT<Eigen::SparseMatrix<double>, Eigen::Upper> solver;
    solver.analyzePattern(eig_sparse);
    solver.factorize(eig_sparse);
    if (solver.info() != Eigen::Success)
      throw std::runtime_error("Decomp failure.");
    Eigen::VectorXd x1 = solver.solve(b);
    std::cout << "x1: " << x1.transpose() << std::endl;

    // Get only block-level sparsity (important for re-using pattern)
    Eigen::SparseMatrix<double> eig_blk_sparse = tri.toEigen(false);
    std::cout << "case2: " << eig_blk_sparse << std::endl;
    std::cout << "nonzeros: " << eig_blk_sparse.nonZeros() << std::endl;

    // Solve sparse
    solver.analyzePattern(eig_blk_sparse);
    solver.factorize(eig_blk_sparse);
    if (solver.info() != Eigen::Success)
      throw std::runtime_error("Decomp failure.");
    Eigen::VectorXd x2 = solver.solve(b);
    std::cout << "x2: " << x2.transpose() << std::endl;
    EXPECT_LT((x1 - x2).norm(), 1e-6);
  }

  // Test solve, setting pattern with ones
  {
    // Solve using regular tri-block diagonal
    Eigen::SparseMatrix<double> eig_tri = tri.toEigen();
    std::cout << "case1: " << eig_tri << std::endl;
    std::cout << "nonzeros: " << eig_tri.nonZeros() << std::endl;

    // Solve sparse
    Eigen::SimplicialLLT<Eigen::SparseMatrix<double>, Eigen::Upper> solver;
    solver.analyzePattern(eig_tri);
    solver.factorize(eig_tri);
    if (solver.info() != Eigen::Success) {
      throw std::runtime_error("Decomp failure.");
    }
    Eigen::VectorXd x1 = solver.solve(b);
    std::cout << "x1: " << x1.transpose() << std::endl;

    // Set pattern using ones and then solve with tri-block
    Eigen::SparseMatrix<double> eig_tri_ones = tri_ones.toEigen();
    std::cout << "case2: " << eig_tri_ones << std::endl;
    std::cout << "nonzeros: " << eig_tri_ones.nonZeros() << std::endl;

    // Solve sparse
    solver.analyzePattern(eig_tri_ones);
    solver.factorize(eig_tri);
    if (solver.info() != Eigen::Success) {
      throw std::runtime_error("Decomp failure.");
    }
    Eigen::VectorXd x2 = solver.solve(b);
    std::cout << "x2: " << x2.transpose() << std::endl;
    EXPECT_LT((x1 - x2).norm(), 1e-6);
  }

  // Test solve of matrix with zero blocks, setting pattern with ones
  {
    // Solve using regular tri-block diagonal
    Eigen::SparseMatrix<double> eig_blkdiag = blkdiag.toEigen();
    std::cout << "case1: " << eig_blkdiag << std::endl;
    std::cout << "nonzeros: " << eig_blkdiag.nonZeros() << std::endl;

    // Solve sparse
    Eigen::SimplicialLLT<Eigen::SparseMatrix<double>, Eigen::Upper> solver;
    solver.analyzePattern(eig_blkdiag);
    solver.factorize(eig_blkdiag);
    if (solver.info() != Eigen::Success) {
      throw std::runtime_error("Decomp failure.");
    }
    Eigen::VectorXd x1 = solver.solve(b);
    std::cout << "x1: " << x1.transpose() << std::endl;

    // Set pattern using ones and then solve with tri-block
    Eigen::SparseMatrix<double> eig_tri_ones = tri_ones.toEigen();
    std::cout << "case2: " << eig_tri_ones << std::endl;
    std::cout << "nonzeros: " << eig_tri_ones.nonZeros() << std::endl;

    // Solve sparse
    solver.analyzePattern(eig_tri_ones);
    solver.factorize(eig_blkdiag);
    if (solver.info() != Eigen::Success) {
      throw std::runtime_error("Decomp failure.");
    }
    Eigen::VectorXd x2 = solver.solve(b);
    std::cout << "x2: " << x2.transpose() << std::endl;
    EXPECT_LT((x1 - x2).norm(), 1e-6);
  }
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}