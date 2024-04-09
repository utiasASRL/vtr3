/**
 * @file   assert_macros.hpp
 * @authors Paul Furgale and Chi Hay Tong
 * @date   Thu Feb 18 18:18:47 2010
 *
 * @brief  Handy assert macros that throw exceptions when something goes awry
 *
 * These handy macros will build and throw standard exceptions that include the
 * current file, function name, and line number.
 *
 */
 
/*
Copyright (c) 2010, Paul Furgale and Chi Hay Tong
All rights reserved.

Redistribution and use in source and binary forms, with or without 
modification, are permitted provided that the following conditions are 
met:

* Redistributions of source code must retain the above copyright notice, 
  this list of conditions and the following disclaimer.
* Redistributions in binary form must reproduce the above copyright 
  notice, this list of conditions and the following disclaimer in the 
  documentation and/or other materials provided with the distribution.
* The names of its contributors may not be used to endorse or promote 
  products derived from this software without specific prior written 
  permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A 
PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER
OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, 
EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, 
PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR 
PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS 
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#ifndef ASRL_ASSERT_MACROS_HPP
#define ASRL_ASSERT_MACROS_HPP

#include <stdexcept>
#include <sstream>

namespace asrl {

  namespace detail {
    /**
     * Throws a runtime error with a nice message indicating the function
     * and file that the error originated from.
     *
     * @param function The function that the exception originated from
     * @param file     The file that the exception originated from
     * @param line     The line number that the exception originated from
     * @param message  The message describing the exception
     */
    inline void throw_runtime_error(std::string function, std::string file,
				    int line, std::string message)
    {
      std::ostringstream asrl_assert_stringstream;
      asrl_assert_stringstream << file << ":" << line << " " << function << "() " << message;


      throw(std::runtime_error(asrl_assert_stringstream.str()));
    }

  }} // namespace asrl::detail

#define ASRL_TRACE std::cout << __FILE__ << ":" << __LINE__ << " " << __FUNCTION__ << "()" << std::endl;

/**
 * Throws a runtime error including the function, file, and line number.
 *
 * @param message An informative message to be included in the exception
 *
 */
#define ASRL_THROW(message){						\
    std::ostringstream asrl_assert_stringstream;			\
    asrl_assert_stringstream << message;				\
    asrl::detail::throw_runtime_error(__FUNCTION__,__FILE__,__LINE__, asrl_assert_stringstream.str()); \
  }

/**
 * A macro to check if "condition" is true. If it isn't a runtime error is thrown.
 *
 * @param condition A runtime error is thrown if this condition evaluates to false
 * @param message An informative message to be included in the exception
 *
 */
#define ASRL_ASSERT(condition, message)					\
  if(!(condition))							\
    {									\
      std::ostringstream asrl_assert_stringstream;			\
      asrl_assert_stringstream << "assert " << #condition << " failed: " << message; \
      asrl::detail::throw_runtime_error(__FUNCTION__,__FILE__,__LINE__, asrl_assert_stringstream.str()); \
    }

/**
 * This macro asserts that \f$lowerBound <= value < upperBound\f$
 * If it is not, a runtime error is thrown
 *
 * @param message An informative message included in the exception
 *
 */
#define ASRL_ASSERT_GE_LT(value, lowerBound, upperBound, message)	\
  if(value < lowerBound || value >= upperBound)				\
    {									\
      std::ostringstream asrl_assert_stringstream;			\
      asrl_assert_stringstream << "assert " << #lowerBound << " <= " << #value << " < " << #upperBound << " failed [" << lowerBound << " <= " << value << " < " << upperBound << "]: " << message; \
      asrl::detail::throw_runtime_error(__FUNCTION__,__FILE__,__LINE__,asrl_assert_stringstream.str());	\
    }

/**
 * This macro asserts that \f$value < upperBound\f$
 * If it is not, a runtime error is thrown
 *
 * @param message An informative message included in the exception
 *
 */
#define ASRL_ASSERT_LT(value, upperBound, message)			\
  if(value >= upperBound)						\
    {									\
      std::ostringstream asrl_assert_stringstream;			\
      asrl_assert_stringstream << "assert " << #value << " < " << #upperBound << " failed [" << value << " < " << upperBound << "]: " <<  message; \
      asrl::detail::throw_runtime_error(__FUNCTION__,__FILE__,__LINE__,asrl_assert_stringstream.str());	\
    }

/**
 * This macro asserts that \f$value >= lowerBound\f$
 * If it is not, a runtime error is thrown
 *
 * @param message An informative message included in the exception
 *
 */
#define ASRL_ASSERT_GE(value, lowerBound, message)			\
  if(value < lowerBound)						\
    {									\
      std::ostringstream asrl_assert_stringstream;			\
      asrl_assert_stringstream << "assert " << #value << " >= " << #lowerBound << " failed [" << value << " >= " << lowerBound << "]: " <<  message; \
      asrl::detail::throw_runtime_error(__FUNCTION__,__FILE__,__LINE__,asrl_assert_stringstream.str());	\
    }

/**
 * This macro asserts that \f$value <= upperBound\f$
 * If it is not, a runtime error is thrown
 *
 * @param message An informative message included in the exception
 *
 */
#define ASRL_ASSERT_LE(value, upperBound, message)			\
  if(value > upperBound)						\
    {									\
      std::ostringstream asrl_assert_stringstream;			\
      asrl_assert_stringstream << "assert " << #value << " <= " << #upperBound << " failed [" << value << " <= " << upperBound << "]: " <<  message; \
      asrl::detail::throw_runtime_error(__FUNCTION__,__FILE__,__LINE__,asrl_assert_stringstream.str());	\
    }

/**
 * This macro asserts that \f$value > lowerBound\f$
 * If it is not, a runtime error is thrown
 *
 * @param message An informative message included in the exception
 *
 */
#define ASRL_ASSERT_GT(value, lowerBound, message)			\
  if(value <= lowerBound)						\
    {									\
      std::ostringstream asrl_assert_stringstream;			\
      asrl_assert_stringstream << "assert " << #value << " > " << #lowerBound << " failed [" << value << " > " << lowerBound << "]: " <<  message; \
      asrl::detail::throw_runtime_error(__FUNCTION__,__FILE__,__LINE__,asrl_assert_stringstream.str());	\
    }

/**
 * This macro asserts that \f$value == testValue\f$
 * If it is not, a runtime error is thrown
 *
 * @param message An informative message included in the exception
 *
 */
#define ASRL_ASSERT_EQ(value, testValue, message)			\
  if(value != testValue)						\
    {									\
      std::ostringstream asrl_assert_stringstream;			\
      asrl_assert_stringstream << "assert " << #value << " == " << #testValue << " failed [" << value << " == " << testValue << "]: " <<  message; \
      asrl::detail::throw_runtime_error(__FUNCTION__,__FILE__,__LINE__,asrl_assert_stringstream.str());	\
    }

/**
 * This macro asserts that \f$value != testValue\f$
 * If it is not, a runtime error is thrown
 *
 * @param message An informative message included in the exception
 *
 */
#define ASRL_ASSERT_NE(value, testValue, message)			\
  if(value == testValue)						\
    {									\
      std::ostringstream asrl_assert_stringstream;			\
      asrl_assert_stringstream << "assert " << #value << " != " << #testValue << " failed [" << value << " != " << testValue << "]: " <<  message; \
      asrl::detail::throw_runtime_error(__FUNCTION__,__FILE__,__LINE__,asrl_assert_stringstream.str());	\
    }

/** 
 * This macro checks for a cuda error. If there is one. it throws an error
 * 
 * @param errorMessage the message to return
 * 
 */
#define ASRL_CHECK_CUDA_ERROR(errorMessage) {			\
    cudaDeviceSynchronize();						\
    cudaError_t err = cudaGetLastError();				\
    if( cudaSuccess != err) {						\
      std::ostringstream asrl_assert_stringstream;			\
      asrl_assert_stringstream << "CUDA error: " << cudaGetErrorString( err ) << " : " <<  errorMessage; \
      asrl::detail::throw_runtime_error(__FUNCTION__,__FILE__,__LINE__,asrl_assert_stringstream.str());	\
    } \
  }

/**
 * This macro asserts that \f$value = testValue\f$ to within some tolerance
 * If it is not, a runtime error is thrown
 *
 * @param message An informative message included in the exception
 *
 */
#define ASRL_ASSERT_EQ_TOL(value, testValue, tolerance, message)		\
  if(fabs((double)value - (double)testValue) > (double)tolerance)	\
    {									\
      std::ostringstream asrl_assert_stringstream;			\
      asrl_assert_stringstream << "assert " << #value << " == " << #testValue << " (tolerance " << #tolerance ") failed [" << value << " != " << testValue << ", " << tolerance << "]: " <<  message; \
      asrl::detail::throw_runtime_error(__FUNCTION__,__FILE__,__LINE__,asrl_assert_stringstream.str());	\
    }

#ifndef NDEBUG

/**
 * Throws a runtime error including the function, file, and line number.
 * The exception is only thrown in debug mode.
 *
 * @param message An informative message to be included in the exception
 *
 */
#define ASRL_THROW_DBG(message){					\
    std::ostringstream asrl_assert_stringstream;			\
    asrl_assert_stringstream << message;				\
    asrl::detail::throw_runtime_error(__FUNCTION__,__FILE__,__LINE__, asrl_assert_stringstream.str()); \
  }

/**
 * Throws a runtime error including the function, file, and line number.
 * The exception is only thrown in debug mode.
 *
 * @param message An informative message to be included in the exception
 *
 */
#define ASRL_ASSERT_DBG(condition, message)				\
  if(!(condition))							\
    {									\
      std::ostringstream asrl_assert_stringstream;			\
      asrl_assert_stringstream << "debug assert " << #condition << " failed: " << message; \
      asrl::detail::throw_runtime_error(__FUNCTION__,__FILE__,__LINE__, asrl_assert_stringstream.str()); \
    }
/**
 * This macro asserts that \f$lowerBound <= value < upperBound\f$
 * If it is not, a runtime error is thrown
 * The exception is only thrown in debug mode.
 *
 * @param message An informative message included in the exception
 *
 */

#define ASRL_ASSERT_GE_LT_DBG(value, lowerBound, upperBound, message)	\
  if(value < lowerBound || value >= upperBound)				\
    {									\
      std::ostringstream asrl_assert_stringstream;			\
      asrl_assert_stringstream << "debug assert " << #lowerBound << " <= " << #value << " < " << #upperBound << " failed [" << lowerBound << " <= " << value << " < " << upperBound << "]: " << message; \
      asrl::detail::throw_runtime_error(__FUNCTION__,__FILE__,__LINE__,asrl_assert_stringstream.str());	\
    }

/**
 * This macro asserts that \f$value < upperBound\f$
 * If it is not, a runtime error is thrown
 * The exception is only thrown in debug mode.
 *
 * @param message An informative message included in the exception
 *
 */

#define ASRL_ASSERT_LT_DBG(value, upperBound, message)			\
  if(value >= upperBound)						\
    {									\
      std::ostringstream asrl_assert_stringstream;			\
      asrl_assert_stringstream << "debug assert " << #value << " < " << #upperBound << " failed [" << value << " < " << upperBound << "]: " <<  message; \
      asrl::detail::throw_runtime_error(__FUNCTION__,__FILE__,__LINE__,asrl_assert_stringstream.str());	\
    }

/**
 * This macro asserts that \f$value >= lowerBound\f$
 * If it is not, a runtime error is thrown
 * The exception is only thrown in debug mode.
 *
 * @param message An informative message included in the exception
 *
 */

#define ASRL_ASSERT_GE_DBG(value, lowerBound, message)			\
  if(value < lowerBound)						\
    {									\
      std::ostringstream asrl_assert_stringstream;			\
      asrl_assert_stringstream << "debug assert " << #value << " >= " << #lowerBound << " failed [" << value << " >= " << lowerBound << "]: " <<  message; \
      asrl::detail::throw_runtime_error(__FUNCTION__,__FILE__,__LINE__,asrl_assert_stringstream.str());	\
    }

/**
 * This macro asserts that \f$value < upperBound\f$
 * If it is not, a runtime error is thrown
 * The exception is only thrown in debug mode.
 *
 * @param message An informative message included in the exception
 *
 */

#define ASRL_ASSERT_LE_DBG(value, upperBound, message)			\
  if(value > upperBound)						\
    {									\
      std::ostringstream asrl_assert_stringstream;			\
      asrl_assert_stringstream << "debug assert " << #value << " <= " << #upperBound << " failed [" << value << " <= " << upperBound << "]: " <<  message; \
      asrl::detail::throw_runtime_error(__FUNCTION__,__FILE__,__LINE__,asrl_assert_stringstream.str());	\
    }


/**
 * This macro asserts that \f$value > lowerBound\f$
 * If it is not, a runtime error is thrown
 * The exception is only thrown in debug mode.
 *
 * @param message An informative message included in the exception
 *
 */
#define ASRL_ASSERT_GT_DBG(value, lowerBound, message)			\
  if(value <= lowerBound)						\
    {									\
      std::ostringstream asrl_assert_stringstream;			\
      asrl_assert_stringstream << "debug assert " << #value << " > " << #lowerBound << " failed [" << value << " > " << lowerBound << "]: " <<  message; \
      asrl::detail::throw_runtime_error(__FUNCTION__,__FILE__,__LINE__,asrl_assert_stringstream.str());	\
    }


/**
 * This macro asserts that \f$value == testValue\f$
 * If it is not, a runtime error is thrown
 * The exception is only thrown in debug mode.
 *
 * @param message An informative message included in the exception
 *
 */
#define ASRL_ASSERT_EQ_DBG(value, testValue, message)			\
  if(value != testValue)						\
    {									\
      std::ostringstream asrl_assert_stringstream;			\
      asrl_assert_stringstream << "debug assert " << #value << " == " << #testValue << " failed [" << value << " == " << testValue << "]: " <<  message; \
      asrl::detail::throw_runtime_error(__FUNCTION__,__FILE__,__LINE__,asrl_assert_stringstream.str());	\
    }

/**
 * This macro asserts that \f$value != testValue\f$
 * If it is not, a runtime error is thrown
 * The exception is only thrown in debug mode.
 *
 * @param message An informative message included in the exception
 *
 */
#define ASRL_ASSERT_NE_DBG(value, testValue, message)			\
  if(value == testValue)						\
    {									\
      std::ostringstream asrl_assert_stringstream;			\
      asrl_assert_stringstream << "debug assert " << #value << " != " << #testValue << " failed [" << value << " != " << testValue << "]: " <<  message; \
      asrl::detail::throw_runtime_error(__FUNCTION__,__FILE__,__LINE__,asrl_assert_stringstream.str());	\
    }


/** 
 * This macro checks for a cuda error. If there is one. it throws an error
 * 
 * @param errorMessage the message to return
 * 
 */
#define ASRL_CHECK_CUDA_ERROR_DBG(errorMessage) {			\
    cudaDeviceSynchronize();						\
    cudaError_t err = cudaGetLastError();				\
    if( cudaSuccess != err) {						\
      std::ostringstream asrl_assert_stringstream;			\
      asrl_assert_stringstream << "CUDA error: " << cudaGetErrorString( err ) << " : " <<  errorMessage; \
      asrl::detail::throw_runtime_error(__FUNCTION__,__FILE__,__LINE__,asrl_assert_stringstream.str());	\
    } \
  }


#else

#define COUT_P(x)
#define ASRL_THROW_DBG(message)
#define ASRL_ASSERT_DBG(condition, message)
#define ASRL_ASSERT_GE_LT_DBG(value, lowerBound, upperBound, message)
#define ASRL_ASSERT_LT_DBG(value, upperBound, message)
#define ASRL_ASSERT_GT_DBG(value, lowerBound, message)
#define ASRL_ASSERT_LE_DBG(value, upperBound, message)
#define ASRL_ASSERT_GE_DBG(value, lowerBound, message)
#define ASRL_ASSERT_NE_DBG(value, testValue, message)
#define ASRL_ASSERT_EQ_DBG(value, testValue, message)
#define ASRL_ASSERT_EQ_TOL_DBG(value, testValue, tolerance, message)
#define ASRL_CHECK_CUDA_ERROR_DBG(errorMessage)

#endif // DEBUG

#endif // ASRL_ASSERT_MACROS_HPP
