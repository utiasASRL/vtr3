#pragma once

#include <sstream>
#include <stdexcept>
#include <typeinfo>
#include "source_file_pos.hpp"

//! Macro for defining an exception with a given parent
//  (std::runtime_error should be top parent)
// adapted from ros/drivers/laser/hokuyo_driver/hokuyo.h
#define ASRL_DEFINE_EXCEPTION(exceptionName, exceptionParent)               \
  class exceptionName : public exceptionParent {                            \
   public:                                                                  \
    exceptionName(const char* message) : exceptionParent(message) {}        \
    exceptionName(std::string const& message) : exceptionParent(message) {} \
    virtual ~exceptionName() throw() {}                                     \
  }

namespace asrl {

namespace detail {

template <typename E>
inline void throw_exception(std::string const& exceptionType,
                            const asrl::source_file_pos& sfp,
                            std::string const& message) {
  std::stringstream asrl_assert_stringstream;
  asrl_assert_stringstream << exceptionType << sfp << " " << message;

  throw(E(asrl_assert_stringstream.str()));
}

template <typename E>
inline void throw_exception(std::string const& exceptionType,
                            std::string const& function,
                            std::string const& file, int line,
                            std::string const& message) {
  throw_exception<E>(exceptionType, asrl::source_file_pos(function, file, line),
                     message);
}

}  // namespace detail

template <typename E>
inline void assert_throw(bool assert_condition, std::string message,
                         asrl::source_file_pos sfp) {
  if (!assert_condition) {
    detail::throw_exception<E>("", sfp, message);
  }
}

}  // namespace asrl

#define ASRL_THROW(exceptionType, message)                         \
  {                                                                \
    std::stringstream asrl_assert_stringstream;                    \
    asrl_assert_stringstream << message;                           \
    asrl::detail::throw_exception<exceptionType>(                  \
        "[" #exceptionType "] ", __FUNCTION__, __FILE__, __LINE__, \
        asrl_assert_stringstream.str());                           \
  }

#define ASRL_THROW_RE(message) ASRL_THROW(std::runtime_error, message)

#define ASRL_THROW_SFP(exceptionType, SourceFilePos, message) \
  {                                                           \
    std::stringstream asrl_assert_stringstream;               \
    asrl_assert_stringstream << message;                      \
    asrl::detail::throw_exception<exceptionType>(             \
        "[" #exceptionType "] ", SourceFilePos,               \
        asrl_assert_stringstream.str());                      \
  }

#define ASRL_ASSERT(exceptionType, condition, message)             \
  if (!(condition)) {                                              \
    std::stringstream asrl_assert_stringstream;                    \
    asrl_assert_stringstream << "assert(" << #condition            \
                             << ") failed: " << message;           \
    asrl::detail::throw_exception<exceptionType>(                  \
        "[" #exceptionType "] ", __FUNCTION__, __FILE__, __LINE__, \
        asrl_assert_stringstream.str());                           \
  }

#define ASRL_ASSERT_GE_LT(exceptionType, value, lowerBound, upperBound,      \
                          message)                                           \
  if (value < lowerBound || value >= upperBound) {                           \
    std::stringstream asrl_assert_stringstream;                              \
    asrl_assert_stringstream << "assert(" << #lowerBound << " <= " << #value \
                             << " < " << #upperBound << ") failed ["         \
                             << lowerBound << " <= " << value << " < "       \
                             << upperBound << "]: " << message;              \
    asrl::detail::throw_exception<exceptionType>(                            \
        "[" #exceptionType "] ", __FUNCTION__, __FILE__, __LINE__,           \
        asrl_assert_stringstream.str());                                     \
  }

#define ASRL_ASSERT_LT(exceptionType, value, upperBound, message)            \
  if (value >= upperBound) {                                                 \
    std::stringstream asrl_assert_stringstream;                              \
    asrl_assert_stringstream << "assert(" << #value << " < " << #upperBound  \
                             << ") failed [" << value << " < " << upperBound \
                             << "]: " << message;                            \
    asrl::detail::throw_exception<exceptionType>(                            \
        "[" #exceptionType "] ", __FUNCTION__, __FILE__, __LINE__,           \
        asrl_assert_stringstream.str());                                     \
  }

#define ASRL_ASSERT_GE(exceptionType, value, lowerBound, message)             \
  if (value < lowerBound) {                                                   \
    std::stringstream asrl_assert_stringstream;                               \
    asrl_assert_stringstream << "assert(" << #value << " >= " << #lowerBound  \
                             << ") failed [" << value << " >= " << lowerBound \
                             << "]: " << message;                             \
    asrl::detail::throw_exception<exceptionType>(                             \
        "[" #exceptionType "] ", __FUNCTION__, __FILE__, __LINE__,            \
        asrl_assert_stringstream.str());                                      \
  }

#define ASRL_ASSERT_LE(exceptionType, value, upperBound, message)             \
  if (value > upperBound) {                                                   \
    std::stringstream asrl_assert_stringstream;                               \
    asrl_assert_stringstream << "assert(" << #value << " <= " << #upperBound  \
                             << ") failed [" << value << " <= " << upperBound \
                             << "]: " << message;                             \
    asrl::detail::throw_exception<exceptionType>(                             \
        "[" #exceptionType "] ", __FUNCTION__, __FILE__, __LINE__,            \
        asrl_assert_stringstream.str());                                      \
  }

#define ASRL_ASSERT_GT(exceptionType, value, lowerBound, message)            \
  if (value <= lowerBound) {                                                 \
    std::stringstream asrl_assert_stringstream;                              \
    asrl_assert_stringstream << "assert(" << #value << " > " << #lowerBound  \
                             << ") failed [" << value << " > " << lowerBound \
                             << "]: " << message;                            \
    asrl::detail::throw_exception<exceptionType>(                            \
        "[" #exceptionType "] ", __FUNCTION__, __FILE__, __LINE__,           \
        asrl_assert_stringstream.str());                                     \
  }

#define ASRL_ASSERT_EQ(exceptionType, value, testValue, message)             \
  if (value != testValue) {                                                  \
    std::stringstream asrl_assert_stringstream;                              \
    asrl_assert_stringstream << "assert(" << #value << " == " << #testValue  \
                             << ") failed [" << value << " == " << testValue \
                             << "]: " << message;                            \
    asrl::detail::throw_exception<exceptionType>(                            \
        "[" #exceptionType "] ", __FUNCTION__, __FILE__, __LINE__,           \
        asrl_assert_stringstream.str());                                     \
  }

#define ASRL_ASSERT_NE(exceptionType, value, testValue, message)             \
  if (value == testValue) {                                                  \
    std::stringstream asrl_assert_stringstream;                              \
    asrl_assert_stringstream << "assert(" << #value << " != " << #testValue  \
                             << ") failed [" << value << " != " << testValue \
                             << "]: " << message;                            \
    asrl::detail::throw_exception<exceptionType>(                            \
        "[" #exceptionType "] ", __FUNCTION__, __FILE__, __LINE__,           \
        asrl_assert_stringstream.str());                                     \
  }

#define ASRL_ASSERT_EQ_TOL(exceptionType, value, testValue, tolerance,       \
                           message)                                          \
  if (fabs(testValue - value) > fabs(tolerance)) {                           \
    std::stringstream asrl_assert_stringstream;                              \
    asrl_assert_stringstream << "assert(" << #value << " == " << #testValue  \
                             << ") failed [" << value << " == " << testValue \
                             << " (" << fabs(testValue - value) << " > "     \
                             << fabs(tolerance) << ")]: " << message;        \
    asrl::detail::throw_exception<exceptionType>(                            \
        "[" #exceptionType "] ", __FUNCTION__, __FILE__, __LINE__,           \
        asrl_assert_stringstream.str());                                     \
  }

#ifndef NDEBUG

#define ASRL_THROW_DBG(exceptionType, message)                     \
  {                                                                \
    std::stringstream asrl_assert_stringstream;                    \
    asrl_assert_stringstream << message;                           \
    asrl::detail::throw_exception<exceptionType>(                  \
        "[" #exceptionType "] ", __FUNCTION__, __FILE__, __LINE__, \
        asrl_assert_stringstream.str());                           \
  }

#define ASRL_ASSERT_DBG(exceptionType, condition, message)         \
  if (!(condition)) {                                              \
    std::stringstream asrl_assert_stringstream;                    \
    asrl_assert_stringstream << "debug assert(" << #condition      \
                             << ") failed: " << message;           \
    asrl::detail::throw_exception<exceptionType>(                  \
        "[" #exceptionType "] ", __FUNCTION__, __FILE__, __LINE__, \
        asrl_assert_stringstream.str());                           \
  }

#define ASRL_ASSERT_DBG_RE(condition, message) \
  ASRL_ASSERT_DBG(std::runtime_error, condition, message)

#define ASRL_ASSERT_GE_LT_DBG(exceptionType, value, lowerBound, upperBound,   \
                              message)                                        \
  if (value < lowerBound || value >= upperBound) {                            \
    std::stringstream asrl_assert_stringstream;                               \
    asrl_assert_stringstream << "debug assert(" << #lowerBound                \
                             << " <= " << #value << " < " << #upperBound      \
                             << ") failed [" << lowerBound << " <= " << value \
                             << " < " << upperBound << "]: " << message;      \
    asrl::detail::throw_exception<exceptionType>(                             \
        "[" #exceptionType "] ", __FUNCTION__, __FILE__, __LINE__,            \
        asrl_assert_stringstream.str());                                      \
  }

#define ASRL_ASSERT_LT_DBG(exceptionType, value, upperBound, message)         \
  if (value >= upperBound) {                                                  \
    std::stringstream asrl_assert_stringstream;                               \
    asrl_assert_stringstream << "debug assert(" << #value << " < "            \
                             << #upperBound << ") failed [" << value << " < " \
                             << upperBound << "]: " << message;               \
    asrl::detail::throw_exception<exceptionType>(                             \
        "[" #exceptionType "] ", __FUNCTION__, __FILE__, __LINE__,            \
        asrl_assert_stringstream.str());                                      \
  }

#define ASRL_ASSERT_GE_DBG(exceptionType, value, lowerBound, message)          \
  if (value < lowerBound) {                                                    \
    std::stringstream asrl_assert_stringstream;                                \
    asrl_assert_stringstream << "debug assert(" << #value                      \
                             << " >= " << #lowerBound << ") failed [" << value \
                             << " >= " << lowerBound << "]: " << message;      \
    asrl::detail::throw_exception<exceptionType>(                              \
        "[" #exceptionType "] ", __FUNCTION__, __FILE__, __LINE__,             \
        asrl_assert_stringstream.str());                                       \
  }

#define ASRL_ASSERT_LE_DBG(exceptionType, value, upperBound, message)          \
  if (value > upperBound) {                                                    \
    std::stringstream asrl_assert_stringstream;                                \
    asrl_assert_stringstream << "debug assert(" << #value                      \
                             << " <= " << #upperBound << ") failed [" << value \
                             << " <= " << upperBound << "]: " << message;      \
    asrl::detail::throw_exception<exceptionType>(                              \
        "[" #exceptionType "] ", __FUNCTION__, __FILE__, __LINE__,             \
        asrl_assert_stringstream.str());                                       \
  }

#define ASRL_ASSERT_GT_DBG(exceptionType, value, lowerBound, message)         \
  if (value <= lowerBound) {                                                  \
    std::stringstream asrl_assert_stringstream;                               \
    asrl_assert_stringstream << "debug assert(" << #value << " > "            \
                             << #lowerBound << ") failed [" << value << " > " \
                             << lowerBound << "]: " << message;               \
    asrl::detail::throw_exception<exceptionType>(                             \
        "[" #exceptionType "] ", __FUNCTION__, __FILE__, __LINE__,            \
        asrl_assert_stringstream.str());                                      \
  }

#define ASRL_ASSERT_EQ_DBG(exceptionType, value, testValue, message)          \
  if (value != testValue) {                                                   \
    std::stringstream asrl_assert_stringstream;                               \
    asrl_assert_stringstream << "debug assert(" << #value                     \
                             << " == " << #testValue << ") failed [" << value \
                             << " == " << testValue << "]: " << message;      \
    asrl::detail::throw_exception<exceptionType>(                             \
        "[" #exceptionType "] ", __FUNCTION__, __FILE__, __LINE__,            \
        asrl_assert_stringstream.str());                                      \
  }

#define ASRL_ASSERT_NE_DBG(exceptionType, value, testValue, message)          \
  if (value == testValue) {                                                   \
    std::stringstream asrl_assert_stringstream;                               \
    asrl_assert_stringstream << "debug assert(" << #value                     \
                             << " != " << #testValue << ") failed [" << value \
                             << " != " << testValue << "]: " << message;      \
    asrl::detail::throw_exception<exceptionType>(                             \
        "[" #exceptionType "] ", __FUNCTION__, __FILE__, __LINE__,            \
        asrl_assert_stringstream.str());                                      \
  }

#define ASRL_ASSERT_EQ_TOL_DBG(exceptionType, value, testValue, tolerance,   \
                               message)                                      \
  if (fabs(testValue - value) > fabs(tolerance)) {                           \
    std::stringstream asrl_assert_stringstream;                              \
    asrl_assert_stringstream                                                 \
        << "debug assert(" << #value << " == " << #testValue << ") failed [" \
        << value << " == " << testValue << " (" << fabs(testValue - value)   \
        << " > " << fabs(tolerance) << ")]: " << message;                    \
    asrl::detail::throw_exception<exceptionType>(                            \
        "[" #exceptionType "] ", __FUNCTION__, __FILE__, __LINE__,           \
        asrl_assert_stringstream.str());                                     \
  }

#define COUT_P(X) std::cout << #X << ": " << X << std::endl

#else

#define COUT_P(X)
#define ASRL_THROW_DBG(exceptionType, message)
#define ASRL_ASSERT_DBG(exceptionType, condition, message)
#define ASRL_ASSERT_GE_LT_DBG(exceptionType, value, lowerBound, upperBound, \
                              message)
#define ASRL_ASSERT_LT_DBG(exceptionType, value, upperBound, message)
#define ASRL_ASSERT_GT_DBG(exceptionType, value, lowerBound, message)
#define ASRL_ASSERT_LE_DBG(exceptionType, value, upperBound, message)
#define ASRL_ASSERT_GE_DBG(exceptionType, value, lowerBound, message)
#define ASRL_ASSERT_NE_DBG(exceptionType, value, testValue, message)
#define ASRL_ASSERT_EQ_DBG(exceptionType, value, testValue, message)
#define ASRL_ASSERT_EQ_TOL_DBG(exceptionType, value, testValue, tolerance, \
                               message)
#endif

#define SRE_THROW_SFP(exceptionType, SourceFilePos, message) \
  ASRL_THROW_SFP(std::runtime_error, SourceFilePos, message)
#define SRE_ASSERT(condition, message) \
  ASRL_ASSERT(std::runtime_error, condition, message)
#define SRE_ASSERT_GE_LT(value, lowerBound, upperBound, message) \
  ASRL_ASSERT_GE_LT(std::runtime_error, value, lowerBound, upperBound, message)
#define SRE_ASSERT_GE(value, lowerBound, message) \
  ASRL_ASSERT_GE(std::runtime_error, value, lowerBound, message)
#define SRE_ASSERT_LE(value, upperBound, message) \
  ASRL_ASSERT_LE(std::runtime_error, value, upperBound, message)
#define SRE_ASSERT_GT(value, lowerBound, message) \
  ASRL_ASSERT_GT(std::runtime_error, value, lowerBound, message)
#define SRE_ASSERT_EQ_TOL(value, testValue, tolerance, message) \
  ASRL_ASSERT_EQ_TOL(std::runtime_error, value, testValue, tolerance, message)
#define SRE_THROW_DBG(message) ASRL_THROW_DBG(std::runtime_error, message)
#define SRE_ASSERT_GE_LT_DBG(value, lowerBound, upperBound, message)       \
  ASRL_ASSERT_GE_LT_DBG(std::runtime_error, value, lowerBound, upperBound, \
                        message)
#define SRE_ASSERT_LT_DBG(value, upperBound, message) \
  ASRL_ASSERT_LT_DBG(std::runtime_error, value, upperBound, message)
#define SRE_ASSERT_GE_DBG(value, lowerBound, message) \
  ASRL_ASSERT_GE_DBG(std::runtime_error, value, lowerBound, message)
#define SRE_ASSERT_GT_DBG(value, lowerBound, message) \
  ASRL_ASSERT_GT_DBG(std::runtime_error, value, lowerBound, message)
#define SRE_ASSERT_EQ_DBG(value, testValue, message) \
  ASRL_ASSERT_EQ_DBG(std::runtime_error, value, testValue, message)
#define SRE_ASSERT_NE_DBG(value, testValue, message) \
  ASRL_ASSERT_NE_DBG(std::runtime_error, value, testValue, message)
#define SRE_ASSERT_EQ_TOL_DBG(value, testValue, tolerance, message)       \
  ASRL_ASSERT_EQ_TOL_DBG(std::runtime_error, value, testValue, tolerance, \
                         message)
