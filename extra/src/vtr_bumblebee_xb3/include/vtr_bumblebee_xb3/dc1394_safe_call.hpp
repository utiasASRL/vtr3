#pragma once

#include <dc1394/utils.h>

#include <vtr_bumblebee_xb3/assert_macros.hpp>

// Utilities used with the PGR libraries.

template <typename FT, typename T1>
void dc1394SafeCall(std::string const& message, FT fn, T1 t1,
                    asrl::source_file_pos const& sfp) {
  dc1394error_t fe = fn(t1);
  asrl::assert_throw<std::runtime_error>(
      fe == DC1394_SUCCESS,
      message + " dc1394 error: " + dc1394_error_get_string(fe), sfp);
}

template <typename FT, typename T1, typename T2>
void dc1394SafeCall(std::string const& message, FT fn, T1 t1, T2 t2,
                    asrl::source_file_pos const& sfp) {
  dc1394error_t fe = fn(t1, t2);
  asrl::assert_throw<std::runtime_error>(
      fe == DC1394_SUCCESS,
      message + " dc1394 error: " + dc1394_error_get_string(fe), sfp);
}

template <typename FT, typename T1, typename T2, typename T3>
void dc1394SafeCall(std::string const& message, FT fn, T1 t1, T2 t2, T3 t3,
                    asrl::source_file_pos const& sfp) {
  dc1394error_t fe = fn(t1, t2, t3);
  asrl::assert_throw<std::runtime_error>(
      fe == DC1394_SUCCESS,
      message + " dc1394 error: " + dc1394_error_get_string(fe), sfp);
}

template <typename FT, typename T1, typename T2, typename T3, typename T4>
void dc1394SafeCall(std::string const& message, FT fn, T1 t1, T2 t2, T3 t3,
                    T4 t4, asrl::source_file_pos const& sfp) {
  dc1394error_t fe = fn(t1, t2, t3, t4);
  asrl::assert_throw<std::runtime_error>(
      fe == DC1394_SUCCESS,
      message + " dc1394 error: " + dc1394_error_get_string(fe), sfp);
}

template <typename FT, typename T1, typename T2, typename T3, typename T4,
          typename T5>
void dc1394SafeCall(std::string const& message, FT fn, T1 t1, T2 t2, T3 t3,
                    T4 t4, T5 t5, asrl::source_file_pos const& sfp) {
  dc1394error_t fe = fn(t1, t2, t3, t4, t5);
  asrl::assert_throw<std::runtime_error>(
      fe == DC1394_SUCCESS,
      message + " dc1394 error: " + dc1394_error_get_string(fe), sfp);
}

template <typename FT, typename T1, typename T2, typename T3, typename T4,
          typename T5, typename T6>
void dc1394SafeCall(std::string const& message, FT fn, T1 t1, T2 t2, T3 t3,
                    T4 t4, T5 t5, T6 t6, asrl::source_file_pos const& sfp) {
  dc1394error_t fe = fn(t1, t2, t3, t4, t5, t6);
  asrl::assert_throw<std::runtime_error>(
      fe == DC1394_SUCCESS,
      message + " dc1394 error: " + dc1394_error_get_string(fe), sfp);
}

template <typename FT, typename T1, typename T2, typename T3, typename T4,
          typename T5, typename T6, typename T7>
void dc1394SafeCall(std::string const& message, FT fn, T1 t1, T2 t2, T3 t3,
                    T4 t4, T5 t5, T6 t6, T7 t7,
                    asrl::source_file_pos const& sfp) {
  dc1394error_t fe = fn(t1, t2, t3, t4, t5, t6, t7);
  asrl::assert_throw<std::runtime_error>(
      fe == DC1394_SUCCESS,
      message + " dc1394 error: " + dc1394_error_get_string(fe), sfp);
}

template <typename FT, typename T1, typename T2, typename T3, typename T4,
          typename T5, typename T6, typename T7, typename T8>
void dc1394SafeCall(std::string const& message, FT fn, T1 t1, T2 t2, T3 t3,
                    T4 t4, T5 t5, T6 t6, T7 t7, T8 t8,
                    asrl::source_file_pos const& sfp) {
  dc1394error_t fe = fn(t1, t2, t3, t4, t5, t6, t7, t8);
  asrl::assert_throw<std::runtime_error>(
      fe == DC1394_SUCCESS,
      message + " dc1394 error: " + dc1394_error_get_string(fe), sfp);
}

template <typename FT, typename T1, typename T2, typename T3, typename T4,
          typename T5, typename T6, typename T7, typename T8, typename T9>
void dc1394SafeCall(std::string const& message, FT fn, T1 t1, T2 t2, T3 t3,
                    T4 t4, T5 t5, T6 t6, T7 t7, T8 t8, T9 t9,
                    asrl::source_file_pos const& sfp) {
  dc1394error_t fe = fn(t1, t2, t3, t4, t5, t6, t7, t8, t9);
  asrl::assert_throw<std::runtime_error>(
      fe == DC1394_SUCCESS,
      message + " dc1394 error: " + dc1394_error_get_string(fe), sfp);
}
