/*
 * Unpublished Work Copyright 2019 Waymo LLC.  All rights reserved.
 * Waymo Proprietary and Confidential - Contains Trade Secrets
 *
 * This is the proprietary software of Waymo LLC ("Waymo") and/or its licensors,
 * and may only be used, duplicated, modified or distributed pursuant to the
 * terms and conditions of a separate, written license agreement executed
 * between you and Waymo (an "Authorized License"). Except as set forth in an
 * Authorized License, Waymo grants no license (express or implied), right to
 * use, or waiver of any kind with respect to the Software, and Waymo expressly
 * reserves all rights in and to the Software and all intellectual property
 * rights therein. IF YOU HAVE NO AUTHORIZED LICENSE, THEN YOU HAVE NO RIGHT TO
 * USE THIS SOFTWARE IN ANY WAY, AND SHOULD IMMEDIATELY NOTIFY WAYMO AND
 * DISCONTINUE ALL USE OF THE SOFTWARE.
 *
 * Except as expressly set forth in the Authorized License:
 *
 * 1. This software includes trade secrets of Waymo, and you shall use all
 * reasonable efforts to protect the confidentiality thereof.  You shall use
 * this software only in connection with your authorized use of Waymo products.
 *
 * 2. TO THE MAXIMUM EXTENT PERMITTED BY APPLICABLE LAW, WAYMO PROVIDES THE
 * WAYMO SOFTWARE ON AN “AS IS” AND “AS AVAILABLE” BASIS WITH ALL FAULTS AND
 * WITHOUT ANY REPRESENTATIONS OR WARRANTIES OF ANY KIND.  WAYMO EXPRESSLY
 * DISCLAIMS ALL WARRANTIES, WHETHER IMPLIED, STATUTORY OR OTHERWISE, INCLUDING
 * IMPLIED WARRANTIES OF MERCHANTABILITY, NON-INFRINGEMENT AND FITNESS FOR
 * PARTICULAR PURPOSE OR ARISING FROM COURSE OF PERFORMANCE, COURSE OF DEALING
 * OR USAGE OF TRADE.  SOME JURISDICTIONS DO NOT ALLOW THE EXCLUSION OF CERTAIN
 * WARRANTIES, REPRESENTATIONS OR CONDITIONS, THE LIMITATION OR EXCLUSION OF
 * IMPLIED WARRANTIES, LIMITATIONS ON HOW LONG AN IMPLIED WARRANTY MAY LAST OR
 * EXCLUSIONS OR LIMITATIONS FOR CONSEQUENTIAL OR INCIDENTAL DAMAGES, SO SOME OF
 * THE ABOVE LIMITATIONS MAY NOT APPLY IN FULL TO YOU, AND WAYMO’S LIABILITY
 * SHALL BE LIMITED TO THE EXTENT SUCH LIMITATIONS ARE PERMITTED BY LAW.
 *
 * 3. TO THE MAXIMUM EXTENT PERMITTED UNDER APPLICABLE LAW, IN NO EVENT WILL
 * WAYMO OR ITS LICENSORS BE LIABLE FOR ANY DIRECT, INDIRECT, SPECIAL,
 * INCIDENTAL, EXEMPLARY, PUNITIVE OR CONSEQUENTIAL DAMAGES OF ANY KIND ARISING
 * OUT OF OR IN CONNECTION WITH THE WAYMO SOFTWARE, REGARDLESS OF THE FORM OF
 * ACTION, WHETHER IN CONTRACT, TORT (INCLUDING NEGLIGENCE), STRICT LIABILITY OR
 * OTHERWISE, EVEN IF WAYMO HAS BEEN ADVISED OR IS OTHERWISE AWARE OF THE
 * POSSIBILITY OF SUCH DAMAGES. THE FOREGOING LIMITATIONS, EXCLUSIONS, AND
 * DISCLAIMERS WILL APPLY TO THE MAXIMUM EXTENT PERMITTED BY APPLICABLE LAW,
 * EVEN IF ANY REMEDY FAILS ITS ESSENTIAL PURPOSE.
 */
#include "hc/pitch_table_utils.h"

#include <algorithm>
#include <sstream>

namespace honeycomb {
namespace {

std::vector<std::string> StringSplit(std::string str, std::string del) {
  std::vector<std::string> result;
  size_t last = 0;
  size_t index;

  do {
    index = str.find(del, last);
    result.push_back(str.substr(last, index - last));
    last = index + del.size();
  } while (index != std::string::npos);

  return result;
}

bool ConvertToDouble(std::string s, double* d) {
  std::istringstream buf(s);
  buf >> *d;
  if (buf.fail()) {
    return false;
  }
  return true;
}

std::vector<double> RangeVector(double start, double end, double step,
                                std::string* error_str) {
  constexpr int kMaxRangeSteps = 1000;

  // Make sure the range will terminate
  if (step == 0 || ((end - start) / step) < 0) {
    *error_str = "Range will not terminate.";
    return std::vector<double>();
  }

  int steps = static_cast<int>((end - start) / step) + 1;
  if (steps > kMaxRangeSteps) {
    *error_str = "Range exceeds maximum size (" +
                 std::to_string(kMaxRangeSteps) + ").";  // NOLINT
    return std::vector<double>();
  }

  std::vector<double> v(steps);
  double n = start - step;
  std::generate(v.begin(), v.end(), [n, step]() mutable { return n += step; });
  return v;
}

std::vector<double> ExpandRange(std::vector<std::string> range_str,
                                std::string* error_str) {
  std::vector<double> values;
  const std::vector<double> kEmpty;
  double d;

  for (const auto& str : range_str) {
    if (ConvertToDouble(str, &d)) {
      values.push_back(d);
    } else {
      *error_str = "Unable to convert '" + str + "' to a number.";
      return kEmpty;
    }
  }

  if (range_str.empty() || range_str.size() > 3 ||
      range_str.size() != values.size()) {
    *error_str = "Range expression has the wrong number of terms.";
    return kEmpty;
  }

  double start = values[0];
  double end = values.size() > 1 ? values[1] : start;
  double step = values.size() > 2 ? values[2] : 1.0;
  return RangeVector(start, end, step, error_str);
}

}  // namespace

std::vector<double> ParsePitchTableString(std::string pitch_table_str,
                                          std::string* error_str) {
  std::vector<std::string> clumps;
  std::vector<std::vector<std::string>> ranges(clumps.size());

  *error_str = "";

  clumps = StringSplit(pitch_table_str, ",");
  for (const auto& clump : clumps) {
    ranges.push_back(StringSplit(clump, ":"));
  }

  std::vector<double> pitch_table;

  for (const auto& range : ranges) {
    auto v = ExpandRange(range, error_str);
    if (v.empty()) {
      return std::vector<double>();
    }
    pitch_table.insert(pitch_table.end(), v.begin(), v.end());
  }
  std::sort(pitch_table.begin(), pitch_table.end(), std::greater<double>());
  return pitch_table;
}
}  // namespace honeycomb
