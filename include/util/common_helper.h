/** PerceptIn license **/

#ifndef UTIL_INCLUDE_UTIL_COMMON_HEALPER_H_
#define UTIL_INCLUDE_UTIL_COMMON_HEALPER_H_

#include <sstream>
#include <string>

namespace PI {
namespace util {

/**
 * @brief Convert any value to string with << operator.
 * @note Use this function instead of std::to_string() because std::to_string()
 *       cannot be compiled in JNI.
 */
template <typename T>
std::string ToString(T value) {
  std::ostringstream os ;
  os << value ;
  return os.str() ;
}

/**
 *
 */
void ParseGflagsParam(const int argc, char * const * const argv);

}  // namespace util
}  // namespace PI

#endif  // UTIL_INCLUDE_UTIL_COMMON_HEALPER_H_
