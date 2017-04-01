/** PerceptIn license **/

#ifndef UTIL_INCLUDE_UTIL_KBHIT_H_
#define UTIL_INCLUDE_UTIL_KBHIT_H_

namespace PI {
namespace util {
namespace kbhit {

/**
 * @brief Checks if there is something in the input buffer
 * @return
 */
int kbhit();

/**
 * @brief Set the input buffer.
 */
void init();

/**
 * @brief Clear the input buffer.
 */
void finish();

}  // namespace kbhit
}  // namespace util
}  // namespace PI

#endif  // UTIL_INCLUDE_UTIL_KBHIT_H_
