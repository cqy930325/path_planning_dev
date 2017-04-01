/** PerceptIn license **/

#ifndef UTIL_INCLUDE_UTIL_DEBUG_H_
#define UTIL_INCLUDE_UTIL_DEBUG_H_

/**
 * Define a macro function DEBUG(x) to process the enclosed code only when the
 * __DEBUG__ flag is defined. The __DEBUG__ flag guards information that is only
 * intended to be used for sanity checks to catch segfaults and undesired
 * behaviors in the code.
 *
 * @note: Internal CHECKs for sanity checks should be guarded under DEBUG, while
 *        all the GLog messages should be guarded under INTERNAL.
 */
#ifdef __DEBUG__
#define DEBUG(x) x
#else
#define DEBUG(x)
#endif  // __DEBUG__

/**
 * Define a macro function INTERNAL(x) to process the enclosed code only when
 * the __INTERNAL__ flag is defined. The __INTERNAL__ flag guards information
 * that is only intended to be viewed internally for algorithm evaluation or
 * debugging.
 *
 * @note: All the GLog messages should be guarded under INTERNAL, while internal
 *        CHECKs should be guarded by DEBUG.
 */
#ifdef __INTERNAL__
#define INTERNAL(x) x
#else
#define INTERNAL(x)
#endif  // __INTERNAL__

/**
 * Define a macro function JNI_LOG(x) to process the enclosed code only when the
 * __ANDROID__ flag is defined. The __ANDROID__ is predefined in Android JNI
 * compiler.
 *
 * Define JNI_LOG_ID_MONO_SDK2 to be a string to identify the log in monoSDK2.
 */
#ifdef __ANDROID__
#define JNI_LOG(x) x
#define JNI_LOG_ID_MONO_SDK2 "io.perceptin.monoSDK2 - "
#else
#define JNI_LOG(x)
#endif  // __ANDROID__

/**
 * Define a macro function PUBLIC(x) to process the enclosed code only when
 * __PUBLIC__ flag is defined. The __PUBLIC__ flag guards information to be
 * printed on the screen for developers outside of the company.
 */
#ifdef __PUBLIC__
#define PUBLIC(x) x
#else
#define PUBLIC(x)
#endif  // __PUBLIC__

/**
 * Define a macro function VIZ(x) to process the enclosed code only when the
 * __VIZ__ flag is defined. The __VIZ__ flag guards visualization using
 * cv::imshow or cv::imwrite.
 */
#ifdef __VIZ__
#define VIZ(x) x
#else
#define VIZ(x)
#endif  // __VIZ__

#endif  // UTIL_INCLUDE_UTIL_DEBUG_H_
