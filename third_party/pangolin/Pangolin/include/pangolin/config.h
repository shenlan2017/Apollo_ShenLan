#ifndef PANGOLIN_CONFIG_H
#define PANGOLIN_CONFIG_H

/*
 * Configuration Header for Pangolin
 */

/// Version
#define PANGOLIN_VERSION_MAJOR 0
#define PANGOLIN_VERSION_MINOR 6
#define PANGOLIN_VERSION_STRING "0.6"

/// Pangolin options
#define BUILD_PANGOLIN_GUI
#define BUILD_PANGOLIN_VARS
#define BUILD_PANGOLIN_VIDEO

/// Configured libraries
/* #undef HAVE_CUDA */
/* #undef HAVE_PYTHON */

/* #undef HAVE_EIGEN */
/* #undef HAVE_TOON */

/* #undef HAVE_DC1394 */
/* #undef HAVE_V4L */
/* #undef HAVE_OPENNI */
/* #undef HAVE_LIBREALSENSE */
/* #undef HAVE_OPENNI2 */
/* #undef HAVE_UVC */
/* #undef HAVE_DEPTHSENSE */
/* #undef HAVE_TELICAM */
/* #undef HAVE_PLEORA */

/* #undef HAVE_FFMPEG */
/* #undef HAVE_FFMPEG_MAX_ANALYZE_DURATION2 */
/* #undef HAVE_FFMPEG_AVFORMAT_ALLOC_OUTPUT_CONTEXT2 */
/* #undef HAVE_FFMPEG_AVPIXELFORMAT */

/* #undef HAVE_GLEW */
/* #undef GLEW_STATIC */

/* #undef HAVE_APPLE_OPENGL_FRAMEWORK */
/* #undef HAVE_GLES */
/* #undef HAVE_GLES_2 */
/* #undef HAVE_OCULUS */

/* #undef HAVE_PNG */
/* #undef HAVE_JPEG */
/* #undef HAVE_TIFF */
/* #undef HAVE_OPENEXR */
/* #undef HAVE_ZSTD */
/* #undef HAVE_LZ4 */

/// Platform
#define _UNIX_
/* #undef _WIN_ */
/* #undef _OSX_ */
#define _LINUX_
/* #undef _ANDROID_ */
/* #undef _IOS_ */

/// Compiler
#define _GCC_
/* #undef _CLANG_ */
/* #undef _MSVC_ */

#if (__cplusplus > 199711L) || (_MSC_VER >= 1800)
#define CALLEE_HAS_VARIADIC_TEMPLATES
#endif

#endif //PANGOLIN_CONFIG_H
