
#ifndef TRIANGLE_EXPORT_H
#define TRIANGLE_EXPORT_H

#define TRIANGLE_STATIC_DEFINE

#ifdef TRIANGLE_STATIC_DEFINE
#  define TRIANGLE_EXPORT
#  define TRIANGLE_NO_EXPORT
#else
#  ifndef TRIANGLE_EXPORT
#    ifdef triangle_api_EXPORTS
        /* We are building this library */
#      define TRIANGLE_EXPORT __attribute__((visibility("default")))
#    else
        /* We are using this library */
#      define TRIANGLE_EXPORT __attribute__((visibility("default")))
#    endif
#  endif

#  ifndef TRIANGLE_NO_EXPORT
#    define TRIANGLE_NO_EXPORT __attribute__((visibility("hidden")))
#  endif
#endif

#ifndef TRIANGLE_DEPRECATED
#  define TRIANGLE_DEPRECATED __attribute__ ((__deprecated__))
#endif

#ifndef TRIANGLE_DEPRECATED_EXPORT
#  define TRIANGLE_DEPRECATED_EXPORT TRIANGLE_EXPORT TRIANGLE_DEPRECATED
#endif

#ifndef TRIANGLE_DEPRECATED_NO_EXPORT
#  define TRIANGLE_DEPRECATED_NO_EXPORT TRIANGLE_NO_EXPORT TRIANGLE_DEPRECATED
#endif

#if 0 /* DEFINE_NO_DEPRECATED */
#  ifndef TRIANGLE_NO_DEPRECATED
#    define TRIANGLE_NO_DEPRECATED
#  endif
#endif

#endif /* TRIANGLE_EXPORT_H */
